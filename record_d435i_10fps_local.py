#!/usr/bin/env python3
"""D435i 本机单文件录包：USB 相机直接连接本机，30 FPS 采集后同步抽帧录成 10 FPS bag。"""
import argparse
import collections
import copy
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import select
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

RGB = "/camera_10fps/color/image_raw"
DEPTH = "/camera_10fps/aligned_depth_to_color/image_raw"
STOP = threading.Event()

def say(message):
    print(message, flush=True)


def save(path, obj):
    path = Path(path)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n")
    temp.replace(path)


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def on_stop(signum, frame):
    STOP.set()


class Images:
    def __init__(self, rgb=RGB, depth=DEPTH):
        self.rgb, self.depth = rgb, depth
        self.streams = {t: {"count": 0, "stamps": [], "errors": [],
                            "depth_valid_sum": 0.0} for t in (rgb, depth)}

    def add(self, topic, msg):
        import numpy as np
        s = self.streams[topic]
        s["count"] += 1
        try:
            if msg._type != "sensor_msgs/Image":
                raise ValueError("类型不是 sensor_msgs/Image")
            stamp = msg.header.stamp.to_nsec()
            if stamp <= 0 or (s["stamps"] and stamp <= s["stamps"][-1]):
                raise ValueError("时间戳为零、重复或倒退")
            shape = [msg.width, msg.height, msg.encoding, msg.header.frame_id]
            if "shape" in s and s["shape"] != shape:
                raise ValueError("尺寸/编码/frame_id 在流中变化")
            s["shape"] = shape
            if not msg.header.frame_id or msg.width <= 0 or msg.height <= 0:
                raise ValueError("尺寸或 frame_id 为空")
            encodings = {"rgb8": 3, "bgr8": 3, "mono8": 1, "rgba8": 4, "bgra8": 4}
            pixel = {"32FC1": 4, "16UC1": 2} if topic == self.depth else encodings
            if msg.encoding not in pixel:
                raise ValueError("不支持的图像编码: " + msg.encoding)
            if msg.step < msg.width * pixel[msg.encoding] or len(msg.data) != msg.step * msg.height:
                raise ValueError("图像 data/step/尺寸不匹配")
            if topic == self.depth:
                dt = (">" if msg.is_bigendian else "<") + ("f4" if msg.encoding == "32FC1" else "u2")
                depth = np.ndarray((msg.height, msg.width), dtype=dt, buffer=msg.data,
                                   strides=(msg.step, pixel[msg.encoding]))[::8, ::8]
                valid = float(np.mean(np.isfinite(depth) & (depth > 0)))
                s["depth_valid_sum"] += valid
            s["stamps"].append(stamp)
        except Exception as exc:
            if len(s["errors"]) < 10:
                s["errors"].append(str(exc))

    def report(self, min_frames=10, min_hz=3.0, sync_ms=5.0, min_pair_ratio=0.95):
        errors, result = [], {}
        for topic, s in self.streams.items():
            stamps = s["stamps"]
            span = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0
            hz = (len(stamps) - 1) / span if span > 0 else 0
            result[topic] = {"messages": s["count"], "hz_by_header": hz,
                             "shape": s.get("shape"), "errors": s["errors"]}
            if s["count"] < min_frames or hz < min_hz or s["errors"]:
                errors.append(topic + ": 消息不足、低帧率或图像/时间戳异常")
        a, b = self.streams[self.rgb], self.streams[self.depth]
        if a.get("shape") and b.get("shape"):
            if a["shape"][:2] != b["shape"][:2] or a["shape"][3] != b["shape"][3]:
                errors.append("RGB/depth 尺寸或 frame_id 不一致；需要对齐深度")
        valid = b["depth_valid_sum"] / max(1, b["count"])
        if valid < 0.01:
            errors.append("深度有效像素均值不足 1%")
        left, right = a["stamps"], b["stamps"]
        i = j = paired = 0
        tolerance = sync_ms * 1e6
        while i < len(left) and j < len(right):
            delta = left[i] - right[j]
            if abs(delta) <= tolerance:
                paired += 1
                i += 1
                j += 1
            elif delta < 0:
                i += 1
            else:
                j += 1
        ratio = paired / max(1, a["count"], b["count"])
        if ratio < min_pair_ratio:
            errors.append("RGB/depth 时间戳配对率不足 %.1f%%" % (100 * min_pair_ratio))
        return {"ok": not errors, "errors": errors, "topics": result,
                "paired_frames": paired, "pair_ratio": ratio,
                "sync_tolerance_ms": sync_ms, "depth_valid_fraction": valid}


def probe(args):
    import rospy
    from sensor_msgs.msg import Image
    stats = Images(args.rgb_topic, args.depth_topic)
    lock = threading.Lock()
    seen = set()
    ready_at = None
    first = {}
    last = {}

    def callback(msg, topic):
        nonlocal ready_at
        with lock:
            seen.add(topic)
            if len(seen) == 2 and ready_at is None:
                ready_at = time.monotonic()
            if ready_at is None or time.monotonic() - ready_at < args.warmup:
                return
            stats.add(topic, msg)
            first.setdefault(topic, time.monotonic())
            last[topic] = time.monotonic()

    rospy.init_node("rgbd_preflight", anonymous=True, disable_signals=True)
    subs = [rospy.Subscriber(t, Image, callback, callback_args=t, queue_size=20,
                            buff_size=16 * 1024 * 1024, tcp_nodelay=True)
            for t in stats.streams]
    deadline = time.monotonic() + args.timeout
    start = None
    while time.monotonic() < deadline and not STOP.is_set():
        with lock:
            if len(first) == 2 and start is None:
                start = time.monotonic()
        if start and time.monotonic() - start >= 5:
            break
        time.sleep(0.1)
    for sub in subs:
        sub.unregister()
    with lock:
        report = stats.report(min_hz=args.min_hz, sync_ms=args.sync_ms)
        wall_rates = {}
        for t, s in stats.streams.items():
            span = last.get(t, 0) - first.get(t, 0)
            hz = (s["count"] - 1) / span if span > 0 else 0
            wall_rates[t] = hz
            if hz < args.min_hz or time.monotonic() - last.get(t, 0) > 2:
                report["errors"].append(t + ": 接收帧率不足或数据已停流")
        report["receive_hz"] = wall_rates
        report["ok"] = not report["errors"] and not STOP.is_set()
    save(args.report, report)
    rospy.signal_shutdown("probe complete")
    say(json.dumps(report, ensure_ascii=False))
    return 0 if report["ok"] else 1


def check(args):
    import rosbag
    stats = Images(args.rgb_topic, args.depth_topic)
    counts = collections.Counter()
    report = {"ok": False, "bag": str(args.bag)}
    try:
        if str(args.bag).endswith(".active"):
            raise ValueError("bag 仍是 .active，未正常封包")
        with rosbag.Bag(str(args.bag), "r", allow_unindexed=False) as bag:
            expected = bag.get_message_count()
            for topic, msg, stamp in bag.read_messages():
                counts[topic] += 1
                if topic in stats.streams:
                    stats.add(topic, msg)
            if sum(counts.values()) != expected:
                raise ValueError("顺序读取数与索引消息数不同")
        report.update(stats.report(min_hz=args.min_hz, sync_ms=args.sync_ms))
        report.update({"all_topic_counts": dict(counts), "full_read_messages": sum(counts.values()),
                       "bytes": Path(args.bag).stat().st_size})
    except Exception as exc:
        report.update({"ok": False, "errors": [str(exc)], "all_topic_counts": dict(counts)})
    save(args.report, report)
    say(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 1


def start(command, log, env=None):
    with open(log, "wb") as out:
        return subprocess.Popen(command, stdout=out, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, env=env, start_new_session=True)


def stop_process(proc, grace=30):
    """Signal only our session, including descendants if the leader exited."""
    if proc is None:
        return True
    graceful = True
    for sig, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            proc.poll()
            try:
                os.killpg(proc.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.1)
        else:
            graceful = False
            continue
        break
    proc.wait(timeout=5)
    return graceful


def wait_for(test, seconds, children=()):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if STOP.is_set():
            raise RuntimeError("用户请求停止")
        if any(p.poll() is not None for p in children):
            raise RuntimeError("子进程提前退出，请查看日志")
        if test():
            return
        time.sleep(0.2)
    raise RuntimeError("等待就绪超时")


def positive(value):
    number = float(value)
    if not 0 < number < float("inf"):
        raise argparse.ArgumentTypeError("必须为有限正数")
    return number


CAMERA_LAUNCH = '''<launch>
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="camera" value="camera"/>
    <arg name="color_width" value="640"/>
    <arg name="color_height" value="480"/>
    <arg name="color_fps" value="30"/>
    <arg name="enable_color" value="true"/>
    <arg name="depth_width" value="640"/>
    <arg name="depth_height" value="480"/>
    <arg name="depth_fps" value="30"/>
    <arg name="enable_depth" value="true"/>
    <arg name="enable_infra" value="false"/>
    <arg name="enable_infra1" value="false"/>
    <arg name="enable_infra2" value="false"/>
    <arg name="enable_gyro" value="false"/>
    <arg name="enable_accel" value="false"/>
    <arg name="enable_sync" value="true"/>
    <arg name="align_depth" value="true"/>
    <arg name="enable_pointcloud" value="false"/>
    <arg name="initial_reset" value="false"/>
  </include>
</launch>
'''


def write_camera_files(run):
    launch = run / "d435i_camera_30fps.launch"
    launch.write_text(CAMERA_LAUNCH)
    script = run / "start_rgbd10.sh"
    script.write_text("\n".join([
        "#!/usr/bin/env bash", "set -e",
        shlex.join([sys.executable, str(Path(__file__).resolve()), "decimate"]) + " &",
        "decimator_pid=$!", shlex.join(["roslaunch", str(launch)]) + " &",
        "driver_pid=$!", 'wait -n "$decimator_pid" "$driver_pid"', "exit 1", "",
    ]))
    script.chmod(0o755)
    return script


class RgbdDecimator:
    """Select pairs 1, 4, 7, ... after exact RGB/depth timestamp matching."""

    def __init__(self):
        import message_filters
        import rospy
        from sensor_msgs.msg import CameraInfo, Image

        self.lock = threading.Lock()
        self.pairs = 0
        self.info = {}
        self.color = rospy.Publisher(RGB, Image, queue_size=2)
        self.depth = rospy.Publisher(DEPTH, Image, queue_size=2)
        self.info_pubs = {
            "color": rospy.Publisher("/camera_10fps/color/camera_info", CameraInfo, queue_size=2),
            "depth": rospy.Publisher("/camera_10fps/aligned_depth_to_color/camera_info", CameraInfo, queue_size=2),
        }
        self.info_subs = [
            rospy.Subscriber(topic, CameraInfo, self.info_callback, callback_args=key, queue_size=5)
            for key, topic in (("color", "/camera/color/camera_info"),
                               ("depth", "/camera/aligned_depth_to_color/camera_info"))]
        self.subs = [message_filters.Subscriber(topic, Image, queue_size=10,
                                               buff_size=16 * 1024 * 1024)
                     for topic in ("/camera/color/image_raw", "/camera/aligned_depth_to_color/image_raw")]
        self.sync = message_filters.TimeSynchronizer(self.subs, 30)
        self.sync.registerCallback(self.pair_callback)

    def info_callback(self, msg, key):
        with self.lock:
            self.info[key] = msg

    def pair_callback(self, color, depth):
        with self.lock:
            index = self.pairs
            self.pairs += 1
            if index % 3:
                return
            self.color.publish(color)
            self.depth.publish(depth)
            for key, msg in (("color", color), ("depth", depth)):
                if key in self.info:
                    info = copy.deepcopy(self.info[key])
                    info.header = msg.header
                    self.info_pubs[key].publish(info)


def decimate():
    import rospy
    rospy.init_node("rgbd_portable_decimator")
    node = RgbdDecimator()
    rospy.loginfo("RGB-D exact-time 30 -> 10 FPS adapter ready")
    rospy.spin()
    return 0


def bootstrap_ros(args):
    ros_setup = Path(args.ros_setup).expanduser().resolve()
    if not ros_setup.is_file():
        raise RuntimeError("未找到本机 ROS1 环境文件，请用 --ros-setup 指定：" + str(ros_setup))
    lines = ["set -e", 'record_args=("$@")', "set --", "source " + shlex.quote(str(ros_setup))]
    if args.camera_setup:
        camera_setup = Path(args.camera_setup).expanduser().resolve()
        if not camera_setup.is_file():
            raise RuntimeError("未找到本机 RealSense 驱动环境文件：" + str(camera_setup))
        lines.append("source " + shlex.quote(str(camera_setup)))
    lines.append("exec " + shlex.join(["/usr/bin/python3", str(Path(__file__).resolve()), "--ros-ready"]) +
                 ' "${record_args[@]}"')
    os.execv("/bin/bash", ["bash", "-c", "\n".join(lines), "d435i-local-env"] + sys.argv[1:])


def validate_environment():
    if sys.version_info < (3, 8):
        raise RuntimeError("需要 Python 3.8 或更高版本")
    if os.environ.get("ROS_VERSION") != "1":
        raise RuntimeError("本机需要 ROS1，请用 --ros-setup 指定 ROS1 setup.bash")
    try:
        import rosbag
        import rospy
        import message_filters
        import numpy
        import roslz4
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as exc:
        raise RuntimeError("缺少 ROS1 Python 依赖（rosbag/rospy/message_filters/sensor_msgs/NumPy/LZ4）：" + str(exc)) from exc
    for command in ("bash", "roscore", "roslaunch", "rosbag", "rospack"):
        if not shutil.which(command):
            raise RuntimeError("本机缺少命令：" + command)
    package = subprocess.run(["rospack", "find", "realsense2_camera"], capture_output=True, text=True)
    if package.returncode:
        raise RuntimeError("未找到 RealSense ROS1 驱动；自编译安装时请传 --camera-setup /工作空间/devel/setup.bash\n" +
                           package.stderr.strip())
    path = Path(package.stdout.strip())
    if not (path / "launch/rs_camera.launch").is_file():
        raise RuntimeError("RealSense ROS1 驱动缺少 launch/rs_camera.launch：" + str(path))
    say("本机环境检查通过：ROS1、Python 依赖和 RealSense 驱动已找到。")
    say("相机驱动目录：" + str(path))


def record_local(args):
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    with open(output / ".d435i_record.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("该保存目录已有录制会话运行") from exc
        run = output / ("d435i10_local_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:6])
        run.mkdir()
        save(run / "config.json", vars(args))
        camera_script = write_camera_files(run)
        master = camera = recorder = preflight = None
        report = {"ok": False, "run": str(run), "errors": []}
        bag = run / "rgbd.bag"
        active = Path(str(bag) + ".active")
        say("本机录制目录：" + str(run))

        try:
            for proc in Path("/proc").glob("[0-9]*/cmdline"):
                try:
                    command = " ".join(os.fsdecode(item) for item in proc.read_bytes().split(b"\0") if item)
                except OSError:
                    continue
                if command and any(pattern in command for pattern in args.busy_process):
                    raise RuntimeError("已有 RealSense 相机或抽帧进程运行，请先结束该录制会话")
            if shutil.disk_usage(run).free < args.reserve_gb * 1024**3:
                raise RuntimeError("本机剩余空间不足保留值 %.2f GiB" % args.reserve_gb)
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            env = os.environ.copy()
            env.pop("ROS_HOSTNAME", None)
            env.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port, ROS_IP="127.0.0.1",
                       ROS_HOME=str(run / "ros_home"), ROS_LOG_DIR=str(run / "ros_logs"),
                       NO_PROXY="localhost,127.0.0.1", no_proxy="localhost,127.0.0.1")
            master = start(["roscore", "-p", str(port)], run / "master.log", env)

            def master_ready():
                try:
                    with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                        return True
                except OSError:
                    return False

            wait_for(master_ready, 20, (master,))
            camera = start(["bash", str(camera_script)], run / "camera.log", env)
            preflight = start([
                sys.executable, str(Path(__file__).resolve()), "probe", "--report", str(run / "stream_check.json"),
                "--rgb-topic", RGB, "--depth-topic", DEPTH, "--min-hz", str(args.min_hz),
                "--sync-ms", str(args.sync_ms), "--warmup", str(args.warmup), "--timeout", str(args.timeout)],
                run / "stream_check.log", env)
            say("相机正在以 640×480@30 FPS 采集，等待 10 FPS RGB-D 预热和稳定数据检查……")
            wait_for(lambda: preflight.poll() is not None, args.timeout + 10, (camera, master))
            if preflight.returncode:
                raise RuntimeError("RGB-D 数据检查失败，请查看 stream_check.json / camera.log")
            topics = [RGB, DEPTH, "/camera_10fps/color/camera_info",
                      "/camera_10fps/aligned_depth_to_color/camera_info", "/tf", "/tf_static"]
            topics = list(dict.fromkeys(topics + args.extra_topic))
            recorder = start(["rosbag", "record", "--lz4", "--buffsize=128", "--chunksize=2048",
                              "--min-space=%dM" % (args.reserve_gb * 1024), "-O", str(bag)] + topics,
                             run / "record.log", env)
            wait_for(lambda: active.exists() and active.stat().st_size > 4096, 20, (master, camera, recorder))
            say("RECORDING: 已开始本机录制。按 Enter 或 Ctrl+C 停止。")
            report["started_at"] = datetime.datetime.now().isoformat()
            began = last_growth = last_status = time.monotonic()
            last_size = active.stat().st_size
            while not STOP.wait(0.2):
                if args.duration and time.monotonic() - began >= args.duration:
                    STOP.set()
                    break
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    STOP.set()
                    break
                if any(proc.poll() is not None for proc in (master, camera, recorder)):
                    raise RuntimeError("相机/ROS/录制进程意外退出")
                if shutil.disk_usage(run).free < args.reserve_gb * 1024**3:
                    raise RuntimeError("磁盘接近保留空间，停止录制并封包")
                size = active.stat().st_size if active.exists() else 0
                if size > last_size:
                    last_size, last_growth = size, time.monotonic()
                if time.monotonic() - last_growth > 15:
                    raise RuntimeError("bag 15 秒没有增长，数据流或写盘异常")
                if time.monotonic() - last_status >= 10:
                    say("录制中：%.0f 秒，bag %.0f MiB，剩余 %.2f GiB。" %
                        (time.monotonic() - began, size / 1024**2, shutil.disk_usage(run).free / 1024**3))
                    last_status = time.monotonic()
            report["ok"] = True
        except Exception as exc:
            report["errors"].append(str(exc))
            say("录制停止：" + str(exc))
        finally:
            say("正在封包、关闭本次启动的相机/ROS 进程，并完整检查 bag……")
            for name, proc, grace in (("录制器", recorder, 90), ("数据检查", preflight, 30),
                                      ("相机/抽帧", camera, 30), ("ROS master", master, 30)):
                try:
                    if not stop_process(proc, grace):
                        report["errors"].append(name + "未正常退出，已尝试终止")
                except Exception as exc:
                    report["errors"].append(name + "停止失败：" + str(exc))
            if recorder is not None and recorder.returncode not in (0, -signal.SIGINT):
                report["errors"].append("录制器退出码：" + str(recorder.returncode))
            if bag.is_file() and not active.exists():
                try:
                    result = check(argparse.Namespace(bag=bag, report=run / "bag_check.json", rgb_topic=RGB,
                                                       depth_topic=DEPTH, min_hz=args.min_hz, sync_ms=args.sync_ms))
                    checksum = digest(bag)
                    validation = json.loads((run / "bag_check.json").read_text())
                    validation["sha256"] = checksum
                    save(run / "bag_check.json", validation)
                    (run / "SHA256SUMS").write_text(checksum + "  rgbd.bag\n")
                    report.update(bag=bag.name, bytes=bag.stat().st_size, sha256=checksum)
                    if result:
                        report["errors"].append("bag 完整读取或 RGB-D 数据检查未通过")
                    with open(bag, "rb") as stream:
                        os.fsync(stream.fileno())
                except Exception as exc:
                    report["errors"].append("bag 校验失败：" + str(exc))
            else:
                report["errors"].append("未生成正常封包的 rgbd.bag；已有文件和日志保留")
            report["ok"] = report["ok"] and not report["errors"]
            report["finished_at"] = datetime.datetime.now().isoformat()
            save(run / "session_result.json", report)
        say("完成，bag 检查通过：" + str(bag) if report["ok"] else "本次录制未通过检查，文件均保留：" + str(run))
        return 0 if report["ok"] else 1


def worker_main():
    parser = argparse.ArgumentParser(description="本机录包内部任务")
    sub = parser.add_subparsers(dest="mode", required=True)
    p = sub.add_parser("probe")
    p.add_argument("--report", type=Path, required=True)
    p.add_argument("--rgb-topic", default=RGB)
    p.add_argument("--depth-topic", default=DEPTH)
    p.add_argument("--min-hz", type=positive, default=9.5)
    p.add_argument("--sync-ms", type=positive, default=0.1)
    p.add_argument("--warmup", type=positive, default=2)
    p.add_argument("--timeout", type=positive, default=45)
    sub.add_parser("decimate")
    args = parser.parse_args()
    return decimate() if args.mode == "decimate" else probe(args)


def main():
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_stop)
    try:
        if len(sys.argv) > 1 and sys.argv[1] in ("probe", "decimate"):
            return worker_main()
        parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.add_argument("--output", default=str(Path.cwd() / "bags"), help="本机保存根目录")
        parser.add_argument("--duration", type=positive, default=0, help="正式录制后自动停止的秒数；默认按 Enter/Ctrl+C 停止")
        parser.add_argument("--ros-setup", default="/opt/ros/noetic/setup.bash", help="本机 ROS1 环境文件")
        parser.add_argument("--camera-setup", default="", help="本机自编译 RealSense 驱动工作空间 setup.bash，可留空")
        parser.add_argument("--reserve-gb", type=positive, default=1.0, help="本机磁盘保留空间 GiB")
        parser.add_argument("--extra-topic", action="append", default=[], help="额外录制话题，可重复")
        parser.add_argument("--check-env", action="store_true", help="只检查本机基础依赖，不打开相机或录制")
        parser.add_argument("--ros-ready", action="store_true", help=argparse.SUPPRESS)
        parser.set_defaults(min_hz=9.5, sync_ms=0.1, warmup=2., timeout=45., busy_process=[
            "realsense2_camera_node", "realsense2_camera_manager", "rgbd_decimator_node",
            str(Path(__file__).resolve()) + " decimate"])
        args = parser.parse_args()
        if not args.ros_ready:
            return bootstrap_ros(args)
        validate_environment()
        return 0 if args.check_env else record_local(args)
    except Exception as exc:
        say("错误：" + str(exc))
        return 1


if __name__ == "__main__":
    sys.exit(main())

