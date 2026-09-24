#!/usr/bin/env python3
"""ROS1 RGB-D capture supervisor, live probe and full bag reader (Python 3.8+)."""
import argparse
import collections
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
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

RGB = "/camera/infra1/image_raw"
DEPTH = "/camera/depth/image_raw"
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
            raise RuntimeError("用户停止或 SSH 控制连接断开")
        if any(p.poll() is not None for p in children):
            raise RuntimeError("子进程提前退出，请查看日志")
        if test():
            return
        time.sleep(0.2)
    raise RuntimeError("等待就绪超时")


def board(args):
    cfg = json.loads(Path(args.config).read_text())
    run = Path(args.config).resolve().parent
    lock = open(run.parent / ".record.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    master = camera = recorder = preflight = None
    report = {"ok": False, "run": str(run)}

    def control():
        # The laptop sends heartbeats; EOF or a lost link finalizes the bag.
        while not STOP.is_set():
            readable, _, _ = select.select([sys.stdin], [], [], 20)
            if not readable or sys.stdin.readline().strip() != "ping":
                STOP.set()
                break

    threading.Thread(target=control, daemon=True).start()
    try:
        busy_patterns = cfg["busy_process"] or ["s316_camera_node"]
        for proc in Path("/proc").glob("[0-9]*/cmdline"):
            try:
                argv = proc.read_bytes().split(b"\0")
            except OSError:
                continue
            command = " ".join(os.fsdecode(item) for item in argv if item)
            if command and any(pattern in command for pattern in busy_patterns):
                raise RuntimeError("已有相机驱动运行；请先结束该会话，以免重复占用相机")
        if shutil.disk_usage(run).free < cfg["reserve_gb"] * 1024**3:
            raise RuntimeError("板卡剩余磁盘空间不足")
        say("板卡可用空间 %.2f GiB；保留 %.2f GiB，空间不足时自动封包。" %
            (shutil.disk_usage(run).free / 1024**3, cfg["reserve_gb"]))
        camera_start_script = cfg["camera_start_script"]
        init = cfg["init_script"]
        if not camera_start_script and not init:
            init = next((p for p in ("/home/mo/mo_init.sh", "/home/metoak/mo_init.sh")
                         if Path(p).is_file()), None)
        if camera_start_script and not Path(camera_start_script).is_file():
            raise RuntimeError("未找到相机启动脚本: " + camera_start_script)
        if not camera_start_script and (not init or not Path(init).is_file()):
            raise RuntimeError("未找到初始化脚本，请指定 --init-script")
        snapshots = []
        if camera_start_script:
            report["camera_start_script"] = camera_start_script
            snapshots.append((camera_start_script, Path(camera_start_script).name + ".snapshot"))
        else:
            report["init_script"] = init
            snapshots.extend(((init, "mo_init.sh.snapshot"),
                              ("/ros1_project/camera_ws/src/s316_camera/config/x5_camera.cfg", "x5_camera.cfg"),
                              ("/ros1_project/camera_ws/src/s316_camera/launch/camera.launch", "camera.launch")))
        for spec in cfg["snapshot"]:
            source, separator, name = spec.partition("=")
            snapshots.append((source, name if separator else Path(source).name))
        for source, name in snapshots:
            if Path(source).is_file():
                shutil.copy2(source, run / name)
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
        if camera_start_script:
            say("启动相机和同步降频节点: " + camera_start_script)
            camera = start([camera_start_script], run / "camera.log", env)
        else:
            # insmod is made idempotent for the already installed vendor initializer.
            # All other initialization errors abort before roslaunch (bash -e).
            camera_shell = r'''
set -e
insmod() {
  local module="${1##*/}"
  module="${module%.ko}"
  module="${module//-/_}"
  if [[ -d "/sys/module/$module" ]]; then
    echo "module already loaded: $module"
  else
    command insmod "$@"
  fi
}
cd -- "$(dirname -- "$1")"
source "$1"
exec roslaunch s316_camera camera.launch
'''
            say("初始化相机: " + init)
            camera = start(["bash", "-c", camera_shell, "camera-init", init], run / "camera.log", env)
        preflight = start([sys.executable, __file__, "probe", "--report", str(run / "stream_check.json"),
                           "--rgb-topic", cfg["rgb_topic"], "--depth-topic", cfg["depth_topic"],
                           "--min-hz", str(cfg["min_hz"]), "--sync-ms", str(cfg["sync_ms"]),
                           "--warmup", str(cfg["warmup"]),
                           "--timeout", str(cfg["timeout"])], run / "stream_check.log", env)
        say("正在检查 RGB-D 数据流（稳定采样 5 秒）……")
        wait_for(lambda: preflight.poll() is not None, cfg["timeout"] + 10, (camera, master))
        if preflight.returncode:
            raise RuntimeError("数据流检查未通过，见 stream_check.json / camera.log")
        stream = json.loads((run / "stream_check.json").read_text())
        say("数据流检查通过：RGB %.2f Hz，depth %.2f Hz，配对率 %.1f%%。" %
            (stream["receive_hz"][cfg["rgb_topic"]], stream["receive_hz"][cfg["depth_topic"]],
             stream["pair_ratio"] * 100))
        bag = run / "rgbd.bag"
        camera_info_topics = cfg["camera_info_topic"] or [
            "/camera/infra1/camera_info", "/camera/depth/camera_info"]
        topics = list(dict.fromkeys([cfg["rgb_topic"], cfg["depth_topic"]] +
                                    camera_info_topics + cfg["default_extra_topic"] +
                                    cfg["extra_topic"]))
        cmd = ["rosbag", "record", "--lz4", "--buffsize=128", "--chunksize=2048",
               "--min-space=%dM" % (cfg["reserve_gb"] * 1024), "-O", str(bag)] + topics
        recorder = start(cmd, run / "record.log", env)
        active = Path(str(bag) + ".active")
        wait_for(lambda: active.exists() and active.stat().st_size > 4096, 20,
                 (camera, master, recorder))
        say("RECORDING: 已开始录制！请完成闭环路线，回到起点并保持几秒。按 Enter 或 Ctrl+C 结束。")
        report["started_at"] = datetime.datetime.now().isoformat()
        last_size = active.stat().st_size
        last_growth = time.monotonic()
        recording_start = last_growth
        last_status = last_growth
        while not STOP.wait(0.5):
            if any(p.poll() is not None for p in (camera, master, recorder)):
                raise RuntimeError("相机/ROS/录制进程意外退出")
            if shutil.disk_usage(run).free < cfg["reserve_gb"] * 1024**3:
                raise RuntimeError("磁盘接近保留空间，已停止录制")
            size = active.stat().st_size if active.exists() else 0
            if size > last_size:
                last_size, last_growth = size, time.monotonic()
            if time.monotonic() - last_growth > 15:
                raise RuntimeError("bag 15 秒没有增长，数据流或写盘异常")
            if time.monotonic() - last_status >= 10:
                say("录制中：%.0f 秒，bag %.0f MiB，板卡剩余 %.2f GiB。" %
                    (time.monotonic() - recording_start, size / 1024**2,
                     shutil.disk_usage(run).free / 1024**3))
                last_status = time.monotonic()
        report["ok"] = True
    except Exception as exc:
        report["error"] = str(exc)
        say("录制流程异常: " + str(exc))
    finally:
        say("停止录制，等待 bag 写入索引，然后关闭相机……")
        # Keep the camera/master alive until the recorder has flushed its buffer.
        if not stop_process(recorder, 90):
            report.update(ok=False, error="录制器未能在 90 秒内正常退出")
        if recorder is not None and recorder.returncode not in (0, -signal.SIGINT):
            report.update(ok=False, error="录制器退出码: %s" % recorder.returncode)
        stop_process(preflight)
        if not stop_process(camera):
            report["camera_forced_stop"] = True
        stop_process(master)
        report["finished_at"] = datetime.datetime.now().isoformat()
        bag = run / "rgbd.bag"
        if bag.is_file() and not Path(str(bag) + ".active").exists():
            report.update(bag=bag.name, bytes=bag.stat().st_size, sha256=digest(bag))
        else:
            report.update(ok=False, error=report.get("error", "bag 未正常封包"))
        save(run / "board_result.json", report)
        say("板卡会话结束，文件保留在: " + str(run))
    return 0 if report["ok"] else 1


def ssh_command(args, remote):
    return ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
            "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3",
            args.board, remote]


def transfer(args, remote, local):
    ssh_transport = "ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3"
    subprocess.run(["rsync", "-a", "--partial", "--protect-args", "--info=progress2",
                    "-e", ssh_transport, args.board + ":" + remote + "/", str(local) + "/"], check=True)


def remove_verified_remote_bag(args, remote, expected_hash):
    if not re.fullmatch(r"[0-9a-f]{64}", expected_hash):
        raise ValueError("无效的 bag SHA-256")
    # Delete exactly this finalized bag, never the session directory or *.active.
    # Recheck its digest in case the remote file changed since it was downloaded.
    command = "\n".join([
        "set -e",
        "cd -- " + shlex.quote(remote),
        "test ! -e rgbd.bag.active",
        "test ! -L rgbd.bag",
        "if [ -e rgbd.bag ]; then",
        "  printf '%s  rgbd.bag\\n' " + shlex.quote(expected_hash) + " | sha256sum --check --status",
        "  rm -- rgbd.bag",
        "fi",
    ])
    subprocess.run(ssh_command(args, command), check=True)


def verify_download(args, remote, local, board_result, status=0):
    if board_result.get("bag") != "rgbd.bag":
        raise RuntimeError("没有正常结束的 rgbd.bag，请查看 " + str(local / "board_result.json"))
    bag = local / "rgbd.bag"
    say("核对 SHA-256 并完整读取 bag 中的消息……")
    if digest(bag) != board_result["sha256"]:
        raise RuntimeError("SHA-256 不一致，回传文件不完整；板卡原包保留")
    cfg = json.loads((local / "config.json").read_text())
    checked = check(argparse.Namespace(bag=bag, report=local / "bag_check.json", rgb_topic=cfg["rgb_topic"],
                                      depth_topic=cfg["depth_topic"], min_hz=cfg["min_hz"], sync_ms=cfg["sync_ms"]))
    if checked or status or not board_result["ok"]:
        raise RuntimeError("文件已回传，但录制流程或 bag 检查未通过；板卡原包保留，请查看 board_result.json 和 bag_check.json")
    # Commit the verified local copy to disk before removing the remote copy.
    with open(bag, "rb") as stream:
        os.fsync(stream.fileno())
    directory = os.open(str(local), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
    say("本地校验通过，正在删除板卡上的对应 bag……")
    try:
        remove_verified_remote_bag(args, remote, board_result["sha256"])
    except Exception as exc:
        raise RuntimeError("本地 bag 已校验通过，但未能确认板卡 bag 删除成功；可用相同 --fetch 命令重试。" + str(exc)) from exc
    say("板卡对应 bag 已清理，日志和配置保留。")
    say("完成：bag 完整性与 RGB-D 基础检查通过。\n" + str(bag))
    return 0


def host(args):
    if not re.fullmatch(r"[A-Za-z0-9_.@-]+", args.board) or args.board.startswith("-"):
        raise ValueError("--board 必须为 SSH 别名或 user@host")
    if not re.fullmatch(r"/[A-Za-z0-9_./-]+", args.remote_root) or ".." in Path(args.remote_root).parts:
        raise ValueError("--remote-root 必须为不含空格的绝对路径")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.run_prefix):
        raise ValueError("--run-prefix 只能包含字母、数字、下划线和连字符")
    if args.fetch and not re.fullmatch(re.escape(args.run_prefix) + r"_[A-Za-z0-9_-]+", args.fetch):
        raise ValueError("--fetch 会话目录名必须以 " + args.run_prefix + "_ 开头")
    for program in ("ssh", "rsync"):
        if not shutil.which(program):
            raise RuntimeError("本机缺少命令: " + program)
    import rosbag  # Fail before powering the camera if local verification is unavailable.
    import numpy  # noqa: F401
    run_id = args.fetch or (args.run_prefix + "_" +
                            datetime.datetime.now().strftime("%Y%m%d_%H%M%S_") +
                            uuid.uuid4().hex[:6])
    local = Path(args.output).expanduser().resolve() / run_id
    local.mkdir(parents=True, exist_ok=bool(args.fetch))
    remote = args.remote_root.rstrip("/") + "/" + run_id
    say("本地保存目录: " + str(local))
    say("板卡保存目录: " + args.board + ":" + remote)
    status = 0
    if not args.fetch:
        cfg = vars(args).copy()
        save(local / "config.json", cfg)
        subprocess.run(ssh_command(args, shlex.join(["mkdir", "-p", remote])), check=True)
        for source, name in ((Path(__file__).resolve(), "rgbd_recorder.py"), (local / "config.json", "config.json")):
            with open(source, "rb") as stream:
                subprocess.run(ssh_command(args, "cat > " + shlex.quote(remote + "/" + name)), stdin=stream, check=True)
        shell = "set -e; source %s; source %s; exec /usr/bin/python3 %s board --config %s" % tuple(
            shlex.quote(x) for x in (args.ros_setup, args.camera_setup, remote + "/rgbd_recorder.py", remote + "/config.json"))
        proc = subprocess.Popen(ssh_command(args, "bash -c " + shlex.quote(shell)),
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1, start_new_session=True)
        lines = queue.Queue()

        def read_output():
            for line in proc.stdout:
                lines.put(line)

        threading.Thread(target=read_output, daemon=True).start()
        started = None
        sent_stop = False
        heartbeat = 0
        with open(local / "session.log", "w") as log:
            while proc.poll() is None or not lines.empty():
                try:
                    line = lines.get(timeout=0.2)
                    say(line.rstrip())
                    log.write(line)
                    log.flush()
                    if line.startswith("RECORDING:"):
                        started = time.monotonic()
                except queue.Empty:
                    pass
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    STOP.set()
                if started and args.duration and time.monotonic() - started >= args.duration:
                    STOP.set()
                if proc.poll() is None and not sent_stop:
                    if STOP.is_set() or time.monotonic() - heartbeat > 2:
                        try:
                            proc.stdin.write("stop\n" if STOP.is_set() else "ping\n")
                            proc.stdin.flush()
                            heartbeat = time.monotonic()
                            if STOP.is_set():
                                sent_stop = True
                                say("已请求停止；正在封包，请等待回传和检查完成。")
                        except BrokenPipeError:
                            sent_stop = True
        status = proc.wait()
        proc.stdin.close()
        proc.stdout.close()
    # Ctrl+C during finalization/transfer does not kill the upload or truncate a bag.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    say("回传录制文件（校验通过后删除板卡 bag，失败时保留）……")
    try:
        metadata = subprocess.check_output(ssh_command(args, "cat " + shlex.quote(remote + "/board_result.json")), text=True)
        board_result = json.loads(metadata)
        if shutil.disk_usage(local).free < board_result.get("bytes", 0) + 256 * 1024**2:
            raise RuntimeError("本机剩余空间不足；可用 --output 指定其他磁盘并 --fetch 重试")
        transfer(args, remote, local)
    except Exception:
        say("回传未完成，文件仍在板卡。重试: %s --board %s --remote-root %s --output %s --fetch %s" %
            tuple(shlex.quote(x) for x in (args.wrapper, args.board, args.remote_root,
                                           args.output, run_id)))
        raise
    return verify_download(args, remote, local, board_result, status)


def positive(value):
    number = float(value)
    if not 0 < number < float("inf"):
        raise argparse.ArgumentTypeError("必须为有限正数")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    h = sub.add_parser("host", help="在笔记本启动录制")
    h.add_argument("--board", default="root@192.168.1.10")
    h.add_argument("--remote-root", default="/userdata/rgbd_bags")
    h.add_argument("--output", default=str(Path(__file__).resolve().parents[1] / "bags"))
    h.add_argument("--init-script", default="", help="自动查找 /home/mo/mo_init.sh 或 /home/metoak/mo_init.sh")
    h.add_argument("--ros-setup", default="/opt/ros/noetic/setup.bash")
    h.add_argument("--camera-setup", default="/ros1_project/camera_ws/install/setup.bash")
    h.add_argument("--camera-start-script", default="", help="直接启动相机流程的板卡脚本；设置后不运行 S316 初始化")
    h.add_argument("--busy-process", action="append", default=[], help="检测到命令行包含该文本时拒绝重复打开相机")
    h.add_argument("--camera-info-topic", action="append", default=[], help="随 RGB-D 一起录制的 CameraInfo 话题")
    h.add_argument("--default-extra-topic", action="append", default=["/camera/imu/data", "/tf", "/tf_static"],
                   help="配置文件定义的默认附加话题；可由专用包装脚本清空后重设")
    h.add_argument("--snapshot", action="append", default=[], metavar="PATH[=NAME]",
                   help="将板卡配置文件快照保存到录制目录")
    h.add_argument("--run-prefix", default="rgbd_loop")
    h.add_argument("--wrapper", default="./record_rgbd_loop.sh", help=argparse.SUPPRESS)
    h.add_argument("--duration", type=positive, default=0, help="可选：正式开始后自动停止的秒数；默认手动停止")
    h.add_argument("--reserve-gb", type=positive, default=1.0)
    h.add_argument("--extra-topic", action="append", default=[])
    h.add_argument("--fetch", metavar="RUN_ID", help="只回传上次会话并检查，不启动相机")
    b = sub.add_parser("board")
    b.add_argument("--config", required=True)
    p = sub.add_parser("probe")
    c = sub.add_parser("check", help="完整读取已有 bag，不需要 ROS master")
    c.add_argument("bag", type=Path)
    for obj in (h, p, c):
        obj.add_argument("--rgb-topic", default=RGB)
        obj.add_argument("--depth-topic", default=DEPTH)
        obj.add_argument("--min-hz", type=positive, default=3.0)
        obj.add_argument("--sync-ms", type=positive, default=5.0)
    for obj in (h, p):
        obj.add_argument("--timeout", type=positive, default=45)
        obj.add_argument("--warmup", type=positive, default=1.0,
                         help="两路首次到达后丢弃的启动预热秒数")
    for obj in (p, c):
        obj.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_stop)
    try:
        return {"host": host, "board": board, "probe": probe, "check": check}[args.mode](args)
    except Exception as exc:
        say("错误: " + str(exc))
        return 1


if __name__ == "__main__":
    sys.exit(main())
