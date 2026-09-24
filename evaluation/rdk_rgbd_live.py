#!/usr/bin/env python3
"""Laptop-controlled D435i RGB-D SLAM with live output and owned-process cleanup."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
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

from d435i_slam import PACKAGE, EXPECTED, RGB, DEPTH, INFO, CAMERA, camera_dict, make_settings
from rgbd_recorder import STOP, on_stop, digest, save, start, stop_process, wait_for

BOARD = 'sunrise@192.168.1.11'
REMOTE = '/userdata/d435i_slam_20260924'
SSH = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
       '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3', BOARD]


def say(message):
    try:
        print(message, flush=True)
    except (BrokenPipeError, OSError):
        STOP.set()


def relay(stream, logfile, prefix=''):
    """Drain merged stdout/stderr line-by-line; do not wait for process exit."""
    with Path(logfile).open('w', buffering=1) as log:
        for line in stream:
            log.write(line)
            say(prefix + line.rstrip('\n'))


def control(stream, timeout=15):
    """EOF, lost heartbeat or explicit stop ends only this owned session."""
    while not STOP.is_set():
        ready, _, _ = select.select([stream], [], [], timeout)
        if not ready or stream.readline().strip() != 'ping':
            STOP.set()
            return


def board(args):
    run = args.run_dir.resolve()
    if (run/'session.json').exists():
        raise RuntimeError('Refusing to reuse an existing session')
    lock = (run.parent/'.camera.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    os.chdir(run)
    threading.Thread(target=control, args=(sys.stdin,), daemon=True).start()
    children, subscribers = {}, []
    output_thread = None
    result = {'started': False, 'package': str(PACKAGE)}
    try:
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                command = path.read_bytes().replace(b'\0', b' ').decode(errors='replace')
            except OSError:
                continue
            if any(x in command for x in ('realsense2_camera_node', 'realsense2_camera_manager',
                                         'rgbd_decimator_node', '/ros_rgbd')):
                raise RuntimeError('已有相机或 SLAM 在运行，拒绝重复启动；不会停止已有进程。')
        result['sha256'] = {name: digest(PACKAGE/name) for name in EXPECTED}
        if result['sha256'] != EXPECTED:
            raise RuntimeError('MEM18/perf20 node/core SHA256 不匹配')
        env = os.environ.copy()
        env.pop('ROS_HOSTNAME', None)
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        env.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
                   ROS_HOME=str(run/'ros_home'), ROS_LOG_DIR=str(run/'ros_logs'),
                   NO_PROXY='localhost,127.0.0.1', no_proxy='localhost,127.0.0.1')
        os.environ.update(env)
        result['master_uri'] = env['ROS_MASTER_URI']
        children['master'] = start(['roscore', '-p', str(port)], run/'master.log', env)
        def master_ready():
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.2):
                    return True
            except OSError:
                return False
        wait_for(master_ready, 20, tuple(children.values()))
        say('[启动] D435i：640×480，30 FPS 采集 → 精确配对 10 FPS；正在预检…')
        children['camera'] = start([CAMERA], run/'camera.log', env)
        children['probe'] = start([sys.executable, str(run/'rgbd_recorder.py'), 'probe',
            '--rgb-topic', RGB, '--depth-topic', DEPTH, '--min-hz', '9.5',
            '--sync-ms', '0.000001', '--warmup', '2', '--timeout', '60',
            '--report', str(run/'preflight.json')], run/'preflight.log', env)
        wait_for(lambda: children['probe'].poll() is not None, 75, (children['master'], children['camera']))
        if children['probe'].returncode:
            raise RuntimeError('相机预检失败：' + (run/'preflight.log').read_text(errors='replace')[-2500:])
        preflight = json.loads((run/'preflight.json').read_text())
        for topic, shape in ((RGB, [640, 480, 'rgb8']), (DEPTH, [640, 480, '16UC1'])):
            if preflight['topics'][topic]['shape'][:3] != shape:
                raise RuntimeError('相机图像尺寸/编码与 D435i 配置不符')
        import rospy
        from sensor_msgs.msg import CameraInfo
        from std_msgs.msg import Bool
        rospy.init_node('rdk_live_control', disable_signals=True)
        info = camera_dict(rospy.wait_for_message(INFO, CameraInfo, timeout=15))
        aligned = camera_dict(rospy.wait_for_message('/camera_10fps/aligned_depth_to_color/camera_info', CameraInfo, timeout=15))
        if info['K'] != aligned['K'] or info['frame_id'] != aligned['frame_id']:
            raise RuntimeError('深度未对齐到彩色相机')
        save(run/'calibration.json', {'color': info, 'aligned_depth': aligned})
        (run/'d435i.yaml').write_text(make_settings(info))
        # Only latest status is retained: no unbounded per-frame diagnostics in live mode.
        tracking = {'ok': None, 'received': 0, 'last': 0.0}
        def status_cb(msg):
            tracking.update(ok=bool(msg.data), received=tracking['received']+1, last=time.monotonic())
        subscribers.append(rospy.Subscriber('/rdk_rgbd/robot_pose_tracking_ok', Bool, status_cb, queue_size=10))
        slam_env = dict(env, LD_LIBRARY_PATH=f'{PACKAGE}:{PACKAGE}/lib:/opt/ros/noetic/lib:/usr/local/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu',
                        ORB_MEMORY_STATS='0')
        checks = []
        for command in (['file', str(PACKAGE/'ros_rgbd'), str(PACKAGE/'liborb_slam3_ros.so')],
                        ['ldd', str(PACKAGE/'ros_rgbd')], ['ldd', str(PACKAGE/'liborb_slam3_ros.so')]):
            output = subprocess.check_output(command, env=slam_env, text=True)
            if 'not found' in output:
                raise RuntimeError('缺少动态库：' + output)
            checks.append(output)
        (run/'file_ldd.txt').write_text('\n'.join(checks))
        command = ['stdbuf', '-oL', '-eL', str(PACKAGE/'ros_rgbd'),
            f'_voc_file:={PACKAGE}/Vocabulary/orbvoc.dbow3', f'_settings_file:={run}/d435i.yaml',
            f'_rgb_topic:={RGB}', f'_depth_topic:={DEPTH}',
            '_rgb_queue:=8', '_depth_queue:=8', '_sync_queue:=10',
            '_enable_pangolin:=false', '_timing_enabled:=false',
            '_field_benchmark_metrics:=false', '_field_diagnostics_enabled:=false',
            '__name:=slam', '__ns:=/rdk_rgbd']
        save(run/'command.json', {'argv': command, 'master_uri': env['ROS_MASTER_URI']})
        say('[启动] MEM18/perf20；以下为 SLAM 的实时 stdout/stderr：')
        children['slam'] = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, text=True, errors='replace', bufsize=1, env=slam_env, start_new_session=True)
        output_thread = threading.Thread(target=relay, args=(children['slam'].stdout, run/'node.log', '[SLAM] '), daemon=True)
        output_thread.start()
        def ready():
            _, _, state = rospy.get_master().getSystemState()
            subscriptions = dict(state[1])
            return all('/rdk_rgbd/slam' in subscriptions.get(t, []) for t in (RGB, DEPTH))
        wait_for(ready, 90, (children['slam'], children['camera']))
        loaded = Path(f'/proc/{children["slam"].pid}/maps').read_text()
        (run/'loaded_maps.txt').write_text(loaded)
        if str(PACKAGE/'liborb_slam3_ros.so') not in loaded:
            raise RuntimeError('实际加载的 core 路径不符')
        result['started'] = True
        say('[运行] 已连接 RGB-D。按 Ctrl+C 停止；启动成功不代表已初始化/跟踪成功。')
        say(f'[板端日志] {run}\n[板端 ROS_MASTER_URI] {env["ROS_MASTER_URI"]}')
        begin = time.monotonic()
        next_status = begin
        while not STOP.is_set() and (args.duration == 0 or time.monotonic()-begin < args.duration):
            if any(children[k].poll() is not None for k in ('slam', 'camera', 'master')):
                raise RuntimeError('SLAM、相机或 ROS master 提前退出')
            if time.monotonic() >= next_status:
                status = ('无新鲜状态' if time.monotonic()-tracking['last'] > 3 else
                          'OK' if tracking['ok'] else '未初始化或跟踪失败（请对准有纹理场景）')
                say(f'[状态] tracking={status}；收到状态 {tracking["received"]} 条')
                if shutil.disk_usage(run).free < 512*1024**2:
                    raise RuntimeError('磁盘可用空间低于 512 MiB，停止本次会话')
                next_status = time.monotonic()+5
            STOP.wait(.2)
        result['last_tracking_status'] = tracking
    except Exception as exc:
        if not STOP.is_set():
            result['error'] = str(exc)
            say('[错误] ' + str(exc))
    finally:
        say('[停止] 正在关闭本次启动的相机、SLAM 和 ROS master…')
        result['cleanup'] = {}
        for key in ('probe', 'camera', 'slam'):
            if key in children:
                result['cleanup'][key] = stop_process(children[key], grace=30)
                result[key+'_exit'] = children[key].returncode
        if output_thread:
            output_thread.join(timeout=5)
        for sub in subscribers:
            sub.unregister()
        if 'rospy' in sys.modules:
            sys.modules['rospy'].signal_shutdown('live session stopped')
        if 'master' in children:
            result['cleanup']['master'] = stop_process(children['master'])
        save(run/'session.json', result)
        say('[停止] 清理完成；板端日志保留。')
    return 0 if not result.get('error') and all(result['cleanup'].values()) else 1


def host(args):
    root = Path(__file__).resolve().parents[1]
    run_id = 'live_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:6]
    remote = REMOTE + '/' + run_id
    local = root/'logs'/'rdk_rgbd'/run_id
    local.mkdir(parents=True)
    say(f'[连接] {BOARD}\n[本机日志] {local}/terminal.log')
    subprocess.run(SSH + [f'set -e; if [ ! -e {REMOTE} ]; then sudo -n install -d -o sunrise -g sunrise -m 775 {REMOTE}; fi; mkdir {remote}'], check=True)
    sources = [root/'evaluation'/name for name in ('rdk_rgbd_live.py', 'd435i_slam.py', 'rgbd_recorder.py')]
    subprocess.run(['scp', '-q', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8'] +
                   [str(p) for p in sources] + [f'{BOARD}:{remote}/'], check=True)
    remote_command = ('source /opt/ros/noetic/setup.bash && exec /usr/bin/python3 -u ' +
                      shlex.join([remote+'/rdk_rgbd_live.py', 'board', '--run-dir', remote,
                                  '--duration', str(args.duration)]))
    process = subprocess.Popen(SSH + [remote_command], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, errors='replace', bufsize=1, start_new_session=True)
    thread = threading.Thread(target=relay, args=(process.stdout, local/'terminal.log'), daemon=True)
    thread.start()
    try:
        while process.poll() is None and not STOP.is_set():
            process.stdin.write('ping\n')
            process.stdin.flush()
            STOP.wait(2)
        if process.poll() is None:
            say('[本机] 已请求停止，请等待板卡清理；不要直接关闭终端。')
            process.stdin.write('stop\n')
            process.stdin.flush()
        process.wait(timeout=180)
    except (BrokenPipeError, OSError, subprocess.TimeoutExpired) as exc:
        say('[连接] ' + str(exc))
        process.stdin.close()  # EOF is also a stop request.
        stop_process(process, grace=5)
    finally:
        if not process.stdin.closed:
            process.stdin.close()
        thread.join(timeout=5)
    # The terminal log is already local; fetch small board metadata/logs, no images/bags/maps.
    copied = subprocess.run(['scp', '-q', '-r', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                             f'{BOARD}:{remote}', str(local/'board')]).returncode == 0
    say(f'[完成] 本机日志：{local}；板端：{remote}' + ('' if copied else '（回传失败，可稍后手动取回）'))
    return process.returncode or (0 if copied else 1)


def main():
    parser = argparse.ArgumentParser(prog='start_rdk_rgbd_slam.sh', usage='%(prog)s [--duration SECONDS]',
        description='本机一键启动 RDK X5 D435i RGB-D SLAM，实时显示 SLAM 输出；Ctrl+C 停止。')
    parser.add_argument('mode', choices=['host', 'board'], help=argparse.SUPPRESS)
    parser.add_argument('--duration', type=int, default=0, help='SLAM 就绪后运行秒数；默认 0 表示持续运行直到 Ctrl+C')
    parser.add_argument('--run-dir', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.duration < 0 or (args.mode == 'board' and args.run_dir is None):
        parser.error('invalid duration/run directory')
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_stop)
    try:
        return host(args) if args.mode == 'host' else board(args)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        say('[错误] ' + str(exc))
        return 1


if __name__ == '__main__':
    sys.exit(main())
