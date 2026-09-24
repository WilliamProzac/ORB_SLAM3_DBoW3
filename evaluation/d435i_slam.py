#!/usr/bin/env python3
"""Finite D435i + pinned perf20 integration run; creates only a new run directory."""
import argparse
import csv
import fcntl
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

from rgbd_recorder import Images, STOP, digest, on_stop, save, start, stop_process, wait_for

PACKAGE = Path('/userdata/rgbd_memory_lifecycle_8g_20260923/mem18_perf20')
EXPECTED = {'ros_rgbd': '4065297335e00b1efd0e2301083f817e62e5f365f61dc96902401024ce55255d',
            'liborb_slam3_ros.so': '647a5b261c84fbf2475e92c9ab0e5fb4cd2aed4e22b873c7ea92af423d3803b7'}
RGB = '/camera_10fps/color/image_raw'
DEPTH = '/camera_10fps/aligned_depth_to_color/image_raw'
INFO = '/camera_10fps/color/camera_info'
CAMERA = '/userdata/ros1_realsense_ws/start_rgbd10.sh'


def camera_dict(msg):
    return dict(width=msg.width, height=msg.height, frame_id=msg.header.frame_id,
                distortion_model=msg.distortion_model, K=list(msg.K), D=list(msg.D),
                R=list(msg.R), P=list(msg.P))


def make_settings(info):
    if (info['width'], info['height']) != (640, 480):
        raise ValueError('Expected 640x480 calibration')
    if info['distortion_model'] != 'plumb_bob' or len(info['D']) not in (4, 5):
        raise ValueError('Unsupported distortion model; do not silently treat raw images as rectified')
    k, d = info['K'], info['D'] + [0.0]
    if len(k) != 9 or not all(math.isfinite(x) for x in k + d) or min(k[0], k[4]) <= 0:
        raise ValueError('Invalid calibration')
    values = {'Camera1.fx': k[0], 'Camera1.fy': k[4], 'Camera1.cx': k[2], 'Camera1.cy': k[5],
              'Camera1.k1': d[0], 'Camera1.k2': d[1], 'Camera1.p1': d[2],
              'Camera1.p2': d[3], 'Camera1.k3': d[4]}
    return '\n'.join(['%YAML:1.0', 'File.version: "1.0"', 'Camera.type: "PinHole"'] +
                     [f'{key}: {float(value)!r}' for key, value in values.items()] + [
        'Camera.width: 640', 'Camera.height: 480', 'Camera.fps: 10', 'Camera.RGB: 1',
        '# D435 nominal 50 mm stereo baseline; used for RGB-D close-point threshold.',
        '# Depth itself is supplied by the aligned image, in millimetres.',
        'Stereo.b: 0.05', 'Stereo.ThDepth: 40.0', 'RGBD.DepthMapFactor: 1000.0',
        'ORBextractor.nFeatures: 700', 'ORBextractor.scaleFactor: 1.2',
        'ORBextractor.nLevels: 5', 'ORBextractor.iniThFAST: 20', 'ORBextractor.minThFAST: 7',
        'ORBextractor.parallelWorkers: 3', 'RGBD.ExactOptimizationLevel: 4',
        'Viewer.KeyFrameSize: 0.05', 'Viewer.KeyFrameLineWidth: 1.0', 'Viewer.GraphLineWidth: 0.9',
        'Viewer.PointSize: 2.0', 'Viewer.CameraSize: 0.08', 'Viewer.CameraLineWidth: 3.0',
        'Viewer.ViewpointX: 0.0', 'Viewer.ViewpointY: -0.7', 'Viewer.ViewpointZ: -3.5',
        'Viewer.ViewpointF: 500.0', '# No automatic atlas save in this finite static test.', ''])


def percentiles(values):
    values = sorted(float(v) for v in values)
    if not values:
        return {'count': 0}
    result = dict(count=len(values), mean=sum(values)/len(values), min=values[0], max=values[-1])
    for p in (50, 95, 99):
        index = (len(values)-1)*p/100
        low, high = math.floor(index), math.ceil(index)
        result[f'p{p}'] = values[low] + (values[high]-values[low])*(index-low)
    return result


def benchmark(path):
    raw = {k: int(v) for k, v in (line.split('=', 1) for line in path.read_text().splitlines())}
    seqs = []
    for channel in range(3):
        seq = set()
        prefix = f'seq.{channel}.word.'
        for key, bits in raw.items():
            if key.startswith(prefix):
                word = int(key[len(prefix):])
                seq.update(raw[f'seq.{channel}.base'] + 64*word + bit
                           for bit in range(64) if bits & (1 << bit))
        seqs.append(seq)
    bins = [int(k.rsplit('.', 1)[1]) + .5 for k, count in raw.items()
            if k.startswith('callback_ms_bin.') for _ in range(count)]
    return raw, seqs, percentiles(bins)


def analyze(run, result):
    def rows(name):
        p = run/name
        return list(csv.DictReader(p.open())) if p.exists() else []
    if not (run/'benchmark.txt').exists():
        return
    raw, seqs, hist = benchmark(run/'benchmark.txt')
    result['benchmark'] = dict(callback_ms=hist, exact_max_ms=raw['max_ns']/1e6,
        histogram_quantiles_approximate_1ms=True, non_ok=raw['nonok'], maps=raw['maps'],
        keyframes=raw['keyframes'], map_points=raw['map_points'],
        counts=[raw[f'count.{i}'] for i in range(3)],
        sequence_overflow=[raw[f'seq.{i}.overflow'] for i in range(3)],
        sync_drop_rgb=raw['sync_drop_rgb'], sync_drop_depth=raw['sync_drop_depth'])
    # Bound the stable comparison by wall receipt time, not first/last processed frames.
    # Missing terminal frames therefore remain visible; startup/shutdown are separate.
    window = result.get('measurement_window', [0, 0])
    expected = {}
    for name, channel in [('rgb', 0), ('depth', 1)]:
        samples = [r for r in rows(name+'.csv') if window[0] <= float(r['steady_s']) < window[1]]
        seq = {int(r['seq']) for r in samples}
        expected[name] = seq
        stamps = [int(r['stamp_ns']) for r in samples]
        result[name] = dict(messages=len(samples), unique=len(seq),
            hz=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0,
            missing_dequeued=sorted(seq-seqs[channel]),
            missing_processed=sorted(seq-seqs[2]) if channel == 0 else None)
    frames = rows('timing/rgbd_frame_timing.csv')
    result['timing'] = {key: percentiles(r[key] for r in frames) for key in
        ('callback_total_ms', 'track_call_ms', 'orb_extract_ms', 'local_map_ms')
        if frames and key in frames[0]}
    result['features'] = {key: percentiles(r[key] for r in frames)
                          for key in ('keypoints', 'valid_depths', 'tracked_inliers')}
    result['tracking_states'] = {s: sum(r['state_after'] == s for r in frames)
                                 for s in sorted({r['state_after'] for r in frames})}
    ba = rows('timing/local_mapping_timing.csv')
    result['local_ba'] = {kind: percentiles(r['local_ba_ms'] for r in ba
        if int(r['lba_executed']) and (kind == 'all' or bool(int(r['lba_aborted'])) == (kind == 'aborted')))
        for kind in ('all', 'completed', 'aborted')}
    memory_rows = rows('memory.csv')
    memory = [r for r in memory_rows if r.get('steady_s', '').replace('.', '', 1).isdigit()]
    result['invalid_memory_rows'] = len(memory_rows)-len(memory)
    result['memory'] = {key: percentiles(r[key] for r in memory if r[key])
        for key in ('rss_kib', 'pss_kib', 'swap_kib', 'available_kib')}
    result['paging_delta'] = {key: int(memory[-1][key])-int(memory[0][key])
        for key in ('majflt', 'pswpin', 'pswpout')} if memory else {}
    poses = rows('poses.csv')
    xyz = [[float(r[k]) for k in ('x', 'y', 'z')] for r in poses]
    result['pose'] = dict(count=len(poses), finite=bool(xyz) and all(math.isfinite(v) for p in xyz for v in p),
        max_displacement_from_first_m=max((math.dist(p, xyz[0]) for p in xyz), default=0))
    log = (run/'node.log').read_text(errors='replace')
    result['map_events'] = {name: log.count(pattern) for name, pattern in
        [('new_map', 'New Map created'), ('loop', '*Loop detected'), ('merge', '*Merge detected'),
         ('active_reset', 'LM: Active map reset, Done!!!')]}
    result['static_integration_ok'] = bool(result.get('completed') and result.get('slam_exit') == 0
        and result.get('preflight', {}).get('ok') and result.get('stream', {}).get('ok')
        and raw['count.2'] > 0 and raw['nonok'] == 0 and raw['maps'] == 1
        and result['pose']['finite'] and not any(result['benchmark']['sequence_overflow'])
        and not result['rgb']['missing_processed'] and not result['rgb']['missing_dequeued']
        and not result['depth']['missing_dequeued']
        and min(result['rgb']['hz'], result['depth']['hz']) >= 9.5
        and not result['map_events']['active_reset'])
    result['static_integration_ok'] = bool(result['static_integration_ok']
        and all(result.get('cleanup', {}).values())
        and result.get('binary_sha256_after') == EXPECTED and not result['invalid_memory_rows'])
    result['strict_100ms_callback_gate'] = raw['max_ns'] <= 100000000 if result['timing_mode'] == 'off' else None
    result['input_adapter_ok'] = bool(result.get('completed') and result.get('preflight', {}).get('ok')
        and result.get('stream', {}).get('ok') and min(result['rgb']['hz'], result['depth']['hz']) >= 9.5
        and not result['rgb']['missing_processed'] and not result['rgb']['missing_dequeued']
        and not result['depth']['missing_dequeued'])
    result['production_realtime_passed'] = bool(result['static_integration_ok']
        and result['strict_100ms_callback_gate'] and result['timing_mode'] == 'off')


def finalize(run):
    # Called again by the host only after the ROS process and its atexit logging exit.
    paths = sorted(p for p in run.rglob('*') if p.is_file() and not p.is_symlink() and p != run/'SHA256SUMS')
    (run/'SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.relative_to(run)}\n' for p in paths))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=120)
    parser.add_argument('--timing', choices=['off', 'on'], default='off')
    parser.add_argument('--finalize', action='store_true', help='hash a stopped run without initializing ROS')
    parser.add_argument('--review', action='store_true', help='write a separate analysis_review.json for an existing run')
    args = parser.parse_args()
    if not 10 <= args.duration <= 1800:
        parser.error('duration must be 10..1800')
    run = args.run_dir.resolve()
    if args.review:
        result = json.loads((run/'result.json').read_text())
        result['original_analysis_error'] = result.pop('analysis_error', None)
        analyze(run, result)
        save(run/'analysis_review.json', result)
        return 0
    if args.finalize:
        finalize(run)
        return 0
    if (run/'result.json').exists() or (run/'node.log').exists():
        parser.error('Refusing to overwrite a previous run')
    lock = (run.parent/'.camera.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    os.chdir(run)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_stop)
    result = dict(completed=False, timing_mode=args.timing, duration_s=args.duration,
                  scene='static; motion/loop/accuracy not validated', package=str(PACKAGE))
    children, subscribers, handles = {}, [], []
    samples = Images(RGB, DEPTH)
    mutex = threading.Lock()
    try:
        for proc in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                argv = proc.read_bytes().split(b'\0')
            except OSError:
                continue
            command = b' '.join(argv).decode(errors='replace')
            if any(x in command for x in ('realsense2_camera_node', 'realsense2_camera_manager',
                                           'rgbd_decimator_node', '/ros_rgbd')):
                raise RuntimeError('Camera or SLAM already running; refusing hardware conflict')
        result['binary_sha256'] = {name: digest(PACKAGE/name) for name in EXPECTED}
        if result['binary_sha256'] != EXPECTED:
            raise RuntimeError('Pinned node/core SHA256 mismatch')
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
        children['camera'] = start([CAMERA], run/'camera.log', env)
        children['probe'] = start([sys.executable, str(run/'rgbd_recorder.py'), 'probe',
            '--rgb-topic', RGB, '--depth-topic', DEPTH, '--min-hz', '9.5', '--sync-ms', '0.000001',
            '--warmup', '2', '--timeout', '60', '--report', str(run/'preflight.json')], run/'preflight.log', env)
        wait_for(lambda: children['probe'].poll() is not None, 75, (children['master'], children['camera']))
        result['preflight'] = json.loads((run/'preflight.json').read_text())
        if children['probe'].returncode or not result['preflight']['ok']:
            raise RuntimeError('Camera preflight failed')
        shapes = result['preflight']['topics']
        if shapes[RGB]['shape'][:3] != [640, 480, 'rgb8'] or shapes[DEPTH]['shape'][:3] != [640, 480, '16UC1']:
            raise RuntimeError('Unexpected image dimensions/encodings')
        import rospy
        from sensor_msgs.msg import CameraInfo, Image
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool
        rospy.init_node('d435i_joint_monitor', disable_signals=True)
        info = camera_dict(rospy.wait_for_message(INFO, CameraInfo, timeout=15))
        aligned_info = camera_dict(rospy.wait_for_message('/camera_10fps/aligned_depth_to_color/camera_info', CameraInfo, timeout=15))
        if info['K'] != aligned_info['K'] or info['frame_id'] != aligned_info['frame_id']:
            raise RuntimeError('Aligned depth calibration/frame differs from color')
        save(run/'calibration.json', dict(color=info, aligned_depth=aligned_info,
             baseline_m=.05, baseline_source='nominal D435 stereo baseline for close-depth threshold',
             depth_unit_m=.001, depth_unit_source='RealSense ROS1 fix_depth_scale millimetre normalization'))
        (run/'d435i.yaml').write_text(make_settings(info))
        import cv2
        import numpy as np
        picture = rospy.wait_for_message(RGB, Image, timeout=10)
        pixels = np.frombuffer(picture.data, np.uint8).reshape(picture.height, picture.step)
        pixels = pixels[:, :picture.width*3].reshape(picture.height, picture.width, 3)
        cv2.imwrite(str(run/'scene.png'), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
        def writer(name, columns):
            handle = (run/name).open('w', newline='')
            handles.append(handle)
            output = csv.writer(handle)
            output.writerow(columns)
            return output
        image_writers = {t: writer(name+'.csv', ['steady_s', 'seq', 'stamp_ns'])
                         for t, name in ((RGB, 'rgb'), (DEPTH, 'depth'))}
        pose_writer = writer('poses.csv', ['steady_s', 'stamp_ns', 'frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        state_writer = writer('tracking.csv', ['steady_s', 'ok'])
        def image_cb(msg, topic):
            with mutex:
                image_writers[topic].writerow([time.monotonic(), msg.header.seq, msg.header.stamp.to_nsec()])
                samples.add(topic, msg)
        def pose_cb(msg):
            p, q = msg.pose.position, msg.pose.orientation
            with mutex:
                pose_writer.writerow([time.monotonic(), msg.header.stamp.to_nsec(), msg.header.frame_id,
                                      p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        def state_cb(msg):
            with mutex:
                state_writer.writerow([time.monotonic(), int(msg.data)])
        subscribers.extend(rospy.Subscriber(t, Image, image_cb, callback_args=t, queue_size=1000,
                            buff_size=16*1024*1024, tcp_nodelay=True) for t in (RGB, DEPTH))
        subscribers.append(rospy.Subscriber('/d435i_validation/robot_pose_slam', PoseStamped, pose_cb, queue_size=1000))
        subscribers.append(rospy.Subscriber('/d435i_validation/robot_pose_tracking_ok', Bool, state_cb, queue_size=1000))
        slam_env = dict(env, LD_LIBRARY_PATH=f'{PACKAGE}:{PACKAGE}/lib:/opt/ros/noetic/lib:/usr/local/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu',
                        ORB_MEMORY_STATS='1', ORB_MEMORY_STATS_FILE=str(run/'stats.csv'))
        provenance = []
        for cmd in (['file', str(PACKAGE/'ros_rgbd'), str(PACKAGE/'liborb_slam3_ros.so')],
                    ['ldd', str(PACKAGE/'ros_rgbd')], ['ldd', str(PACKAGE/'liborb_slam3_ros.so')]):
            output = subprocess.check_output(cmd, env=slam_env, text=True)
            if 'not found' in output:
                raise RuntimeError('Missing runtime library')
            provenance.append(output)
        (run/'file_ldd.txt').write_text('\n'.join(provenance))
        (run/'timing').mkdir()
        command = [str(PACKAGE/'ros_rgbd'), f'_voc_file:={PACKAGE}/Vocabulary/orbvoc.dbow3',
            f'_settings_file:={run}/d435i.yaml', f'_rgb_topic:={RGB}', f'_depth_topic:={DEPTH}',
            '_rgb_queue:=8', '_depth_queue:=8', '_sync_queue:=10', '_enable_pangolin:=false',
            f'_timing_enabled:={str(args.timing == "on").lower()}', f'_timing_output_dir:={run}/timing',
            '_field_benchmark_metrics:=true', '_field_diagnostics_enabled:=false',
            '__name:=slam', '__ns:=/d435i_validation']
        save(run/'command.json', dict(argv=command, env={k: slam_env[k] for k in
            ('ROS_MASTER_URI', 'ROS_IP', 'ROS_HOME', 'ROS_LOG_DIR', 'LD_LIBRARY_PATH',
             'ORB_MEMORY_STATS', 'ORB_MEMORY_STATS_FILE')}))
        children['slam'] = start(command, run/'node.log', slam_env)
        with (run/'memory.csv').open('wb') as output, (run/'memory_sampler.err').open('wb') as error:
            children['sampler'] = subprocess.Popen(
                [sys.executable, str(run/'resource_sample.py'), str(children['slam'].pid)],
                stdout=output, stderr=error, env=env, start_new_session=True)
        def slam_ready():
            _, _, state = rospy.get_master().getSystemState()
            subscriptions = dict(state[1])
            return all('/d435i_validation/slam' in subscriptions.get(topic, []) for topic in (RGB, DEPTH))
        wait_for(slam_ready, 90, (children['slam'], children['camera']))
        loaded = Path(f'/proc/{children["slam"].pid}/maps').read_text()
        (run/'loaded_maps.txt').write_text(loaded)
        if str(PACKAGE/'liborb_slam3_ros.so') not in loaded:
            raise RuntimeError('Process loaded wrong core')
        result['measurement_window'] = [time.monotonic()+2, 0]
        print(f'STATIC RUN STARTED: {args.duration}s timing={args.timing}', flush=True)
        until = result['measurement_window'][0] + args.duration
        while time.monotonic() < until and not STOP.is_set():
            if children['slam'].poll() is not None or children['camera'].poll() is not None:
                raise RuntimeError('SLAM or camera exited during measurement')
            STOP.wait(.2)
        result['measurement_window'][1] = time.monotonic()
        result['completed'] = not STOP.is_set()
    except Exception as exc:
        result['error'] = str(exc)
        print('ERROR:', exc, flush=True)
    finally:
        # Stop only processes started by this invocation, camera first then drain SLAM.
        cleanup = {}
        for key in ('probe', 'camera'):
            if key in children:
                cleanup[key] = stop_process(children[key])
        if 'slam' in children:
            time.sleep(2)
            cleanup['slam'] = stop_process(children['slam'], grace=45)
            result['slam_exit'] = children['slam'].returncode
        if 'sampler' in children:
            try:
                children['sampler'].wait(timeout=4)
                cleanup['sampler'] = children['sampler'].returncode == 0
            except subprocess.TimeoutExpired:
                cleanup['sampler'] = stop_process(children['sampler'])
        for sub in subscribers:
            sub.unregister()
        with mutex:
            for handle in handles:
                handle.close()
            result['stream'] = samples.report(min_hz=9.5, sync_ms=0)
        if 'rospy' in sys.modules:
            sys.modules['rospy'].signal_shutdown('joint validation complete')
        if 'master' in children:
            cleanup['master'] = stop_process(children['master'])
        result['cleanup'] = cleanup
        result['binary_sha256_after'] = {name: digest(PACKAGE/name) for name in EXPECTED}
        try:
            analyze(run, result)
        except Exception as exc:
            result['analysis_error'] = str(exc)
        save(run/'result.json', result)
        logging.shutdown()
        # Exclude this manifest itself; host verifies every transferred artifact.
        finalize(run)
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result.get('static_integration_ok') else 1


if __name__ == '__main__':
    sys.exit(main())
