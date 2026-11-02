#!/usr/bin/env python3
"""Independent source/pose observations and this round's owned process control."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import threading
import time


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def process_start(pid):
    return int(Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()[19])


def owned_process(args):
    run = args.run_dir.resolve(strict=True)
    record_path = run / (args.role + '.process.json')
    if args.mode == 'exec_owned':
        command = args.command[1:] if args.command[:1] == ['--'] else args.command
        if not command or record_path.exists():
            raise ValueError('missing command or existing owned process record')
        os.chdir(run)
        pid = os.getpid()
        write_json(record_path, {'pid': pid, 'start_ticks': process_start(pid),
                                'run_directory': str(run), 'role': args.role})
        (run / (args.role + '.pid')).write_text(str(pid) + '\n')
        if (run / (args.role + '.stop_requested')).exists():
            return 125
        os.execvp(command[0], command)
    if args.mode == 'stop_owned':
        # Covers cleanup racing registration: a delayed exec may not start
        # after its owner has already begun stopping this round.
        (run / (args.role + '.stop_requested')).touch()
    if not record_path.exists():
        return 0
    record = json.loads(record_path.read_text())
    pid = int(record['pid'])
    def alive_owned():
        try:
            started = process_start(pid)
            cwd = Path('/proc/%d/cwd' % pid).resolve(strict=True)
        except FileNotFoundError:
            return False
        if record['run_directory'] != str(run) or record['role'] != args.role or \
                started != int(record['start_ticks']) or cwd != run:
            raise RuntimeError('refusing to signal a process without this round ownership')
        return True
    stages = [(args.signal, 0)] if args.mode == 'signal_owned' else [(args.signal, 3), ('TERM', 2), ('KILL', 2)]
    for requested_signal, seconds in stages:
        if not alive_owned():
            return 0
        try:
            os.kill(pid, getattr(signal, 'SIG' + requested_signal))
        except ProcessLookupError:
            return 0
        if args.mode == 'signal_owned':
            return 0
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not alive_owned():
                return 0
            time.sleep(.05)
    return 124


def published_stamps(path, index):
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        if not {'event', 'source_stamp_ns'} <= set(reader.fieldnames or []):
            raise ValueError('producer events missing schema')
        stamps = [int(row['source_stamp_ns']) for row in reader if row['event'] == 'pose_publish']
    if len(stamps) != len(set(stamps)) or not set(stamps) <= set(index):
        raise ValueError('producer pose identities duplicate or outside frozen index')
    return set(stamps)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--mode', choices=['host', 'board_pose', 'exec_owned', 'signal_owned', 'stop_owned'], required=True)
    parser.add_argument('--rgb-topic', default='/rgbd_v2/rgb')
    parser.add_argument('--depth-topic', default='/rgbd_v2/depth')
    parser.add_argument('--pose-topic', default='/robot_pose_slam')
    parser.add_argument('--seal-file', type=Path)
    parser.add_argument('--producer-events', type=Path)
    parser.add_argument('--drain-timeout', type=float, default=15.)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--role', choices=['slam', 'pose_monitor', 'sampler', 'cpu_sampler'])
    parser.add_argument('--signal', choices=['INT', 'TERM', 'KILL'], default='TERM')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.mode in ('exec_owned', 'signal_owned', 'stop_owned'):
        if args.run_dir is None or args.role is None:
            parser.error('owned process control requires run directory and role')
        return owned_process(args)
    if args.index is None or args.output is None or not 0 < args.drain_timeout <= 60:
        parser.error('observation requires index/output and drain timeout in (0,60]')
    if args.mode == 'board_pose' and (args.seal_file is None or args.producer_events is None):
        parser.error('pose consumer requires producer seal and actual publication events')
    with args.index.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    index = {int(row['source_stamp_ns']): row for row in rows}
    if not rows or len(index) != len(rows):
        raise ValueError('empty/ambiguous frozen source index')
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image
    rospy.init_node('rgbd_v2_' + args.mode + '_observer', anonymous=True)
    lock = threading.Lock()
    count = errors = callback_errors = 0
    closing = False
    seen = set()
    expected = None
    seal_seen = None
    producer_exit = None
    failure = ''
    completed = False
    with args.output.open('x', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(['event', 'source_stamp_ns', 'rgb_seq', 'message_seq',
                         'observer_monotonic_ns', 'observer_wall_ns', 'clock_domain'])

        def record(message, event):
            nonlocal count, errors, callback_errors
            try:
                mono, wall = time.monotonic_ns(), time.time_ns()
                stamp = message.header.stamp.to_nsec()
                with lock:
                    if closing:
                        return
                    row = index.get(stamp)
                    if row is None:
                        errors += 1
                    seq = int(row['rgb_seq']) if row else -1
                    if row and event in ('host_rgb', 'host_depth'):
                        key_name = 'rgb_seq' if event == 'host_rgb' else 'depth_seq'
                        if int(row[key_name]) != message.header.seq:
                            errors += 1
                    key = (event, stamp)
                    if key in seen:
                        errors += 1
                    if row:
                        seen.add(key)
                    if count >= 4 * len(rows):
                        errors += 1
                        return
                    writer.writerow([event, stamp, seq, message.header.seq, mono, wall, args.mode])
                    count += 1
                    if count % 64 == 0:
                        output.flush()
            except Exception:
                with lock:
                    callback_errors += 1

        if args.mode == 'host':
            subscribers = [rospy.Subscriber(args.rgb_topic, Image, record, 'host_rgb', queue_size=100),
                           rospy.Subscriber(args.depth_topic, Image, record, 'host_depth', queue_size=100)]
        else:
            subscribers = [rospy.Subscriber(args.pose_topic, PoseStamped, record, 'pose_receive', queue_size=100)]
            deadline = time.monotonic() + 30
            while not subscribers[0].get_num_connections():
                if rospy.is_shutdown() or time.monotonic() >= deadline:
                    raise RuntimeError('raw pose publisher did not connect')
                time.sleep(.05)
        args.output.with_suffix('.ready').write_text('subscribers_registered\n')
        progress_at = 0.
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                if callback_errors:
                    failure = 'observer_callback_exception'
                    break
                if now - progress_at >= 1.:
                    write_json(args.output.with_suffix('.progress.json'),
                               {'unique_source_events': len(seen), 'records': count,
                                'observer_monotonic_ns': time.monotonic_ns(), 'mode': args.mode})
                    progress_at = now
            if args.mode == 'board_pose':
                if seal_seen is None and args.seal_file.exists():
                    seal_seen = now
                    try:
                        producer_exit = json.loads(args.seal_file.read_text())['node_exit']
                        expected = published_stamps(args.producer_events, index)
                    except Exception as error:
                        failure = 'producer_evidence_unavailable: ' + str(error)
                        break
                if expected is not None:
                    with lock:
                        consumed = {stamp for event, stamp in seen if event == 'pose_receive'}
                        if consumed == expected:
                            completed = True
                            closing = True
                            break
                    if now - seal_seen >= args.drain_timeout:
                        failure = 'pose_consumer_drain_timeout'
                        break
            time.sleep(.05)
        with lock:
            closing = True
        for subscriber in subscribers:
            subscriber.unregister()
        output.flush()
    consumed = {stamp for event, stamp in seen if event == 'pose_receive'}
    if args.mode == 'host':
        completed = seen == {(event, stamp) for event in ('host_rgb', 'host_depth') for stamp in index}
    else:
        if expected is None and not failure:
            failure = 'pose_consumer_stopped_before_producer_seal'
        if producer_exit != 0 and not failure:
            failure = 'producer_nonzero_exit'
    ok = completed and not errors and not callback_errors and not failure
    write_json(args.output.with_suffix('.status.json'), {
        'records': count, 'identity_duplicate_or_overflow_errors': errors,
        'callback_errors': callback_errors, 'mode': args.mode,
        'scope': 'independent subscriber callback observation',
        'normal_spin_return': bool(ok), 'consumer_complete': bool(completed),
        'producer_exit': producer_exit, 'expected_publications': len(expected) if expected is not None else None,
        'missing_source_stamps': sorted(expected-consumed) if expected is not None else None,
        'extra_source_stamps': sorted(consumed-expected) if expected is not None else None,
        'failure': failure})
    return 0 if ok else 2


if __name__ == '__main__':
    raise SystemExit(main())
