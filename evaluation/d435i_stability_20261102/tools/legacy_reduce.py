#!/usr/bin/env python3
"""Independently reduce the finished 003015_7ea2c9 board run; never run ROS."""
import argparse
from collections import Counter
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path


STATES = {-1: 'SYSTEM_NOT_READY', 0: 'NO_IMAGES_YET', 1: 'NOT_INITIALIZED',
          2: 'OK', 3: 'RECENTLY_LOST', 4: 'LOST', 5: 'OK_KLT'}
KINDS = ('received_rgb', 'received_depth', 'sync_drop_rgb', 'sync_drop_depth',
         'paired', 'returned', 'valid_fresh')
STAGES = ('orb', 'track_local_map', 'reference_copy', 'a1', 'a1_vote', 'a1_filter',
          'a3', 'a3_clear', 'a3_gather', 'search_local_points', 'local_pose_optimization',
          'map_update_wait', 'scale_snapshot')
WORKLOAD = ('global_kf', 'global_mp', 'local_kf', 'local_mp', 'voting_kf',
            'gather_slots', 'visited_slots', 'accepted_points', 'frame_keypoints', 'tracking_inliers')
BINARIES = {
    'ros_rgbd': '20a17635abf62bde699f83b6ca832324094986bdd9d1a2c53df28e4ef75981c4',
    'liborb_slam3_ros.so': '471f66f76f683c0ef22eb8e151b0efb056ef49615a887ac9158474937b95b6f4'}
POSE_KEYS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


def read_json(path):
    return json.loads(path.read_text())


def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def stats(values):
    """Finite samples only; callers separately account for invalid/unmeasured data."""
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return dict(count=0, mean=None, p50=None, p95=None, p99=None, min=None, max=None)
    def quantile(q):
        position = (len(values) - 1) * q
        left = int(position)
        right = min(left + 1, len(values) - 1)
        return values[left] + (values[right] - values[left]) * (position - left)
    return dict(count=len(values), mean=math.fsum(values) / len(values),
                p50=quantile(.5), p95=quantile(.95), p99=quantile(.99),
                min=values[0], max=values[-1])


def finite_pose(row, keys=POSE_KEYS):
    return all(math.isfinite(float(row[key])) for key in keys)


def stages_and_workload(frames, expected_count):
    stages = {}
    for name in STAGES:
        executed = [r for r in frames if int(r[name + '_calls']) > 0]
        item = dict(executed_callbacks=len(executed), unexecuted_callbacks=len(frames) - len(executed),
                    total_calls=sum(int(r[name + '_calls']) for r in frames))
        for metric in ('wall', 'cpu'):
            measured = [float(r[name + '_' + metric + '_ms']) for r in executed
                        if math.isfinite(float(r[name + '_' + metric + '_ms'])) and
                        float(r[name + '_' + metric + '_ms']) >= 0]
            total = math.fsum(measured)
            complete = len(measured) == len(executed)
            item[metric + '_ms_executed'] = stats(measured)
            item[metric + '_unmeasured_executed_callbacks'] = len(executed) - len(measured)
            item[metric + '_ms_sum'] = total if complete else None
            item[metric + '_ms_per_observed_callback'] = total / len(frames) if complete and frames else None
            item[metric + '_ms_per_expected_source'] = (total / expected_count
                if complete and len(frames) == expected_count and expected_count else None)
        stages[name] = item
    workload = {}
    for name in WORKLOAD:
        values = [int(r[name]) for r in frames if int(r[name]) >= 0]
        workload[name] = dict(measured=stats(values), unmeasured_callbacks=len(frames) - len(values))
    return stages, workload


def ranges(frames, index, predicate):
    groups = []
    for row in sorted(frames, key=lambda r: int(r['source_ordinal'])):
        if predicate(row):
            if not groups or int(row['source_ordinal']) != int(groups[-1][-1]['source_ordinal']) + 1:
                groups.append([])
            groups[-1].append(row)
    result = []
    for group in groups:
        first, last = int(group[0]['source_ordinal']), int(group[-1]['source_ordinal'])
        start, end = int(index[first]['source_stamp_ns']), int(index[last]['source_stamp_ns'])
        result.append(dict(start_ordinal=first, end_ordinal=last, observed_frames=len(group),
            original_start_stamp_ns=start, original_end_stamp_ns=end,
            header_span_ms=(end - start) / 1e6,
            state_counts=dict(Counter(STATES.get(int(r['state_after']), r['state_after']) for r in group)),
            map_id_counts=dict(Counter(r['map_id'] for r in group if int(r['scale_snapshot_mono_ns']) >= 0))))
    return result


def frame_summary(frames, expected_ordinals):
    duration = [float(r['callback_complete_ms']) for r in frames]
    callback = stats(duration)
    callback['over_100_ms'] = sum(v > 100 for v in duration)
    callback['over_100_fraction_of_observed'] = callback['over_100_ms'] / len(frames) if frames else None
    states = Counter(STATES.get(int(r['state_after']), r['state_after']) for r in frames)
    fresh = {int(r['source_ordinal']) for r in frames if int(r['pose_valid']) and int(r['pose_fresh'])}
    returned = {int(r['source_ordinal']) for r in frames if int(r['track_returned'])}
    seen = {int(r['source_ordinal']) for r in frames}
    snapshots = [r for r in frames if int(r['scale_snapshot_mono_ns']) >= 0 and int(r['map_id']) >= 0]
    transitions = [dict(source_ordinal=int(b['source_ordinal']), from_map=int(a['map_id']), to_map=int(b['map_id']))
                   for a, b in zip(snapshots, snapshots[1:]) if a['map_id'] != b['map_id']]
    stages, workload = stages_and_workload(frames, len(expected_ordinals))
    return dict(expected_sources=len(expected_ordinals), frame_records=len(frames),
        returned_sources=len(returned), fresh_sources=len(fresh),
        fresh_fraction_of_expected=len(fresh & expected_ordinals) / len(expected_ordinals) if expected_ordinals else None,
        missing_callback_ordinals=sorted(expected_ordinals - seen),
        missing_return_ordinals=sorted(expected_ordinals - returned),
        state_counts=dict(states), callback_ms=callback,
        callback_ms_fresh=stats(float(r['callback_complete_ms']) for r in frames
                               if int(r['source_ordinal']) in fresh),
        callback_ms_nonfresh=stats(float(r['callback_complete_ms']) for r in frames
                                  if int(r['source_ordinal']) not in fresh),
        stages=stages, workload=workload,
        map=dict(observed_ids=sorted({int(r['map_id']) for r in snapshots}),
                 switches=len(transitions), transitions=transitions,
                 scope='Current-map snapshots before tracking; not terminal map or full Atlas totals'))


def reduce_run(root, plan, entry, result):
    run = root / 'runs' / entry['id']
    problems = []
    def check(condition, message):
        if not condition:
            problems.append(message)
        return bool(condition)
    def rows(path, required):
        if not path.exists():
            problems.append('missing ' + str(path.relative_to(root)))
            return []
        with path.open(newline='') as stream:
            reader = csv.DictReader(stream)
            if not check(set(required) <= set(reader.fieldnames or []), 'schema mismatch: ' + str(path.relative_to(root))):
                return []
            return list(reader)
    def document(path):
        if not path.exists():
            problems.append('missing ' + str(path.relative_to(root)))
            return {}
        return read_json(path)

    index_path = root / 'inputs' / entry['id'] / 'input_index.csv'
    index_rows = rows(index_path, ('source_ordinal', 'source_stamp_ns', 'rgb_seq', 'depth_seq'))
    index = {int(r['source_ordinal']): r for r in index_rows}
    expected = set(range(1, entry['expected_pairs'] + 1))
    check(entry['expected_pairs'] == 5878, 'PLAN differs from the 5878 original exact-pair denominator')
    check(len(index_rows) == len(expected) and set(index) == expected, 'missing/duplicate/extra input-index ordinal')
    check(sha(index_path) == entry['index_sha256'], 'input index SHA differs from PLAN')
    record_table = root / 'inputs' / entry['id'] / 'original_input_records.csv'
    original_records = rows(record_table, ('source_ordinal', 'source_stamp_ns', 'rgb_seq', 'depth_seq',
                                          'rgb_record_ns', 'depth_record_ns'))
    check(sha(record_table) == entry['record_table_sha256'], 'original record-time table SHA differs from PLAN')
    check(len(original_records) == len(index_rows) and all(
        all(int(original[k]) == int(source[k]) for k in ('source_ordinal', 'source_stamp_ns', 'rgb_seq', 'depth_seq'))
        for original, source in zip(original_records, index_rows)), 'original record table and source index differ')
    ordered_stamps = [int(r['source_stamp_ns']) for r in index_rows]
    check(all(a < b for a, b in zip(ordered_stamps, ordered_stamps[1:])), 'source stamps not strictly increasing')
    by_stamp = {int(r['source_stamp_ns']): r for r in index_rows}
    completion = document(run / 'board/timing/completion.json')
    shutdown = document(run / 'board/timing/shutdown.json')
    required_frame = ('source_ordinal', 'source_stamp_ns', 'rgb_seq', 'depth_seq', 'enter_ns', 'complete_ns',
        'callback_complete_ms', 'callback_cpu_ms', 'track_called', 'track_returned', 'pose_fresh', 'pose_valid',
        'state_after', 'callback_error', 'map_id', 'scale_snapshot_mono_ns', 'clock_failures',
        'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw') + WORKLOAD + tuple(
        name + suffix for name in STAGES for suffix in ('_wall_ms', '_cpu_ms', '_calls'))
    frames = rows(run / 'board/timing/scale_frames.csv', required_frame)
    sources = rows(run / 'board/timing/scale_sources.csv',
                   ('source_ordinal', 'source_stamp_ns', 'rgb_seq', 'depth_seq') + KINDS)
    for name, table in [('frames', frames), ('sources', sources)]:
        counter = Counter(int(r['source_ordinal']) for r in table)
        check(not (set(counter) - expected) and max(counter.values(), default=0) <= 1,
              name + ' duplicate/extra ordinal')
        if name == 'sources':
            check(set(counter) == expected, 'source-accounting rows do not cover all 5878 inputs')
        for row in table:
            ordinal = int(row['source_ordinal'])
            original = index.get(ordinal)
            check(original is not None and all(int(row[k]) == int(original[k])
                for k in ('source_stamp_ns', 'rgb_seq', 'depth_seq')),
                name + ' original stamp/sequence identity mismatch at ordinal ' + str(ordinal))
    counts = {kind: sum(int(r[kind]) for r in sources) for kind in KINDS}
    sets = {kind: {int(r['source_ordinal']) for r in sources if int(r[kind]) > 0} for kind in KINDS}
    duplicates = {kind: sorted(int(r['source_ordinal']) for r in sources if int(r[kind]) > 1) for kind in KINDS}
    check(not any(duplicates[k] for k in ('received_rgb', 'received_depth', 'paired', 'returned', 'valid_fresh')),
          'source received/paired/returned/fresh event duplicates')
    frame_ids = {int(r['source_ordinal']) for r in frames}
    returned = {int(r['source_ordinal']) for r in frames if int(r['track_returned'])}
    fresh_rows = [r for r in frames if int(r['pose_valid']) and int(r['pose_fresh'])]
    fresh_ids = {int(r['source_ordinal']) for r in fresh_rows}
    fresh_stamps = {int(r['source_stamp_ns']) for r in fresh_rows}
    check(frame_ids == sets['paired'], 'paired source counters differ from frame records')
    check(returned == sets['returned'], 'returned source counters differ from frame flags')
    check(fresh_ids == sets['valid_fresh'], 'fresh source counters differ from frame flags')
    check(all(int(r['state_after']) in (2, 5) and finite_pose(r, ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'))
              for r in fresh_rows), 'fresh flag associated with non-OK state/nonfinite pose')
    check(all(int(r['callback_error']) == 0 and int(r['clock_failures']) == 0 for r in frames),
          'callback/core clock error recorded')
    check(all(math.isfinite(float(r['callback_complete_ms'])) and float(r['callback_complete_ms']) >= 0 and
              int(r['complete_ns']) >= int(r['enter_ns']) for r in frames), 'invalid callback duration/boundary')
    check(all(int(r[name + '_calls']) >= 0 and (int(r[name + '_calls']) == 0 or
              all(math.isfinite(float(r[name + '_' + clock + '_ms'])) and
                  float(r[name + '_' + clock + '_ms']) >= 0 for clock in ('wall', 'cpu')))
              for r in frames for name in STAGES), 'invalid executed stage duration or call count')
    for key, value in [('expected_inputs', len(expected)), ('frame_records', len(frames)),
                       ('processed_sources', len(returned)), ('valid_fresh_sources', len(fresh_ids))]:
        check(completion.get(key) == value, 'completion count mismatch: ' + key)
    check(completion.get('input_complete') == (returned == expected), 'completion input_complete mismatch')
    recorder_ok = completion.get('files_ok') is True and all(completion.get(key) == 0 for key in
        ('frame_overflow', 'event_overflow', 'identity_errors', 'callback_errors', 'cpu_clock_errors'))
    check(recorder_ok, 'recorder integrity counters/files not clean')
    check(completion.get('scale_detail') is True, 'scale detail observation was not enabled')
    check(completion.get('front_idle') is True and completion.get('front_timeout') is False,
          'front callbacks did not reach observed idle before cutoff')
    check(shutdown.get('shutdown_returned') is True, 'Shutdown return not recorded')

    host = rows(run / 'host_events.csv', ('event', 'source_stamp_ns', 'message_seq'))
    host_summary = {}
    for kind in ('rgb', 'depth'):
        table = [r for r in host if r['event'] == 'host_' + kind]
        observed = Counter(int(r['source_stamp_ns']) for r in table)
        identity_bad = [int(r['source_stamp_ns']) for r in table if int(r['source_stamp_ns']) not in by_stamp or
                        int(r['message_seq']) != int(by_stamp[int(r['source_stamp_ns'])][kind + '_seq'])]
        host_summary[kind] = dict(records=len(table), unique_stamps=len(observed),
            missing_source_ordinals=sorted(int(by_stamp[s]['source_ordinal']) for s in set(by_stamp) - set(observed)),
            extra_stamps=sorted(set(observed) - set(by_stamp)), duplicate_stamps=sorted(s for s, n in observed.items() if n > 1),
            identity_bad_stamps=identity_bad)
        check(set(observed) == set(by_stamp) and len(table) == len(expected) and not identity_bad,
              'host ' + kind + ' source observation incomplete/duplicate/identity mismatch')
    check(len(host) == sum(v['records'] for v in host_summary.values()), 'unexpected host input event kind')
    host_status = document(run / 'host_events.status.json')
    host_status_ok = (host_status.get('records') == len(host) == 2 * len(expected) and
        host_status.get('identity_duplicate_or_overflow_errors') == 0 and host_status.get('callback_errors') == 0 and
        host_status.get('consumer_complete') is True and host_status.get('normal_spin_return') is True and
        host_status.get('mode') == 'host' and not host_status.get('failure'))
    check(host_status_ok, 'host observer status does not confirm complete clean observations')

    consumers = {}
    frame_by_stamp = {int(r['source_stamp_ns']): r for r in frames}
    for kind in ('raw', 'continuous'):
        poses = rows(run / 'poses' / (kind + '.csv'), ('stamp_ns', 'frame') + POSE_KEYS)
        stamps = Counter(int(r['stamp_ns']) for r in poses)
        finite = sum(finite_pose(r) for r in poses)
        extras = set(stamps) - fresh_stamps
        nonfresh = []
        for stamp in sorted(extras & set(frame_by_stamp)):
            row = frame_by_stamp[stamp]
            nonfresh.append(dict(stamp_ns=stamp, source_ordinal=int(row['source_ordinal']),
                state_after=int(row['state_after']), state_name=STATES.get(int(row['state_after']), row['state_after']),
                pose_valid=bool(int(row['pose_valid'])), pose_fresh=bool(int(row['pose_fresh'])),
                consumer_records=stamps[stamp]))
        extra_states = Counter()
        for item in nonfresh:
            extra_states[item['state_name']] += item['consumer_records']
        missing_fresh = fresh_stamps - set(stamps)
        consumers[kind] = dict(records=len(poses), unique_stamps=len(stamps), finite_records=finite,
            nonfinite_records=len(poses) - finite, frame_counts=dict(Counter(r['frame'] for r in poses)),
            matched_fresh_records=sum(stamps[s] for s in fresh_stamps),
            finite_matched_fresh_records=sum(int(r['stamp_ns']) in fresh_stamps and finite_pose(r) for r in poses),
            missing_fresh_ordinals=sorted(int(by_stamp[s]['source_ordinal']) for s in missing_fresh),
            duplicate_stamps=sorted(s for s, n in stamps.items() if n > 1), extra_stamps=sorted(extras),
            extras_matched_to_nonfresh_frames=nonfresh, extra_nonfresh_state_counts=dict(extra_states),
            extra_stamps_without_frame=sorted(extras - set(frame_by_stamp)),
            all_fresh_sources_observed=not missing_fresh,
            scope='Finite topic messages can include nonfresh outputs; identity matching only, not cross-frame coordinate comparison')
        check(finite == len(poses), kind + ' consumer has nonfinite coordinates')

    memory = rows(run / 'memory.csv', ('steady_s', 'rss_kib', 'pss_kib', 'swap_kib', 'available_kib',
        'majflt', 'pswpin', 'pswpout', 'process_cpu_s', 'cpu_frequency_khz', 'temperature_milli_c', 'oom_kill'))
    mem = dict(samples=len(memory), scope='Approximately 1 Hz over sampler lifetime; sampled maxima, not VmHWM')
    if memory:
        mem.update(sample_interval_s=stats([float(b['steady_s']) - float(a['steady_s']) for a, b in zip(memory, memory[1:])]),
                   first_steady_s=float(memory[0]['steady_s']), last_steady_s=float(memory[-1]['steady_s']),
                   max_rss_kib=max(int(r['rss_kib']) for r in memory), max_pss_kib=max(int(r['pss_kib']) for r in memory),
                   max_vmswap_kib=max(int(r['swap_kib']) for r in memory),
                   min_available_kib=min(int(r['available_kib']) for r in memory),
                   temperature_c=stats([float(r['temperature_milli_c']) / 1000 for r in memory if r['temperature_milli_c']]),
                   cpu_frequency_mhz=stats([float(r['cpu_frequency_khz']) / 1000 for r in memory if r['cpu_frequency_khz']]))
        for key in ('majflt', 'pswpin', 'pswpout', 'oom_kill', 'process_cpu_s'):
            convert = float if key == 'process_cpu_s' else int
            first, last = convert(memory[0][key]), convert(memory[-1][key])
            mem[key] = dict(first=first, last=last, delta=last - first,
                            scope='process' if key in ('majflt', 'process_cpu_s') else 'whole system')
            check(last >= first, 'resource counter decreased: ' + key)
    else:
        check(False, 'no memory/resource samples')

    hashes = document(run / 'remote_sha256.json')
    bad_sha, absent = [], []
    for relative, recorded_sha in hashes.items():
        path = run / 'board' / relative
        if not path.is_file():
            absent.append(relative)
        elif sha(path) != recorded_sha:
            bad_sha.append(relative)
    transfer_ok = bool(hashes) and not absent and not bad_sha
    check(transfer_ok and result.get('transfer_verified') is True, 'board-artifact SHA transfer verification failed')
    loaded = document(run / 'loaded.json')
    preflight = document(run / 'board_preflight.json')
    binary_ok = preflight.get('sha256') == BINARIES
    check(binary_ok, 'preflight binary hashes differ from atomic_02')
    check(loaded.get('exe') == plan['package'] + '/ros_rgbd' and
          plan['package'] + '/liborb_slam3_ros.so' in loaded.get('maps', ''), 'actual loaded binary path mismatch')
    check(loaded.get('settings_sha256') == plan['settings_sha256'], 'loaded settings SHA mismatch')
    for key in ('package', 'source_commit', 'settings_sha256', 'rate', 'queues'):
        check(result.get(key) == plan[key], 'RESULT/PLAN mismatch: ' + key)
    bag_path = Path(entry['bag'])
    bag_stat = bag_path.stat()
    bag_unchanged = bag_stat.st_size == entry['bag_bytes'] and bag_stat.st_mtime_ns == entry['bag_mtime_ns']
    check(bag_unchanged, 'bag size/mtime changed after preparation')
    exit_keys = ('node_exit', 'player_exit', 'seal_exit', 'observer_exit', 'sampler_exit', 'slam_cleanup_exit', 'sampler_cleanup_exit')
    normal = all(result.get(k) == 0 for k in ('node_exit', 'player_exit', 'seal_exit', 'observer_exit'))
    cleanup_ok = all(result.get(k) == 0 for k in ('slam_cleanup_exit', 'sampler_cleanup_exit'))
    check(normal, 'nonzero/missing node/player/seal/host observer exit')
    check(cleanup_ok, 'owned process cleanup did not return zero')
    check(result.get('complete') is True, 'runner did not mark execution complete')
    overall = frame_summary(frames, expected)
    quartiles = []
    for n in range(4):
        begin, end = len(expected) * n // 4 + 1, len(expected) * (n + 1) // 4
        ordinals = set(range(begin, end + 1))
        selected = [r for r in frames if int(r['source_ordinal']) in ordinals]
        quartiles.append(dict(quartile=n + 1, start_ordinal=begin, end_ordinal=end,
                              **frame_summary(selected, ordinals)))
    missing_return = expected - sets['returned']
    missing_layers = dict(
        neither_filter_callback_observed=sorted(missing_return - sets['received_rgb'] - sets['received_depth']),
        only_rgb_filter_callback_observed=sorted(missing_return & (sets['received_rgb'] - sets['received_depth'])),
        only_depth_filter_callback_observed=sorted(missing_return & (sets['received_depth'] - sets['received_rgb'])),
        both_callbacks_without_pair=sorted(missing_return & sets['received_rgb'] & sets['received_depth'] - sets['paired']),
        paired_without_return=sorted(missing_return & sets['paired']),
        with_observed_sync_drop=sorted(missing_return & (sets['sync_drop_rgb'] | sets['sync_drop_depth'])),
        scope='After-deserialization filter/drop observations; wire arrival and subscription queue loss not measured')
    return dict(id=entry['id'], bag=entry['bag'], expected_pairs=len(expected),
        original_rgb_frames=entry.get('rgb_frames', len(expected)), original_depth_frames=entry.get('depth_frames', len(expected)),
        unpaired_rgb=entry.get('unpaired_rgb', 0), unpaired_depth=entry.get('unpaired_depth', 0),
        original_stamp_span_s=(ordered_stamps[-1] - ordered_stamps[0]) / 1e9 if ordered_stamps else None,
        bag_record_duration_s=entry['bag_record_duration_s'], source_header_mean_hz=entry['header_mean_hz'],
        source_header_gap_ms=entry['header_gap_ms'],
        input_adaptation='Native rosbag play with topic remap only; original stamp/sequence/payload preserved',
        direct_publisher_emission_count_measured=None,
        board_event_counts=counts, board_unique_source_counts={k: len(v) for k, v in sets.items()},
        missing_ordinals={k: sorted(expected - v) for k, v in sets.items()},
        repeated_event_ordinals=duplicates, missing_return_classification=missing_layers,
        whole=overall, source_quartiles=quartiles,
        nonfresh_intervals=ranges(frames, index, lambda r: not (int(r['pose_fresh']) and int(r['pose_valid']))),
        non_ok_intervals=ranges(frames, index, lambda r: int(r['state_after']) not in (2, 5)),
        lost_state_intervals=ranges(frames, index, lambda r: int(r['state_after']) in (3, 4)),
        host_input_observation=dict(verified=host_status_ok, streams=host_summary, status=host_status),
        consumers=consumers, memory=mem, completion=completion, shutdown=shutdown,
        execution_complete=result.get('complete') is True and normal and cleanup_ok and transfer_ok,
        paired_input_complete=returned == expected,
        measurement_integrity_ok=not problems, problems=problems,
        tracking_quality_pass=None, stable_performance_pass=None,
        exits={k: result.get(k) for k in exit_keys}, owned_cleanup_returned_zero=cleanup_ok,
        transfer=dict(runner_verified=result.get('transfer_verified'), independently_verified=transfer_ok,
                      files_checked=len(hashes), missing_files=absent, sha_mismatches=bad_sha),
        provenance=dict(source_commit=plan['source_commit'], package=plan['package'],
            preflight_binary_sha256=preflight.get('sha256'), binary_identity_ok=binary_ok,
            loaded_settings_sha256=loaded.get('settings_sha256'),
            prepared_bag_sha256=entry['bag_sha256'], bag_metadata_unchanged=bag_unchanged,
            bag_sha_recomputed_after_run=False), runner_error=result.get('error'), collect_error=result.get('collect_error'))


def fmt(value, places=2):
    return '—' if value is None else f'{value:.{places}f}'


def yes(value):
    return '是' if value else '否'


def report(summary):
    plan = summary['plan']
    calibration = summary['calibration']
    run = summary['runs'][0]
    whole = run['whole']
    callback = whole['callback_ms']
    lines = ['# D435i 003015_7ea2c9 板端回放报告', '',
        f"整段{run['expected_pairs']}对输入全部处理并正常退出；有效新位姿{whole['fresh_sources']}帧"
        f"（{fmt(100*whole['fresh_fraction_of_expected'])}%），失跟状态连续区间{len(run['lost_state_intervals'])}段、"
        f"地图编号变化{whole['map']['switches']}次，跟踪仍不稳定。完整回调均值{fmt(callback['mean'])} ms，"
        f"P95 {fmt(callback['p95'])} ms，P99 {fmt(callback['p99'])} ms，最大{fmt(callback['max'])} ms。", '',
        f"生成时间：{summary['generated_at_Asia_Shanghai']}。使用现有成套 `atomic_02` ARM 程序，"
        f"对应源码 `{plan['source_commit']}`，按原 bag 记录时间1×回放。", '',
        '本包基准为5878对原始精确同步RGB-D帧。保留原header.stamp、header.seq和图像负载，'
        '通过原生rosbag play仅重映射输入话题；没有配对重采样、时间戳改写或序号重编。'
        '本轮没有重建算法或替换既有程序，新进程和新图，关闭查看器并启用scale/detail记录。'
        'ORB沿用700特征、5层、3个执行者，RGB/depth/sync队列为8/8/10；'
        'Camera.fps=10，采用包内本机内参，16UC1对齐深度除以1000转换为米。', '',
        '## 标定与范围', '',
        '输入包含对齐深度和CameraInfo。标定生成依据、有效性检查和限制见'
        '[preflight/calibration.json](preflight/calibration.json)，实际使用配置为'
        '[preflight/d435i.yaml](preflight/d435i.yaml)。'
        f"配置SHA-256：`{calibration['settings_sha256']}`。", '',
        '实际配置摘录：' + '，'.join(f"{k}={v}" for k, v in calibration['settings_values'].items()) + '。', '',
        f"配置与包内CameraInfo的K/D及尺寸匹配复核：{yes(calibration['settings_match_camera_info'])}。"
        f"深度比例依据：{calibration['document']['depth_scale_basis']}", '',
        '没有真值轨迹，不能据此评价轨迹绝对精度、米制尺度误差或给出ATE/RPE通过结论。'
        '这是一轮详细观测测量，不能由单次结果宣称性能稳定、优化因果关系或生产验收通过。', '',
        '## 完整输入与跟踪', '',
        '| 轮次 | 基准对数 | host收到RGB/深度 | 板端收到RGB/深度 | 同步配对 | Track返回 | 有效新鲜 | 新鲜率（全部基准） |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in summary['runs']:
        e, h, whole = r['board_event_counts'], r['host_input_observation']['streams'], r['whole']
        lines.append(f"| {r['id']} | {r['expected_pairs']} | {h['rgb']['records']}/{h['depth']['records']} | "
            f"{e['received_rgb']}/{e['received_depth']} | {e['paired']} | {e['returned']} | {e['valid_fresh']} | "
            f"{fmt(100*whole['fresh_fraction_of_expected'])}% |")
        lines += ['', f"原记录时长{fmt(r['bag_record_duration_s'],3)} s，header平均频率"
            f"{fmt(r['source_header_mean_hz'],3)} Hz；相邻header间隔中位/最大值"
            f"{fmt(r['source_header_gap_ms']['median'],3)}/{fmt(r['source_header_gap_ms']['max'],3)} ms。", '']
    lines += ['', '所有完整性分母固定为原始5878对；没有只保留成功帧或各侧交集。'
        'host和板端“收到”均指独立订阅/成功反序列化后的回调观测，未直接测量publisher逐条发射、网络到达或队列驻留。'
        '完整缺失ordinal和已观察同步drop计数保留于SUMMARY.json；未收到不能定位为网络丢包。', '',
        '## 完整回调和首末四分段', '',
        '| 轮次/范围 | 基准/回调/返回/新鲜 | 平均ms | P50ms | P95ms | P99ms | 最大ms | >100ms |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in summary['runs']:
        segments = [('全段', r['whole'])] + [(f"Q{q['quartile']} {q['start_ordinal']}–{q['end_ordinal']}", q)
                                            for q in r['source_quartiles']]
        for label, q in segments:
            c = q['callback_ms']
            lines.append(f"| {r['id']} {label} | {q['expected_sources']}/{q['frame_records']}/{q['returned_sources']}/{q['fresh_sources']} | "
                f"{fmt(c['mean'])} | {fmt(c['p50'])} | {fmt(c['p95'])} | {fmt(c['p99'])} | {fmt(c['max'])} | {c['over_100_ms']} |")
    for r in summary['runs']:
        fresh, nonfresh = r['whole']['callback_ms_fresh'], r['whole']['callback_ms_nonfresh']
        lines += ['', f"仅有效新位姿的{fresh['count']}次回调：均值{fmt(fresh['mean'])} ms，"
            f"P95 {fmt(fresh['p95'])} ms；非新鲜{nonfresh['count']}次回调：均值{fmt(nonfresh['mean'])} ms，"
            f"P95 {fmt(nonfresh['p95'])} ms。两组共同组成全段统计，不剔除失跟回调。"]
    lines += ['', '四分段按原始source ordinal固定划分，包含初始化、失跟和缺失身份，未按性能或有效位姿择样。'
        '回调分位数仅使用实际执行记录并用线性插值；缺失输入另外列出，不能把未执行回调计为0 ms。'
        '100 ms只作为10 Hz帧间隔的描述阈值，不代表所有实时性门槛已通过。', '',
        '## 阶段与局部工作量', '',
        '| 阶段 | 执行回调数/全部回调 | 执行时wall均值ms | 执行时wall P95ms | 每回调wall成本ms | 执行时调用线程CPU均值ms |',
        '|---|---:|---:|---:|---:|---:|']
    for r in summary['runs']:
        for name, stage in r['whole']['stages'].items():
            lines.append(f"| {name} | {stage['executed_callbacks']}/{r['whole']['frame_records']} | "
                f"{fmt(stage['wall_ms_executed']['mean'])} | {fmt(stage['wall_ms_executed']['p95'])} | "
                f"{fmt(stage['wall_ms_per_observed_callback'])} | {fmt(stage['cpu_ms_executed']['mean'])} |")
    lines += ['', '阶段“执行时”统计只纳入calls>0且计时有效的回调，-1表示未测量，合法0 ms保留。'
        '每回调成本为执行阶段总耗时除以实际全部回调数，未执行阶段贡献0；只有全部基准输入均有回调时，'
        '才另给每个预期source成本。父子阶段包含嵌套，不能相加当作独立耗时或份额。'
        'CPU时间属于调用线程，ORB辅助工作线程和后台线程不包含在内。', '',
        '| 固定source范围 | ORB每回调ms | TrackLocalMap每回调ms | A1每回调ms | A3每回调ms | SearchLocalPoints每回调ms | 局部位姿优化每回调ms | map update等待每回调ms |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in summary['runs']:
        for q in r['source_quartiles']:
            cost = [fmt(q['stages'][name]['wall_ms_per_observed_callback']) for name in
                    ('orb', 'track_local_map', 'a1', 'a3', 'search_local_points', 'local_pose_optimization', 'map_update_wait')]
            lines.append(f"| Q{q['quartile']} {q['start_ordinal']}–{q['end_ordinal']} | " + ' | '.join(cost) + ' |')
    lines += ['',
        '| 范围 | 当前地图KF/MP最大值 | local KF均值/P95 | local MP均值/P95 | voting KF均值 | accepted points均值 | tracking inliers均值 |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for r in summary['runs']:
        for label, q in [('全段', r['whole'])] + [(f"Q{x['quartile']}", x) for x in r['source_quartiles']]:
            w = {k: v['measured'] for k, v in q['workload'].items()}
            lines.append(f"| {label} | {fmt(w['global_kf']['max'],0)}/{fmt(w['global_mp']['max'],0)} | "
                f"{fmt(w['local_kf']['mean'])}/{fmt(w['local_kf']['p95'])} | {fmt(w['local_mp']['mean'])}/{fmt(w['local_mp']['p95'])} | "
                f"{fmt(w['voting_kf']['mean'])} | {fmt(w['accepted_points']['mean'])} | {fmt(w['tracking_inliers']['mean'])} |")
    lines += ['', '工作量仅统计非负有效观测，未执行/未测量数单列在SUMMARY.json。global KF/MP为跟踪前、'
        '地图锁保护下当前地图容器快照，并非Atlas全图总量或停机最终地图。'
        '阶段和工作量随场景、时间及后台活动共同变化，只描述关联，不作单因果归因。', '',
        '## 状态、失跟区间、地图和消费者', '']
    for r in summary['runs']:
        lines += [f"{r['id']}：状态计数 `{r['whole']['state_counts']}`；观察到map_id "
                  f"`{r['whole']['map']['observed_ids']}`，变化{r['whole']['map']['switches']}次。", '']
        intervals = r['lost_state_intervals']
        if intervals:
            # Keep the prose compact; retain every interval in machine-readable evidence.
            examples = sorted(intervals, key=lambda x: x['header_span_ms'], reverse=True)[:8]
            lines += [f"RECENTLY_LOST/LOST连续区间共{len(intervals)}段，最长8段（不足8段则全部）为：" +
                '；'.join(f"{x['start_ordinal']}–{x['end_ordinal']} ({fmt(x['header_span_ms'])} ms, {x['state_counts']})"
                         for x in examples) + '。', '']
        else:
            lines += ['没有观察到RECENTLY_LOST或LOST回调。', '']
        for kind, consumer in r['consumers'].items():
            lines += [f"{kind}：收到{consumer['records']}条，有限{consumer['finite_records']}条，"
                f"匹配新鲜帧{consumer['matched_fresh_records']}条，漏新鲜{len(consumer['missing_fresh_ordinals'])}个；"
                f"相对fresh集合的额外输出{len(consumer['extra_stamps'])}个时间戳，"
                f"匹配非fresh回调状态 `{consumer['extra_nonfresh_state_counts']}`，"
                f"未匹配回调{len(consumer['extra_stamps_without_frame'])}个。", '']
    lines += ['连续区间只连接实际记录且ordinal相邻的帧，时长为首末原header差，单帧区间为0 ms；'
        '不是失跟开始到恢复的精确连续时长。所有非fresh、非OK及失跟区间、状态计数和地图变化保存在SUMMARY.json。', '',
        '有限topic输出不等于新鲜有效位姿。额外输出按同一原时间戳核对scale_frames的状态及valid/fresh标记；'
        'RECENTLY_LOST等状态下已有发布路径可能沿用旧位姿，不能只看topic收到数判断轨迹有效。'
        '该时间戳关联不独立证明数值来源，也不比较raw/continuous两个坐标系的坐标。'
        'trajectory_online.tum是在线新鲜Twc，不是后端校正轨迹。', '',
        '## 资源、退出和证据', '',
        '| 轮次 | 约1 Hz样本数 | RSS/PSS/VmSwap采样最大MiB | 温度°C | CPU频率MHz | 进程major fault首→末 | 系统pswpin/pswpout首→末 | 系统OOM kill首→末 |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in summary['runs']:
        m = r['memory']
        if m['samples']:
            lines.append(f"| {r['id']} | {m['samples']} | {fmt(m['max_rss_kib']/1024)}/{fmt(m['max_pss_kib']/1024)}/{fmt(m['max_vmswap_kib']/1024)} | "
                f"{fmt(m['temperature_c']['min'])}–{fmt(m['temperature_c']['max'])} | "
                f"{fmt(m['cpu_frequency_mhz']['min'])}–{fmt(m['cpu_frequency_mhz']['max'])} | "
                f"{m['majflt']['first']}→{m['majflt']['last']} | {m['pswpin']['first']}→{m['pswpin']['last']} / "
                f"{m['pswpout']['first']}→{m['pswpout']['last']} | {m['oom_kill']['first']}→{m['oom_kill']['last']} |")
            lines += ['', f"资源首末增量：进程major fault {m['majflt']['delta']}，系统pswpin/pswpout "
                f"{m['pswpin']['delta']}/{m['pswpout']['delta']}，系统OOM kill {m['oom_kill']['delta']}。", '']
    lines += ['内存最大值来自约1 Hz采样，覆盖sampler生命周期，可包括准备与关闭；不是VmHWM或绝对峰值。'
        'major fault为进程计数，pswpin/pswpout/OOM为整机累计计数，需看首末差值，不能把历史绝对值归为本轮SLAM故障。', '',
        '| 轮次 | 执行及回传完成 | 全部配对返回 | 归约完整性 | node/player/seal/observer退出 | host status复核 | owned清理返回0 | SHA复核文件数 |',
        '|---|---|---|---|---|---|---|---:|']
    for r in summary['runs']:
        exits = '/'.join(str(r['exits'][k]) for k in ('node_exit', 'player_exit', 'seal_exit', 'observer_exit'))
        lines.append(f"| {r['id']} | {yes(r['execution_complete'])} | {yes(r['paired_input_complete'])} | "
            f"{yes(r['measurement_integrity_ok'])} | {exits} | {yes(r['host_input_observation']['verified'])} | "
            f"{yes(r['owned_cleanup_returned_zero'])} | {r['transfer']['files_checked']} |")
        if r['problems']:
            lines += ['', '需保留的问题：' + '；'.join(r['problems']) + '。', '']
    audit = summary['board_final_audit']
    lines += ['', f"测试后只读核验：本轮登记进程已退出={yes(audit['all_owned_processes_exited'])}，"
        f"成套程序SHA保持={yes(audit['package_unchanged'])}，活动SLAM进程{len(audit['active_slam'])}个，"
        f"本轮新增Atlas文件{len(audit['new_atlas_files'])}个。详见[BOARD_FINAL_AUDIT.json](BOARD_FINAL_AUDIT.json)。", '',
        '执行正常、输入完整、新鲜位姿比例、消费者收到以及性能质量分别记录。'
        '板端回传逐文件对照远端SHA清单独立重算，预检二进制SHA、实际加载路径及配置SHA另行核对。'
        'bag在准备阶段全量计算SHA；本归约只复核原文件大小和mtime，没有再次全量重算bag内容SHA。', '',
        'scale的completion在Shutdown之前写入，backend_joined、pose_consumer_complete及shutdown的joined_verified'
        '保留原记录的null语义。Shutdown返回、节点退出和所属进程清理成功不等同于后台自然排空或所有线程已join。', '',
        '完整原始记录、固定全部输入分母、缺失/重复身份、执行阶段和各四分段数值见'
        '[SUMMARY.json](SUMMARY.json)。本工具只读证据并写此报告和SUMMARY.json，不修改bag、配置或算法。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    root = parser.parse_args().root.resolve()
    plan = read_json(root / 'PLAN.json')
    results = [read_json(root / 'runs' / entry['id'] / 'RESULT.json') for entry in plan['runs']]
    if any(result.get('phase') != 'finished' for result in results):
        raise SystemExit('Refusing reduction while a run is not finished.')
    settings_path = Path(plan['settings'])
    settings_sha = sha(settings_path)
    if settings_sha != plan['settings_sha256']:
        raise SystemExit('Settings SHA differs from PLAN; no summary written.')
    calibration_path = root / 'preflight/calibration.json'
    calibration_document = read_json(calibration_path)
    settings_values = {}
    wanted = ('Camera.type', 'Camera1.fx', 'Camera1.fy', 'Camera1.cx', 'Camera1.cy',
              'Camera1.k1', 'Camera1.k2', 'Camera1.p1', 'Camera1.p2', 'Camera1.k3',
              'Camera.width', 'Camera.height', 'Camera.fps', 'Camera.RGB', 'Stereo.b', 'RGBD.DepthMapFactor')
    for line in settings_path.read_text().splitlines():
        if ':' in line:
            name, value = line.split(':', 1)
            if name.strip() in wanted:
                settings_values[name.strip()] = value.split('#', 1)[0].strip()
    color, depth = calibration_document['color'], calibration_document['aligned_depth']
    scalar_expectations = dict(zip(('Camera1.fx', 'Camera1.fy', 'Camera1.cx', 'Camera1.cy'),
                                  (color['K'][0], color['K'][4], color['K'][2], color['K'][5])))
    scalar_expectations.update(zip(('Camera1.k1', 'Camera1.k2', 'Camera1.p1', 'Camera1.p2', 'Camera1.k3'), color['D']))
    scalar_expectations.update({'Camera.width': color['width'], 'Camera.height': color['height']})
    calibration_ok = (calibration_document['camera_info_constant'] is True and
        all(color[key] == depth[key] for key in ('width', 'height', 'K', 'D', 'frame_id')) and
        all(float(settings_values[key]) == value for key, value in scalar_expectations.items()))
    runs = [reduce_run(root, plan, entry, result) for entry, result in zip(plan['runs'], results)]
    if not calibration_ok:
        for run in runs:
            run['problems'].append('YAML intrinsics/distortion/dimensions differ from recorded CameraInfo')
            run['measurement_integrity_ok'] = False
    summary = dict(schema='rgbd_d435i_board_summary_v1',
        generated_at_Asia_Shanghai=datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds'),
        plan=plan, board_final_audit=read_json(root / 'BOARD_FINAL_AUDIT.json'),
        calibration=dict(path=str(calibration_path), sha256=sha(calibration_path),
            document=calibration_document, settings_path=str(settings_path), settings_sha256=settings_sha,
            settings_values=settings_values, settings_match_camera_info=calibration_ok), runs=runs,
        all_execution_complete=all(r['execution_complete'] for r in runs),
        all_paired_input_complete=all(r['paired_input_complete'] for r in runs),
        all_measurement_integrity_ok=all(r['measurement_integrity_ok'] for r in runs),
        scope='Single detailed observation on all 5878 original exact pairs; no GT accuracy or stable-performance qualification')
    encoded = json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    markdown = report(summary)
    (root / 'SUMMARY.json').write_text(encoded)
    (root / 'REPORT.md').write_text(markdown)
    print(json.dumps({k: summary[k] for k in ('all_execution_complete', 'all_paired_input_complete',
                                            'all_measurement_integrity_ok')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
