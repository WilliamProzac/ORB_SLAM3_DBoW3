#!/usr/bin/env python3
"""Independent raw-CSV reduction of all 16 sealed RGBD36 trials.

No frozen reducer is imported. Only this analysis directory is written.
"""
import csv
import hashlib
import json
import math
import re
import statistics as st
from collections import Counter
from pathlib import Path

OUT = Path(__file__).resolve().parent
ART = OUT.parent
STATE = {0: 'NO_IMAGES_YET', 1: 'NOT_INITIALIZED', 2: 'OK', 3: 'RECENTLY_LOST', 4: 'LOST', 5: 'OK_KLT'}


def read_json(path):
    return json.loads(path.read_text())


def read_csv(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def quantile(a, p):
    a = sorted(a)
    if not a:
        return None
    x = (len(a) - 1) * p
    i = int(x)
    return a[i] + (a[min(i + 1, len(a) - 1)] - a[i]) * (x - i)


def distribution(a):
    return dict(count=len(a), mean=st.mean(a) if a else None,
                p95=quantile(a, .95), p99=quantile(a, .99),
                min=min(a) if a else None, max=max(a) if a else None)


def boolfield(row, k):
    return int(row[k]) == 1


def isfresh(row):
    return boolfield(row, 'pose_fresh') and boolfield(row, 'pose_valid')


def resource(rows):
    def values(k):
        return [float(r[k]) for r in rows if r.get(k) not in ['', None]]
    def maximum(k):
        a = values(k)
        return max(a) if a else None
    def delta(k):
        a = values(k)
        return a[-1] - a[0] if a else None
    rss, pss, swap = [maximum(k) for k in ['rss_kib', 'pss_kib', 'swap_kib']]
    temps = [v / 1000 for v in values('temperature_milli_c')]
    freq = [v / 1000 for v in values('cpu_frequency_khz')]
    return dict(samples=len(rows), max_rss_mib=rss / 1024 if rss is not None else None,
                max_pss_mib=pss / 1024 if pss is not None else None,
                max_vmswap_mib=swap / 1024 if swap is not None else None,
                min_available_mib=min(values('available_kib')) / 1024 if rows else None,
                temperature_c=distribution(temps), frequency_mhz=distribution(freq),
                frequency_counts=dict(sorted(Counter(freq).items())),
                majflt_delta=delta('majflt'), pswpin_delta=delta('pswpin'),
                pswpout_delta=delta('pswpout'), oom_kill_delta=delta('oom_kill'))


def main():
    plan = read_json(ART / 'PLAN.json')
    protocol = read_json(ART / 'preflight/EXPERIMENT_PROTOCOL.json')
    original = read_csv(Path(plan['input']['index']))
    expected = len(original)
    assert expected == plan['input']['expected_pairs'] == 5878
    index = {int(r['source_ordinal']): r for r in original}
    stamps = {n: int(r['source_stamp_ns']) for n, r in index.items()}
    span = (stamps[expected] - stamps[1]) / 1e9
    result = dict(scope='Independent reduction; no GT, no trajectory-accuracy qualification; all 16 rounds retained',
                  source_count=expected, rounds=[], discrepancies=[], inputs_sha256={})
    checks = []

    def check(label, observed, archived, tolerance=1e-8):
        if isinstance(observed, (float, int)) and isinstance(archived, (float, int)):
            passed = abs(observed - archived) <= tolerance
        else:
            passed = observed == archived
        checks.append({'label': label, 'passed': passed})
        if not passed:
            result['discrepancies'].append(dict(label=label, independently_computed=observed, archived=archived))
        return passed

    for declaration in plan['runs']:
        name = declaration['id']
        run = ART / 'runs' / name
        files = {k: run / v for k, v in {
            'frames': 'board/timing/scale_frames.csv', 'sources': 'board/timing/scale_sources.csv',
            'summary': 'SUMMARY.json', 'result': 'RESULT.json', 'loaded': 'loaded.json',
            'memory': 'memory.csv', 'log': 'node.log'}.items()}
        result['inputs_sha256'].update({str(p.relative_to(ART)): sha(p) for p in files.values()})
        frames = read_csv(files['frames'])
        sources = read_csv(files['sources'])
        summary, execution, loaded = [read_json(files[k]) for k in ['summary', 'result', 'loaded']]
        by_n = {int(r['source_ordinal']): r for r in frames}
        source_n = {int(r['source_ordinal']): r for r in sources}
        count_before = len(result['discrepancies'])
        check(name + '.frame_ordinal_unique', len(by_n), len(frames))
        check(name + '.source_ordinal_unique', len(source_n), len(sources))
        check(name + '.source_set', sorted(source_n), sorted(index))
        identity_errors = []
        for scope, rows in [('frames', frames), ('sources', sources)]:
            for r in rows:
                n = int(r['source_ordinal'])
                if n not in index or any(int(r[k]) != int(index[n][k]) for k in ['source_stamp_ns', 'rgb_seq', 'depth_seq']):
                    identity_errors.append([scope, n])
        check(name + '.exact_stamp_seq_identity', identity_errors, [])
        fresh = {n for n, r in by_n.items() if isfresh(r)}
        callback = distribution([float(r['callback_complete_ms']) for r in frames])
        over100 = sum(float(r['callback_complete_ms']) > 100 for r in frames)
        states = dict(Counter(STATE.get(int(r['state_after']), r['state_after']) for r in frames))
        source_counts = {k: sum(int(r[k]) for r in sources) for k in ['received_rgb', 'received_depth', 'paired', 'returned', 'valid_fresh', 'sync_drop_rgb', 'sync_drop_depth']}
        check(name + '.fresh_source_flags', sorted(n for n, r in source_n.items() if int(r['valid_fresh']) > 0), sorted(fresh))
        for k in ['received_rgb', 'received_depth', 'paired', 'returned']:
            check(name + '.' + k, source_counts[k], expected)
        for k in ['sync_drop_rgb', 'sync_drop_depth']:
            check(name + '.' + k, source_counts[k], 0)
        check(name + '.callback_errors', sum(int(r['callback_error']) for r in frames), 0)
        check(name + '.clock_failures', sum(int(r['clock_failures']) for r in frames), 0)
        check(name + '.state_counts', states, summary['whole']['state_counts'])
        for k, actual in [('frame_records', len(frames)), ('returned_sources', source_counts['returned']), ('fresh_sources', len(fresh))]:
            check(name + '.summary_' + k, actual, summary['whole'][k])
        for k in ['mean', 'p95', 'p99', 'max']:
            check(name + '.callback_' + k, callback[k], summary['whole']['callback_ms'][k])
        check(name + '.over100', over100, summary['whole']['callback_ms']['over_100_ms'])
        check(name + '.run_complete', execution['complete'], True)
        for k in ['node_exit', 'player_exit', 'seal_exit', 'observer_exit', 'pose_observer_exit', 'sampler_exit', 'slam_cleanup_exit', 'sampler_cleanup_exit']:
            check(name + '.' + k, execution[k], 0)
        check(name + '.transfer_verified', execution['transfer_verified'], True)
        check(name + '.summary_trial_valid', summary['trial_valid'], True)
        config = plan['configs'][execution['config_id']]
        package = plan['packages'][execution['package_id']]
        check(name + '.loaded_binary_sha', loaded['actual_sha256'], package['binaries'])
        check(name + '.summary_binary_sha', summary['provenance']['preflight_binary_sha256'], package['binaries'])
        check(name + '.config_sha', execution['settings_sha256'], config['sha256'])
        check(name + '.loaded_config_sha', loaded['settings_sha256'], config['sha256'])
        check(name + '.loaded_eager_bow', loaded['eager_bow'], '1' if execution['bow_ready'] else '0')
        check(name + '.loaded_observation', bool(loaded['stability_directory']), execution['observation'])
        check(name + '.loaded_executable_path', loaded['exe'], package['remote_path'] + '/ros_rgbd')
        check(name + '.loaded_core_path', package['remote_path'] + '/liborb_slam3_ros.so' in loaded['maps'], True)
        check(name + '.stage', execution['stage'], declaration['stage'])
        check(name + '.config', execution['config_id'], declaration['config_id'])

        # Current-map snapshots; distinguish observed ID changes from reset logs.
        changes, resets, previous = [], [], None
        for n, r in sorted(by_n.items()):
            if previous is not None:
                pn, pr = previous
                if int(r['map_id']) != int(pr['map_id']):
                    changes.append(n)
                if n == pn + 1 and int(pr['state_after']) == 4 and int(r['state_after']) == 2 and int(r['global_kf']) == int(r['global_mp']) == 0 and r['map_id'] == pr['map_id']:
                    resets.append(pn)
            previous = (n, r)
        log = files['log'].read_text(errors='replace')
        creations = [int(n) for n in re.findall(r'Creation of new map with id:\s*(\d+)', log)]
        new_maps = len(creations) - 1 if creations else None
        reset_logs = log.count('LM: Active map reset, Done!!!')
        merge_detected = log.count('*Merge detected')
        merge_finished = log.count('Merge finished!')
        check(name + '.new_map_logs', new_maps, summary['dynamics']['new_map_creation_logs']['after_initial_atlas_creation'])
        check(name + '.map_id_changes', len(changes), summary['dynamics']['map_id_changes']['count'])
        check(name + '.same_id_reset_count', len(resets), summary['dynamics']['same_id_active_resets']['raw_count'])
        check(name + '.reset_logs', reset_logs, summary['dynamics']['same_id_active_resets']['completed_log_count'])
        check(name + '.reset_log_matches_raw', reset_logs, len(resets))
        nonfresh_s = fresh_s = unknown_s = 0.0
        for n in range(1, expected):
            dt = (stamps[n + 1] - stamps[n]) / 1e9
            if n not in by_n:
                unknown_s += dt
            elif n in fresh:
                fresh_s += dt
            else:
                nonfresh_s += dt
        runs, current = [], []
        for n in range(1, expected + 1):
            if n in fresh:
                current.append(n)
            elif current:
                runs.append(current)
                current = []
        if current:
            runs.append(current)
        episodes, current_loss = [], []
        for n in range(1, expected + 1):
            if n not in fresh:
                current_loss.append(n)
            elif current_loss:
                episodes.append(current_loss)
                current_loss = []
        if current_loss:
            episodes.append(current_loss)
        outcomes = Counter()
        for episode in episodes:
            end = episode[-1]
            if end == expected:
                outcomes['eof_unrecovered'] += 1
            elif int(by_n[end]['map_id']) != int(by_n[end + 1]['map_id']):
                outcomes['new_map'] += 1
            elif end in resets:
                outcomes['same_id_reset'] += 1
            else:
                outcomes['recovered_without_new_map_or_same_id_reset'] += 1
        check(name + '.episode_new_map_count', outcomes['new_map'], new_maps)
        check(name + '.episode_reset_count', outcomes['same_id_reset'], len(resets))
        longest = max(runs, key=len) if runs else []
        longest_duration = (stamps[longest[-1]] - stamps[longest[0]]) / 1e9 if longest else 0
        check(name + '.weighted_nonfresh_s', nonfresh_s, summary['dynamics']['time_weights']['nonfresh_s'])
        check(name + '.weighted_fresh_s', fresh_s, summary['dynamics']['time_weights']['fresh_s'])
        check(name + '.longest_fresh_frames', len(longest), summary['dynamics']['contiguous_fresh']['longest_frames'])
        samples = read_csv(files['memory'])
        memory = resource(samples)
        # Both use board monotonic time; no host-board subtraction is used.
        first_enter = min(int(v['enter_ns']) for v in frames) / 1e9
        last_complete = max(int(v['complete_ns']) for v in frames) / 1e9
        envelope = [r for r in samples if first_enter <= float(r['steady_s']) <= last_complete]
        envelope_resource = resource(envelope)
        for raw, report in [('max_rss_mib', 'max_rss_kib'), ('max_pss_mib', 'max_pss_kib'), ('max_vmswap_mib', 'max_vmswap_kib')]:
            check(name + '.memory_' + raw, memory[raw] * 1024, summary['memory'][report])
        for k, report in [('majflt_delta', 'majflt'), ('pswpin_delta', 'pswpin'), ('pswpout_delta', 'pswpout'), ('oom_kill_delta', 'oom_kill')]:
            check(name + '.' + k, memory[k], summary['memory'][report]['delta'])
        for k, report in [('temperature_c', 'temperature_c'), ('frequency_mhz', 'cpu_frequency_mhz')]:
            for stat in ['min', 'max']:
                check(name + '.' + k + '_' + stat, memory[k][stat], summary['memory'][report][stat])
        entry = dict(id=name, stage=execution['stage'], config=execution['config_id'],
            observation=execution['observation'], bow_ready=execution['bow_ready'],
            processed=len(frames), returned=source_counts['returned'], fresh=len(fresh),
            nonfresh=expected - len(fresh), fresh_fraction=len(fresh) / expected, state_counts=states,
            callback=callback, over100=over100, new_maps=new_maps, map_id_changes=len(changes),
            same_id_resets=len(resets), reset_request_ordinals=resets, new_map_log_ids=creations,
            loss_onsets=len(episodes), transient_recoveries=outcomes['recovered_without_new_map_or_same_id_reset'],
            recovery_category_definition='Returned to fresh without detected new-map or same-ID reset; merge may change coordinates.',
            merge_detected_logs=merge_detected, merge_finished_logs=merge_finished,
            loss_episode_outcomes=dict(outcomes),
            fresh_segments=len(runs), longest_fresh_frames=len(longest), longest_fresh_s=longest_duration,
            longest_fresh_start=longest[0] if longest else None, longest_fresh_end=longest[-1] if longest else None,
            weighted_nonfresh_s=nonfresh_s, weighted_nonfresh_fraction=nonfresh_s / span,
            weighted_fresh_s=fresh_s, unknown_duration_s=unknown_s, right_censored_eof_nonfresh=expected not in fresh,
            memory=memory, callback_envelope_memory=envelope_resource,
            trial_valid=summary['trial_valid'], input_complete=sorted(by_n) == sorted(index) and all(source_counts[k] == expected for k in ['received_rgb', 'received_depth', 'paired', 'returned']),
            discrepancies_in_this_round=len(result['discrepancies']) - count_before)
        result['rounds'].append(entry)

    def stage_groups(stage, switch):
        rows = [r for r in result['rounds'] if r['stage'] == stage]
        return [r for r in rows if not r[switch]], [r for r in rows if r[switch]]

    off, on = stage_groups('observation', 'observation')
    off_p95, on_p95 = [st.mean(r['callback']['p95'] for r in rs) for rs in [off, on]]
    off_fresh, on_fresh = [st.mean(r['fresh_fraction'] for r in rs) for rs in [off, on]]
    fresh_range = max(r['fresh_fraction'] for r in off) - min(r['fresh_fraction'] for r in off)
    obs = dict(off_ids=[r['id'] for r in off], on_ids=[r['id'] for r in on],
        off_p95_mean_ms=off_p95, on_p95_mean_ms=on_p95, p95_increase_ms=on_p95 - off_p95,
        p95_allowed_increase_ms=max(2, .05 * off_p95), off_fresh_mean=off_fresh,
        on_fresh_mean=on_fresh, off_fresh_range=fresh_range, absolute_fresh_difference=abs(on_fresh - off_fresh),
        allowed_fresh_difference=max(.02, fresh_range))
    obs['gate_passed'] = obs['p95_increase_ms'] <= obs['p95_allowed_increase_ms'] and obs['absolute_fresh_difference'] <= obs['allowed_fresh_difference']
    off, on = stage_groups('bow', 'bow_ready')
    off_p95, on_p95 = [st.mean(r['callback']['p95'] for r in rs) for rs in [off, on]]
    off_fresh, on_fresh = [st.mean(r['fresh_fraction'] for r in rs) for rs in [off, on]]
    fresh_range = max(r['fresh_fraction'] for r in off) - min(r['fresh_fraction'] for r in off)
    bow = dict(off_ids=[r['id'] for r in off], on_ids=[r['id'] for r in on],
        off_fresh_mean=off_fresh, on_fresh_mean=on_fresh, off_replicate_range=fresh_range,
        allowed_fresh_decline=max(.02, fresh_range), off_p95_mean_ms=off_p95, on_p95_mean_ms=on_p95,
        on_input_completion_failure=any(not r['input_complete'] or not r['trial_valid'] for r in on),
        fresh_gate_failed=off_fresh - on_fresh > max(.02, fresh_range),
        latency_gate_failed=on_p95 > 100 and off_p95 <= 100)
    bow['gate_passed'] = not any(bow[k] for k in ['on_input_completion_failure', 'fresh_gate_failed', 'latency_gate_failed'])
    for gate, path in [(obs, ART / 'preflight/stage_bindings/bow.json'), (bow, ART / 'preflight/stage_bindings/matrix.json')]:
        stored = read_json(path)['decision_evidence']
        for k in gate:
            if k in stored:
                check(path.name + '.' + k, gate[k], stored[k])
    result['stage_gates'] = {'observation': obs, 'bow': bow}
    matrix = [r for r in result['rounds'] if r['stage'] == 'matrix']
    baseline = st.mean(r['nonfresh'] for r in matrix if r['config'] == 'P0')
    group_stats = {}
    for config in ['P0', 'P1', 'P2', 'P3']:
        rows = [r for r in matrix if r['config'] == config]
        fields = {'nonfresh': [r['nonfresh'] for r in rows], 'fresh': [r['fresh'] for r in rows],
            'callback_mean_ms': [r['callback']['mean'] for r in rows], 'p95_ms': [r['callback']['p95'] for r in rows],
            'p99_ms': [r['callback']['p99'] for r in rows], 'over100': [r['over100'] for r in rows],
            'new_maps': [r['new_maps'] for r in rows], 'same_id_resets': [r['same_id_resets'] for r in rows],
            'loss_onsets': [r['loss_onsets'] for r in rows],
            'transient_recoveries': [r['transient_recoveries'] for r in rows],
            'merge_detected_logs': [r['merge_detected_logs'] for r in rows],
            'merge_finished_logs': [r['merge_finished_logs'] for r in rows],
            'weighted_nonfresh_s': [r['weighted_nonfresh_s'] for r in rows],
            'weighted_nonfresh_fraction': [r['weighted_nonfresh_fraction'] for r in rows],
            'longest_fresh_frames': [r['longest_fresh_frames'] for r in rows],
            'longest_fresh_s': [r['longest_fresh_s'] for r in rows]}
        aggregates = {k: {'values': v, 'mean': st.mean(v), 'min': min(v), 'max': max(v)} for k, v in fields.items()}
        quality_each = [r['nonfresh'] <= .70 * baseline for r in rows]
        input_each = [r['input_complete'] for r in rows]
        latency_each = [r['callback']['p95'] < 100 for r in rows]
        group_stats[config] = dict(config=plan['configs'][config], ids=[r['id'] for r in rows], aggregates=aggregates,
            nonfresh_reduction_vs_P0_mean=1 - aggregates['nonfresh']['mean'] / baseline,
            nonfresh_reduction_each_vs_P0_mean=[1 - r['nonfresh'] / baseline for r in rows],
            quality_threshold_nonfresh=.70 * baseline, quality_each=quality_each,
            input_each=input_each, latency_each=latency_each,
            strict_numeric_screen_pass=config != 'P0' and all(quality_each + input_each + latency_each),
            full_acceptance=None, note='P0 is reference; no GT or wrong-relocalization diagnosis supplied by these numerical checks.')
    result['matrix'] = group_stats
    result['comparison_checks'] = {'checked': len(checks), 'passed': sum(v['passed'] for v in checks), 'failed': len(result['discrepancies'])}
    for p in [ART / 'PLAN.json', ART / 'preflight/EXPERIMENT_PROTOCOL.json', ART / 'preflight/stage_bindings/bow.json', ART / 'preflight/stage_bindings/matrix.json']:
        result['inputs_sha256'][str(p.relative_to(ART))] = sha(p)
    flat = []
    for r in result['rounds']:
        flat.append({k: r[k] for k in ['id', 'stage', 'config', 'observation', 'bow_ready', 'processed', 'returned', 'fresh', 'nonfresh', 'fresh_fraction', 'over100', 'new_maps', 'map_id_changes', 'same_id_resets', 'loss_onsets', 'transient_recoveries', 'merge_detected_logs', 'merge_finished_logs', 'fresh_segments', 'longest_fresh_frames', 'longest_fresh_s', 'weighted_nonfresh_s', 'weighted_nonfresh_fraction', 'input_complete', 'trial_valid', 'discrepancies_in_this_round']})
        flat[-1].update({'state_' + k: v for k, v in r['state_counts'].items()})
        flat[-1].update({'callback_' + k: v for k, v in r['callback'].items()})
        flat[-1].update({k: v for k, v in r['memory'].items() if not isinstance(v, dict)})
        flat[-1].update({k + '_' + s: r['memory'][k][s] for k in ['temperature_c', 'frequency_mhz'] for s in ['min', 'max', 'mean']})
    keys = list(dict.fromkeys(k for r in flat for k in r))
    with (OUT / 'round_results.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(flat)
    (OUT / 'COMPARISON.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    generate_report(result)
    print('RAW_COMPARISON_COMPLETE', result['comparison_checks'])
    for k, v in group_stats.items():
        print(k, 'nonfresh', v['aggregates']['nonfresh'], 'reduction', v['nonfresh_reduction_vs_P0_mean'], 'screen', v['strict_numeric_screen_pass'])


def generate_report(r):
    lines = ['# 16轮稳定性试验：独立数值复核', '',
        '原scale_frames/scale_sources逐行对照原5878个header/seq；独立重算回调、状态、时间权重、地图与资源，再对照RESULT、loaded及封存SUMMARY。没有改写封存数据或复用冻结归约函数。', '',
        f"共{r['comparison_checks']['checked']}项对照，{r['comparison_checks']['failed']}项差异；全部差异（如有）保留在COMPARISON.json。回调分位数采用线性插值，全部完成回调均纳入，不删失跟帧。", '',
        '| 轮次 | fresh / 5878 | 非fresh | 回调均值 ms | P95 ms | P99 ms | >100 ms帧 | 新图/同ID重置 |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for x in r['rounds']:
        c = x['callback']
        lines.append(f"| {x['id']} | {x['fresh']} | {x['nonfresh']} | {c['mean']:.3f} | {c['p95']:.3f} | {c['p99']:.3f} | {x['over100']} | {x['new_maps']}/{x['same_id_resets']} |")
    o, b = r['stage_gates']['observation'], r['stage_gates']['bow']
    lines += ['', '## 两阶段门槛', '',
        f"观测开关：OFF/ON两轮P95均值{o['off_p95_mean_ms']:.3f}/{o['on_p95_mean_ms']:.3f} ms，增加{o['p95_increase_ms']:.3f} ms，允许max(2 ms, OFF的5%)={o['p95_allowed_increase_ms']:.3f} ms；两组fresh均值差{o['absolute_fresh_difference']*100:.3f}个百分点，允许max(2个百分点, OFF重复范围)={o['allowed_fresh_difference']*100:.3f}个百分点。门槛通过，后续观测ON；这不证明零观测扰动。", '',
        f"BoW开关：OFF/ON fresh均值{b['off_fresh_mean']*100:.3f}%/{b['on_fresh_mean']*100:.3f}%，允许下降{b['allowed_fresh_decline']*100:.3f}个百分点；P95均值{b['off_p95_mean_ms']:.3f}/{b['on_p95_mean_ms']:.3f} ms。ON无输入/完成失败，且未触发ON>100而OFF≤100条件，门槛通过；后续BoW ON。P95确实增加约{b['on_p95_mean_ms']-b['off_p95_mean_ms']:.3f} ms，不能报告成无成本或大型跟踪改善。", '',
        '## 参数矩阵', '',
        '| 配置（特征/层） | 非fresh均值〔两轮范围〕 | 相对P0减少 | P95两轮范围 ms | 新图均值 | 同ID重置均值 | 严格30%门 |',
        '|---|---:|---:|---:|---:|---:|---|']
    for k, x in r['matrix'].items():
        a = x['aggregates']; nf, p95 = a['nonfresh'], a['p95_ms']; cfg = x['config']
        lines.append(f"| {k}（{cfg['features']}/{cfg['levels']}） | {nf['mean']:.1f}〔{nf['min']}–{nf['max']}〕 | {x['nonfresh_reduction_vs_P0_mean']*100:.2f}% | {p95['min']:.3f}–{p95['max']:.3f} | {a['new_maps']['mean']:.1f} | {a['same_id_resets']['mean']:.1f} | {'参考' if k=='P0' else ('通过' if all(x['quality_each']) else '未过')} |")
    threshold = r['matrix']['P3']['quality_threshold_nonfresh']
    ref, candidate = r['matrix']['P0']['aggregates'], r['matrix']['P3']['aggregates']
    lines += ['', f"P0非fresh两轮均值{ref['nonfresh']['mean']:.0f}，因此冻结质量门要求候选**每轮**≤0.70×P0均值={threshold:.1f}。P1/P2/P3均未通过；全部矩阵轮次输入完整、P95<100 ms。P3的{candidate['nonfresh']['values']}均高于该门，不能称完成30%减少目标，也不能把未过门等同没有改善：其均值{candidate['nonfresh']['mean']:.0f}比P0少{r['matrix']['P3']['nonfresh_reduction_vs_P0_mean']*100:.2f}%，新图由平均{ref['new_maps']['mean']:.0f}降到{candidate['new_maps']['mean']:.0f}，同ID重置由{ref['same_id_resets']['mean']:.0f}降到{candidate['same_id_resets']['mean']:.0f}。两轮重复不足以证明稳定因果增益，不能仅按最好一轮筛选。", '',
        '| 配置 | 总失跟起始：两轮 | 新图：两轮 | 同ID重置：两轮 | 未触发新图或同ID重置的恢复：两轮 |',
        '|---|---:|---:|---:|---:|']
    for k, x in r['matrix'].items():
        a = x['aggregates']
        def pair(field): return ' / '.join(str(v) for v in a[field]['values'])
        lines.append(f"| {k} | {pair('loss_onsets')} | {pair('new_maps')} | {pair('same_id_resets')} | {pair('transient_recoveries')} |")
    lines += ['', '每轮另有1段到EOF仍未恢复。P3总起始为69/82，更多于P0的61/69；本批新图和同ID重置更少，未触发这两类事件的恢复更多。不能泛称“失跟次数减少”。', '',
        '恢复类别只描述返回fresh前未触发新图或同ID重置，不保证沿用原坐标系。P3两轮日志分别有2/3组Merge detected及Merge finished，P0两轮均为0；合图可能改变坐标，现有epoch边界不保证排除合图影响，因此不拼接为连续轨迹。', '',
        '| 配置 | 时间加权非fresh秒：均值〔范围〕 | 最长连续fresh帧：两轮 | 回调均值：两轮均值 ms |',
        '|---|---:|---:|---:|']
    for k, x in r['matrix'].items():
        a = x['aggregates']; w = a['weighted_nonfresh_s']; v = a['longest_fresh_frames']['values']
        lines.append(f"| {k} | {w['mean']:.3f}〔{w['min']:.3f}–{w['max']:.3f}〕 | {v[0]} / {v[1]} | {a['callback_mean_ms']['mean']:.3f} |")
    lines += ['', '时间加权以每帧状态乘到下一原header的间隔；最后一帧右删失，不虚构EOF后的持续时间。最长fresh只按原ordinal连续段，不跨失跟/重置拼接。新图是日志去掉初始Atlas创建的次数，同ID重置同时核对LOST→空图再初始化和后台reset完成日志，不能只数map_id变化。', '',
        '## 资源与适用范围', '', '| 配置 | RSS采样峰值范围 MiB | 温度采样范围 °C | 频率采样范围 MHz | major fault增量两轮 |', '|---|---:|---:|---:|---:|']
    for k in ['P0', 'P1', 'P2', 'P3']:
        rows = [x for x in r['rounds'] if x['stage'] == 'matrix' and x['config'] == k]
        mem = [x['memory'] for x in rows]
        lines.append(f"| {k} | {min(v['max_rss_mib'] for v in mem):.2f}–{max(v['max_rss_mib'] for v in mem):.2f} | {min(v['temperature_c']['min'] for v in mem):.2f}–{max(v['temperature_c']['max'] for v in mem):.2f} | {min(v['frequency_mhz']['min'] for v in mem):.0f}–{max(v['frequency_mhz']['max'] for v in mem):.0f} | {mem[0]['majflt_delta']:.0f} / {mem[1]['majflt_delta']:.0f} |")
    lines += ['', '表中资源覆盖约1 Hz sampler生命周期，包含启停，不是绝对峰值或逐帧调度；COMPARISON.json另列回调包络内资源。VmSwap、系统换页/OOM和进程fault分别保存，整机计数不能归因于SLAM。完整状态、PSS、时间权重、温频、每轮身份与SHA见round_results.csv/COMPARISON.json。', '',
        '筛选结论：保留P0作为冻结参考；P3是有改善迹象但未达到预定目标的候选，P1收益很小，P2本批非fresh更多。P3增加计算成本且P99/>100ms仍需保留，P95通过10 Hz门不代表零慢帧，也不替代原项目90 ms资格。没有GT，无法给出ATE/RPE、米制精度或错误重定位被排除的完整资格；本文件只给数值筛选，不宣布新的正式赢家。', '',
        '复算：`python3 -B artifacts/rgbd_stability_trials_20261102/analysis/compare_results.py`。只读原档，仅覆盖本目录四项派生输出。']
    (OUT / 'NUMERICAL_COMPARISON.md').write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
