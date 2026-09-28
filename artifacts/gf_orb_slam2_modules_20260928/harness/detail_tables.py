#!/usr/bin/env python3
"""Export transparent per-run metrics, without inferring repeatability."""
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODES = ['baseline', 'f', 'h', 'g', 'fh', 'fg', 'hg', 'fhg']

def value(obj, *keys):
    for key in keys:
        if not isinstance(obj, dict) or key not in obj:
            return None
        obj = obj[key]
    return obj

def number(x, digits=2):
    return '—' if x is None else f'{x:.{digits}f}'

def main():
    rows = []
    text = ['# GF03 单轮详细结果', '', '所有延时单位为 ms。每种模式仅一次；表中不使用跨轮中位数。', '']
    for tag in ['euroc_modules', 'tum_modules']:
        data = json.loads((ROOT/'runs'/tag/'analysis.json').read_text())
        runs = {r['mode']: r for r in data['runs']}
        text += ['## '+tag, '', '| 模式 | 输入 | 初始化后 non-OK | ATE m | 1s RPE m | 1s RPE ° | 真值匹配/输出 |', '|---|---:|---:|---:|---:|---:|---:|']
        for mode in MODES:
            r = runs[mode]; a = r.get('accuracy', {})
            text.append(f"| {mode} | {r['processed']}/{r['expected_frames']} | {r['non_ok_after_initialization']} | {number(a.get('ate_rmse_m'),4)} | {number(a.get('rpe_translation_rmse_m'),4)} | {number(a.get('rpe_rotation_rmse_deg'),4)} | {a.get('associated_poses','—')}/{a.get('estimated_poses','—')} |")
        text += ['', '误差数字保留失败轨迹的可计算部分；只有完整且无丢跟踪的运行可用于质量筛选。EuRoC 使用一个全局 Sim(3)，TUM 使用 SE(3)；TUM 真值覆盖有限。', '', '| 模式 | 全段 API P95 | 初始化后 API mean/P50/P95/P99/max | 固定后25% P95/P99 | pose-ready mean/P95 | post mean/P95 |', '|---|---:|---|---|---|---|']
        for mode in MODES:
            r = runs[mode]; s = r['after_initialization']; t = r['tail25']
            def fmt(section, metric, keys):
                return '/'.join(number(value(section,metric,k)) for k in keys)
            text.append(f"| {mode} | {number(value(r,'all','track_ms','p95'))} | {fmt(s,'track_ms',['mean','p50','p95','p99','max'])} | {fmt(t,'track_ms',['p95','p99'])} | {fmt(s,'pose_ready_ms',['mean','p95'])} | {fmt(s,'post_ms',['mean','p95'])} |")
        text += ['', '| 模式 | F 主动/跳过帧 | H 非空帧 | OTS 完成/调用 | G 返回/触发 | 局部阶段均值 | 局部点均值 | 最终 KF/MP |', '|---|---:|---:|---:|---:|---:|---:|---:|']
        for mode in MODES:
            r = runs[mode]; e = r['module_execution']; o = r['ots']; s = r['after_initialization']
            text.append(f"| {mode} | {e['gfm_active_frames']}/{e['gfm_skip_local_frames']} | {r['hash_query_nonzero_frames']} | {o['completed']}/{o['updates']} | {e['good_graph_completed']}/{e['good_graph_triggered']} | {number(value(s,'local_ms','mean'))} | {number(value(s,'local_mp','mean'),0)} | {r['final_kfs']}/{r['final_mps']} |")
        text += ['', '局部阶段/点数按全部初始化后帧平均，包含 F 跳过局部跟踪的帧；不能据此认为每次实际构图都同等缩小。按实际分支条件计算的分布另见 analysis.json 的 conditional_diagnostics。G 返回表示选图函数返回，不是 BA 完成率。', '', '| 模式 | BA 计时调用 | BA wall mean/P95/max | 自由 KF mean/max | 固定 KF mean/max | BA MP mean/max | G 插图/Schur 均值 |', '|---|---:|---|---|---|---|---|']
        for mode in MODES:
            r = runs[mode]; b = r.get('local_ba', {}); g = r.get('good_graph_times', {})
            text.append(f"| {mode} | {b.get('timed_calls','—')} | {'/'.join(number(value(b,'wall_ms',k)) for k in ['mean','p95','max'])} | {'/'.join(number(value(b,'free_kf',k),1) for k in ['mean','max'])} | {'/'.join(number(value(b,'fixed_kf',k),1) for k in ['mean','max'])} | {'/'.join(number(value(b,'points',k),0) for k in ['mean','max'])} | {number(value(g,'insert','mean'))}/{number(value(g,'schur','mean'))} |")
        text += ['', 'BA wall 是现有 LocalBundleAdjustment 调用计时，包含选图及可能的中断；没有新增 BA 完成/中断结果埋点，不声称测得完成率。选图阶段均值按所有 BA 计时调用统计，包含未触发选择的调用。', '', '| 模式 | 平均 CPU 核数 | RSS 峰值 MiB | 进程 Swap 峰值 MiB | 输入滞后 P95/max | 超标称帧周期调用数 |', '|---|---:|---:|---:|---|---:|']
        for mode in MODES:
            r = runs[mode]; res = r['resources']; s = r['after_initialization']; late = s.get('input_lateness_ms', {})
            text.append(f"| {mode} | {number(res.get('mean_process_cores'))} | {number(res['rss_peak_kib']/1024,1)} | {number(res['swap_peak_kib']/1024,1)} | {number(late.get('p95'))}/{number(late.get('max'))} | {r['track_calls_over_nominal_period_after_init']} |")
            row = dict(dataset=r['dataset'], mode=mode, processed=r['processed'], expected=r['expected_frames'], non_ok=r['non_ok_after_initialization'], tracking_pass=r['tracking_pass'])
            for label, section in [('all',r['all']),('initialized',s),('tail25',r['tail25'])]:
                for metric in ['track_ms','pose_ready_ms','post_ms','local_ms','local_mp','gfm_matrix_ms','input_lateness_ms']:
                    for stat in ['mean','p50','p95','p99','max']:
                        row[f'{label}_{metric}_{stat}'] = value(section,metric,stat)
            for key in ['ate_rmse_m','rpe_translation_rmse_m','rpe_rotation_rmse_deg','associated_poses','estimated_poses','truth_max_gap_s']:
                row[key] = value(r,'accuracy',key)
            row.update(res)
            row.update(r['module_execution'])
            row.update(ots_calls=r['ots']['updates'],ots_completed=r['ots']['completed'],hash_nonempty_frames=r['hash_query_nonzero_frames'],final_kfs=r['final_kfs'],final_mps=r['final_mps'])
            rows.append(row)
        text += ['', 'CPU/RSS/Swap 每 2 秒采样；CPU 为进程累计 CPU 秒/墙钟秒，不是单核百分比。输入滞后包含读图及前序处理积压；API 耗时不包含 PNG 解码。FG/HG/FHG 缺少输入滞后列，破折号表示未测，见 diagnostic_gaps.json。所有帧依次处理，零主动跳帧不代表严格跟上实时节拍。', '']
    (ROOT/'DETAILS.md').write_text('\n'.join(text).rstrip()+'\n')
    with (ROOT/'run_details.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    print('DETAILS_WRITTEN',len(rows),'runs')

if __name__ == '__main__':
    main()
