#!/usr/bin/env python3
"""Render final measurements without hand-transcribing numeric results."""
import argparse,json,statistics
from pathlib import Path
modes=['baseline','mih_ots','paper_ots','paper_all']
p=argparse.ArgumentParser();p.add_argument('matrices',type=Path,nargs='+');a=p.parse_args()
for matrix in a.matrices:
 d=json.loads((matrix/'analysis.json').read_text());g=d['groups']
 lines=['# '+matrix.name,'','| 模式 | 无丢跟踪 | API 平均 ms | API P95 ms | API P99 ms | 后25% P95 ms | 位姿就绪平均 ms | 位姿就绪P95 ms | ATE m | 1s RPE m / ° |','|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
 for m in modes:
  q=g[m];v=[q[k] for k in ['median_run_track_mean','median_run_track_p95','median_run_track_p99','median_run_tail_p95','median_run_pose_ready_mean','median_run_pose_ready_p95']]
  acc='—' if not q['valid_accuracy_runs'] else f"{q['median_valid_ate_rmse_m']:.5f}"
  rpe='—' if not q['valid_accuracy_runs'] else f"{q['median_valid_rpe_translation_rmse_m']:.5f} / {q['median_valid_rpe_rotation_rmse_deg']:.4f}"
  label=m+('†' if q['timing_includes_failed_runs'] else '')
  lines.append(f"| {label} | {q['tracking_pass_runs']}/{q['runs']} | "+' | '.join(f'{x:.3f}' for x in v)+f' | {acc} | {rpe} |')
 lines+=['','| 模式 | 局部MP平均 | Local Map ms | 更新+搜索 ms | 优化统计 ms | 后处理 ms | RSS峰值 MiB | 平均CPU核 | 结束时桶条目 | 最大桶 |','|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
 for m in modes:
  rr=[r for r in d['runs'] if r['mode']==m]
  v=[statistics.median(r['after_initialization'][k]['mean'] for r in rr) for k in ['local_mp','local_ms','match_ms','opt_ms','post_ms']]
  v += [statistics.median(r['resources']['rss_peak_kib']/1024 for r in rr),statistics.median(r['resources']['mean_process_cores'] for r in rr)]
  ds=[r['index_diagnostics'] for r in rr if 'index_diagnostics' in r]
  index='— | —' if not ds else f"{statistics.median(z['entries'] for z in ds):.0f} | {max(z['max_bucket'] for z in ds)}"
  lines.append('| '+m+' | '+' | '.join(f'{x:.3f}' for x in v)+' | '+index+' |')
 lines+=['','† marks a configuration with tracking failures: its raw timing includes LOST frames and is not evidence of acceleration. All timing and quality numbers above are medians of per-run aggregates. Failed runs remain in timing; accuracy uses only eligible runs. RSS is the median of per-run sampled peaks; maximum bucket is the maximum of end-of-run diagnostics.']
 (matrix/'tables.md').write_text('\n'.join(lines)+'\n')
 print(matrix/'tables.md')
