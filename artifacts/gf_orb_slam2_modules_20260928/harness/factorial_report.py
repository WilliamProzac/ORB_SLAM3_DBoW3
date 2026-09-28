#!/usr/bin/env python3
"""Report the entire factorial, retaining failures and activation evidence."""
import csv, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
MODES=['baseline','f','h','g','fh','fg','hg','fhg']

def main():
    allrows=[]; comparison={}
    for tag in ['euroc_modules','tum_modules']:
        matrix=ROOT/'runs'/tag
        if not (matrix/'analysis.json').exists():continue
        data=json.loads((matrix/'analysis.json').read_text()); groups=data['groups']; base=groups['baseline']
        lines=['# '+tag,'','F=Good Feature Matching; H=legacy MIH+OTS with common synchronization; G=fixed-cardinality Good Graph.','','| mode | complete/total | tracking pass | API mean ms | API P95 ms | API P99 ms | tail25 P95 ms | ATE m* | F active / skipped | G selected | OTS updates |','|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
        decisions={}
        for mode in MODES:
            g=groups[mode]; rr=[r for r in data['runs'] if r['mode']==mode]; ex=g['module_execution_totals']
            row={'dataset':rr[0]['dataset'],'mode':mode,'runs':len(rr),'complete_runs':sum(r['complete'] for r in rr),'tracking_pass_runs':g['tracking_pass_runs'],'api_mean_ms':g['median_run_track_mean'],'api_p95_ms':g['median_run_track_p95'],'api_p99_ms':g['median_run_track_p99'],'tail25_p95_ms':g['median_run_tail_p95'],'ate_m':g.get('median_valid_ate_rmse_m'),'f_active':ex['gfm_active_frames'],'f_skip':ex['gfm_skip_local_frames'],'g_selected':ex['good_graph_completed'],'ots_updates':g['ots_update_count']}
            row['p95_change_percent']=100*(row['api_p95_ms']/base['median_run_track_p95']-1)
            accuracy_checks={}
            for key,tolerance in [('median_valid_ate_rmse_m',.005),('median_valid_rpe_translation_rmse_m',.005),('median_valid_rpe_rotation_rmse_deg',.1)]:
                accuracy_checks[key]=key in g and key in base and g[key]<=(base[key]*1.05 if base[key]>=tolerance else base[key]+tolerance)
            activation=('f' not in mode or ex['gfm_active_frames']>0) and ('h' not in mode or g['ots_update_count']>0) and ('g' not in mode or ex['good_graph_completed']>0)
            checks={'all_repeats_complete_and_loss_free':len(rr)==3 and g['all_runs_tracking_pass'],'baseline_all_loss_free':base['all_runs_tracking_pass'],'p95_gain_at_least_5pct':row['api_p95_ms']<=.95*base['median_run_track_p95'],'p99_regression_at_most_5pct':row['api_p99_ms']<=1.05*base['median_run_track_p99'],'accuracy':all(accuracy_checks.values()),'modules_exercised':activation}
            decisions[mode]={'checks':checks,'accuracy_checks':accuracy_checks,'relative_host_gate_pass':mode!='baseline' and all(checks.values()),'p95_change_percent':row['p95_change_percent']}
            allrows.append(row)
            ate='—' if row['ate_m'] is None else f"{row['ate_m']:.4f}"
            lines.append(f"| {mode} | {row['complete_runs']}/{len(rr)} | {g['tracking_pass_runs']}/{len(rr)} | {row['api_mean_ms']:.3f} | {row['api_p95_ms']:.3f} | {row['api_p99_ms']:.3f} | {row['tail25_p95_ms']:.3f} | {ate} | {row['f_active']}/{row['f_skip']} | {row['g_selected']} | {row['ots_updates']} |")
        lines+=['','* ATE only from complete loss-free repeats. Timing retains failed repeats and cannot prove gains for failed groups. TUM GT covers only about 24% of frames; relative gates are not full-trajectory accuracy acceptance.','']
        (matrix/'tables.md').write_text('\n'.join(lines))
        comparison[tag]=decisions
        fig,ax=plt.subplots(1,2,figsize=(12,4.5))
        for panel,(section,title) in enumerate([('after_initialization','After initialization'),('tail25','Fixed last 25%')]):
            values=[]; lo=[]; hi=[]
            for mode in MODES:
                vals=[r[section]['track_ms']['p95'] for r in data['runs'] if r['mode']==mode and r.get(section,{}).get('track_ms')]
                v=np.median(vals);values.append(v);lo.append(v-min(vals));hi.append(max(vals)-v)
            ax[panel].bar(MODES,values,yerr=[lo,hi],capsize=3,color=['#687785']+['#4b8bb8']*6+['#bd6d42'])
            ax[panel].set_title(title);ax[panel].set_ylabel('Full Track API P95 (ms)');ax[panel].grid(axis='y',alpha=.2);ax[panel].set_axisbelow(True)
        fig.suptitle(tag+' — median of repeats; error bars: min/max, failures retained')
        fig.tight_layout();fig.savefig(matrix/'comparison.png',dpi=170);fig.savefig(matrix/'comparison.pdf');plt.close(fig)
    (ROOT/'comparison_summary.json').write_text(json.dumps(comparison,indent=2)+'\n')
    if allrows:
        with (ROOT/'run_summary.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(allrows[0]));w.writeheader();w.writerows(allrows)
    print(json.dumps(comparison,indent=2))
if __name__=='__main__':main()
