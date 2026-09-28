#!/usr/bin/env python3
import csv,json
from run import ROOT

def main():
    rows=[]
    for name,loop in [('analysis.json',False),('analysis_fullsystem.json',True)]:
        for r in json.loads((ROOT/name).read_text())['runs']:
            t=r['after_initialization']['track_ms'];a=r.get('accuracy',{})
            rows.append({'dataset':r['dataset'],'mode':r['mode'],'loop_closing':loop,'processed':r['processed'],'expected':r['expected_frames'],'non_ok':r['non_ok_after_initialization'],'maps':r['max_maps'],'api_mean_ms':t['mean'],'api_p95_ms':t['p95'],'api_p99_ms':t['p99'],'api_max_ms':t['max'],'tail25_p95_ms':r['tail25']['track_ms']['p95'],'ate_m':a.get('ate_rmse_m'),'rpe_translation_m':a.get('rpe_translation_rmse_m'),'rpe_rotation_deg':a.get('rpe_rotation_rmse_deg'),'exit_code':r['exit_code'],'complete_including_exit':r['complete'],'tracking_and_exit_pass':r['tracking_pass']})
    with (ROOT/'comparison_all.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys(),lineterminator='\n');w.writeheader();w.writerows(rows)
    print('ALL_RESULTS',len(rows),'runs',sum(r['processed'] for r in rows),'frames')
if __name__=='__main__':main()
