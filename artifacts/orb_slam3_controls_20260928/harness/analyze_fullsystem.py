#!/usr/bin/env python3
import json
from pathlib import Path
import numpy as np
from analyze import summarize,stats
from run import ROOT,sha

def main():
    results=[]
    for p in sorted((ROOT/'runs/fullsystem_formal/tum_large').glob('*/run.json')):
        r=summarize(p.parent);s=(p.parent/'console.log').read_text(errors='replace')
        r['loop_closure']=True
        r['loop_events']={key:s.count(text) for key,text in [('detected','*Loop detected'),('merge_detected','*Merge detected'),('gba_start_log_messages','Starting Global Bundle Adjustment'),('gba_finished_log_messages','Global Bundle Adjustment finished')]}
        r['gba_log_visibility']='System sets VERBOSITY_QUIET; missing NORMAL-level GBA messages do not imply no GBA activity.'
        r['all_input_rows_processed']=r['processed']==r['expected_frames'] and r['indices_contiguous'] and r['finite'] and r['timestamps_increasing']
        if r['exit_code']==7:r['diagnostic_exit_meaning']='60 s shutdown wait timed out; LocalMapping / LoopClosing / running-GBA are checked jointly. Per-thread terminal states were not recorded.'
        r['loop_log']=[z for z in s.splitlines() if any(t in z for t in ['Loop detected','Merge detected','Global Bundle Adjustment','Map updated','Shutdown','Waiting'])]
        meta=json.loads(p.read_text());assert all(sha(f)==h for f,h in meta['hashes'].items())
        results.append(r)
    cutoff=max(r['first_ok_index'] for r in results)
    end=min(r['non_ok_ranges'][0][0] if r['non_ok_ranges'] else r['expected_frames'] for r in results)
    for r in results:
        x=np.atleast_1d(np.genfromtxt(Path(r['run'])/'frames.csv',names=True,delimiter=','))
        r['common_clean_prefix']={'start':cutoff,'end_exclusive':end,'track_ms':stats(x['track_ms'][(x['index']>=cutoff)&(x['index']<end)])}
    frozen=json.loads((ROOT/'frozen.json').read_text());assert all(sha(p)==h for p,h in frozen.items())
    (ROOT/'analysis_fullsystem.json').write_text(json.dumps({'scope':'RGB-D loop-on, one pass; frozen original core libraries, shutdown waits for GBA','frozen_main_files_pass':True,'runs':results},indent=2)+'\n')
    print(json.dumps([{**{k:r[k] for k in ['mode','processed','complete','non_ok_after_initialization','map_ids','loop_events']},'timing':r['after_initialization']['track_ms'],'accuracy':r.get('accuracy')} for r in results],indent=2))
if __name__=='__main__':main()
