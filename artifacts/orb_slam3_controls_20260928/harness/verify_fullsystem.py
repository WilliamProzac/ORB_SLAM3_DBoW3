#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
from run import ROOT,sha

def main():
    p=argparse.ArgumentParser();p.add_argument('--phase',choices=['smoke','formal'],required=True);a=p.parse_args()
    source=(ROOT/'harness/replay.cc').read_text()
    expected=source.replace('while(!s.mpLocalMapper->isFinished() || !s.mpLoopCloser->isFinished())','while(!s.mpLocalMapper->isFinished() || !s.mpLoopCloser->isFinished() || s.mpLoopCloser->isRunningGBA())')
    assert (ROOT/'fullsystem/replay.cc').read_text()==expected
    manifest=json.loads((ROOT/'fullsystem/build_manifest.json').read_text())
    for v,m in manifest.items():
        assert sha(ROOT/'fullsystem'/v/'replay')==m['executable_sha256']
        assert sha(ROOT/'fullsystem/replay.cc')==m['driver_sha256']
    runs=[]
    for mode in ['official800','delivered800','delivered700']:
        p=ROOT/'runs'/f'fullsystem_{a.phase}'/'tum_large'/mode
        m=json.loads((p/'run.json').read_text());x=np.atleast_1d(np.genfromtxt(p/'frames.csv',names=True,delimiter=','))
        assert m['loop_closure'] is True
        assert 'loopClosing: 1' in (ROOT/'fullsystem'/f'{mode}.yaml').read_text()
        assert all(sha(f)==h for f,h in m['hashes'].items())
        complete=m['exit_code']==0 and len(x)==m['expected_frames'] and np.array_equal(x['index'],np.arange(len(x))) and all(np.isfinite(x[k]).all() for k in x.dtype.names) and bool(np.all(np.diff(x['timestamp'])>0))
        complete=bool(complete and f"REPRO_COMPLETE {m['expected_frames']}" in (p/'console.log').read_text(errors='replace'))
        # A failed formal test remains a valid recorded negative result.
        if a.phase=='smoke':assert complete and np.any(x['state']==2),mode
        runs.append({'mode':mode,'complete':complete,'frames':len(x),'exit_code':m['exit_code']})
    frozen=json.loads((ROOT/'frozen.json').read_text());assert all(sha(f)==h for f,h in frozen.items())
    result={'phase':a.phase,'frozen_main_unchanged':True,'same_driver_except_shutdown_GBA_wait':True,'runs':runs,'all_complete':all(r['complete'] for r in runs)}
    (ROOT/('verification_fullsystem.json' if a.phase=='formal' else 'verification_fullsystem_smoke.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
if __name__=='__main__':main()
