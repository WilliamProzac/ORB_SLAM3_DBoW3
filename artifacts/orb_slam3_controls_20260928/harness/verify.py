#!/usr/bin/env python3
"""Check actual replay schemas, pinned algorithms and build provenance."""
import argparse,json,subprocess
from pathlib import Path
import numpy as np
from run import ROOT,MODES,sha

def main():
    a=argparse.ArgumentParser();a.add_argument('--phase',choices=['smoke','formal'],required=True);args=a.parse_args()
    evidence=Path(__file__).resolve().parents[1]
    original=json.loads((evidence/'source_manifest.json').read_text())
    source_audit={}
    for variant,info in original.items():
        different=[]
        for rel,digest in info['original_files'].items():
            if sha(ROOT/variant/rel)!=digest:different.append(rel)
        assert set(different)<= {'include/System.h','Thirdparty/g2o/config.h','src/LoopClosing.cc'},different
        # Verify the ONLY loop source edit is the loop-off guard, byte for byte.
        loop=(ROOT/variant/'src/LoopClosing.cc').read_text()
        reverted=loop.replace('mbActiveLC && CheckNewKeyFrames()', 'CheckNewKeyFrames()')
        import hashlib
        assert hashlib.sha256(reverted.encode()).hexdigest()==info['original_files']['src/LoopClosing.cc']
        source_audit[variant]={'changed_files':different,'original_sha':info['sha']}
    runs=[];expected_schema=None
    for ds in ['V1_01_easy','tum_large']:
        for mode in MODES:
            run=ROOT/'runs'/args.phase/ds/mode
            meta=json.loads((run/'run.json').read_text())
            f=np.atleast_1d(np.genfromtxt(run/'frames.csv',names=True,delimiter=','))
            assert meta['exit_code']==0 and len(f)==meta['expected_frames'],run
            assert np.array_equal(f['index'],np.arange(len(f))),run
            assert np.all(np.diff(f['timestamp'])>0) and all(np.isfinite(f[n]).all() for n in f.dtype.names),run
            assert np.any(f['state']==2),run
            text=(run/'console.log').read_text(errors='replace')
            assert f"REPRO_COMPLETE {len(f)}" in text,run
            if mode!='gf_bridge':
                if expected_schema is None:expected_schema=f.dtype.names
                assert expected_schema==f.dtype.names,run
            if mode.startswith('delivered'):
                assert '- Parallel Workers: 3' in text,run
                if ds=='tum_large':assert 'RGB-D exact optimization level: 4' in text,run
            runs.append({'dataset':ds,'mode':mode,'frames':len(f),'post_init_non_ok':int(np.sum(f['state'][np.flatnonzero(f['state']==2)[0]:]!=2)),'csv_sha256':sha(run/'frames.csv')})
    builds={}
    for variant in ['official','delivered']:
        b=ROOT/f'build-{variant}';commands=json.loads((b/'compile_commands.json').read_text())
        driver=[c for c in commands if Path(c['file']).name=='replay.cc'];assert len(driver)==1
        ldd=subprocess.check_output(['ldd',str(b/'replay')],text=True)
        assert 'not found' not in ldd
        for name in ['libg2o','libDBoW','liborb3']:
            selected=[line for line in ldd.splitlines() if name in line];assert len(selected)==1
            assert str(b) in selected[0],selected
        builds[variant]={'driver_command':driver[0],'driver_sha256':sha(driver[0]['file']),'translation_units':len(commands),'executable_sha256':sha(b/'replay'),'ldd':ldd}
    assert builds['official']['driver_sha256']==builds['delivered']['driver_sha256']
    output={'phase':args.phase,'passed':True,'source_audit':source_audit,'runs':runs,'builds':builds}
    if args.phase=='formal':
        frozen=json.loads((ROOT/'frozen.json').read_text());assert all(sha(p)==h for p,h in frozen.items())
        output['frozen_files']=len(frozen)
    (ROOT/f'verification_{args.phase}.json').write_text(json.dumps(output,indent=2)+'\n')
    print(args.phase,'PASS',len(runs),'runs',sum(r['frames'] for r in runs),'frames')
if __name__=='__main__':main()
