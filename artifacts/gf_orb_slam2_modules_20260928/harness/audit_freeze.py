#!/usr/bin/env python3
import hashlib,json
from pathlib import Path
import numpy as np
from freeze import sha, ROOT

def main():
    paths={};runs=[]
    for tag in ['euroc_modules','tum_modules']:
        matrix=ROOT/'runs'/tag
        metas=list(matrix.glob('*/run.json'));assert len(metas)==8,(tag,len(metas))
        assert {json.loads(f.read_text())['mode'] for f in metas}=={'baseline','f','h','g','fh','fg','hg','fhg'}
        for f in metas:
            m=json.loads(f.read_text())
            for p,h in m['hashes'].items():
                assert p not in paths or paths[p]==h,'Mixed frozen artifacts'
                paths[p]=h
            x=np.atleast_1d(np.genfromtxt(f.parent/'frames.csv',names=True,delimiter=','))
            ok=x['state']==2
            check={'run':str(f.parent),'exit_zero':m['exit_code']==0,'processed':len(x),'expected_count':len(x)==m['expected_frames'],'contiguous':bool(np.array_equal(x['index'],np.arange(len(x)))),'finite':bool(all(np.isfinite(x[k]).all() for k in x.dtype.names)),'timestamps_increasing':bool(np.all(np.diff(x['timestamp'])>0)),'pose_ready_within_api':bool(np.all(x['pose_ready_ms']<=x['track_ms']+.001)),'non_ok_after_init':int(np.sum(~ok[np.flatnonzero(ok)[0]:])) if np.any(ok) else len(x)}
            assert check['contiguous'] and check['timestamps_increasing']
            check['input_lateness_measured']='input_lateness_ms' in x.dtype.names
            runs.append(check)
    frozen={p:sha(Path(p))==h for p,h in paths.items()};assert all(frozen.values())
    initial=json.loads((ROOT/'source_manifest_initial.json').read_text())
    source={p:sha(ROOT/p)==h for p,h in initial['source_sha256'].items()};assert all(source.values())
    binaries={p:sha(ROOT/p)==v['sha256'] for p,v in json.loads((ROOT/'build_manifest.json').read_text()).items()};assert all(binaries.values())
    previous={p:sha(Path(p))==h for p,h in json.loads((ROOT/'previous_builds.json').read_text()).items()};assert all(previous.values())
    result={'frozen_runtime_files':frozen,'source':source,'binaries':binaries,'previous_builds':previous,'runs':runs,'total_processed':sum(r['processed'] for r in runs),'loss_free_complete_runs':sum(r['non_ok_after_init']==0 and r['expected_count'] and r['exit_zero'] for r in runs),'scope':'integrity audit; runtime failures retained as failures'}
    result['all_record_integrity_checks_pass']=all(all(r[k] for k in ['exit_zero','expected_count','contiguous','finite','timestamps_increasing','pose_ready_within_api']) for r in runs)
    result['input_lateness_available_all_runs']=all(r['input_lateness_measured'] for r in runs)
    result['driver_provenance_limitation']='FG/HG/FHG retain the historical driver without input_lateness_ms. Immutable file hashes do not prove every binary was rebuilt from the latest driver source. See diagnostic_gaps.json and preserved source snapshots.'
    (ROOT/'freeze_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print('FREEZE_AUDIT_PASS',len(runs),'runs',result['total_processed'],'frames;',result['loss_free_complete_runs'],'loss-free complete')
if __name__=='__main__':main()
