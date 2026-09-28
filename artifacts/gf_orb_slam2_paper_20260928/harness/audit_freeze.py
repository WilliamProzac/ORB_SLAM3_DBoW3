#!/usr/bin/env python3
"""Final source/binary/input and result-integrity audit for the executed GF02 matrix."""
import hashlib,json,subprocess,datetime
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 paths={};runs=[]
 for tag in ['euroc_paper','tum_paper']:
  matrix=ROOT/'runs'/tag
  metas=list(matrix.glob('*/run.json'));assert len(metas)==12,tag
  for f in metas:
   m=json.loads(f.read_text())
   for p,h in m['hashes'].items():
    assert p not in paths or paths[p]==h,'mixed frozen artifacts'
    paths[p]=h
   x=np.genfromtxt(f.parent/'frames.csv',names=True,delimiter=',')
   ok=x['state']==2
   check={'run':str(f.parent),'exit_zero':m['exit_code']==0,'expected_count':len(x)==m['expected_frames'],
          'contiguous':bool(np.array_equal(x['index'],np.arange(len(x)))),
          'finite':bool(all(np.isfinite(x[k]).all() for k in x.dtype.names)),
          'timestamps_increasing':bool(np.all(np.diff(x['timestamp'])>0)),
          'pose_ready_within_api':bool(np.all(x['pose_ready_ms']<=x['track_ms']+.001)),
          'ok_pose_ready_positive':bool(np.all(x['pose_ready_ms'][ok]>0)),
          'processed':len(x),'non_ok_after_init':int(np.sum(x['state'][np.flatnonzero(ok)[0]:]!=2))}
   for k in ['exit_zero','expected_count','contiguous','finite','timestamps_increasing','pose_ready_within_api','ok_pose_ready_positive']:assert check[k],(f,k)
   if m['mode'].startswith('paper_'):
    d=json.loads((f.parent/'index_diagnostics.json').read_text())
    check['bounded_index']=d['max_bucket']<=10 and d['entries']<=81920 and d['indexed_points']<=d['entries']
    assert check['bounded_index'],f
   runs.append(check)
 frozen={p:sha(Path(p))==h for p,h in paths.items()};assert all(frozen.values())
 initial=json.loads((ROOT/'source_manifest_initial.json').read_text())
 source={p:sha(ROOT/p)==h for p,h in initial['source_sha256'].items() if p.startswith(('upstream/src/','upstream/include/','configs/')) or p in ['harness/CMakeLists.txt','harness/compat.h','harness/replay.cc','harness/run_matrix.py','harness/test_hash_index.cc','harness/test_hash_stubs.cc','harness/run_index_sanitizers.sh']}
 assert all(source.values())
 binaries={p:sha(ROOT/p)==v['sha256'] for p,v in json.loads((ROOT/'build_manifest.json').read_text()).items()};assert all(binaries.values())
 result={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'frozen_runtime_files':frozen,'compiled_source_and_fixtures':source,'built_artifacts':binaries,'runs':runs,'total_processed':sum(r['processed'] for r in runs),'loss_free_runs':sum(r['non_ok_after_init']==0 for r in runs),'source_commit':subprocess.check_output(['git','-C',str(ROOT/'upstream'),'rev-parse','HEAD'],text=True).strip()}
 assert result['total_processed']==95628
 (ROOT/'freeze_audit.json').write_text(json.dumps(result,indent=2)+'\n');print('AUDIT_PASS',len(runs),'rounds',result['total_processed'],'frames; loss-free',result['loss_free_runs'])
if __name__=='__main__':main()
