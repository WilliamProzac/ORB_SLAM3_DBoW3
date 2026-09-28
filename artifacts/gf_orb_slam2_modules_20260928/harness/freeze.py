#!/usr/bin/env python3
import datetime, hashlib, json, platform, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MODES=['baseline','f','h','g','fh','fg','hg','fhg']
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def main():
    assert not (ROOT/'source_manifest_initial.json').exists(), 'Refuse overwriting a frozen manifest'
    source=ROOT/'upstream'
    assert subprocess.check_output(['git','-C',str(source),'status','--porcelain'],text=True).strip()==''
    files=list((source/'src').glob('*.cc'))+list((source/'include').glob('*'))+list((ROOT/'configs').glob('*.yaml'))
    files += [ROOT/'harness'/n for n in ['CMakeLists.txt','compat.h','replay.cc','run_matrix.py']]
    files += [source/'Thirdparty/SLAM++'/n for n in ['CMakeLists.txt','src/good_graph_testbed/BAOptimizer.cpp','include/slam/NonlinearSolver_GoodGraph.h']]
    manifest={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_commit':subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip(),'source_clean':True,'official_upstream':'2a60498cb889965bd5b4ab05c918ab28b433e882','parent':'33e506481bf4daf8bf37b1f4377950c28a8c54a7','platform':platform.platform(),'compiler':subprocess.check_output(['g++','--version'],text=True).splitlines()[0],'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files if p.is_file()}}
    (ROOT/'source_manifest_initial.json').write_text(json.dumps(manifest,indent=2)+'\n')
    bins=[ROOT/'build'/name for mode in MODES for name in ['replay_'+mode,'libgf_'+mode+'.so']]
    bins += [source/'Thirdparty/g2o/lib/libg2o.so',source/'Thirdparty/DBoW2/lib/libDBoW2.so']+list((source/'Thirdparty/SLAM++/lib').glob('*.so'))
    (ROOT/'build_manifest.json').write_text(json.dumps({str(p.relative_to(ROOT)):{'sha256':sha(p),'bytes':p.stat().st_size} for p in bins},indent=2)+'\n')
    linked=''
    for mode in MODES:
        result=subprocess.check_output(['ldd',str(ROOT/'build'/('replay_'+mode))],text=True)
        assert 'not found' not in result
        linked+=mode+'\n'+result+'\n'
    (ROOT/'logs/linked_libraries.log').write_text(linked)
    previous=json.loads((ROOT/'previous_builds.json').read_text())
    assert all(sha(Path(p))==h for p,h in previous.items()),'Prior build changed'
    (ROOT/'upstream.patch').write_text(subprocess.check_output(['git','-C',str(source),'diff','2a60498cb889965bd5b4ab05c918ab28b433e882','HEAD'],text=True))
    (ROOT/'module_delta.patch').write_text(subprocess.check_output(['git','-C',str(source),'diff','33e5064','HEAD'],text=True))
    print('FROZEN',manifest['source_commit'],len(bins),'binaries/dependencies; prior builds preserved')
if __name__=='__main__':main()
