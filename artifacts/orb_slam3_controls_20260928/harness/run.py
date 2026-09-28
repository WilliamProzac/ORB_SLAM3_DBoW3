#!/usr/bin/env python3
"""One pass per configuration; isolated outputs and immutable formal inputs."""
import argparse, hashlib, json, os, shutil, signal, subprocess, time
from pathlib import Path

ROOT=Path('/home/ywl/Project/RDK_X5_orb_slam/orb_slam3_controls_20260928')
GF=Path('/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_modules_20260928')
MODES=['official800','delivered800','delivered700','gf_bridge']
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def spec(dataset,mode):
    prefix='tum' if dataset=='tum_large' else 'euroc'
    if mode=='gf_bridge':
        return GF/'build/replay_baseline', GF/'configs'/f'{prefix}800.yaml', Path('/home/ywl/Project/ORB_SLAM3_DBoW3/Vocabulary/ORBvoc.txt')
    variant='official' if mode.startswith('official') else 'delivered'
    return ROOT/f'build-{variant}/replay', ROOT/'configs'/f'{prefix}_{mode}.yaml', ROOT/'vocabulary/ORBvoc.txt'
def freeze():
    paths=[ROOT/'harness/replay.cc',ROOT/'harness/CMakeLists.txt',ROOT/'harness/run.py']
    paths+=list((ROOT/'configs').glob('*.yaml'))
    for variant in ['official','delivered']:
        paths.extend(p for p in (ROOT/variant).rglob('*') if p.is_file())
        paths.extend((ROOT/f'build-{variant}').glob('*.so'))
        paths.extend((ROOT/f'build-{variant}/lib').glob('*.so'))
        paths.extend([ROOT/f'build-{variant}/replay',ROOT/f'build-{variant}/compile_commands.json'])
    paths.extend([GF/'build/replay_baseline',GF/'build/libgf_baseline.so',GF/'upstream/Thirdparty/DBoW2/lib/libDBoW2.so',GF/'upstream/Thirdparty/g2o/lib/libg2o.so'])
    for dataset in ['V1_01_easy','tum_large']:
        paths.append(GF/'inputs'/f'{dataset}.txt')
        for mode in MODES:paths.extend(spec(dataset,mode))
    return {str(p):sha(p) for p in sorted(set(paths))}
def main():
    a=argparse.ArgumentParser();a.add_argument('--phase',choices=['smoke','formal'],required=True)
    a.add_argument('--modes',nargs='+',choices=MODES,default=MODES)
    a.add_argument('--datasets',nargs='+',default=['V1_01_easy','tum_large']);args=a.parse_args()
    if args.phase=='formal':
        path=ROOT/'frozen.json'
        assert not path.exists(),'Formal freeze exists; never overwrite formal runs'
        hashes=freeze();path.write_text(json.dumps(hashes,indent=2)+'\n')
    else:hashes={}
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    for dataset in args.datasets:
        for mode in args.modes:
            if hashes:assert all(sha(k)==v for k,v in hashes.items()),'Frozen inputs changed'
            assert shutil.disk_usage(ROOT).free>4*1024**3
            limit=150 if args.phase=='smoke' else 0
            manifest=GF/'inputs'/f'{dataset}.txt'
            count=len(manifest.read_text().splitlines());count=min(count,limit) if limit else count
            exe,config,vocab=spec(dataset,mode)
            out=ROOT/'runs'/args.phase/dataset/mode;out.mkdir(parents=True,exist_ok=False)
            cmd=list(map(str,[exe,vocab,config,'rgbd' if dataset=='tum_large' else 'mono',manifest,out,limit]))
            start=time.monotonic();deadline=start+count/(30 if dataset=='tum_large' else 20)+300
            meta={'command':cmd,'mode':mode,'dataset':dataset,'phase':args.phase,'repeat':1,'expected_frames':count,'hashes':{str(p):sha(p) for p in [exe,config,vocab,manifest]},'start_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'thread_env':{k:env[k] for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']},'loop_closure':False,'rate':1,'drop_policy':'none'}
            (out/'started.json').write_text(json.dumps(meta,indent=2)+'\n')
            print('START',dataset,mode,args.phase,flush=True);samples=[];reason=None
            with (out/'console.log').open('w') as log:
                proc=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
                while proc.poll() is None:
                    try:
                        s=Path(f'/proc/{proc.pid}/status').read_text();stat=Path(f'/proc/{proc.pid}/stat').read_text().split()
                        sample={'elapsed_s':time.monotonic()-start,'cpu_ticks':int(stat[13])+int(stat[14]),'clock_ticks_per_second':os.sysconf('SC_CLK_TCK')}
                        sample.update({k:l.split(':',1)[1].strip() for l in s.splitlines() for k in ['VmRSS','VmHWM','VmSwap','Threads'] if l.startswith(k+':')})
                        samples.append(sample)
                    except FileNotFoundError:pass
                    if time.monotonic()>deadline or shutil.disk_usage(ROOT).free<2*1024**3:
                        reason='timeout' if time.monotonic()>deadline else 'disk_guard';os.killpg(proc.pid,signal.SIGTERM)
                        try:proc.wait(timeout=10)
                        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                        break
                    time.sleep(2)
            meta.update(exit_code=proc.returncode,elapsed_s=time.monotonic()-start,failure_reason=reason,end_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
            (out/'run.json').write_text(json.dumps(meta,indent=2)+'\n');(out/'resources.json').write_text(json.dumps(samples,indent=2)+'\n')
            print('END',dataset,mode,proc.returncode,round(meta['elapsed_s'],1),flush=True)
            if args.phase=='smoke' and proc.returncode:raise RuntimeError(f'Smoke failed: {out}')
    if hashes:
        assert all(sha(k)==v for k,v in hashes.items()),'Frozen inputs changed'
        (ROOT/'freeze_audit.json').write_text(json.dumps({'passed':True,'files':len(hashes)},indent=2)+'\n')
    print('COMPLETE',args.phase,flush=True)
if __name__=='__main__':main()
