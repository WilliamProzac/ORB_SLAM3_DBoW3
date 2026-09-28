#!/usr/bin/env python3
"""Finite, sequential, interleaved upstream runs; retain failed runs."""
import argparse, hashlib, json, os, signal, subprocess, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset',choices=['V1_01_easy','tum_large'],required=True)
    p.add_argument('--modes',nargs='+',default=['baseline','mih32','mih_ots']);p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--max-frames',type=int,default=0);p.add_argument('--tag',required=True);a=p.parse_args()
    rgbd=a.dataset=='tum_large'; settings=ROOT/'configs'/('tum800.yaml' if rgbd else 'euroc800.yaml')
    manifest=ROOT/'inputs'/(a.dataset+'.txt');n=len(manifest.read_text().splitlines());n=min(n,a.max_frames) if a.max_frames else n
    frozen=[settings,manifest,Path(__file__),ROOT/'harness/replay.cc',Path('/home/ywl/Project/ORB_SLAM3_DBoW3/Vocabulary/ORBvoc.txt')]
    for mode in a.modes:frozen.extend([ROOT/'build'/('replay_'+mode),ROOT/'build'/('libgf_'+mode+'.so')])
    frozen.extend([ROOT/'upstream/Thirdparty/g2o/lib/libg2o.so',ROOT/'upstream/Thirdparty/DBoW2/lib/libDBoW2.so'])
    hashes={str(x):sha(x) for x in frozen}
    dest=ROOT/'runs'/a.tag;dest.mkdir(parents=True,exist_ok=False)
    (dest/'frozen.json').write_text(json.dumps(hashes,indent=2))
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    for r in range(a.repeats):
        order=a.modes[r%len(a.modes):]+a.modes[:r%len(a.modes)]
        for mode in order:
            assert all(sha(Path(k))==v for k,v in hashes.items()),'Frozen input changed'
            out=dest/(str(r+1)+'_'+mode);out.mkdir()
            cmd=[str(ROOT/'build'/('replay_'+mode)), '/home/ywl/Project/ORB_SLAM3_DBoW3/Vocabulary/ORBvoc.txt',str(settings),'rgbd' if rgbd else 'mono',str(manifest),str(out),str(a.max_frames)]
            meta={'command':cmd,'expected_frames':n,'start_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'mode':mode,'dataset':a.dataset,'repeat':r+1,'hashes':hashes,'loop_closure':False,'ground_truth_prior':False,'playback_rate':1,'drop_policy':'none; wait/overrun'}
            print('START',out.name,flush=True);start=time.monotonic();deadline=start+n/(30 if rgbd else 20)+300
            samples=[];reason=None
            with open(out/'console.log','w') as log:
                proc=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
                while proc.poll() is None:
                    try:
                        status=Path('/proc')/str(proc.pid)/'status';s=status.read_text()
                        procstat=(Path('/proc')/str(proc.pid)/'stat').read_text().split();
                        samples.append({'cpu_ticks':int(procstat[13])+int(procstat[14]),'clock_ticks_per_second':os.sysconf('SC_CLK_TCK'),'elapsed_s':time.monotonic()-start,**{k:l.split(':',1)[1].strip() for l in s.splitlines() for k in ['VmRSS','VmHWM','VmSwap','Threads'] if l.startswith(k+':')}})
                    except FileNotFoundError:pass
                    if time.monotonic()>deadline:
                        reason='timeout';os.killpg(proc.pid,signal.SIGTERM)
                        try:proc.wait(timeout=10)
                        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                        break
                    time.sleep(2)
            meta.update(exit_code=proc.returncode,elapsed_s=time.monotonic()-start,failure_reason=reason)
            (out/'run.json').write_text(json.dumps(meta,indent=2));(out/'resources.json').write_text(json.dumps(samples))
            print('END',out.name,proc.returncode,round(meta['elapsed_s'],2),flush=True)
    print('MATRIX_COMPLETE',dest,flush=True)
if __name__=='__main__':main()
