#!/usr/bin/env python3
"""Failure-triggered RGB-D loop-on diagnostic; fresh drivers, frozen core libraries."""
import argparse,json,os,shlex,signal,subprocess,time,shutil
from pathlib import Path
from run import ROOT,GF,sha

def build():
    target=ROOT/'fullsystem';target.mkdir(exist_ok=False)
    s=(ROOT/'harness/replay.cc').read_text()
    needle='while(!s.mpLocalMapper->isFinished() || !s.mpLoopCloser->isFinished())'
    assert s.count(needle)==1
    s=s.replace(needle,'while(!s.mpLocalMapper->isFinished() || !s.mpLoopCloser->isFinished() || s.mpLoopCloser->isRunningGBA())')
    source=target/'replay.cc';source.write_text(s)
    records={}
    for variant in ['official','delivered']:
        original=ROOT/f'build-{variant}';out=target/variant;out.mkdir()
        commands=json.loads((original/'compile_commands.json').read_text())
        cmd=next(c for c in commands if Path(c['file']).name=='replay.cc')
        compile=shlex.split(cmd['command']);compile[compile.index('-o')+1]=str(out/'replay.o');compile[compile.index('-c')+1]=str(source)
        link=shlex.split((original/'CMakeFiles/replay.dir/link.txt').read_text())
        link[link.index('-o')+1]=str(out/'replay')
        objects=[i for i,x in enumerate(link) if x.endswith('replay.cc.o')];assert len(objects)==1;link[objects[0]]=str(out/'replay.o')
        with (out/'build.log').open('w') as log:
            subprocess.run(compile,cwd=original,stdout=log,stderr=subprocess.STDOUT,check=True)
            subprocess.run(link,cwd=original,stdout=log,stderr=subprocess.STDOUT,check=True)
        records[variant]={'compile':compile,'link':link,'cwd':str(original),'driver_sha256':sha(source),'executable_sha256':sha(out/'replay'),'ldd':subprocess.check_output(['ldd',str(out/'replay')],text=True)}
        assert 'not found' not in records[variant]['ldd']
    for mode in ['official800','delivered800','delivered700']:
        s=(ROOT/'configs'/f'tum_{mode}.yaml').read_text();assert s.count('loopClosing: 0')==1
        (target/f'{mode}.yaml').write_text(s.replace('loopClosing: 0','loopClosing: 1'))
    (target/'build_manifest.json').write_text(json.dumps(records,indent=2)+'\n')
def replay(phase):
    target=ROOT/'fullsystem';limit=150 if phase=='smoke' else 0
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    original=json.loads((ROOT/'frozen.json').read_text())
    frozen={str(p):sha(p) for p in target.rglob('*') if p.is_file() and 'runs' not in p.parts}
    for mode in ['official800','delivered800','delivered700']:
        assert all(sha(p)==h for p,h in original.items()),'Main experiment changed'
        assert all(sha(p)==h for p,h in frozen.items()),'Loop-on input changed'
        assert shutil.disk_usage(ROOT).free>4*1024**3
        v='official' if mode=='official800' else 'delivered';out=ROOT/'runs'/f'fullsystem_{phase}'/'tum_large'/mode;out.mkdir(parents=True,exist_ok=False)
        command=list(map(str,[target/v/'replay',ROOT/'vocabulary/ORBvoc.txt',target/f'{mode}.yaml','rgbd',GF/'inputs/tum_large.txt',out,limit]))
        count=limit or 5057
        meta={'mode':mode,'dataset':'tum_large','phase':phase,'expected_frames':count,'loop_closure':True,'repeat':1,'command':command,'hashes':frozen,'start_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
        start=time.monotonic();deadline=start+count/30+300;samples=[];reason=None
        print('START loop-on',phase,mode,flush=True)
        with (out/'console.log').open('w') as log:
            proc=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            while proc.poll() is None:
                try:
                    s=Path(f'/proc/{proc.pid}/status').read_text();stat=Path(f'/proc/{proc.pid}/stat').read_text().split()
                    samples.append({'elapsed_s':time.monotonic()-start,'cpu_ticks':int(stat[13])+int(stat[14]),'clock_ticks_per_second':os.sysconf('SC_CLK_TCK'),**{k:l.split(':',1)[1].strip() for l in s.splitlines() for k in ['VmRSS','VmHWM','VmSwap','Threads'] if l.startswith(k+':')}})
                except FileNotFoundError:pass
                if time.monotonic()>deadline or shutil.disk_usage(ROOT).free<2*1024**3:
                    reason='timeout' if time.monotonic()>deadline else 'disk_guard';os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    break
                time.sleep(2)
        meta.update(exit_code=proc.returncode,elapsed_s=time.monotonic()-start,failure_reason=reason)
        (out/'run.json').write_text(json.dumps(meta,indent=2)+'\n');(out/'resources.json').write_text(json.dumps(samples,indent=2)+'\n')
        print('END loop-on',phase,mode,proc.returncode,flush=True)
        if phase=='smoke' and proc.returncode:raise RuntimeError(f'Loop-on smoke failed: {out}')
    assert all(sha(p)==h for p,h in original.items())
def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['build','smoke','formal']);args=p.parse_args()
    if args.action=='build':build()
    else:replay(args.action)
if __name__=='__main__':main()
