#!/usr/bin/env python3
"""Archive compact evidence; retain untouched full runs in the experiment root."""
import gzip,hashlib,json,shutil
from pathlib import Path
from run import ROOT,sha

def main():
    dest=Path(__file__).resolve().parents[1]
    for name in ['analysis.json','analysis_fullsystem.json','summary.csv','DETAILS.md','frozen.json','freeze_audit.json','verification_formal.json','verification_smoke.json','verification_fullsystem.json','host.json','initial_build_sha256.json']:
        shutil.copy2(ROOT/name,dest/name)
    shutil.copytree(ROOT/'figures',dest/'figures',dirs_exist_ok=True)
    index=[]
    for p in sorted((ROOT/'runs').rglob('*')):
        if not p.is_file():continue
        rel=p.relative_to(ROOT);q=dest/rel
        q.parent.mkdir(parents=True,exist_ok=True)
        raw=p.read_bytes()
        if p.suffix in ['.csv','.txt','.log']:
            q=q.with_name(q.name+'.gz')
            with q.open('wb') as f:
                with gzip.GzipFile(filename='',mode='wb',fileobj=f,mtime=0) as z:z.write(raw)
            assert gzip.decompress(q.read_bytes())==raw
        else:shutil.copy2(p,q)
        index.append({'source':str(p),'size':len(raw),'sha256':hashlib.sha256(raw).hexdigest(),'archived':str(q.relative_to(dest)),'archive_sha256':sha(q)})
    logs=dest/'build_logs';logs.mkdir(exist_ok=True)
    for p in sorted(ROOT.glob('*log')):
        q=logs/(p.name+'.gz')
        with q.open('wb') as f:
            with gzip.GzipFile(filename='',mode='wb',fileobj=f,mtime=0) as z:z.write(p.read_bytes())
    for v in ['official','delivered']:
        q=dest/'build_provenance'/v;q.mkdir(parents=True,exist_ok=True)
        for name in ['compile_commands.json','CMakeCache.txt']:
            shutil.copy2(ROOT/f'build-{v}'/name,q/name)
    extra=dest/'fullsystem';extra.mkdir(exist_ok=True)
    for p in (ROOT/'fullsystem').glob('*'):
        if p.is_file():shutil.copy2(p,extra/p.name)
    for v in ['official','delivered']:
        q=extra/v;q.mkdir(exist_ok=True)
        shutil.copy2(ROOT/'fullsystem'/v/'build.log',q/'build.log')
    (dest/'raw_index.json').write_text(json.dumps(index,indent=2)+'\n')
    print('ARCHIVED',len(index),'files',sum(r['size'] for r in index),'raw bytes')
if __name__=='__main__':main()
