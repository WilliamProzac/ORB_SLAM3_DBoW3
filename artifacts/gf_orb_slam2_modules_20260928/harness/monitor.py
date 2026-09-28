#!/usr/bin/env python3
"""Small read-only progress inspection; never edits a frozen run."""
import csv,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for tag in ['euroc_modules','tum_modules']:
    m=ROOT/'runs'/tag
    if not m.exists():continue
    finished=sorted(m.glob('*/run.json')); active=[p for p in m.iterdir() if p.is_dir() and not (p/'run.json').exists()]
    print(tag, 'finished',len(finished),'/8')
    for d in active:
        f=d/'frames.csv'
        if f.exists() and f.stat().st_size:
            with f.open('rb') as x:x.seek(max(0,f.stat().st_size-4096)); tail=x.read().decode(errors='replace').splitlines()
            rows=[s.split(',') for s in tail if s and s[0].isdigit() and len(s.split(','))>=18]
            if rows:print('ACTIVE',d.name,'last_flushed_index',rows[-1][0],'state',rows[-1][2])
    if finished:
        f=max(finished,key=lambda x:x.stat().st_mtime);meta=json.loads(f.read_text())
        with (f.parent/'frames.csv').open() as h:rows=list(csv.DictReader(h))
        first=next((i for i,r in enumerate(rows) if r['state']=='2'),len(rows));bad=sum(r['state']!='2' for r in rows[first:]);ms=sorted(float(r['track_ms']) for r in rows[first:]);p95=ms[min(len(ms)-1,int(.95*len(ms)))] if ms else None
        print('LATEST',f.parent.name,'exit',meta['exit_code'],'frames',len(rows),'nonOK',bad,'P95approx',round(p95,2) if p95 is not None else None)
