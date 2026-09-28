#!/usr/bin/env python3
"""Archive compact GF03 evidence; keep raw logs/trajectories with hash manifests."""
import argparse,hashlib,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def record(p):
    return {'path':str(p.resolve()),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def main():
    p=argparse.ArgumentParser();p.add_argument('destination',type=Path);a=p.parse_args();dest=a.destination.resolve()
    assert dest!=ROOT and ROOT not in dest.parents;dest.mkdir(parents=True,exist_ok=True)
    for name in ['README.md','DESIGN.md','REPORT.md','upstream.patch','module_delta.patch','source_manifest_initial.json','previous_builds.json','build_manifest.json','freeze_audit.json','comparison_summary.json','run_summary.csv','smoke_summary_initial.json','smoke_summary_final.json']:
        shutil.copy2(ROOT/name,dest/name)
    for folder in ['harness','configs','provenance']:
        (dest/folder).mkdir(exist_ok=True)
        for f in sorted((ROOT/folder).iterdir()):
            if f.is_file():shutil.copy2(f,dest/folder/f.name)
    (dest/'input_manifest.json').write_text(json.dumps([record(ROOT/'inputs'/x) for x in ['V1_01_easy.txt','tum_large.txt']],indent=2)+'\n')
    for tag in ['euroc_modules','tum_modules']:
        matrix=ROOT/'runs'/tag;target=dest/tag;target.mkdir(exist_ok=True)
        assert len(list(matrix.glob('*/run.json')))==24
        for name in ['analysis.json','frozen.json','tables.md','comparison.png','comparison.pdf']:
            shutil.copy2(matrix/name,target/name)
        (target/'raw_index.json').write_text(json.dumps([record(x) for x in sorted(matrix.glob('*/*')) if x.is_file()],indent=2)+'\n')
    (dest/'logs').mkdir(exist_ok=True)
    for f in (ROOT/'logs').glob('*'):
        if f.is_file() and f.stat().st_size<500_000:shutil.copy2(f,dest/'logs'/f.name)
    (dest/'build_log_index.json').write_text(json.dumps([record(x) for x in sorted((ROOT/'logs').glob('*')) if x.is_file()],indent=2)+'\n')
    (dest/'smoke_index.json').write_text(json.dumps([record(x) for x in sorted((ROOT/'runs').glob('smoke*/*/*')) if x.is_file()],indent=2)+'\n')
    print('Archived GF03 evidence:',dest)
if __name__=='__main__':main()
