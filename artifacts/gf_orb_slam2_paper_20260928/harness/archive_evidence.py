#!/usr/bin/env python3
"""Archive compact GF02 evidence and hashes; retain raw runs in the isolated root."""
import argparse,hashlib,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def record(p):
    return {'path':str(p.resolve()),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def main():
    p=argparse.ArgumentParser();p.add_argument('destination',type=Path);a=p.parse_args();dest=a.destination.resolve()
    assert dest!=ROOT and ROOT not in dest.parents
    dest.mkdir(parents=True,exist_ok=True)
    for name in ['README.md','DESIGN.md','upstream.patch','paper_delta.patch','source_manifest.json','source_manifest_initial.json','gf01_preservation.json','build_manifest.json','freeze_audit.json','comparison_summary.json','run_summary.csv','startup_failure.json']:
        shutil.copy2(ROOT/name,dest/name)
    for folder in ['harness','configs']:
        (dest/folder).mkdir(exist_ok=True)
        for f in sorted((ROOT/folder).iterdir()):
            if f.is_file() and f.name not in ('test_bucket.cc','run_bucket_probe.sh'):
                shutil.copy2(f,dest/folder/f.name)
    (dest/'input_manifest.json').write_text(json.dumps([record(ROOT/'inputs'/x) for x in ['V1_01_easy.txt','tum_large.txt']],indent=2)+'\n')
    for tag in ['euroc_paper','tum_paper']:
        matrix=ROOT/'runs'/tag;target=dest/tag;target.mkdir(exist_ok=True)
        assert len(list(matrix.glob('*/run.json')))==12
        assert len(json.loads((matrix/'analysis.json').read_text())['runs'])==12
        for name in ['analysis.json','frozen.json','tables.md','comparison.png','comparison.pdf','timing_boundary.png','timing_boundary.pdf']:
            shutil.copy2(matrix/name,target/name)
        (target/'raw_index.json').write_text(json.dumps([record(x) for x in sorted(matrix.glob('*/*')) if x.is_file()],indent=2)+'\n')
    (dest/'logs').mkdir(exist_ok=True)
    for name in ['configure.log','index_asan.log','index_tsan.log','index_sanitizers_build.log','index_release.log','accuracy_tests.log','smoke.log','euroc_paper.log','tum_paper.log','freeze_audit.log','linked_libraries.log','euroc_analysis.log','tum_analysis.log','export.log']:
        shutil.copy2(ROOT/'logs'/name,dest/'logs'/name)
    (dest/'build_log_index.json').write_text(json.dumps([record(x) for x in sorted((ROOT/'logs').glob('build*.log'))],indent=2)+'\n')
    (dest/'smoke_index.json').write_text(json.dumps([record(x) for x in sorted((ROOT/'runs/smoke').glob('*/*')) if x.is_file()],indent=2)+'\n')
    print('Archived GF02 evidence:',dest)
if __name__=='__main__':main()
