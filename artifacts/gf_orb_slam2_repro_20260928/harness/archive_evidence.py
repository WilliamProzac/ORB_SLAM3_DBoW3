#!/usr/bin/env python3
"""Copy bounded evidence; keep cloned source, binaries, images and raw runs local."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def record(path):
    return {'path': str(path.resolve()), 'bytes': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    dest = args.destination.resolve()
    assert dest != ROOT and ROOT not in dest.parents
    dest.mkdir(parents=True, exist_ok=True)
    names = ['README.md', 'upstream.patch', 'source_manifest.json',
             'source_manifest_initial.json', 'build_manifest.json',
             'build_manifest_initial.json', 'groundtruth_check.json',
             'tum_input_check.json', 'freeze_audit.json', 'bucket_probe.json',
             'comparison_summary.json', 'run_summary.csv']
    for name in names:
        shutil.copy2(ROOT / name, dest / name)
    for folder in ['harness', 'configs', 'provenance']:
        (dest / folder).mkdir(exist_ok=True)
        for path in sorted((ROOT / folder).iterdir()):
            if path.is_file():
                shutil.copy2(path, dest / folder / path.name)
    inputs = [record(ROOT / 'inputs' / name) for name in ['V1_01_easy.txt', 'tum_large.txt']]
    (dest / 'input_manifest.json').write_text(json.dumps(inputs, indent=2) + '\n')
    for tag, expected in [('euroc_formal', 9), ('euroc_default', 6), ('tum_bgr_formal', 9)]:
        matrix = ROOT / 'runs' / tag
        assert len(list(matrix.glob('*/run.json'))) == expected, tag
        analysis = json.loads((matrix / 'analysis.json').read_text())
        assert len(analysis['runs']) == expected
        target = dest / tag
        target.mkdir(exist_ok=True)
        for name in ['analysis.json', 'frozen.json', 'comparison.png', 'comparison.pdf']:
            shutil.copy2(matrix / name, target / name)
        index = [record(path) for path in sorted(matrix.glob('*/*')) if path.is_file()]
        (target / 'raw_index.json').write_text(json.dumps(index, indent=2) + '\n')
    invalid = ROOT / 'runs/tum_formal'
    target = dest / 'invalid_tum_rgb_input'
    target.mkdir(exist_ok=True)
    shutil.copy2(invalid / 'INVALIDATED.json', target / 'INVALIDATED.json')
    index = [record(path) for path in sorted(invalid.rglob('*')) if path.is_file()]
    (target / 'raw_index.json').write_text(json.dumps(index, indent=2) + '\n')
    (dest / 'logs').mkdir(exist_ok=True)
    for name in ['accuracy_tests.log', 'bucket_probe_build.log', 'configure.log',
                 'configure_default.log', 'euroc_formal.log', 'euroc_default.log',
                 'tum_bgr_formal.log']:
        shutil.copy2(ROOT / 'logs' / name, dest / 'logs' / name)
    print('Archived bounded evidence:', dest)


if __name__ == '__main__':
    main()
