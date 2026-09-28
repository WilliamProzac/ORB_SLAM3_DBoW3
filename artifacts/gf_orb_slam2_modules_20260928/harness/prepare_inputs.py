#!/usr/bin/env python3
"""Rebuild the native manifests from original dataset indices, without resampling."""
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--euroc-root', type=Path, default=Path('/home/ywl/dataset/vicon_room1/V1_01_easy'))
    p.add_argument('--tum-root', type=Path, default=Path('/home/ywl/dataset/rgbd_dataset_freiburg2_large_with_loop'))
    p.add_argument('--check-existing', action='store_true', help='Verify only; never overwrite frozen manifests')
    args = p.parse_args()
    camera = args.euroc_root.resolve() / 'mav0/cam0'
    euroc = []
    for line in (camera / 'data.csv').read_text().splitlines():
        if not line.strip() or line.startswith('#'):
            continue
        stamp, name = line.strip().split(',')
        image = camera / 'data' / name
        assert image.is_file(), image
        euroc.append(f'{int(stamp) / 1e9:.9f} {image}\n')
    tum = []
    for line in (args.tum_root / 'associate.txt').read_text().splitlines():
        if not line.strip() or line.startswith('#'):
            continue
        stamp, rgb, depth_stamp, depth = line.split()
        assert abs(float(stamp) - float(depth_stamp)) <= .02
        rgb, depth = args.tum_root.resolve() / rgb, args.tum_root.resolve() / depth
        assert rgb.is_file() and depth.is_file(), (rgb, depth)
        tum.append(f'{stamp} {rgb} {depth}\n')
    for name, rows in [('V1_01_easy', euroc), ('tum_large', tum)]:
        stamps = [float(row.split()[0]) for row in rows]
        assert rows and all(a < b for a, b in zip(stamps, stamps[1:]))
        dest = ROOT / 'inputs' / (name + '.txt')
        content = ''.join(rows)
        if args.check_existing:
            assert dest.read_text() == content, f'Input differs: {dest}'
        else:
            dest.parent.mkdir(exist_ok=True)
            with dest.open('x') as output:  # Never silently replace prior experiment inputs.
                output.write(content)
        print(name, len(rows), 'verified' if args.check_existing else 'created')


if __name__ == '__main__':
    main()
