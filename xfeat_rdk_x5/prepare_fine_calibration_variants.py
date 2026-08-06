#!/usr/bin/env python3
"""Slice fixed-M Fine Matcher calibration tensors from the M=512 corpus."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=Path(__file__).resolve().parent
        / "calibration_data"
        / "fine_matcher_m512",
    )
    parser.add_argument("--sizes", type=int, nargs="+", default=[256, 384])
    args = parser.parse_args()
    source_files = sorted(args.source.glob("*.f32"))
    if not source_files:
        raise SystemExit(f"No calibration tensors found under {args.source}")
    root = args.source.parent
    for size in sorted(set(args.sizes)):
        if size <= 0 or size > 512:
            raise SystemExit(f"Unsupported match count: {size}")
        output = root / f"fine_matcher_m{size}"
        output.mkdir(parents=True, exist_ok=True)
        for path in source_files:
            values = np.fromfile(path, dtype=np.float32)
            if values.size != 512 * 128:
                raise RuntimeError(f"Unexpected tensor size in {path}: {values.size}")
            values.reshape(512, 128)[:size].tofile(output / path.name)
        print(f"M={size}: {len(source_files)} tensors -> {output}")


if __name__ == "__main__":
    main()
