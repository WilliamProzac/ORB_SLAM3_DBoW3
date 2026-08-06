#!/usr/bin/env python3
"""Run a fixed-size ONNX model with Horizon's quantization-aware ORT executor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from horizon_nn.executor import ORTExecutor


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--matches", required=True, type=int)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    values = np.fromfile(args.input, dtype=np.float32)
    expected = args.matches * 128
    if values.size != expected or not np.isfinite(values).all():
        raise RuntimeError(f"expected {expected} finite floats, got {values.size}")
    executor = ORTExecutor(str(args.model))
    outputs = executor.inference(
        {"descriptor_pairs": values.reshape(1, args.matches, 128)}
    )
    if "fine_logits" not in outputs:
        raise RuntimeError(f"fine_logits missing from outputs: {list(outputs)}")
    logits = np.asarray(outputs["fine_logits"], dtype=np.float32)
    if logits.shape != (1, args.matches, 64) or not np.isfinite(logits).all():
        raise RuntimeError(f"unexpected output: shape={logits.shape}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / "fine_logits.f32"
    logits.tofile(output)
    metadata = {
        "model": str(args.model),
        "input": str(args.input),
        "output": str(output),
        "shape": list(logits.shape),
        "minimum": float(logits.min()),
        "maximum": float(logits.max()),
        "mean": float(logits.mean()),
    }
    (args.output_dir / "outputs.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
