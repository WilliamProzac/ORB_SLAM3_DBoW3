#!/usr/bin/env python3
"""Compare XFeat float, hb_mapper quantized, and RDK X5 BPU outputs."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


OUTPUTS = (
    (
        "dense_descriptors",
        "_block_fusion_block_fusion.2_Conv_float.bin",
        "dense_descriptors_float.bin",
        "output_0_dense_descriptors.f32",
        (1, 64, 60, 80),
    ),
    (
        "keypoint_logits",
        "_keypoint_tail_keypoint_tail.3_Conv_float.bin",
        "keypoint_logits_float.bin",
        "output_1_keypoint_logits.f32",
        (1, 65, 60, 80),
    ),
    (
        "reliability",
        "reliability_float.bin",
        "reliability_float.bin",
        "output_2_reliability.f32",
        (1, 1, 60, 80),
    ),
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load(path, shape):
    values = np.fromfile(path, dtype=np.float32)
    expected = int(np.prod(shape))
    if values.size == expected + len(shape):
        header = np.fromfile(path, dtype=np.int32, count=len(shape))
        if tuple(int(value) for value in header) != tuple(shape):
            raise RuntimeError(f"{path}: unexpected shape header {header.tolist()}")
        values = np.fromfile(path, dtype=np.float32, offset=4 * len(shape))
    if values.size != expected:
        raise RuntimeError(f"{path}: expected {expected} floats, got {values.size}")
    if not np.isfinite(values).all():
        raise RuntimeError(f"{path}: contains non-finite values")
    return values


def metrics(reference, candidate):
    difference = candidate.astype(np.float64) - reference.astype(np.float64)
    denominator = np.linalg.norm(reference) * np.linalg.norm(candidate)
    cosine = float(np.dot(reference, candidate) / denominator) if denominator else 1.0
    return {
        "cosine_similarity": cosine,
        "mean_absolute_error": float(np.mean(np.abs(difference))),
        "root_mean_squared_error": float(np.sqrt(np.mean(difference * difference))),
        "max_absolute_error": float(np.max(np.abs(difference))),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("float_dir", type=Path)
    parser.add_argument("quantized_dir", type=Path)
    parser.add_argument("board_dir", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--minimum-cosine", type=float, default=0.99)
    args = parser.parse_args()

    results = {}
    for name, float_name, quantized_name, board_name, shape in OUTPUTS:
        paths = {
            "float": args.float_dir / float_name,
            "quantized": args.quantized_dir / quantized_name,
            "board": args.board_dir / board_name,
        }
        values = {kind: load(path, shape) for kind, path in paths.items()}
        results[name] = {
            "shape": list(shape),
            "files": {
                kind: {"path": str(path), "sha256": sha256(path)}
                for kind, path in paths.items()
            },
            "quantized_vs_float": metrics(values["float"], values["quantized"]),
            "board_vs_quantized": metrics(values["quantized"], values["board"]),
            "board_vs_float": metrics(values["float"], values["board"]),
        }

    minimum = min(
        output[comparison]["cosine_similarity"]
        for output in results.values()
        for comparison in ("quantized_vs_float", "board_vs_float")
    )
    report = {
        "status": "passing" if minimum >= args.minimum_cosine else "failing",
        "minimum_required_cosine": args.minimum_cosine,
        "minimum_observed_cosine": minimum,
        "outputs": results,
    }
    board_metadata = args.board_dir / "outputs.json"
    if board_metadata.is_file():
        report["board_runtime"] = json.loads(board_metadata.read_text(encoding="utf-8"))
    if args.model:
        report["model"] = {
            "path": str(args.model),
            "byte_size": args.model.stat().st_size,
            "sha256": sha256(args.model),
        }
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["status"] == "passing" else 1


if __name__ == "__main__":
    raise SystemExit(main())
