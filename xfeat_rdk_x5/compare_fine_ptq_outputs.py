#!/usr/bin/env python3
"""Compare Fine Matcher float, quantized simulation, and board logits."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path, matches: int) -> np.ndarray:
    values = np.fromfile(path, dtype=np.float32)
    if values.size != matches * 64 or not np.isfinite(values).all():
        raise RuntimeError(f"{path}: expected {matches * 64} finite floats")
    return values.reshape(matches, 64)


def softmax(values: np.ndarray) -> np.ndarray:
    shifted = values - values.max(axis=1, keepdims=True)
    exponent = np.exp(shifted)
    return exponent / exponent.sum(axis=1, keepdims=True)


def decoded(logits: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    probabilities = softmax(logits.astype(np.float64) * 3.0)
    axis = np.arange(8, dtype=np.float64) - 4.0
    x, y = np.meshgrid(axis, axis)
    offsets = np.stack(
        [probabilities @ x.reshape(-1), probabilities @ y.reshape(-1)], axis=1
    )
    return offsets, probabilities.max(axis=1)


def tensor_metrics(reference: np.ndarray, candidate: np.ndarray) -> dict[str, float]:
    difference = candidate.astype(np.float64) - reference.astype(np.float64)
    denominator = np.linalg.norm(reference) * np.linalg.norm(candidate)
    return {
        "cosine_similarity": float(np.vdot(reference, candidate) / denominator),
        "mean_absolute_error": float(np.mean(np.abs(difference))),
        "maximum_absolute_error": float(np.max(np.abs(difference))),
    }


def decode_metrics(
    reference_logits: np.ndarray, candidate_logits: np.ndarray, threshold: float
) -> dict[str, float]:
    reference_offsets, reference_confidence = decoded(reference_logits)
    candidate_offsets, candidate_confidence = decoded(candidate_logits)
    coordinate_error = np.linalg.norm(candidate_offsets - reference_offsets, axis=1)
    decisions = (reference_confidence >= threshold) == (
        candidate_confidence >= threshold
    )
    return {
        "offset_error_median_px": float(np.percentile(coordinate_error, 50)),
        "offset_error_p95_px": float(np.percentile(coordinate_error, 95)),
        "offset_error_maximum_px": float(coordinate_error.max()),
        "confidence_decision_agreement": float(decisions.mean()),
        "confidence_maximum_absolute_error": float(
            np.max(np.abs(candidate_confidence - reference_confidence))
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matches", type=int, required=True)
    parser.add_argument("--float", dest="float_path", type=Path, required=True)
    parser.add_argument("--quantized", type=Path, required=True)
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--threshold", type=float, default=0.25)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    values = {
        "float": load(args.float_path, args.matches),
        "quantized": load(args.quantized, args.matches),
        "board": load(args.board, args.matches),
    }
    quantized_decode = decode_metrics(values["float"], values["quantized"], args.threshold)
    board_decode = decode_metrics(values["float"], values["board"], args.threshold)
    board_quantized_exact = bool(np.array_equal(values["board"], values["quantized"]))
    pass_flags = {
        "quantized_offset_p95": quantized_decode["offset_error_p95_px"] <= 0.25,
        "board_offset_p95": board_decode["offset_error_p95_px"] <= 0.25,
        "quantized_decisions": quantized_decode["confidence_decision_agreement"] >= 0.99,
        "board_decisions": board_decode["confidence_decision_agreement"] >= 0.99,
        "board_matches_quantized": board_quantized_exact,
    }
    report = {
        "status": "passing" if all(pass_flags.values()) else "failing",
        "matches": args.matches,
        "confidence_threshold": args.threshold,
        "model": {
            "path": str(args.model.resolve()),
            "sha256": sha256(args.model),
            "bytes": args.model.stat().st_size,
        },
        "files": {
            name: {"path": str(path.resolve()), "sha256": sha256(path)}
            for name, path in {
                "float": args.float_path,
                "quantized": args.quantized,
                "board": args.board,
            }.items()
        },
        "logits": {
            "quantized_vs_float": tensor_metrics(values["float"], values["quantized"]),
            "board_vs_float": tensor_metrics(values["float"], values["board"]),
            "board_vs_quantized_exact": board_quantized_exact,
        },
        "decoded": {
            "quantized_vs_float": quantized_decode,
            "board_vs_float": board_decode,
        },
        "gates": pass_flags,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report["status"] == "passing" else 1)


if __name__ == "__main__":
    main()
