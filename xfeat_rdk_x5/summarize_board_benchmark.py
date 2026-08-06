#!/usr/bin/env python3
"""Summarize the RDK X5 ROS XFeat/ORB benchmark CSV without extra deps."""

import argparse
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path


NUMERIC_FIELDS = {
    "frame_index",
    "stamp_ns",
    "tracking_state",
    "extract_a_ms",
    "extract_b_ms",
    "preprocess_a_ms",
    "preprocess_b_ms",
    "bpu_queue_a_ms",
    "bpu_queue_b_ms",
    "bpu_a_ms",
    "bpu_b_ms",
    "postprocess_a_ms",
    "postprocess_b_ms",
    "keypoints_a",
    "keypoints_b",
    "extraction_wall_ms",
    "match_ms",
    "geometry_ms",
    "end_to_end_pair_ms",
    "matches",
    "ransac_inliers",
    "strict_inliers",
    "strict_inlier_ratio",
    "strict_grid_coverage",
    "median_vertical_error_px",
    "median_disparity_px",
}


def percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - position) + ordered[high] * (position - low)


def distribution(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {"count": 0}
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "p10": percentile(finite, 0.10),
        "p90": percentile(finite, 0.90),
        "min": min(finite),
        "max": max(finite),
    }


def parse_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        for field in NUMERIC_FIELDS:
            if field in row:
                row[field] = float(row[field])
    return rows


def summarize_pair_rows(rows):
    robust = [
        row
        for row in rows
        if row["strict_inliers"] >= 30 and row["strict_inlier_ratio"] >= 0.30
    ]
    return {
        "pair_count": len(rows),
        "robust_pair_count": len(robust),
        "robust_pair_rate": len(robust) / len(rows) if rows else 0.0,
        "matches": distribution(row["matches"] for row in rows),
        "ransac_inliers": distribution(row["ransac_inliers"] for row in rows),
        "strict_inliers": distribution(row["strict_inliers"] for row in rows),
        "strict_inlier_ratio": distribution(row["strict_inlier_ratio"] for row in rows),
        "strict_grid_coverage": distribution(
            row["strict_grid_coverage"] for row in rows
        ),
        "match_latency_ms": distribution(row["match_ms"] for row in rows),
        "extraction_wall_latency_ms": distribution(
            row.get(
                "extraction_wall_ms", row["extract_a_ms"] + row["extract_b_ms"]
            )
            for row in rows
        ),
        "geometry_latency_ms": distribution(row["geometry_ms"] for row in rows),
        "end_to_end_pair_latency_ms": distribution(
            row["end_to_end_pair_ms"] for row in rows
        ),
    }


def summarize_method(rows, method):
    method_rows = [row for row in rows if row["method"] == method]
    stereo_rows = [row for row in method_rows if row["pair_type"] == "stereo"]
    # Each stereo row owns two independently extracted images. Temporal rows reuse
    # those same extractions, so excluding them avoids double counting latency.
    extract = [value for row in stereo_rows for value in (row["extract_a_ms"], row["extract_b_ms"])]
    keypoints = [value for row in stereo_rows for value in (row["keypoints_a"], row["keypoints_b"])]
    preprocess = [value for row in stereo_rows for value in (row["preprocess_a_ms"], row["preprocess_b_ms"])]
    bpu = [value for row in stereo_rows for value in (row["bpu_a_ms"], row["bpu_b_ms"])]
    bpu_queue = [
        value
        for row in stereo_rows
        for value in (
            row.get("bpu_queue_a_ms", 0.0),
            row.get("bpu_queue_b_ms", 0.0),
        )
    ]
    postprocess = [value for row in stereo_rows for value in (row["postprocess_a_ms"], row["postprocess_b_ms"])]
    pairs = {}
    for pair_type in ("stereo", "temporal"):
        pair_rows = [row for row in method_rows if row["pair_type"] == pair_type]
        pairs[pair_type] = summarize_pair_rows(pair_rows)
        pairs[pair_type]["by_tracking_state"] = {
            "unavailable": summarize_pair_rows(
                [row for row in pair_rows if row["tracking_state"] < 0]
            ),
            "failed": summarize_pair_rows(
                [row for row in pair_rows if row["tracking_state"] == 0]
            ),
            "ok": summarize_pair_rows(
                [row for row in pair_rows if row["tracking_state"] == 1]
            ),
        }
    return {
        "image_extraction_latency_ms": distribution(extract),
        "preprocess_latency_ms": distribution(preprocess),
        "bpu_latency_ms": distribution(bpu),
        "bpu_queue_latency_ms": distribution(bpu_queue),
        "postprocess_latency_ms": distribution(postprocess),
        "detected_keypoints": distribution(keypoints),
        "pairs": pairs,
    }


def mean(summary, path):
    current = summary
    for key in path:
        current = current[key]
    return current["mean"]


def ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-stereo", type=int, default=130)
    parser.add_argument("--expected-temporal", type=int, default=65)
    parser.add_argument("--quality-reference", type=Path)
    args = parser.parse_args()

    rows = parse_csv(args.csv)
    expected_counts = {
        ("xfeat_bpu", "stereo"): args.expected_stereo,
        ("orb_cpu", "stereo"): args.expected_stereo,
        ("xfeat_bpu", "temporal"): args.expected_temporal,
        ("orb_cpu", "temporal"): args.expected_temporal,
    }
    if len(rows) != sum(expected_counts.values()):
        raise ValueError(
            f"Expected {sum(expected_counts.values())} rows, found {len(rows)}"
        )
    for key, expected in expected_counts.items():
        actual = sum(
            row["method"] == key[0] and row["pair_type"] == key[1]
            for row in rows
        )
        if actual != expected:
            raise ValueError(f"Expected {expected} rows for {key}, found {actual}")
    required_finite = (
        "extract_a_ms",
        "extract_b_ms",
        "match_ms",
        "end_to_end_pair_ms",
        "matches",
        "strict_inliers",
        "strict_inlier_ratio",
        "strict_grid_coverage",
    )
    for index, row in enumerate(rows, start=2):
        for field in required_finite:
            if not math.isfinite(row[field]):
                raise ValueError(f"Non-finite {field} at CSV line {index}")
    methods = {
        "xfeat_bpu": summarize_method(rows, "xfeat_bpu"),
        "orb_cpu": summarize_method(rows, "orb_cpu"),
    }
    comparison = {
        "image_extraction_xfeat_over_orb": ratio(
            mean(methods, ("xfeat_bpu", "image_extraction_latency_ms")),
            mean(methods, ("orb_cpu", "image_extraction_latency_ms")),
        )
    }
    for pair_type in ("stereo", "temporal"):
        comparison[pair_type] = {
            "end_to_end_latency_xfeat_over_orb": ratio(
                mean(methods, ("xfeat_bpu", "pairs", pair_type, "end_to_end_pair_latency_ms")),
                mean(methods, ("orb_cpu", "pairs", pair_type, "end_to_end_pair_latency_ms")),
            ),
            "matches_xfeat_over_orb": ratio(
                mean(methods, ("xfeat_bpu", "pairs", pair_type, "matches")),
                mean(methods, ("orb_cpu", "pairs", pair_type, "matches")),
            ),
            "strict_inliers_xfeat_over_orb": ratio(
                mean(methods, ("xfeat_bpu", "pairs", pair_type, "strict_inliers")),
                mean(methods, ("orb_cpu", "pairs", pair_type, "strict_inliers")),
            ),
            "strict_grid_coverage_xfeat_over_orb": ratio(
                mean(methods, ("xfeat_bpu", "pairs", pair_type, "strict_grid_coverage")),
                mean(methods, ("orb_cpu", "pairs", pair_type, "strict_grid_coverage")),
            ),
        }

    digest = hashlib.sha256(args.csv.read_bytes()).hexdigest()
    frame_indices = sorted({int(row["frame_index"]) for row in rows})
    result = {
        "source": {
            "csv": str(args.csv.resolve()),
            "sha256": digest,
            "row_count": len(rows),
            "sampled_frame_count": len(frame_indices),
            "first_sampled_frame": frame_indices[0],
            "last_sampled_frame": frame_indices[-1],
            "first_sampled_stamp_ns": int(min(row["stamp_ns"] for row in rows)),
            "last_sampled_stamp_ns": int(max(row["stamp_ns"] for row in rows)),
            "configuration": {
                "bag_source_frame_count": 644,
                "image_shape_hw": [544, 640],
                "top_k": 1024,
                "anchor_stride": 10,
                "temporal_gap": 1,
                "execution_order": "alternate XFeat-first and ORB-first by anchor",
                "stereo_execution": "left/right worker threads; one serialized BPU0",
                "ransac_rng_seed": "0x58464541",
                "xfeat_min_cosine": 0.82,
                "orb_max_hamming": 64.0,
                "fundamental_ransac_threshold_px": 1.5,
                "stereo_vertical_threshold_px": 2.0,
                "stereo_disparity_range_px": [0.0, 160.0],
                "robust_pair_definition": "strict_inliers >= 30 and strict_inlier_ratio >= 0.30",
            },
        },
        "methods": methods,
        "comparison": comparison,
    }
    if args.quality_reference:
        reference_rows = parse_csv(args.quality_reference)
        reference = {
            (int(row["frame_index"]), row["method"], row["pair_type"]): row
            for row in reference_rows
        }
        quality_fields = (
            "stamp_ns",
            "keypoints_a",
            "keypoints_b",
            "matches",
            "ransac_inliers",
            "strict_inliers",
            "strict_inlier_ratio",
            "strict_grid_coverage",
            "median_vertical_error_px",
            "median_disparity_px",
        )
        mismatches = []
        for row in rows:
            key = (int(row["frame_index"]), row["method"], row["pair_type"])
            if key not in reference:
                mismatches.append({"key": key, "field": "row", "reason": "missing"})
                continue
            for field in quality_fields:
                actual = row[field]
                expected = reference[key][field]
                equal = actual == expected or (
                    isinstance(actual, float)
                    and isinstance(expected, float)
                    and math.isnan(actual)
                    and math.isnan(expected)
                )
                if not equal:
                    mismatches.append(
                        {
                            "key": key,
                            "field": field,
                            "expected": expected,
                            "actual": actual,
                        }
                    )
        result["quality_parity"] = {
            "reference_csv": str(args.quality_reference.resolve()),
            "compared_rows": len(rows),
            "fields_per_row": len(quality_fields),
            "mismatch_count": len(mismatches),
            "status": "PASS" if not mismatches else "FAIL",
            "first_mismatches": mismatches[:20],
        }
        if mismatches:
            raise ValueError(f"Quality parity failed with {len(mismatches)} mismatches")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(comparison, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
