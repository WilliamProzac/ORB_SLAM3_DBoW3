#!/usr/bin/env python3
"""Summarize the F21 ORB/XFeat production-baseline promotion gates."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Callable, Iterable


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--orb-csv", action="append", required=True, type=Path)
    parser.add_argument("--xfeat-csv", action="append", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--xfeat-method", default="xfeat_guided")
    parser.add_argument("--required-runs", type=int, default=5)
    parser.add_argument("--tracking-json", type=Path)
    parser.add_argument("--tracking-orb-method", default="orb_slam3_production")
    parser.add_argument("--tracking-xfeat-method", default="xfeat_global_fine")
    return parser.parse_args()


def load_runs(
    paths: Iterable[Path],
    warmup: int,
    predicate: Callable[[dict[str, str]], bool] | None = None,
) -> list[list[dict[str, str]]]:
    runs: list[list[dict[str, str]]] = []
    for path in paths:
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        if predicate is not None:
            rows = [row for row in rows if predicate(row)]
        if rows and "frame_index" in rows[0]:
            rows = [row for row in rows if number(row, "frame_index") >= warmup]
        else:
            rows = rows[warmup:]
        runs.append(rows)
    return runs


def flatten(runs: Iterable[Iterable[dict[str, str]]]) -> list[dict[str, str]]:
    return [row for run in runs for row in run]


def number(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}: {row[name]}")
    return value


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("cannot summarize an empty metric")
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def metric(rows: list[dict[str, str]], field: str) -> dict[str, float]:
    values = [number(row, field) for row in rows]
    return {
        "mean": sum(values) / len(values),
        "median": percentile(values, 0.5),
        "p90": percentile(values, 0.9),
        "minimum": min(values),
        "maximum": max(values),
    }


def optional_metric(
    rows: list[dict[str, str]], field: str
) -> dict[str, float] | None:
    if not rows or field not in rows[0]:
        return None
    return metric(rows, field)


def derived_metric(
    rows: list[dict[str, str]], function: Callable[[dict[str, str]], float]
) -> dict[str, float]:
    values = [function(row) for row in rows]
    return {
        "mean": sum(values) / len(values),
        "median": percentile(values, 0.5),
        "p90": percentile(values, 0.9),
        "minimum": min(values),
        "maximum": max(values),
    }


def file_record(path: Path) -> dict[str, object]:
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"path": str(path.resolve()), "sha256": digest, "bytes": path.stat().st_size}


def tracking_failures(rows: list[dict[str, str]]) -> dict[str, object]:
    failure_states = {3, 4}
    initialized = False
    failure_frames = 0
    segments: list[tuple[int, int]] = []
    segment_start: int | None = None
    previous_stamp: int | None = None
    for row in rows:
        state = int(number(row, "tracking_state"))
        stamp = int(number(row, "stamp_ns"))
        if state in {2, 5}:
            initialized = True
        failed = initialized and state in failure_states
        if failed:
            failure_frames += 1
            if segment_start is None:
                segment_start = stamp
        elif segment_start is not None:
            segments.append((segment_start, previous_stamp or segment_start))
            segment_start = None
        previous_stamp = stamp
    if segment_start is not None:
        segments.append((segment_start, previous_stamp or segment_start))
    durations = [(end - start) / 1e9 for start, end in segments]
    return {
        "failure_frames": failure_frames,
        "failure_segments": len(segments),
        "total_failure_seconds": sum(durations),
        "longest_failure_seconds": max(durations, default=0.0),
    }


def valid_orb_row(row: dict[str, str]) -> bool:
    return (
        0.0 < number(row, "keypoints") < 10000.0
        and number(row, "feature_extract_ms") > 0.0
        and number(row, "stereo_match_ms") > 0.0
        and number(row, "frontend_ms") > 0.0
    )


def gate(actual: float, operator: str, limit: float) -> dict[str, object]:
    comparisons = {
        "<=": actual <= limit,
        "<": actual < limit,
        ">=": actual >= limit,
    }
    return {"pass": comparisons[operator], "actual": actual, "operator": operator, "limit": limit}


def main() -> None:
    args = parse_args()
    if args.warmup < 0:
        raise ValueError("warmup must be non-negative")

    orb_runs = load_runs(args.orb_csv, args.warmup)
    xfeat_runs = load_runs(
        args.xfeat_csv,
        args.warmup,
        lambda row: row.get("method") == args.xfeat_method
        and row.get("pair_type") == "stereo",
    )
    orb_rows_unfiltered = flatten(orb_runs)
    orb_rows = [row for row in orb_rows_unfiltered if valid_orb_row(row)]
    xfeat_rows = flatten(xfeat_runs)
    if not orb_rows or not xfeat_rows:
        raise ValueError("no post-warmup ORB or XFeat rows remain")

    orb = {
        "frames": len(orb_rows),
        "discarded_invalid_rows": len(orb_rows_unfiltered) - len(orb_rows),
        "feature_extract_ms": metric(orb_rows, "feature_extract_ms"),
        "stereo_match_ms": metric(orb_rows, "stereo_match_ms"),
        "frontend_ms": metric(orb_rows, "frontend_ms"),
        "track_total_ms": metric(orb_rows, "track_total_ms"),
        "keypoints": metric(orb_rows, "keypoints"),
        "valid_depths": metric(orb_rows, "valid_depths"),
        "valid_depth_ratio": metric(orb_rows, "valid_depth_ratio"),
        "depth_grid_coverage": metric(orb_rows, "depth_grid_coverage"),
        "tracked_inliers": metric(orb_rows, "tracked_inliers"),
        "tracking": tracking_failures(orb_rows),
    }
    if "end_to_end_pair_ms" in xfeat_rows[0]:
        xfeat_frontend = metric(xfeat_rows, "end_to_end_pair_ms")
        xfeat_latency_definition = "end_to_end_pair_ms"
    else:
        xfeat_frontend = derived_metric(
            xfeat_rows,
            lambda row: number(row, "extraction_wall_ms")
            + number(row, "match_ms"),
        )
        xfeat_latency_definition = "extraction_wall_ms + match_ms"
    xfeat = {
        "frames": len(xfeat_rows),
        "extraction_wall_ms": metric(xfeat_rows, "extraction_wall_ms"),
        "match_ms": metric(xfeat_rows, "match_ms"),
        "frontend_ms": xfeat_frontend,
        "strict_inliers": metric(xfeat_rows, "strict_inliers"),
        "strict_inlier_ratio": metric(xfeat_rows, "strict_inlier_ratio"),
        "strict_grid_coverage": metric(xfeat_rows, "strict_grid_coverage"),
        "median_vertical_error_px": metric(xfeat_rows, "median_vertical_error_px"),
        "fine_total_ms": optional_metric(xfeat_rows, "fine_total_ms"),
        "photometric_inliers": optional_metric(xfeat_rows, "photometric_inliers"),
        "photometric_inlier_ratio": optional_metric(
            xfeat_rows, "photometric_inlier_ratio"
        ),
        "photometric_grid_coverage": optional_metric(
            xfeat_rows, "photometric_grid_coverage"
        ),
        "patch_ncc_median": optional_metric(xfeat_rows, "patch_ncc_median"),
    }

    gates = {
        "five_orb_runs": gate(float(len(args.orb_csv)), ">=", float(args.required_runs)),
        "five_xfeat_runs": gate(float(len(args.xfeat_csv)), ">=", float(args.required_runs)),
        "mean_frontend_latency": gate(
            xfeat["frontend_ms"]["mean"], "<=", 0.90 * orb["frontend_ms"]["mean"]
        ),
        "p90_frontend_latency": gate(
            xfeat["frontend_ms"]["p90"], "<", orb["frontend_ms"]["p90"]
        ),
        "mean_strict_inliers": gate(
            xfeat["strict_inliers"]["mean"], ">=", 0.95 * orb["valid_depths"]["mean"]
        ),
        "strict_inlier_ratio": gate(
            xfeat["strict_inlier_ratio"]["mean"],
            ">=",
            orb["valid_depth_ratio"]["mean"] - 0.03,
        ),
        "legacy_strict_grid_coverage": gate(
            xfeat["strict_grid_coverage"]["mean"],
            ">=",
            orb["depth_grid_coverage"]["mean"],
        ),
        "minimum_strict_inliers": gate(xfeat["strict_inliers"]["minimum"], ">=", 30.0),
        "median_vertical_error": gate(
            xfeat["median_vertical_error_px"]["median"], "<=", 0.5
        ),
    }
    if xfeat["photometric_inliers"] is not None:
        gates["board_photometric_inliers"] = gate(
            xfeat["photometric_inliers"]["mean"],
            ">=", 0.95 * orb["valid_depths"]["mean"],
        )
    if xfeat["patch_ncc_median"] is not None:
        gates["board_patch_ncc_median"] = gate(
            xfeat["patch_ncc_median"]["median"], ">=", 0.5
        )
    tracking_comparison = {
        "status": "not_evaluated",
        "reason": "--tracking-json was not supplied",
    }
    if args.tracking_json:
        tracking_document = json.loads(args.tracking_json.read_text())
        methods = tracking_document["methods"]
        orb_tracking = methods[args.tracking_orb_method]
        xfeat_tracking = methods[args.tracking_xfeat_method]
        tracking_gates = {
            "stereo_ncc_depths": gate(
                xfeat_tracking["stereo_ncc_depths"]["mean"],
                ">=", 0.95 * orb_tracking["stereo_ncc_depths"]["mean"],
            ),
            "stereo_ncc_coverage": gate(
                xfeat_tracking["stereo_ncc_coverage"]["mean"],
                ">=", orb_tracking["stereo_ncc_coverage"]["mean"],
            ),
            "pnp_inliers": gate(
                xfeat_tracking["pnp_inliers"]["mean"],
                ">=", 0.95 * orb_tracking["pnp_inliers"]["mean"],
            ),
            "pnp_grid_coverage": gate(
                xfeat_tracking["pnp_grid_coverage"]["mean"],
                ">=", orb_tracking["pnp_grid_coverage"]["mean"],
            ),
            "pnp_success_rate": gate(
                xfeat_tracking["pnp_success_rate"],
                ">=", orb_tracking["pnp_success_rate"],
            ),
            "fragile_pnp_pairs": gate(
                float(xfeat_tracking["fragile_pnp_pairs"]),
                "<=", float(orb_tracking["fragile_pnp_pairs"]),
            ),
        }
        gates.update({f"tracking_{name}": value for name, value in tracking_gates.items()})
        tracking_comparison = {
            "status": "passing" if all(item["pass"] for item in tracking_gates.values()) else "failing",
            "input": file_record(args.tracking_json),
            "orb_method": args.tracking_orb_method,
            "xfeat_method": args.tracking_xfeat_method,
            "orb": orb_tracking,
            "xfeat": xfeat_tracking,
            "gates": tracking_gates,
        }
    # The legacy depth-coverage fields are not comparable: production ORB
    # reports every Frame depth while XFeat reports strict/NCC-filtered pairs.
    # They stay in the report for diagnosis, but common Stereo/NCC/PnP gates
    # above own the geometric promotion decision.
    gates["legacy_strict_grid_coverage"]["decision_metric"] = False
    decision_gates = {
        name: item for name, item in gates.items()
        if item.get("decision_metric", True)
    }
    frontend_pass = all(item["pass"] for item in decision_gates.values())
    report = {
        "schema": "rdk-x5-f21-gate-v2",
        "warmup_frames_per_run": args.warmup,
        "xfeat_method": args.xfeat_method,
        "xfeat_latency_definition": xfeat_latency_definition,
        "inputs": {
            "orb": [file_record(path) for path in args.orb_csv],
            "xfeat": [file_record(path) for path in args.xfeat_csv],
        },
        "orb": orb,
        "xfeat": xfeat,
        "gates": gates,
        "frontend_pass": frontend_pass,
        "tracking_comparison": tracking_comparison,
        "promotion_to_f22": frontend_pass and tracking_comparison["status"] == "passing",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "frontend_pass": frontend_pass,
        "promotion_to_f22": report["promotion_to_f22"],
    }, indent=2))


if __name__ == "__main__":
    main()
