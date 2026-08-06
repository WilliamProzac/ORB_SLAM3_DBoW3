#!/usr/bin/env python3
"""Summarize board XFeat* coarse/Fine benchmark rows."""

import argparse
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path


FIELDS = (
    "extraction_wall_ms",
    "match_ms",
    "geometry_ms",
    "fine_total_ms",
    "fine_bpu_ms",
    "fine_postprocess_ms",
    "end_to_end_pair_ms",
    "matches",
    "ransac_inliers",
    "strict_inliers",
    "strict_inlier_ratio",
    "strict_grid_coverage",
    "photometric_inliers",
    "photometric_inlier_ratio",
    "photometric_grid_coverage",
    "patch_ncc_median",
    "patch_ncc_p10",
    "photometric_ms",
)


def percentile(values, fraction):
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - position) + ordered[high] * (position - low)


def distribution(values):
    values = [float(value) for value in values if math.isfinite(float(value))]
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p90": percentile(values, 0.9),
        "min": min(values),
        "max": max(values),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    with args.csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    methods = {}
    for method in sorted({row["method"] for row in rows}):
        method_rows = [row for row in rows if row["method"] == method]
        by_type = {}
        for pair_type in sorted({row["pair_type"] for row in method_rows}):
            pair_rows = [row for row in method_rows if row["pair_type"] == pair_type]
            by_type[pair_type] = {
                "row_count": len(pair_rows),
                **{
                    field: distribution(row.get(field, "nan") for row in pair_rows)
                    for field in FIELDS
                },
            }
        methods[method] = by_type

    comparison = {}
    stereo = {
        method: values["stereo"]
        for method, values in methods.items()
        if "stereo" in values
    }
    for method, values in stereo.items():
        comparison[method] = {
            "mean_frontend_ms": values["end_to_end_pair_ms"].get("mean"),
            "p90_frontend_ms": values["end_to_end_pair_ms"].get("p90"),
            "mean_strict_inliers": values["strict_inliers"].get("mean"),
            "mean_coverage": values["strict_grid_coverage"].get("mean"),
            "mean_photometric_inliers": values["photometric_inliers"].get("mean"),
            "mean_patch_ncc_median": values["patch_ncc_median"].get("mean"),
        }

    result = {
        "source": {
            "path": str(args.csv.resolve()),
            "sha256": hashlib.sha256(args.csv.read_bytes()).hexdigest(),
            "rows": len(rows),
        },
        "methods": methods,
        "comparison": comparison,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(comparison, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
