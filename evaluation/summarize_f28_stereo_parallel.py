#!/usr/bin/env python3
import argparse
import csv
import json
import math
import re
from pathlib import Path

NAME = re.compile(r"f28_(\d{6})_(stereo(?:_inertial)?)_w([1-4])_([^/]+)\.csv$")
SOURCE_PAIRS = {"094639": 996, "094814": 811, "095014": 715}

def percentile(values, q):
    values = sorted(values)
    if not values:
        return None
    index = (len(values) - 1) * q
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return values[lower]
    return values[lower] * (upper - index) + values[upper] * (index - lower)

def numeric(rows, key):
    result = []
    for row in rows:
        try:
            result.append(float(row[key]))
        except (KeyError, TypeError, ValueError):
            pass
    return result

def timing(rows, key):
    values = numeric(rows, key)
    return {
        "mean": sum(values) / len(values) if values else None,
        "p50": percentile(values, 0.50),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
    }

def summarize_csv(path):
    if path.name.endswith("_telemetry.csv"):
        return None
    match = NAME.match(path.name)
    if not match:
        return None
    bag, mode, workers, repeat = match.groups()
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    stamps = numeric(rows, "stamp_ns")
    duration = (max(stamps) - min(stamps)) / 1e9 if len(stamps) > 1 else 0.0
    states = [int(float(value)) for value in numeric(rows, "tracking_state")]
    final = rows[-1] if rows else {}
    result = {
        "bag": bag,
        "mode": mode,
        "workers": int(workers),
        "repeat": repeat,
        "csv": str(path),
        "callbacks": len(rows),
        "source_pairs": SOURCE_PAIRS[bag],
        "delivery_ratio": len(rows) / SOURCE_PAIRS[bag],
        "effective_hz": (len(rows) - 1) / duration if duration > 0 else None,
        "lost_frames": sum(state == 3 for state in states),
        "pose_valid_frames": sum(int(float(row.get("pose_valid", 0))) for row in rows),
        "orb_wall_ms": timing(rows, "feature_extract_ms"),
        "stereo_match_ms": timing(rows, "stereo_match_ms"),
        "frontend_ms": timing(rows, "frontend_ms"),
        "track_total_ms": timing(rows, "track_total_ms"),
        "tracked_inliers": timing(rows, "tracked_inliers"),
        "valid_depths": timing(rows, "valid_depths"),
        "final": {key: int(float(final[key])) for key in
                  ("atlas_maps", "keyframes", "mappoints", "map_change_idx")
                  if final.get(key, "") != ""},
    }
    if mode == "stereo_inertial":
        initialized = numeric(rows, "imu_initialized")
        result["imu_initialized_rows"] = sum(value > 0 for value in initialized)
        result["imu_samples"] = sum(numeric(rows, "imu_samples"))
    telemetry = path.with_name(path.stem + "_telemetry.csv")
    if telemetry.exists():
        with telemetry.open(newline="") as stream:
            telemetry_rows = list(csv.DictReader(stream))
        result["telemetry"] = {
            "node_cpu_mean": timing(telemetry_rows, "node_cpu_pct")["mean"],
            "node_cpu_p95": timing(telemetry_rows, "node_cpu_pct")["p95"],
            "node_rss_kb_max": max(numeric(telemetry_rows, "node_rss_kb"), default=None),
            "node_threads_max": max(numeric(telemetry_rows, "node_threads"), default=None),
            "camera_cpu_mean": timing(telemetry_rows, "camera_cpu_pct")["mean"],
            "temp_mC_max": max(numeric(telemetry_rows, "temp_mC"), default=None),
            "max_freq_kHz_min": min(numeric(telemetry_rows, "max_freq_kHz"), default=None),
        }
    return result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    runs = [summary for path in sorted(args.artifacts.glob("f28_*.csv"))
            if (summary := summarize_csv(path)) is not None]
    output = {"source_pairs": SOURCE_PAIRS, "runs": runs}
    text = json.dumps(output, indent=2, ensure_ascii=False)
    if args.output:
        args.output.write_text(text + "\n")
    print(text)

if __name__ == "__main__":
    main()
