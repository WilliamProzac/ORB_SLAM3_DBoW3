#!/usr/bin/env python3
"""Summarize the F33 RGB-D ORB feature/level matrix."""

import argparse
import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


SOURCE_PAIRS = 888
SOURCE_PERIOD_MS = 90.0


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def distribution(values):
    data = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(data)),
        "p95": float(np.percentile(data, 95)),
        "p99": float(np.percentile(data, 99)),
        "max": float(np.max(data)),
    }


def parse_pose(rows):
    xyz = np.asarray([
        [float(row["field.pose.position.x"]),
         float(row["field.pose.position.y"]),
         float(row["field.pose.position.z"])]
        for row in rows
    ], dtype=np.float64)
    quat = np.asarray([
        [float(row["field.pose.orientation.x"]),
         float(row["field.pose.orientation.y"]),
         float(row["field.pose.orientation.z"]),
         float(row["field.pose.orientation.w"])]
        for row in rows
    ], dtype=np.float64)
    stamps = np.asarray([
        float(row["field.header.stamp"]) * 1.0e-9 for row in rows
    ], dtype=np.float64)
    return xyz, quat, stamps


def quaternion_delta_deg(first, second):
    first = first / np.linalg.norm(first)
    second = second / np.linalg.norm(second)
    return math.degrees(2.0 * math.acos(min(1.0, abs(float(np.dot(first, second))))))


def summarize_run(run_dir):
    match = re.fullmatch(r"f33_f(\d+)_l(\d+)_r(\d+)", run_dir.name)
    if not match:
        raise ValueError(f"unexpected run directory: {run_dir.name}")
    features, levels, repeat = map(int, match.groups())
    frame_rows = read_csv(run_dir / "timing/rgbd_frame_timing.csv")
    local_rows = read_csv(run_dir / "timing/local_mapping_timing.csv")
    pose_rows = read_csv(run_dir / "robot_pose.csv")
    tracking_rows = read_csv(run_dir / "tracking_ok.csv")
    telemetry_rows = read_csv(run_dir / "telemetry.csv")
    log = (run_dir / "slam.log").read_text(errors="replace")

    values = lambda key: [float(row[key]) for row in frame_rows]
    xyz, quat, pose_stamps = parse_pose(pose_rows)
    steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    frame_by_source = {int(row["rgb_seq"]): row for row in frame_rows}
    repeat_pose = None
    if len(pose_stamps):
        first_source = min(frame_by_source, key=lambda value: abs(value - 199))
        second_source = min(frame_by_source, key=lambda value: abs(value - 642))
        first_stamp = float(frame_by_source[first_source]["image_timestamp"])
        second_stamp = float(frame_by_source[second_source]["image_timestamp"])
        first_index = int(np.argmin(np.abs(pose_stamps - first_stamp)))
        second_index = int(np.argmin(np.abs(pose_stamps - second_stamp)))
        repeat_pose = {
            "first_source_frame": first_source,
            "second_source_frame": second_source,
            "translation_m": float(np.linalg.norm(xyz[second_index] - xyz[first_index])),
            "rotation_deg": quaternion_delta_deg(quat[first_index], quat[second_index]),
        }

    callback = values("callback_total_ms")
    header_age = values("header_age_ms")
    tracked_inliers = values("tracked_inliers")
    state_counts = dict(Counter(row["state_after"] for row in frame_rows))
    true_tracking = sum(row.get("field.data", "").lower() in {"1", "true"}
                        for row in tracking_rows)
    false_tracking = sum(row.get("field.data", "").lower() in {"0", "false"}
                         for row in tracking_rows)

    cpu = [float(row["node_cpu_pct"]) for row in telemetry_rows
           if row.get("node_cpu_pct", "")]
    rss = [float(row["node_rss_kb"]) for row in telemetry_rows
           if row.get("node_rss_kb", "")]
    temperature = [float(row["temp_mC"]) for row in telemetry_rows
                   if row.get("temp_mC", "")]
    loop_lines = [line for line in log.splitlines()
                  if "*Loop detected" in line]
    intermediate_loop_lines = [line for line in log.splitlines()
                               if "PR: Loop detected with Reffine Sim3" in line]
    map_ids = re.findall(r"Creation of new map with id:\s*(\d+)", log)
    error_lines = [line for line in log.splitlines() if re.search(
        r"runtime error|segmentation fault|terminate called|cv_bridge exception|"
        r"Unsupported depth|size mismatch", line, re.IGNORECASE)]

    return {
        "run": run_dir.name,
        "features": features,
        "levels": levels,
        "repeat": repeat,
        "callbacks": len(frame_rows),
        "delivery_percent": 100.0 * len(frame_rows) / SOURCE_PAIRS,
        "rgb_sequence_gaps": sum(int(row["rgb_seq_gap"]) for row in frame_rows),
        "states_after": state_counts,
        "tracking_status_true": true_tracking,
        "tracking_status_false": false_tracking,
        "keypoints_mean": float(np.mean(values("keypoints"))),
        "tracked_inliers": distribution(tracked_inliers),
        "orb_extract_ms": distribution(values("orb_extract_ms")),
        "callback_total_ms": distribution(callback),
        "callbacks_over_90_ms": sum(value > SOURCE_PERIOD_MS for value in callback),
        "header_age_growth_ms": header_age[-1] - header_age[0],
        "header_age_range_ms": max(header_age) - min(header_age),
        "inserted_keyframes": sum(int(row["inserted_keyframe"])
                                  for row in frame_rows),
        "local_mapping_rows": len(local_rows),
        "final_keyframe_id": int(local_rows[-1]["keyframe_id"]),
        "map_ids_created": map_ids,
        "explicit_loop_events": len(loop_lines),
        "intermediate_loop_events": len(intermediate_loop_lines),
        "runtime_error_lines": error_lines,
        "trajectory": {
            "poses": len(pose_rows),
            "path_length_m": float(np.sum(steps)),
            "end_to_start_m": float(np.linalg.norm(xyz[-1] - xyz[0])),
            "max_step_m": float(np.max(steps)),
            "p99_step_m": float(np.percentile(steps, 99)),
            "extent_xyz_m": (np.max(xyz, axis=0) - np.min(xyz, axis=0)).tolist(),
            "repeat_199_642": repeat_pose,
        },
        "resources": {
            "cpu_percent_max": max(cpu),
            "rss_mib_max": max(rss) / 1024.0,
            "temperature_c_max": max(temperature) / 1000.0,
        },
        "rosbag_play_exit": int((run_dir / "rosbag_play_exit_code.txt").read_text()),
        "slam_exit": int((run_dir / "slam_exit_code.txt").read_text()),
        "clean_shutdown": "Shutdown: All threads finished." in log,
    }, xyz


def aggregate(runs):
    grouped = defaultdict(list)
    for run in runs:
        grouped[f"{run['features']}x{run['levels']}"] .append(run)
    result = {}
    for setting, items in grouped.items():
        result[setting] = {
            "runs": len(items),
            "callbacks_min_max": [min(item["callbacks"] for item in items),
                                  max(item["callbacks"] for item in items)],
            "gaps_total": sum(item["rgb_sequence_gaps"] for item in items),
            "callback_mean_ms": float(np.mean([
                item["callback_total_ms"]["mean"] for item in items])),
            "callback_p95_ms": float(np.mean([
                item["callback_total_ms"]["p95"] for item in items])),
            "orb_mean_ms": float(np.mean([
                item["orb_extract_ms"]["mean"] for item in items])),
            "inliers_mean": float(np.mean([
                item["tracked_inliers"]["mean"] for item in items])),
            "keyframes_mean": float(np.mean([
                item["local_mapping_rows"] for item in items])),
            "loop_events": sum(item["explicit_loop_events"] for item in items),
        }
    return result


def plot_results(runs, trajectories, output_dir):
    labels = [run["run"].replace("f33_f", "") for run in runs]
    x = np.arange(len(runs))
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    axes[0, 0].bar(x, [run["callback_total_ms"]["p95"] for run in runs])
    axes[0, 0].axhline(SOURCE_PERIOD_MS, color="red", linestyle="--", label="90 ms")
    axes[0, 0].set_ylabel("callback P95 (ms)")
    axes[0, 0].legend()
    axes[0, 1].bar(x, [run["callbacks"] for run in runs])
    axes[0, 1].axhline(SOURCE_PAIRS, color="green", linestyle="--")
    axes[0, 1].set_ylabel("delivered callbacks")
    axes[1, 0].bar(x, [run["tracked_inliers"]["mean"] for run in runs])
    axes[1, 0].set_ylabel("mean tracked inliers")
    axes[1, 1].bar(x, [run["local_mapping_rows"] for run in runs])
    axes[1, 1].set_ylabel("keyframes / LM rows")
    for axis in axes.flat:
        axis.set_xticks(x)
        axis.set_xticklabels(labels, rotation=35, ha="right")
        axis.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "parameter_matrix.png", dpi=160)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(8, 6))
    for name, xyz in trajectories.items():
        axis.plot(xyz[:, 0], xyz[:, 1], label=name.replace("f33_f", ""), alpha=0.8)
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_title("F33 RGB-D trajectories (internal SLAM frame)")
    axis.axis("equal")
    axis.grid(True)
    axis.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(output_dir / "trajectory_overlay.png", dpi=160)
    plt.close(fig)


def write_report(output_dir, runs, aggregated):
    lines = [
        "# F33 RGB-D ORB feature/level matrix",
        "",
        "## Conclusion",
        "",
        "`700 features / 5 levels` is the recommended 11 Hz operating point for this bag. "
        "Both cold starts delivered 888/888 pairs in state 2 with one map and no sequence gap. "
        "It raised mean tracked inliers by about 14% over the 600x5 matrix baseline while keeping "
        "callback P95 near 83--84 ms without sustained header-age growth.",
        "",
        "No tested setting produced an explicit loop event. Increasing to 800--1200 features raised "
        "local tracking inliers but introduced repeatable input loss and did not solve loop confirmation. "
        "The loop issue therefore cannot be attributed only to insufficient ORB feature count or pyramid depth.",
        "",
        "## Runs",
        "",
        "| Run | callbacks | gaps | state 2 | ORB mean/P95 ms | callback mean/P95/max ms | >90 ms | mean inliers | KFs | loops |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for run in runs:
        lines.append(
            f"| {run['run']} | {run['callbacks']} | {run['rgb_sequence_gaps']} | "
            f"{run['states_after'].get('2', 0)} | "
            f"{run['orb_extract_ms']['mean']:.2f}/{run['orb_extract_ms']['p95']:.2f} | "
            f"{run['callback_total_ms']['mean']:.2f}/{run['callback_total_ms']['p95']:.2f}/"
            f"{run['callback_total_ms']['max']:.2f} | {run['callbacks_over_90_ms']} | "
            f"{run['tracked_inliers']['mean']:.1f} | {run['local_mapping_rows']} | "
            f"{run['explicit_loop_events']} |"
        )
    lines.extend([
        "",
        "## Selection notes",
        "",
        "- `600x5` remains the conservative maximum-margin setting, but produced fewer inliers and more keyframes.",
        "- `700x5` was repeatably complete and improved local constraints with acceptable P95 latency.",
        "- `800x5` lost 2 and 3 pairs in its two runs; `800x6` lost 4, so adding a sixth level did not justify its cost.",
        "- `1000x6` and `1200x8` lost 18 and 83 pairs respectively and are rejected for 11 Hz use.",
        "- Every run used the same production binary/library/vocabulary, host-side 1.0x playback, LAN transport, "
        "viewer disabled, parallel workers 3, and a cold in-memory map.",
        "- The bag has no external ground truth; trajectory metrics are internal consistency evidence, not ATE/RPE.",
        "",
        "## Evidence",
        "",
        "- `summary.json`",
        "- `parameter_matrix.png`",
        "- `trajectory_overlay.png`",
        "- `configs/` and `runs/`",
    ])
    (output_dir / "report.md").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", type=Path)
    args = parser.parse_args()
    run_dirs = sorted(path for path in (args.artifacts / "runs").iterdir()
                      if path.is_dir() and path.name.startswith("f33_f"))
    runs = []
    trajectories = {}
    for run_dir in run_dirs:
        summary, xyz = summarize_run(run_dir)
        runs.append(summary)
        trajectories[run_dir.name] = xyz
    runs.sort(key=lambda item: (item["features"], item["levels"], item["repeat"]))
    aggregated = aggregate(runs)
    result = {
        "feature": "F33",
        "bag": "2026-08-25-09-56-06.bag",
        "source_exact_pairs": SOURCE_PAIRS,
        "playback_rate": 1.0,
        "transport": "host ROS master/bag to RDK X5 SLAM over LAN",
        "recommendation": {"nFeatures": 700, "nLevels": 5,
                           "scaleFactor": 1.2, "parallelWorkers": 3},
        "runs": runs,
        "aggregated": aggregated,
        "limitations": [
            "No external trajectory ground truth; ATE/RPE are unavailable.",
            "No setting produced an explicit loop event.",
            "The live camera driver was not running during bag-only LAN replay.",
        ],
    }
    (args.artifacts / "summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    plot_results(runs, trajectories, args.artifacts)
    write_report(args.artifacts, runs, aggregated)
    print(json.dumps(result["aggregated"], indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
