#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from summarize_f27_stereo_vs_imu import pose


MODES = ("stereo", "stereo_inertial")
MODE_LABELS = {"stereo": "Stereo", "stereo_inertial": "Stereo + IMU"}
COLORS = {"stereo": "#4472c4", "stereo_inertial": "#ed7d31"}


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def save_health(summary, output):
    bag_ids = list(summary["runs"])
    labels = [f"{bag[:2]}:{bag[2:4]}:{bag[4:]}" for bag in bag_ids]
    x = list(range(len(bag_ids)))
    width = 0.36
    metrics = (
        ("Frame delivery (%)", lambda run: 100.0 * run["delivery_ratio"]),
        ("Tracking-loss duration (s)", lambda run: run["lost_duration_s"]),
        ("Final Atlas map count", lambda run: run["final_row_map"]["atlas_maps"]),
        ("Final active-map points", lambda run: run["final_row_map"]["mappoints"]),
    )
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for axis, (title, getter) in zip(axes.flat, metrics):
        for offset, mode in ((-width / 2, "stereo"), (width / 2, "stereo_inertial")):
            values = [getter(summary["runs"][bag][mode]) for bag in bag_ids]
            bars = axis.bar(
                [value + offset for value in x], values, width,
                label=MODE_LABELS[mode], color=COLORS[mode],
            )
            if title.startswith("Frame"):
                value_format = "{:.1f}"
            elif title.startswith("Tracking"):
                value_format = "{:.2f}"
            else:
                value_format = "{:.0f}"
            for bar, value in zip(bars, values):
                axis.annotate(
                    value_format.format(value),
                    (bar.get_x() + bar.get_width() / 2.0, bar.get_height()),
                    xytext=(0, 3), textcoords="offset points",
                    ha="center", va="bottom", fontsize=8,
                )
        axis.set_title(title)
        axis.set_xticks(x)
        axis.set_xticklabels(labels)
        axis.grid(axis="y", alpha=0.25)
    axes[0, 0].legend()
    figure.suptitle("RDK X5 ORB-SLAM3 mapping at rosbag rate 1.0")
    figure.savefig(output, dpi=160)
    plt.close(figure)


def trajectory_segments(rows, mode, final_initialized):
    segments = []
    current = []
    previous_index = None
    previous_map = None
    previous_stable = None
    origin = None
    for index, row in enumerate(rows):
        value = pose(row)
        if value is None:
            if current:
                segments.append((previous_map, previous_stable, current))
                current = []
            previous_index = None
            continue
        center = value[0]
        if origin is None:
            origin = center
        center = tuple(center[axis] - origin[axis] for axis in range(3))
        map_id = int(row["map_id"])
        stable = mode == "stereo" or index >= final_initialized
        boundary = (
            previous_index is None
            or index != previous_index + 1
            or map_id != previous_map
            or stable != previous_stable
        )
        if boundary and current:
            segments.append((previous_map, previous_stable, current))
            current = []
        current.append(center)
        previous_index = index
        previous_map = map_id
        previous_stable = stable
    if current:
        segments.append((previous_map, previous_stable, current))
    return segments


def save_trajectories(summary, artifact_dir, output):
    bag_ids = list(summary["runs"])
    figure, axes = plt.subplots(3, 2, figsize=(11, 13), constrained_layout=True)
    map_colors = ("#4472c4", "#70ad47", "#a5a5a5", "#ffc000")
    for row_index, bag_id in enumerate(bag_ids):
        for column, mode in enumerate(MODES):
            axis = axes[row_index, column]
            run = summary["runs"][bag_id][mode]
            rows = load_rows(artifact_dir / f"f27_{bag_id}_{mode}.csv")
            final_initialized = 0
            if mode == "stereo_inertial":
                final_initialized = run["imu"]["final_initialized_callback"] - 1
            horizontal_axis = 2 if mode == "stereo" else 1
            horizontal_name = "Z" if mode == "stereo" else "Y"
            labels_seen = set()
            for map_id, stable, points in trajectory_segments(rows, mode, final_initialized):
                label = f"map {map_id}"
                color = map_colors[map_id % len(map_colors)]
                alpha = 1.0
                if mode == "stereo_inertial" and not stable:
                    label = "IMU startup/reset"
                    color = "#a5a5a5"
                    alpha = 0.65
                if label in labels_seen:
                    label = None
                else:
                    labels_seen.add(label)
                axis.plot(
                    [point[0] for point in points],
                    [point[horizontal_axis] for point in points],
                    color=color, alpha=alpha, linewidth=1.3, label=label,
                )
            axis.set_title(
                f"{bag_id[:2]}:{bag_id[2:4]}:{bag_id[4:]}  {MODE_LABELS[mode]}\n"
                f"maps={run['final_row_map']['atlas_maps']}, "
                f"lost={run['lost_frames']} frames"
            )
            axis.set_xlabel("relative camera-center X (m)")
            axis.set_ylabel(f"relative camera-center {horizontal_name} (m)")
            axis.axis("equal")
            axis.grid(alpha=0.25)
            axis.legend(fontsize=8)
    figure.suptitle(
        "Estimated ground-plane trajectories: Stereo X-Z; IMU gravity-aligned X-Y\n"
        "No ground truth; modes use independent map frames",
        fontsize=14,
    )
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact_dir", type=Path)
    args = parser.parse_args()
    summary = json.loads((args.artifact_dir / "summary.json").read_text())
    health = args.artifact_dir / "mapping_health.png"
    trajectories = args.artifact_dir / "trajectories_xz.png"
    save_health(summary, health)
    save_trajectories(summary, args.artifact_dir, trajectories)
    print(health)
    print(trajectories)


if __name__ == "__main__":
    main()
