#!/usr/bin/env python3
"""Analyze the F34 host-bag/RDK-X5 RGB-D mapping run."""

import argparse
import csv
import json
import math
import re
from pathlib import Path
from statistics import mean


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def stats(values):
    return {
        "mean": mean(values) if values else None,
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def closest_index(values, target):
    return min(range(len(values)), key=lambda index: abs(values[index] - target))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact_root", type=Path)
    args = parser.parse_args()
    root = args.artifact_root.resolve()
    run = next((root / "runs").glob("f34_f700_l5_r1"))

    timing = rows(run / "timing" / "rgbd_frame_timing.csv")
    mapping = rows(run / "timing" / "local_mapping_timing.csv")
    poses = rows(run / "robot_pose.csv")
    tracking = rows(run / "tracking_ok.csv")
    telemetry = rows(run / "telemetry.csv")
    log = (run / "slam.log").read_text(errors="replace")
    metadata = (run / "metadata.txt").read_text(errors="replace")

    floats = lambda key: [float(row[key]) for row in timing]
    callbacks = len(timing)
    tracking_states = {}
    for row in timing:
        state = row["state_after"]
        tracking_states[state] = tracking_states.get(state, 0) + 1

    source_sequences = [int(row["rgb_seq"]) for row in timing]
    sequence_gaps = [
        [source_sequences[index - 1], source_sequences[index]]
        for index in range(1, len(source_sequences))
        if source_sequences[index] != source_sequences[index - 1] + 1
    ]

    pose_stamps = [int(row["field.header.stamp"]) / 1e9 for row in poses]
    xyz = [
        (
            float(row["field.pose.position.x"]),
            float(row["field.pose.position.y"]),
            float(row["field.pose.position.z"]),
        )
        for row in poses
    ]
    pose_steps = [distance(xyz[index - 1], xyz[index]) for index in range(1, len(xyz))]
    path_length = sum(pose_steps)
    loop_kfs = [int(value) for value in re.findall(r"KeyFrame (\d+).*?\n\*Loop detected", log)]
    if not loop_kfs:
        # Production log places the marker after the current keyframe's BoW line.
        lines = log.splitlines()
        last_kf = None
        for line in lines:
            match = re.search(r"KeyFrame (\d+)", line)
            if match:
                last_kf = int(match.group(1))
            if "*Loop detected" in line and last_kf is not None:
                loop_kfs.append(last_kf)

    mapping_by_kf = {int(row["keyframe_id"]): row for row in mapping}
    image_stamps = [float(row["image_timestamp"]) for row in timing]
    loop_events = []
    for keyframe_id in loop_kfs:
        map_row = mapping_by_kf.get(keyframe_id)
        if not map_row:
            continue
        stamp = float(map_row["image_timestamp"])
        frame_index = closest_index(image_stamps, stamp)
        pose_index = closest_index(pose_stamps, stamp)
        first_pose = max(1, pose_index - 5)
        last_pose = min(len(xyz) - 1, pose_index + 5)
        nearby_pose_indices = list(range(first_pose, last_pose + 1))
        correction_pose_index = max(
            nearby_pose_indices, key=lambda index: pose_steps[index - 1]
        )
        loop_events.append(
            {
                "keyframe_id": keyframe_id,
                "image_timestamp": stamp,
                "frame_id": int(timing[frame_index]["frame_id"]),
                "rgb_seq": int(timing[frame_index]["rgb_seq"]),
                "position_m": list(xyz[pose_index]),
                "step_into_frame_m": pose_steps[pose_index - 1] if pose_index else 0.0,
                "correction_jump_m": pose_steps[correction_pose_index - 1],
                "correction_lag_frames": correction_pose_index - pose_index,
                "correction_lag_s": pose_stamps[correction_pose_index] - pose_stamps[pose_index],
                "correction_frame_id": int(timing[correction_pose_index]["frame_id"]),
                "correction_rgb_seq": int(timing[correction_pose_index]["rgb_seq"]),
            }
        )

    tracking_values = [int(row["field.data"]) for row in tracking]
    cpu = [float(row["node_cpu_pct"]) for row in telemetry]
    rss_mib = [float(row["node_rss_kb"]) / 1024 for row in telemetry]
    temperatures = [float(row["temp_mC"]) / 1000 for row in telemetry]
    local_total = [float(row["total_ms"]) for row in mapping]

    result = {
        "run": str(run),
        "source": {
            "bag": "/home/ywl/Project/ORB_SLAM3_DBoW3/2026-08-25-13-12-57.bag",
            "sha256": "1759c6fdf1369296919c7ec9a729741c4e084988fcddc57ed9ef8c91783e9655",
            "duration_s": 112.214042,
            "rgb_frames": 1247,
            "depth_frames": 1247,
            "exact_rgb_depth_pairs": 1247,
        },
        "configuration": {
            "orb_features": 700,
            "orb_levels": 5,
            "orb_scale_factor": 1.2,
            "fast_initial": 20,
            "fast_min": 7,
            "extractor_workers": 3,
            "playback_rate": 1.0,
            "bag_location": "host 192.168.1.3",
            "slam_location": "RDK-X5 192.168.1.10",
        },
        "delivery_and_tracking": {
            "callbacks": callbacks,
            "sequence_gaps": sequence_gaps,
            "state_after_counts": tracking_states,
            "tracking_ok_messages": len(tracking_values),
            "tracking_ok_true": sum(tracking_values),
            "tracking_ok_false": len(tracking_values) - sum(tracking_values),
            "slam_fail_log_lines": len(re.findall(r"fail|lost", log, flags=re.IGNORECASE)),
        },
        "timing_ms": {
            "orb_extract": stats(floats("orb_extract_ms")),
            "callback_total": stats(floats("callback_total_ms")),
            "callback_over_90_ms": sum(value > 90 for value in floats("callback_total_ms")),
            "header_age_growth": floats("header_age_ms")[-1] - floats("header_age_ms")[0],
            "header_age_range": max(floats("header_age_ms")) - min(floats("header_age_ms")),
            "local_mapping_total": stats(local_total),
        },
        "features": {
            "keypoints": stats(floats("keypoints")),
            "tracked_inliers_all_frames": stats(floats("tracked_inliers")),
            "tracked_inliers_after_initialization": stats(floats("tracked_inliers")[1:]),
        },
        "map": {
            "keyframes_processed": len(mapping),
            "last_keyframe_id": max(int(row["keyframe_id"]) for row in mapping),
            "loop_detections": len(loop_events),
            "loop_events": loop_events,
            "clean_shutdown": "Shutdown: All threads finished" in log,
        },
        "trajectory": {
            "published_poses": len(xyz),
            "path_length_m": path_length,
            "start_to_end_m": distance(xyz[0], xyz[-1]),
            "step_p99_m": percentile(pose_steps, 0.99),
            "step_max_m": max(pose_steps),
            "extent_m": {
                axis: max(point[index] for point in xyz) - min(point[index] for point in xyz)
                for index, axis in enumerate("xyz")
            },
        },
        "resources": {
            "samples": len(telemetry),
            "node_cpu_pct": stats(cpu),
            "node_rss_mib": stats(rss_mib),
            "temperature_c": stats(temperatures),
        },
        "integrity": {
            "play_exit_code": (run / "rosbag_play_exit_code.txt").read_text().strip(),
            "node_exit_code": (run / "slam_exit_code.txt").read_text().strip(),
            "loop_markers": log.count("*Loop detected"),
            "local_mapping_stop_markers": log.count("Local Mapping STOP"),
            "local_mapping_release_markers": log.count("Local Mapping RELEASE"),
            "metadata_contains_production_hashes": all(
                value in metadata
                for value in (
                    "a41fd8359313a0c78bb690743c08b964a18491b94c21d797019986666a99cc7b",
                    "51c3e4d720e5e8c22e63405b44c2b39a782e55464347016ff2bf1db9987a2860",
                    "0cbd4c6a210d095613e04d7f928ae89920bf9ae24b482875151e10b3968a1b10",
                )
            ),
        },
    }

    output_json = root / "summary.json"
    output_json.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(1, 2, figsize=(12, 5))
        axes[0].plot([point[0] for point in xyz], [point[1] for point in xyz], linewidth=1.2)
        for event in loop_events:
            axes[0].scatter(event["position_m"][0], event["position_m"][1], s=55, label=f'loop KF {event["keyframe_id"]}')
        axes[0].set_title("Published map-frame trajectory (X-Y)")
        axes[0].set_xlabel("X [m]")
        axes[0].set_ylabel("Y [m]")
        axes[0].axis("equal")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        distance_to_start = [distance(xyz[0], point) for point in xyz]
        axes[1].plot(range(len(xyz)), distance_to_start, linewidth=1.2)
        for event in loop_events:
            event_index = closest_index(pose_stamps, event["image_timestamp"])
            axes[1].axvline(event_index, linestyle="--", label=f'loop KF {event["keyframe_id"]}')
        axes[1].set_title("Distance from starting pose")
        axes[1].set_xlabel("source frame index")
        axes[1].set_ylabel("distance [m]")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        figure.tight_layout()
        figure.savefig(root / "trajectory_loop_events.png", dpi=160)
        plt.close(figure)
    except ImportError:
        pass

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
