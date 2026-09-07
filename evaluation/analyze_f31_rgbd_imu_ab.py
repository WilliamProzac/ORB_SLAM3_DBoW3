#!/usr/bin/env python3
"""Analyze the F31 RGB-D versus RGB-D-inertial board experiment."""

import argparse
import csv
import hashlib
import json
import math
import re
from collections import Counter
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


RUNS = {
    "rgbd_mapping": "f31_rgbd_mapping_131409_rate1_final",
    "rgbd_localization": "f31_rgbd_localization_133822_rate1_queue20_final",
    "rgbd_imu_mapping": "f31_rgbd_imu_mapping_131409_rate1",
    "rgbd_imu_localization": "f31_rgbd_imu_localization_133822_rate1_final2",
}


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_values(rows, field):
    result = []
    for row in rows:
        value = row.get(field, "")
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            result.append(number)
    return result


def stats(values):
    data = np.asarray(values, dtype=np.float64)
    if data.size == 0:
        return None
    return {
        "mean": float(np.mean(data)),
        "median": float(np.median(data)),
        "p95": float(np.percentile(data, 95)),
        "p99": float(np.percentile(data, 99)),
        "max": float(np.max(data)),
    }


def load_pose(path):
    rows = read_rows(path)
    stamps = np.asarray([int(row["field.header.stamp"]) for row in rows],
                        dtype=np.int64)
    xyz = np.asarray([
        [float(row["field.pose.position.x"]),
         float(row["field.pose.position.y"]),
         float(row["field.pose.position.z"])] for row in rows
    ], dtype=np.float64)
    quat = np.asarray([
        [float(row["field.pose.orientation.x"]),
         float(row["field.pose.orientation.y"]),
         float(row["field.pose.orientation.z"]),
         float(row["field.pose.orientation.w"])] for row in rows
    ], dtype=np.float64)
    return stamps, xyz, quat


def quat_to_rotation(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])


def rigid_alignment(source, target):
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    u, _, vt = np.linalg.svd(source_zero.T @ target_zero)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    aligned = (rotation @ source.T).T + translation
    return rotation, translation, aligned


def lookup_ok(ok_map, stamp_ns):
    if ok_map is None:
        return True
    stamp_us = (int(stamp_ns) + 500) // 1000
    return any(ok_map.get(stamp_us + offset, False)
               for offset in range(-2, 3))


def common_timestamp_comparison(first_pose, second_pose, second_ok=None):
    first_stamp, first_xyz, first_q = first_pose
    second_stamp, second_xyz, second_q = second_pose
    first_index = {stamp: index for index, stamp in enumerate(first_stamp)}
    pairs = [(first_index[stamp], index) for index, stamp in enumerate(second_stamp)
             if stamp in first_index and lookup_ok(second_ok, stamp)]
    first_ids = np.asarray([pair[0] for pair in pairs], dtype=np.int64)
    second_ids = np.asarray([pair[1] for pair in pairs], dtype=np.int64)
    rotation, translation, aligned = rigid_alignment(
        second_xyz[second_ids], first_xyz[first_ids])
    errors = np.linalg.norm(aligned - first_xyz[first_ids], axis=1)
    angles = []
    for first_id, second_id in pairs:
        relative = (quat_to_rotation(first_q[first_id]).T @ rotation @
                    quat_to_rotation(second_q[second_id]))
        cosine = min(1.0, max(-1.0, (np.trace(relative) - 1.0) / 2.0))
        angles.append(math.degrees(math.acos(cosine)))
    return {
        "common_pose_timestamps": len(pairs),
        "se3_translation_alignment_m": translation.tolist(),
        "se3_rotation_alignment": rotation.tolist(),
        "aligned_position_difference_m": stats(errors),
        "aligned_orientation_difference_deg": stats(angles),
    }, aligned, first_xyz[first_ids]


def nearest_path_comparison(mapping_pose, localization_pose,
                            localization_ok=None, mapping_ok=None):
    map_stamp, map_xyz, map_q = mapping_pose
    loc_stamp, loc_xyz, loc_q = localization_pose
    if mapping_ok is not None:
        mask = np.asarray([lookup_ok(mapping_ok, stamp)
                           for stamp in map_stamp], dtype=bool)
        map_xyz = map_xyz[mask]
        map_q = map_q[mask]
    if localization_ok is not None:
        mask = np.asarray([lookup_ok(localization_ok, stamp)
                           for stamp in loc_stamp], dtype=bool)
        loc_xyz = loc_xyz[mask]
        loc_q = loc_q[mask]
    distances = np.linalg.norm(loc_xyz[:, None, :] - map_xyz[None, :, :], axis=2)
    nearest_ids = np.argmin(distances, axis=1)
    nearest = distances[np.arange(len(loc_xyz)), nearest_ids]
    angles = []
    for index, map_index in enumerate(nearest_ids):
        relative = (quat_to_rotation(map_q[map_index]).T @
                    quat_to_rotation(loc_q[index]))
        cosine = min(1.0, max(-1.0, (np.trace(relative) - 1.0) / 2.0))
        angles.append(math.degrees(math.acos(cosine)))
    return {
        "localization_poses_compared": len(loc_xyz),
        "nearest_mapping_path_distance_m": stats(nearest),
        "within_0_10_m_percent": 100.0 * float(np.mean(nearest <= 0.10)),
        "within_0_25_m_percent": 100.0 * float(np.mean(nearest <= 0.25)),
        "within_0_50_m_percent": 100.0 * float(np.mean(nearest <= 0.50)),
        "nearest_orientation_difference_deg": stats(angles),
        "nondecreasing_nearest_index_percent":
            100.0 * float(np.mean(np.diff(nearest_ids) >= 0)),
    }, nearest


def pose_state_map(timing, require_imu=False):
    return {int(round(float(row["image_timestamp"]) * 1e6)):
            row["state_after"] == "2" and
            (not require_imu or row.get("imu_initialized") == "1")
            for row in timing}


def summarize_trajectory(pose, ok=None):
    stamps, xyz, _ = pose
    if ok is not None:
        mask = np.asarray([lookup_ok(ok, stamp)
                           for stamp in stamps], dtype=bool)
        xyz = xyz[mask]
    steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    return {
        "poses": len(xyz),
        "path_length_m": float(np.sum(steps)),
        "extent_m": np.ptp(xyz, axis=0).tolist(),
        "end_to_start_m": float(np.linalg.norm(xyz[-1] - xyz[0])),
        "step_m": stats(steps),
    }


def parse_saved_map(log):
    match = re.search(r"Map (\d+) has (\d+) KFs.*?MPs=(\d+), KFs=(\d+)",
                      log, re.S)
    if not match:
        return None
    return {
        "map_id": int(match.group(1)),
        "keyframes": int(match.group(2)),
        "map_points": int(match.group(3)),
        "presaved_keyframes": int(match.group(4)),
    }


def summarize_run(run_dir, expected_frames):
    timing = read_rows(run_dir / "timing/rgbd_frame_timing.csv")
    local_mapping = read_rows(run_dir / "timing/local_mapping_timing.csv")
    telemetry = read_rows(run_dir / "telemetry.csv")
    log = (run_dir / "slam.log").read_text(errors="replace")
    pose = load_pose(run_dir / "robot_pose.csv")
    ok_map = pose_state_map(timing)
    state_counts = Counter(row["state_after"] for row in timing)
    ok_rows = [row for row in timing if row["state_after"] == "2"]
    map_files = list(run_dir.glob("*.osa"))
    callback = finite_values(timing, "callback_total_ms")
    result = {
        "run": run_dir.name,
        "frames": len(timing),
        "expected_exact_pairs": expected_frames,
        "delivery_percent": 100.0 * len(timing) / expected_frames,
        "state_after": dict(state_counts),
        "ok_percent": 100.0 * state_counts.get("2", 0) / len(timing),
        "tracked_inliers_ok": stats(finite_values(ok_rows, "tracked_inliers")),
        "valid_depths": stats(finite_values(timing, "valid_depths")),
        "orb_extract_ms": stats(finite_values(timing, "orb_extract_ms")),
        "track_call_ms": stats(finite_values(timing, "track_call_ms")),
        "tracking_stages_ms": {
            field: stats(finite_values(timing, field))
            for field in ("initialization_ms", "reference_kf_ms",
                          "motion_model_ms", "relocalization_ms",
                          "local_map_ms")
        },
        "callback_total_ms": stats(callback),
        "callback_over_90_909_ms_percent":
            100.0 * sum(value > 90.909 for value in callback) / len(callback),
        "queue_wait_ms": stats(finite_values(timing, "queue_wait_ms")),
        "imu_samples": stats(finite_values(timing, "imu_samples")),
        "last_imu_delta_ms": stats(finite_values(timing, "last_imu_delta_ms")),
        "imu_initialized_rows": sum(row.get("imu_initialized") == "1"
                                    for row in timing),
        "inserted_keyframes": sum(int(row["inserted_keyframe"])
                                  for row in timing),
        "local_mapping_rows": len(local_mapping),
        "resources": {
            "cpu_percent": stats(finite_values(telemetry, "cpu_pct")),
            "rss_kB": stats(finite_values(telemetry, "rss_kB")),
            "cpu_temperature_C": stats([
                value / 1000.0
                for value in finite_values(telemetry, "cpu_temp_mC")
            ]),
        },
        "trajectory": summarize_trajectory(pose, ok_map),
        "relocalized_count": log.count("Relocalized!!"),
        "accepted_relocalization_inliers": [int(value) for value in
                                             re.findall(
            r"Relocalization: accepted with (\d+) inliers", log)],
        "new_map_ids": re.findall(r"Creation of new map with id:\s*(\d+)", log),
        "saved_map": parse_saved_map(log),
        "rosbag_play_exit": (run_dir / "rosbag_play_exit_code.txt").read_text().strip(),
        "slam_exit": (run_dir / "slam_exit_code.txt").read_text().strip(),
    }
    if map_files:
        result["atlas"] = {
            "name": map_files[0].name,
            "bytes": map_files[0].stat().st_size,
            "sha256": sha256(map_files[0]),
        }
    return result, timing, pose, ok_map


def plot_results(poses, nearest_rgbd, nearest_imu, aligned_map, target_map,
                 output):
    figure, axes = plt.subplots(1, 3, figsize=(17, 5))
    for key, label in [("rgbd_mapping", "RGB-D map"),
                       ("rgbd_imu_mapping", "RGB-D+IMU map")]:
        xyz = poses[key][1]
        axes[0].plot(xyz[:, 0], xyz[:, 1], label=label)
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()
    axes[0].set_title("Raw mapping trajectories")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")

    axes[1].plot(target_map[:, 0], target_map[:, 1], label="RGB-D")
    axes[1].plot(aligned_map[:, 0], aligned_map[:, 1], label="RGB-D+IMU aligned")
    axes[1].axis("equal")
    axes[1].grid(True)
    axes[1].legend()
    axes[1].set_title("Same-bag SE(3) alignment")

    axes[2].plot(nearest_rgbd, label="RGB-D")
    axes[2].plot(nearest_imu, label="RGB-D+IMU (OK frames)")
    axes[2].axhline(0.25, color="orange", linestyle="--")
    axes[2].axhline(0.50, color="red", linestyle="--")
    axes[2].grid(True)
    axes[2].legend()
    axes[2].set_title("Localization to own mapping path")
    axes[2].set_xlabel("localization pose index")
    axes[2].set_ylabel("nearest distance (m)")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", type=Path)
    args = parser.parse_args()
    board_results = args.artifacts / "board_results"
    summaries = {}
    timings = {}
    poses = {}
    ok_maps = {}
    for key, name in RUNS.items():
        expected = 670 if "mapping" in key else 803
        summaries[key], timings[key], poses[key], ok_maps[key] = summarize_run(
            board_results / name, expected)

    imu_mapping_ready = pose_state_map(timings["rgbd_imu_mapping"], True)
    map_ab, aligned_map, target_map = common_timestamp_comparison(
        poses["rgbd_mapping"], poses["rgbd_imu_mapping"], imu_mapping_ready)
    loc_ab, _, _ = common_timestamp_comparison(
        poses["rgbd_localization"], poses["rgbd_imu_localization"],
        ok_maps["rgbd_imu_localization"])
    rgbd_repeat, nearest_rgbd = nearest_path_comparison(
        poses["rgbd_mapping"], poses["rgbd_localization"])
    imu_repeat, nearest_imu = nearest_path_comparison(
        poses["rgbd_imu_mapping"], poses["rgbd_imu_localization"],
        ok_maps["rgbd_imu_localization"], imu_mapping_ready)

    pre_fix_name = "f31_rgbd_imu_localization_133822_rate1"
    pre_fix, _, _, _ = summarize_run(board_results / pre_fix_name, 803)
    output = {
        "test_conditions": {
            "mapping_bag": "2026-08-20-13-14-09.bag",
            "localization_bag": "2026-08-20-13-38-22.bag",
            "play_rate": 1.0,
            "slam_cpuset": "2-7",
            "live_camera_driver": False,
            "orb_parallel_workers": 3,
            "rgb_depth_queue_capacity": 20,
            "imu_time_shift_seconds": 0.016355351,
            "absolute_ground_truth": False,
        },
        "runs": summaries,
        "comparisons": {
            "same_mapping_bag_rgbd_vs_rgbd_imu": map_ab,
            "same_localization_bag_rgbd_vs_rgbd_imu_ok_frames": loc_ab,
            "rgbd_localization_repeatability": rgbd_repeat,
            "rgbd_imu_localization_repeatability_ok_frames": imu_repeat,
        },
        "debug_evidence": {
            "pre_fix_rgbd_imu_localization": pre_fix,
        },
    }
    (args.artifacts / "summary.json").write_text(
        json.dumps(output, indent=2, ensure_ascii=False) + "\n")
    plot_results(poses, nearest_rgbd, nearest_imu, aligned_map, target_map,
                 args.artifacts / "trajectory_comparison.png")
    print(json.dumps(output, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
