#!/usr/bin/env python3
import argparse
import csv
import json
import math
import re
from pathlib import Path
from statistics import mean


SOURCE_COUNTS = {
    "094639": {"left": 996, "right": 996, "imu": 17940},
    "094814": {"left": 811, "right": 811, "imu": 14618},
    "095014": {"left": 716, "right": 715, "imu": 12880},
}


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    return values[round(fraction * (len(values) - 1))]


def pose(row):
    if int(row["pose_valid"]) == 0:
        return None
    translation = tuple(float(row[name]) for name in ("tcw_tx", "tcw_ty", "tcw_tz"))
    quaternion = tuple(float(row[name]) for name in ("tcw_qx", "tcw_qy", "tcw_qz", "tcw_qw"))
    if not all(math.isfinite(value) for value in translation + quaternion):
        return None
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return None
    x, y, z, w = (value / norm for value in quaternion)
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    center = tuple(
        -sum(rotation[row_index][column] * translation[row_index]
             for row_index in range(3))
        for column in range(3)
    )
    return center, (x, y, z, w)


def distance(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def angle_degrees(left, right):
    dot = abs(sum(a * b for a, b in zip(left, right)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def pose_step_metrics(indexed_poses):
    translation_steps = []
    rotation_steps = []
    translation_speeds = []
    rotation_speeds = []
    for left, right in zip(indexed_poses, indexed_poses[1:]):
        translation_step = distance(left[2][0], right[2][0])
        rotation_step = angle_degrees(left[2][1], right[2][1])
        translation_steps.append(translation_step)
        rotation_steps.append(rotation_step)
        delta_s = (int(right[1]["stamp_ns"]) - int(left[1]["stamp_ns"])) / 1e9
        if delta_s > 0.0:
            translation_speeds.append(translation_step / delta_s)
            rotation_speeds.append(rotation_step / delta_s)
    return {
        "step_count": len(translation_steps),
        "path_m": sum(translation_steps),
        "translation_step_m_p90": percentile(translation_steps, 0.90),
        "translation_step_m_max": max(translation_steps, default=None),
        "rotation_step_deg_p90": percentile(rotation_steps, 0.90),
        "rotation_step_deg_max": max(rotation_steps, default=None),
        "translation_speed_mps_p90": percentile(translation_speeds, 0.90),
        "translation_speed_mps_max": max(translation_speeds, default=None),
        "rotation_speed_degps_p90": percentile(rotation_speeds, 0.90),
        "rotation_speed_degps_max": max(rotation_speeds, default=None),
    }


def log_metrics(path):
    text = path.read_text(encoding="utf-8", errors="replace")
    drops = {}
    worker = re.search(
        r"worker stopped: frames=(\d+) imu=(\d+) dropped_left=(\d+) "
        r"dropped_right=(\d+) dropped_imu=(\d+) non_monotonic_imu=(\d+)",
        text,
    )
    if worker:
        names = ("worker_frames", "worker_imu", "dropped_left", "dropped_right",
                 "dropped_imu", "non_monotonic_imu")
        drops = dict(zip(names, (int(value) for value in worker.groups())))
    final = re.search(
        r"Final stereo(?:-inertial)? map: map_id=(\d+) atlas_maps=(\d+) "
        r"keyframes=(\d+) mappoints=(\d+) map_change_idx=(\d+)",
        text,
    )
    final_map = None
    if final:
        final_map = dict(zip(
            ("map_id", "atlas_maps", "keyframes", "mappoints", "map_change_idx"),
            (int(value) for value in final.groups()),
        ))
    return {
        "map_creation_messages": text.count("Creation of new map with id"),
        "imu_initialization_messages": text.count("IMU initialization completed"),
        "insufficient_motion_resets": text.count("Not enough motion for initializing"),
        "active_map_reset_lost_frames": [
            int(value) for value in re.findall(r"(\d+) Frames set to lost", text)
        ],
        "stereo_queue_overflow_warnings": text.count(
            "Stereo image queue overflow; dropping oldest left image"
        ),
        "unmatched_left_warnings": text.count("Dropping unmatched left image"),
        "unmatched_right_warnings": text.count("Dropping unmatched right image"),
        "runtime_error_markers": len(re.findall(
            r"segmentation|terminate called|fatal|exception", text, re.IGNORECASE
        )),
        "queue": drops,
        "final_map": final_map,
    }


def summarize_csv(path, bag_id, mode):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    states = [int(row["tracking_state"]) for row in rows]
    stamps = [int(row["stamp_ns"]) / 1e9 for row in rows]
    lost_flags = [state in (3, 4) for state in states]
    lost_segments = 0
    lost_duration = 0.0
    previous = False
    for index, lost in enumerate(lost_flags):
        if lost and not previous:
            lost_segments += 1
        if lost and index + 1 < len(stamps):
            lost_duration += max(0.0, stamps[index + 1] - stamps[index])
        previous = lost

    def floats(name):
        return [float(row[name]) for row in rows if math.isfinite(float(row[name]))]

    def ints(name):
        return [int(row[name]) for row in rows]

    indexed_poses = [
        (index, row, value)
        for index, row in enumerate(rows)
        if (value := pose(row)) is not None
    ]
    all_pose_steps = pose_step_metrics(indexed_poses)
    same_map_poses = []
    same_map_translation_steps = []
    same_map_rotation_steps = []
    for left, right in zip(indexed_poses, indexed_poses[1:]):
        if int(left[1]["map_id"]) != int(right[1]["map_id"]):
            continue
        same_map_poses.append((left, right))
        same_map_translation_steps.append(distance(left[2][0], right[2][0]))
        same_map_rotation_steps.append(angle_degrees(left[2][1], right[2][1]))
    track = floats("track_total_ms")
    valid_depths = ints("valid_depths")
    tracked_inliers = ints("tracked_inliers")
    deadline_ms = 1000.0 / 11.0
    source_pairs = min(SOURCE_COUNTS[bag_id]["left"], SOURCE_COUNTS[bag_id]["right"])
    result = {
        "csv": str(path),
        "mode": mode,
        "source": SOURCE_COUNTS[bag_id],
        "source_pairs": source_pairs,
        "callbacks": len(rows),
        "delivery_ratio": len(rows) / source_pairs,
        "effective_callback_hz": (
            (len(stamps) - 1) / (stamps[-1] - stamps[0])
            if len(stamps) > 1 and stamps[-1] > stamps[0] else None
        ),
        "timestamp_gap_s_p90": percentile(
            [right - left for left, right in zip(stamps, stamps[1:])], 0.90
        ),
        "timestamp_gap_s_max": max(
            [right - left for left, right in zip(stamps, stamps[1:])], default=None
        ),
        "state_counts": {str(state): states.count(state) for state in sorted(set(states))},
        "lost_frames": sum(lost_flags),
        "lost_segments": lost_segments,
        "lost_duration_s": lost_duration,
        "pose_valid_frames": sum(ints("pose_valid")),
        "valid_depths_mean": mean(valid_depths),
        "valid_depths_p10": percentile(valid_depths, 0.10),
        "tracked_inliers_mean": mean(tracked_inliers),
        "tracked_inliers_p10": percentile(tracked_inliers, 0.10),
        "track_ms_mean": mean(track),
        "track_ms_p90": percentile(track, 0.90),
        "track_ms_p99": percentile(track, 0.99),
        "deadline_miss_ratio": sum(value > deadline_ms for value in track) / len(track),
        "trajectory_path_m": all_pose_steps["path_m"],
        "translation_step_m_p90": all_pose_steps["translation_step_m_p90"],
        "translation_step_m_max": all_pose_steps["translation_step_m_max"],
        "rotation_step_deg_p90": all_pose_steps["rotation_step_deg_p90"],
        "rotation_step_deg_max": all_pose_steps["rotation_step_deg_max"],
        "translation_speed_mps_p90": all_pose_steps["translation_speed_mps_p90"],
        "translation_speed_mps_max": all_pose_steps["translation_speed_mps_max"],
        "rotation_speed_degps_p90": all_pose_steps["rotation_speed_degps_p90"],
        "rotation_speed_degps_max": all_pose_steps["rotation_speed_degps_max"],
        "same_map_pose_steps": {
            "step_count": len(same_map_poses),
            "translation_step_m_p90": percentile(same_map_translation_steps, 0.90),
            "translation_step_m_max": max(same_map_translation_steps, default=None),
            "rotation_step_deg_p90": percentile(same_map_rotation_steps, 0.90),
            "rotation_step_deg_max": max(same_map_rotation_steps, default=None),
        },
        "final_row_map": {
            key: int(rows[-1][key])
            for key in ("map_id", "atlas_maps", "keyframes", "mappoints", "map_change_idx")
        } if rows else None,
    }
    if mode == "stereo_inertial":
        initialized = [int(row["imu_initialized"]) for row in rows]
        first_initialized = next((index for index, value in enumerate(initialized) if value), None)
        last_uninitialized = max(
            (index for index, value in enumerate(initialized) if not value), default=None
        )
        final_initialized = (
            last_uninitialized + 1
            if last_uninitialized is not None and last_uninitialized + 1 < len(initialized)
            else (0 if initialized and initialized[0] else None)
        )
        stable_poses = (
            [value for value in indexed_poses if value[0] >= final_initialized]
            if final_initialized is not None else []
        )
        result["imu"] = {
            "samples_consumed": sum(ints("imu_samples")),
            "samples_per_callback_mean": mean(ints("imu_samples")),
            "initialized_rows": sum(initialized),
            "first_initialized_callback": (
                first_initialized + 1 if first_initialized is not None else None
            ),
            "initialization_reset_events": sum(
                left == 1 and right == 0
                for left, right in zip(initialized, initialized[1:])
            ),
            "final_initialized_callback": (
                final_initialized + 1 if final_initialized is not None else None
            ),
            "post_final_initialization_pose_steps": pose_step_metrics(stable_poses),
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    runs = {}
    for bag_id in SOURCE_COUNTS:
        runs[bag_id] = {}
        for mode in ("stereo", "stereo_inertial"):
            path = args.artifact_dir / f"f27_{bag_id}_{mode}.csv"
            log = path.with_suffix(".log")
            result = summarize_csv(path, bag_id, mode)
            result["log"] = log_metrics(log)
            runs[bag_id][mode] = result
    summary = {
        "playback_rate": 1.0,
        "camera_fps": 11.0,
        "ground_truth_available": False,
        "runs": runs,
    }
    args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
