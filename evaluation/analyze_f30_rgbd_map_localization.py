#!/usr/bin/env python3
"""Summarize the F30 RGB-D map-save/load and relocalization validation."""

import argparse
import csv
import hashlib
import json
import math
import re
from collections import Counter
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import rosbag

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stats(values):
    values = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(values)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def bag_summary(path):
    image_topics = ["/camera/infra1/image_raw", "/camera/depth/image_raw"]
    samples = {topic: [] for topic in image_topics}
    with rosbag.Bag(str(path)) as bag:
        imu_count = bag.get_message_count("/camera/imu/data")
        for topic, message, _ in bag.read_messages(topics=image_topics):
            samples[topic].append((message.header.seq,
                                   message.header.stamp.to_nsec()))

    def sequence_gaps(values):
        return [{"previous_seq": values[index - 1][0],
                 "next_seq": values[index][0],
                 "missing": values[index][0] - values[index - 1][0] - 1}
                for index in range(1, len(values))
                if values[index][0] != values[index - 1][0] + 1]

    rgb = samples[image_topics[0]]
    depth = samples[image_topics[1]]
    rgb_stamps = {stamp for _, stamp in rgb}
    depth_stamps = {stamp for _, stamp in depth}
    return {
        "path": str(path),
        "sha256": sha256(path),
        "rgb_frames": len(rgb),
        "depth_frames": len(depth),
        "imu_messages": imu_count,
        "exact_timestamp_pairs": len(rgb_stamps & depth_stamps),
        "rgb_only_timestamps": len(rgb_stamps - depth_stamps),
        "depth_only_timestamps": len(depth_stamps - rgb_stamps),
        "rgb_sequence_gaps": sequence_gaps(rgb),
        "depth_sequence_gaps": sequence_gaps(depth),
    }


def load_pose(path):
    data = rows(path)
    xyz = np.asarray([
        [float(row["field.pose.position.x"]),
         float(row["field.pose.position.y"]),
         float(row["field.pose.position.z"])] for row in data
    ])
    quaternions = np.asarray([
        [float(row["field.pose.orientation.x"]),
         float(row["field.pose.orientation.y"]),
         float(row["field.pose.orientation.z"]),
         float(row["field.pose.orientation.w"])] for row in data
    ])
    return data, xyz, quaternions


def summarize_run(run_dir):
    timing = rows(run_dir / "timing/rgbd_frame_timing.csv")
    local_mapping = rows(run_dir / "timing/local_mapping_timing.csv")
    tracking = rows(run_dir / "tracking_ok.csv")
    pose_rows, xyz, quaternions = load_pose(run_dir / "robot_pose.csv")
    log = (run_dir / "slam.log").read_text(errors="replace")
    first_ok = next((index for index, row in enumerate(timing)
                     if row["state_after"] == "2"), None)
    post_ok = timing[first_ok:] if first_ok is not None else []
    tracked = [int(row["tracked_inliers"]) for row in post_ok]
    steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    result = {
        "run": run_dir.name,
        "frames": len(timing),
        "state_before": dict(Counter(row["state_before"] for row in timing)),
        "state_after": dict(Counter(row["state_after"] for row in timing)),
        "tracking_ok_messages": dict(Counter(row["field.data"] for row in tracking)),
        "first_ok_input_index": first_ok,
        "first_ok": ({
            "frame_id": int(timing[first_ok]["frame_id"]),
            "rgb_seq": int(timing[first_ok]["rgb_seq"]),
            "image_timestamp": float(timing[first_ok]["image_timestamp"]),
            "relocalization_ms": float(timing[first_ok]["relocalization_ms"]),
            "tracked_inliers": int(timing[first_ok]["tracked_inliers"]),
        } if first_ok is not None else None),
        "lost_after_first_ok": sum(row["state_after"] != "2" for row in post_ok),
        "inserted_keyframes": sum(int(row["inserted_keyframe"]) for row in timing),
        "local_mapping_rows": len(local_mapping),
        "valid_depths": stats([float(row["valid_depths"]) for row in timing]),
        "tracked_inliers_after_ok": stats(tracked) if tracked else None,
        "orb_extract_ms": stats([float(row["orb_extract_ms"]) for row in timing]),
        "track_call_ms": stats([float(row["track_call_ms"]) for row in timing]),
        "callback_total_ms": stats([float(row["callback_total_ms"]) for row in timing]),
        "trajectory": {
            "pose_rows": len(pose_rows),
            "path_length_m": float(np.sum(steps)),
            "extent_m": np.ptp(xyz, axis=0).tolist(),
            "end_to_start_m": float(np.linalg.norm(xyz[-1] - xyz[0])),
            "max_step_m": float(np.max(steps)),
        },
        "map_ids_created": re.findall(r"Creation of new map with id:\s*(\d+)", log),
        "relocalized_count": log.count("Relocalized!!"),
        "no_candidate_count": log.count("Relocalization: No candidates found!"),
        "loaded_keyframe_ids": sorted(set(int(value) for value in
                                         re.findall(r"PostLoad KF (\d+):", log))),
        "rosbag_play_exit": (run_dir / "rosbag_play_exit_code.txt").read_text().strip(),
        "slam_exit": (run_dir / "slam_exit_code.txt").read_text().strip(),
    }
    accepted = re.search(r"Relocalization: Found (\d+) candidates.*?"
                         r"Relocalization candidate KF (\d+): BoW matches=(\d+).*?"
                         r"inliers after 1st PoseOpt=(\d+).*?"
                         r"additional via projection\+covis=(\d+).*?"
                         r"inliers after 2nd PoseOpt=(\d+).*?"
                         r"Relocalization: accepted with (\d+) inliers",
                         log, re.S)
    if accepted:
        result["accepted_relocalization"] = {
            "candidate_count": int(accepted.group(1)),
            "matched_keyframe_id": int(accepted.group(2)),
            "bow_matches": int(accepted.group(3)),
            "first_pose_opt_inliers": int(accepted.group(4)),
            "projection_and_covisible_additions": int(accepted.group(5)),
            "second_pose_opt_inliers": int(accepted.group(6)),
            "accepted_inliers": int(accepted.group(7)),
        }
    map_save = re.search(r"Map (\d+) has (\d+) KFs.*?MPs=(\d+), KFs=(\d+)",
                         log, re.S)
    if map_save:
        result["saved_map"] = {
            "map_id": int(map_save.group(1)),
            "keyframes": int(map_save.group(2)),
            "map_points": int(map_save.group(3)),
            "presaved_keyframes": int(map_save.group(4)),
        }
    return result, xyz, quaternions


def compare_trajectory(mapping_xyz, mapping_q, localization_xyz, localization_q):
    distances = np.linalg.norm(
        localization_xyz[:, None, :] - mapping_xyz[None, :, :], axis=2)
    nearest_indices = np.argmin(distances, axis=1)
    nearest = distances[np.arange(len(localization_xyz)), nearest_indices]
    angles = []
    for index, mapping_index in enumerate(nearest_indices):
        first = localization_q[index] / np.linalg.norm(localization_q[index])
        second = mapping_q[mapping_index] / np.linalg.norm(mapping_q[mapping_index])
        angles.append(math.degrees(2.0 * math.acos(
            min(1.0, abs(float(np.dot(first, second)))))))
    angles = np.asarray(angles)
    return {
        "nearest_mapping_distance_m": {
            "mean": float(np.mean(nearest)),
            "median": float(np.median(nearest)),
            "p90": float(np.percentile(nearest, 90)),
            "p95": float(np.percentile(nearest, 95)),
            "max": float(np.max(nearest)),
        },
        "within_0_10_m_percent": 100.0 * float(np.mean(nearest <= 0.10)),
        "within_0_25_m_percent": 100.0 * float(np.mean(nearest <= 0.25)),
        "within_0_50_m_percent": 100.0 * float(np.mean(nearest <= 0.50)),
        "nearest_orientation_difference_deg": {
            "median": float(np.median(angles)),
            "p90": float(np.percentile(angles, 90)),
        },
        "nearest_mapping_index_start": int(nearest_indices[0]),
        "nearest_mapping_index_end": int(nearest_indices[-1]),
        "nondecreasing_nearest_index_percent":
            100.0 * float(np.mean(np.diff(nearest_indices) >= 0)),
    }, nearest


def read_image_near(path, timestamp):
    best = None
    with rosbag.Bag(str(path)) as bag:
        for _, message, _ in bag.read_messages(topics=["/camera/infra1/image_raw"]):
            error = abs(message.header.stamp.to_sec() - timestamp)
            if best is None or error < best[0]:
                rgb = np.frombuffer(message.data, dtype=np.uint8).reshape(
                    message.height, message.step // 3, 3)[:, :message.width]
                gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
                best = (error, gray.copy(), message.header.seq,
                        message.header.stamp.to_sec())
    return best


def visualize_relocalization(mapping_bag, localization_bag, results,
                             mapping_run, localization_run, output_dir):
    match = localization_run["accepted_relocalization"]
    keyframe_id = str(match["matched_keyframe_id"])
    mapping_rows = rows(results / "f30_mapping_131409_rate1" /
                        "timing/local_mapping_timing.csv")
    keyframe = next(row for row in mapping_rows if row["keyframe_id"] == keyframe_id)
    first_ok = localization_run["first_ok"]
    mapping_image = read_image_near(mapping_bag, float(keyframe["image_timestamp"]))
    localization_image = read_image_near(
        localization_bag, first_ok["image_timestamp"])

    orb = cv2.ORB_create(nfeatures=600, scaleFactor=1.2, nlevels=5,
                         fastThreshold=20, edgeThreshold=19)
    first_kp, first_desc = orb.detectAndCompute(mapping_image[1], None)
    second_kp, second_desc = orb.detectAndCompute(localization_image[1], None)
    knn = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(
        first_desc, second_desc, k=2)
    good = [first for first, second in knn
            if first.distance < 0.75 * second.distance]
    first_points = np.float32([first_kp[item.queryIdx].pt for item in good])
    second_points = np.float32([second_kp[item.trainIdx].pt for item in good])
    _, fundamental_mask = cv2.findFundamentalMat(
        first_points, second_points, cv2.FM_RANSAC, 2.0, 0.99)
    _, homography_mask = cv2.findHomography(
        first_points, second_points, cv2.RANSAC, 3.0)
    image = cv2.drawMatches(
        mapping_image[1], first_kp, localization_image[1], second_kp,
        sorted(good, key=lambda item: item.distance)[:80], None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(output_dir / "accepted_relocalization_match.jpg"), image)
    return {
        "map_keyframe_id": int(keyframe_id),
        "mapping_rgb_seq": mapping_image[2],
        "localization_rgb_seq": localization_image[2],
        "mapping_timestamp_error_ms": mapping_image[0] * 1000.0,
        "localization_timestamp_error_ms": localization_image[0] * 1000.0,
        "ratio_matches": len(good),
        "fundamental_inliers": int(np.count_nonzero(fundamental_mask)),
        "homography_inliers": int(np.count_nonzero(homography_mask)),
    }


def plot_result(mapping_xyz, localization_xyz, nearest, output):
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(mapping_xyz[:, 0], mapping_xyz[:, 1], label="mapping bag")
    axes[0].plot(localization_xyz[:, 0], localization_xyz[:, 1],
                 label="localization bag")
    axes[0].scatter(mapping_xyz[0, 0], mapping_xyz[0, 1], marker="*", s=100,
                    label="mapping start")
    axes[0].scatter(localization_xyz[0, 0], localization_xyz[0, 1], marker="x",
                    s=70, label="first relocalized pose")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].set_title("Shared-map RGB-D trajectories")
    axes[0].legend()
    axes[1].plot(nearest)
    axes[1].axhline(0.25, color="orange", linestyle="--", label="0.25 m")
    axes[1].axhline(0.50, color="red", linestyle="--", label="0.50 m")
    axes[1].set_xlabel("localization pose index")
    axes[1].set_ylabel("nearest mapping-path distance (m)")
    axes[1].set_title("Localization path consistency")
    axes[1].grid(True)
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mapping_bag", type=Path)
    parser.add_argument("localization_bag", type=Path)
    parser.add_argument("artifacts", type=Path)
    args = parser.parse_args()
    results = args.artifacts / "board_results"
    run_names = ["f30_mapping_131409_rate1",
                 "f30_localization_133822_rate1",
                 "f30_localization_133822_rate1_r2"]
    summaries = {}
    poses = {}
    for name in run_names:
        summaries[name], xyz, quaternions = summarize_run(results / name)
        poses[name] = (xyz, quaternions)

    mapping_xyz, mapping_q = poses[run_names[0]]
    localization_xyz, localization_q = poses[run_names[1]]
    comparison, nearest = compare_trajectory(
        mapping_xyz, mapping_q, localization_xyz, localization_q)
    first_pose = results / run_names[1] / "robot_pose.csv"
    second_pose = results / run_names[2] / "robot_pose.csv"
    comparison["localization_pose_csv_byte_identical"] = (
        first_pose.read_bytes() == second_pose.read_bytes())
    comparison["atlas_sha256"] = sha256(
        results / run_names[0] / "f30_rgbd_map.osa")
    comparison["atlas_copies_byte_identical"] = len({sha256(
        results / name / "f30_rgbd_map.osa") for name in run_names}) == 1
    comparison["accepted_frame_visual_match"] = visualize_relocalization(
        args.mapping_bag, args.localization_bag, results,
        summaries[run_names[0]], summaries[run_names[1]], args.artifacts)
    plot_result(mapping_xyz, localization_xyz, nearest,
                args.artifacts / "map_localization_trajectory.png")

    output = {
        "bags": {
            "mapping": bag_summary(args.mapping_bag),
            "localization": bag_summary(args.localization_bag),
        },
        "runs": summaries,
        "comparison": comparison,
    }
    (args.artifacts / "summary.json").write_text(
        json.dumps(output, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(output, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
