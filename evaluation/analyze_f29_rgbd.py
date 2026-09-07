#!/usr/bin/env python3
"""Summarize F29 RGB-D board runs and endpoint appearance overlap."""

import argparse
import csv
import json
import math
import re
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import rosbag

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else None


def timing(values):
    return {
        "mean": float(np.mean(values)) if values else None,
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values) if values else None,
    }


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def summarize_run(run_dir):
    rows = read_csv(run_dir / "timing/rgbd_frame_timing.csv")
    poses = read_csv(run_dir / "robot_pose.csv")
    telemetry_rows = read_csv(run_dir / "telemetry.csv")
    states_before = {}
    states_after = {}
    for row in rows:
        states_before[row["state_before"]] = states_before.get(row["state_before"], 0) + 1
        states_after[row["state_after"]] = states_after.get(row["state_after"], 0) + 1

    def floats(key, filtered_rows=rows):
        return [float(row[key]) for row in filtered_rows if row.get(key, "") != ""]

    tracked_rows = [row for row in rows if row["state_after"] == "2" and int(row["frame_id"]) > 0]
    xyz = np.asarray([
        [float(row["field.pose.position.x"]), float(row["field.pose.position.y"]),
         float(row["field.pose.position.z"])] for row in poses
    ])
    quaternions = np.asarray([
        [float(row["field.pose.orientation.x"]), float(row["field.pose.orientation.y"]),
         float(row["field.pose.orientation.z"]), float(row["field.pose.orientation.w"])]
        for row in poses
    ])
    pose_timestamps = np.asarray([
        float(row["field.header.stamp"]) * 1.0e-9 for row in poses
    ])
    steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1) if len(xyz) > 1 else np.asarray([])
    end_distance = float(np.linalg.norm(xyz[-1] - xyz[0])) if len(xyz) else None
    orientation_change = None
    if len(quaternions):
        q0 = quaternions[0] / np.linalg.norm(quaternions[0])
        q1 = quaternions[-1] / np.linalg.norm(quaternions[-1])
        orientation_change = math.degrees(2.0 * math.acos(min(1.0, abs(float(np.dot(q0, q1))))))

    log_text = (run_dir / "slam.log").read_text(errors="replace")
    loop_lines = [line for line in log_text.splitlines() if re.search(
        r"\*Loop detected|Loop detected|CorrectLoop|\*Merge detected|Merge finished|BAD LOOP", line)]
    failure_lines = [line for line in log_text.splitlines() if re.search(
        r"Fail to track|Track Lost|Relocalized|Creation of new map", line)]
    new_map_ids = re.findall(r"Creation of new map with id:\s*(\d+)", log_text)

    return {
        "run": run_dir.name,
        "frames": len(rows),
        "rgb_sequence_gaps": sum(int(row["rgb_seq_gap"]) for row in rows),
        "states_before": states_before,
        "states_after": states_after,
        "valid_depths": timing(floats("valid_depths", tracked_rows)),
        "tracked_inliers": timing(floats("tracked_inliers", tracked_rows)),
        "orb_extract_ms": timing(floats("orb_extract_ms")),
        "track_call_ms": timing(floats("track_call_ms")),
        "callback_total_ms": timing(floats("callback_total_ms")),
        "resources": {
            "cpu_percent": timing([
                float(row["cpu_pct"]) for row in telemetry_rows
                if float(row["cpu_pct"]) > 0.0
            ]),
            "max_rss_mib": max(float(row["rss_kB"]) for row in telemetry_rows)
            / 1024.0,
            "max_temperature_c": max(float(row["cpu_temp_mC"])
                                      for row in telemetry_rows) / 1000.0,
        },
        "inserted_keyframes": sum(int(row["inserted_keyframe"]) for row in rows),
        "local_mapping_rows": len(read_csv(run_dir / "timing/local_mapping_timing.csv")),
        "trajectory": {
            "poses": len(poses),
            "path_length_m": float(np.sum(steps)),
            "end_to_start_m": end_distance,
            "orientation_change_deg": orientation_change,
            "max_step_m": float(np.max(steps)) if len(steps) else None,
            "p99_step_m": percentile(steps.tolist(), 99),
            "extent_m": (np.max(xyz, axis=0) - np.min(xyz, axis=0)).tolist(),
        },
        "loop_event_lines": loop_lines,
        "failure_event_lines": failure_lines,
        "map_ids_created": new_map_ids,
        "rosbag_play_exit": (run_dir / "rosbag_play_exit_code.txt").read_text().strip(),
        "slam_exit": (run_dir / "slam_exit_code.txt").read_text().strip(),
    }, xyz, pose_timestamps, quaternions


def image_from_message(message):
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding.lower() == "rgb8":
        image = data.reshape(message.height, message.step // 3, 3)[:, :message.width]
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    if message.encoding.lower() == "bgr8":
        image = data.reshape(message.height, message.step // 3, 3)[:, :message.width]
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return data.reshape(message.height, message.step)[:, :message.width].copy()


def analyze_bag_messages(bag_path):
    topics = ["/camera/infra1/image_raw", "/camera/depth/image_raw"]
    messages = {topic: [] for topic in topics}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=topics):
            messages[topic].append((message.header.seq,
                                    message.header.stamp.to_nsec()))

    def gaps(rows):
        return [{"previous_seq": rows[index - 1][0],
                 "next_seq": rows[index][0],
                 "missing": rows[index][0] - rows[index - 1][0] - 1}
                for index in range(1, len(rows))
                if rows[index][0] != rows[index - 1][0] + 1]

    rgb = messages[topics[0]]
    depth = messages[topics[1]]
    rgb_stamps = {stamp for _, stamp in rgb}
    depth_stamps = {stamp for _, stamp in depth}
    return {
        "rgb_frames": len(rgb),
        "depth_frames": len(depth),
        "exact_timestamp_pairs": len(rgb_stamps & depth_stamps),
        "rgb_only_timestamps": len(rgb_stamps - depth_stamps),
        "depth_only_timestamps": len(depth_stamps - rgb_stamps),
        "rgb_sequence_gaps": gaps(rgb),
        "depth_sequence_gaps": gaps(depth),
    }


def pair_score(orb, matcher, first, second):
    kp1, des1 = orb.detectAndCompute(first, None)
    kp2, des2 = orb.detectAndCompute(second, None)
    if des1 is None or des2 is None:
        return {"keypoints_a": len(kp1), "keypoints_b": len(kp2), "ratio_matches": 0,
                "fundamental_inliers": 0, "homography_inliers": 0}, kp1, kp2, []
    knn = matcher.knnMatch(des1, des2, k=2)
    good = [a for a, b in knn if a.distance < 0.75 * b.distance]
    pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good])
    f_inliers = 0
    h_inliers = 0
    if len(good) >= 8:
        _, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 2.0, 0.99)
        f_inliers = int(np.count_nonzero(mask)) if mask is not None else 0
    if len(good) >= 4:
        _, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 3.0)
        h_inliers = int(np.count_nonzero(mask)) if mask is not None else 0
    return {
        "keypoints_a": len(kp1),
        "keypoints_b": len(kp2),
        "ratio_matches": len(good),
        "fundamental_inliers": f_inliers,
        "homography_inliers": h_inliers,
    }, kp1, kp2, good


def analyze_appearance(bag_path, output_dir, keyframe_timestamps):
    frames = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, message, _ in bag.read_messages(topics=["/camera/infra1/image_raw"]):
            frames.append((message.header.stamp.to_sec(), image_from_message(message)))
    orb = cv2.ORB_create(nfeatures=600, scaleFactor=1.2, nlevels=5,
                         fastThreshold=20, edgeThreshold=19)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    endpoint, kp1, kp2, good = pair_score(orb, matcher, frames[0][1], frames[-1][1])
    endpoint["time_separation_s"] = frames[-1][0] - frames[0][0]

    first_indices = list(range(0, min(140, len(frames)), 7))
    last_indices = list(range(max(0, len(frames) - 140), len(frames), 7))
    best = None
    for i in first_indices:
        for j in last_indices:
            score, _, _, _ = pair_score(orb, matcher, frames[i][1], frames[j][1])
            candidate = (score["fundamental_inliers"], score["ratio_matches"], i, j, score)
            if best is None or candidate[:2] > best[:2]:
                best = candidate
    _, _, i, j, best_score = best
    best_score.update({
        "first_index": i,
        "last_index": j,
        "time_separation_s": frames[j][0] - frames[i][0],
        "first_time_from_start_s": frames[i][0] - frames[0][0],
        "last_time_from_start_s": frames[j][0] - frames[0][0],
        "first_timestamp": frames[i][0],
        "last_timestamp": frames[j][0],
    })

    score, kp1, kp2, good = pair_score(orb, matcher, frames[i][1], frames[j][1])
    matched = cv2.drawMatches(frames[i][1], kp1, frames[j][1], kp2,
                              sorted(good, key=lambda match: match.distance)[:80], None,
                              flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(output_dir / "best_endpoint_region_matches.jpg"), matched)

    keyframe_indices = sorted(set(min(
        range(len(frames)), key=lambda index: abs(frames[index][0] - timestamp))
        for timestamp in keyframe_timestamps))
    first_keyframes = [index for index in keyframe_indices
                       if frames[index][0] - frames[0][0] < 15.0]
    last_keyframes = [index for index in keyframe_indices
                      if frames[index][0] - frames[0][0] > 45.0]
    best_keyframe = None
    for first_index in first_keyframes:
        for last_index in last_keyframes:
            score, _, _, _ = pair_score(
                orb, matcher, frames[first_index][1], frames[last_index][1])
            candidate = (score["fundamental_inliers"], score["ratio_matches"],
                         first_index, last_index, score)
            if best_keyframe is None or candidate[:2] > best_keyframe[:2]:
                best_keyframe = candidate
    _, _, first_index, last_index, best_keyframe_score = best_keyframe
    best_keyframe_score.update({
        "first_index": first_index,
        "last_index": last_index,
        "time_separation_s": frames[last_index][0] - frames[first_index][0],
        "first_time_from_start_s": frames[first_index][0] - frames[0][0],
        "last_time_from_start_s": frames[last_index][0] - frames[0][0],
        "first_timestamp": frames[first_index][0],
        "last_timestamp": frames[last_index][0],
    })
    _, first_kp, last_kp, keyframe_matches = pair_score(
        orb, matcher, frames[first_index][1], frames[last_index][1])
    matched_keyframes = cv2.drawMatches(
        frames[first_index][1], first_kp, frames[last_index][1], last_kp,
        sorted(keyframe_matches, key=lambda match: match.distance)[:80], None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(output_dir / "best_keyframe_region_matches.jpg"),
                matched_keyframes)

    indices = np.linspace(0, len(frames) - 1, 7, dtype=int)
    tiles = []
    for index in indices:
        tile = cv2.cvtColor(frames[index][1], cv2.COLOR_GRAY2BGR)
        cv2.putText(tile, f"{frames[index][0] - frames[0][0]:.1f}s", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        tiles.append(cv2.resize(tile, (320, 272)))
    cv2.imwrite(str(output_dir / "route_montage.jpg"), np.hstack(tiles))
    return {"image_frames": len(frames), "first_vs_last": endpoint,
            "best_first_last_region_pair": best_score,
            "best_keyframe_region_pair": best_keyframe_score}


def plot_trajectories(trajectories, output):
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    for name, xyz in trajectories.items():
        axes[0].plot(xyz[:, 0], xyz[:, 1], label=name)
        axes[1].plot(np.arange(len(xyz)), np.linalg.norm(xyz - xyz[0], axis=1), label=name)
    axes[0].scatter([0], [0], marker="*", s=100, label="start")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].set_title("RGB-D trajectory, dominant x-y plane")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[1].set_xlabel("published pose index")
    axes[1].set_ylabel("distance from start (m)")
    axes[1].set_title("Distance to starting position")
    axes[1].grid(True)
    axes[0].legend()
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("artifacts", type=Path)
    args = parser.parse_args()
    summaries = []
    trajectories = {}
    pose_series = {}
    for run_dir in sorted(path for path in args.artifacts.glob("f29_*") if path.is_dir()):
        summary, xyz, pose_timestamps, quaternions = summarize_run(run_dir)
        summaries.append(summary)
        trajectories[run_dir.name] = xyz
        pose_series[run_dir.name] = (xyz, pose_timestamps, quaternions)
    first_run_rows = read_csv(
        sorted(path for path in args.artifacts.glob("f29_*") if path.is_dir())[0]
        / "timing/rgbd_frame_timing.csv")
    keyframe_timestamps = [float(row["image_timestamp"]) for row in first_run_rows
                           if row["inserted_keyframe"] == "1"]
    appearance = analyze_appearance(args.bag, args.artifacts,
                                    keyframe_timestamps)
    source = analyze_bag_messages(args.bag)
    repeat_pair = appearance["best_keyframe_region_pair"]
    for summary in summaries:
        xyz, pose_timestamps, quaternions = pose_series[summary["run"]]
        first_index = int(np.argmin(np.abs(
            pose_timestamps - repeat_pair["first_timestamp"])))
        last_index = int(np.argmin(np.abs(
            pose_timestamps - repeat_pair["last_timestamp"])))
        first_q = quaternions[first_index] / np.linalg.norm(quaternions[first_index])
        last_q = quaternions[last_index] / np.linalg.norm(quaternions[last_index])
        summary["repeat_place_pose_delta"] = {
            "first_pose_index": first_index,
            "last_pose_index": last_index,
            "first_timestamp_error_ms": float(abs(
                pose_timestamps[first_index] - repeat_pair["first_timestamp"]) * 1000.0),
            "last_timestamp_error_ms": float(abs(
                pose_timestamps[last_index] - repeat_pair["last_timestamp"]) * 1000.0),
            "translation_m": float(np.linalg.norm(xyz[last_index] - xyz[first_index])),
            "rotation_deg": math.degrees(2.0 * math.acos(min(
                1.0, abs(float(np.dot(first_q, last_q)))))),
        }
    plot_trajectories(trajectories, args.artifacts / "trajectory_analysis.png")
    for summary in summaries:
        summary["delivered_exact_pairs_percent"] = (
            100.0 * summary["frames"] / source["exact_timestamp_pairs"])
    result = {"bag": str(args.bag), "source": source, "runs": summaries,
              "appearance": appearance}
    (args.artifacts / "summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
