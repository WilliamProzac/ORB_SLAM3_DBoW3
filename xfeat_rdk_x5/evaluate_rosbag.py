#!/usr/bin/env python3
"""Evaluate XFeat and an ORB baseline on synchronized stereo images in a ROS bag."""

import argparse
import csv
import json
import math
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

try:
    import rosbag
except ModuleNotFoundError:
    noetic_python = Path("/opt/ros/noetic/lib/python3/dist-packages")
    if noetic_python.is_dir():
        sys.path.append(str(noetic_python))
    import rosbag


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UPSTREAM = ROOT / "Thirdparty" / "accelerated_features"
LEFT_TOPIC = "/camera/infra1/image_raw"
RIGHT_TOPIC = "/camera/infra2/image_raw"
LEFT_INFO_TOPIC = "/camera/infra1/camera_info"
RIGHT_INFO_TOPIC = "/camera/infra2/camera_info"
TRACKING_TOPIC = "/robot_pose_tracking_ok"


def decode_mono8(message):
    if message.encoding not in ("mono8", "8UC1"):
        raise RuntimeError(f"Unsupported image encoding: {message.encoding}")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    return np.ascontiguousarray(rows[:, : message.width])


def distribution(values):
    values = [float(value) for value in values if math.isfinite(float(value))]
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": len(values),
        "mean": float(array.mean()),
        "median": float(np.median(array)),
        "p10": float(np.percentile(array, 10)),
        "p90": float(np.percentile(array, 90)),
        "min": float(array.min()),
        "max": float(array.max()),
    }


def grid_coverage(points, width, height, columns=8, rows=6):
    if len(points) == 0:
        return 0.0
    x = np.clip((points[:, 0] * columns / width).astype(np.int32), 0, columns - 1)
    y = np.clip((points[:, 1] * rows / height).astype(np.int32), 0, rows - 1)
    occupied = len(set(zip(x.tolist(), y.tolist())))
    return occupied / float(columns * rows)


def fundamental_inliers(points_a, points_b, threshold=1.5):
    if len(points_a) < 8:
        return np.zeros(len(points_a), dtype=bool)
    _, mask = cv2.findFundamentalMat(
        points_a.astype(np.float32),
        points_b.astype(np.float32),
        cv2.FM_RANSAC,
        threshold,
        0.999,
    )
    if mask is None or len(mask) != len(points_a):
        return np.zeros(len(points_a), dtype=bool)
    return mask.reshape(-1).astype(bool)


def draw_matches(image_a, image_b, points_a, points_b, pairs, inliers, output, limit=120):
    if len(pairs) == 0:
        canvas = np.hstack([image_a, image_b])
        cv2.imwrite(str(output), canvas)
        return
    selected = np.flatnonzero(inliers)
    if len(selected) == 0:
        selected = np.arange(min(len(pairs), limit))
    selected = selected[:limit]
    keypoints_a = [cv2.KeyPoint(float(x), float(y), 3) for x, y in points_a]
    keypoints_b = [cv2.KeyPoint(float(x), float(y), 3) for x, y in points_b]
    matches = [
        cv2.DMatch(int(pairs[index, 0]), int(pairs[index, 1]), 0.0)
        for index in selected
    ]
    canvas = cv2.drawMatches(
        image_a,
        keypoints_a,
        image_b,
        keypoints_b,
        matches,
        None,
        matchColor=(0, 255, 0),
        singlePointColor=(0, 0, 255),
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )
    cv2.imwrite(str(output), canvas)


def load_sampled_frames(bag_path, anchor_stride, temporal_gap, max_anchors):
    frames = []
    pending = {}
    left_index = 0
    camera_info = {}
    tracking_status = []
    topics = [LEFT_TOPIC, RIGHT_TOPIC, LEFT_INFO_TOPIC, RIGHT_INFO_TOPIC, TRACKING_TOPIC]

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, message, bag_time in bag.read_messages(topics=topics):
            if topic == TRACKING_TOPIC:
                tracking_status.append((bag_time.to_sec(), bool(message.data)))
                continue
            if topic in (LEFT_INFO_TOPIC, RIGHT_INFO_TOPIC):
                if topic not in camera_info:
                    camera_info[topic] = {
                        "width": int(message.width),
                        "height": int(message.height),
                        "K": list(message.K),
                        "D": list(message.D),
                        "R": list(message.R),
                        "P": list(message.P),
                    }
                continue

            stamp_ns = message.header.stamp.to_nsec()
            item = pending.setdefault(stamp_ns, {})
            if topic == LEFT_TOPIC:
                frame_index = left_index
                left_index += 1
                phase = frame_index % anchor_stride
                anchor_ordinal = frame_index // anchor_stride
                within_limit = not max_anchors or anchor_ordinal < max_anchors
                selected = within_limit and (phase == 0 or phase == temporal_gap)
                item["selected"] = selected
                item["frame_index"] = frame_index
                if selected:
                    item["left"] = decode_mono8(message)
                    item["bag_time_sec"] = bag_time.to_sec()
            else:
                item["right_message"] = message

            if item.get("selected") and "left" in item and "right_message" in item:
                frames.append(
                    {
                        "frame_index": item["frame_index"],
                        "stamp_ns": stamp_ns,
                        "stamp_sec": stamp_ns / 1e9,
                        "bag_time_sec": item["bag_time_sec"],
                        "left": item["left"],
                        "right": decode_mono8(item["right_message"]),
                    }
                )
                del pending[stamp_ns]
                if max_anchors and item["frame_index"] == (max_anchors - 1) * anchor_stride + temporal_gap:
                    break
            elif item.get("selected") is False and "right_message" in item:
                del pending[stamp_ns]

    frames.sort(key=lambda item: item["frame_index"])
    if tracking_status:
        status_times = np.asarray([item[0] for item in tracking_status])
        for frame in frames:
            insertion = int(np.searchsorted(status_times, frame["bag_time_sec"]))
            candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(status_times)]
            nearest = min(candidates, key=lambda index: abs(status_times[index] - frame["bag_time_sec"]))
            delta = abs(status_times[nearest] - frame["bag_time_sec"])
            frame["orb_slam_tracking_ok"] = tracking_status[nearest][1] if delta <= 0.25 else None
            frame["tracking_status_delta_sec"] = delta
    else:
        for frame in frames:
            frame["orb_slam_tracking_ok"] = None
            frame["tracking_status_delta_sec"] = math.nan
    return frames, camera_info, left_index, tracking_status


class XFeatMethod:
    name = "xfeat"

    def __init__(self, upstream, top_k, min_cossim):
        sys.path.insert(0, str(upstream.resolve()))
        import torch
        from modules.xfeat import XFeat

        self.torch = torch
        torch.set_num_threads(8)
        self.model = XFeat(top_k=top_k)
        self.top_k = top_k
        self.min_cossim = min_cossim

    def warmup(self, image):
        self.extract(image)

    def extract(self, image):
        start = time.perf_counter()
        output = self.model.detectAndCompute(image, top_k=self.top_k)[0]
        latency_ms = (time.perf_counter() - start) * 1000.0
        return {
            "points": output["keypoints"].cpu().numpy().astype(np.float32),
            "descriptors": output["descriptors"],
            "scores": output["scores"].cpu().numpy(),
            "latency_ms": latency_ms,
        }

    def match(self, features_a, features_b):
        start = time.perf_counter()
        indices_a, indices_b = self.model.match(
            features_a["descriptors"],
            features_b["descriptors"],
            min_cossim=self.min_cossim,
        )
        latency_ms = (time.perf_counter() - start) * 1000.0
        pairs = np.column_stack(
            [indices_a.cpu().numpy(), indices_b.cpu().numpy()]
        ).astype(np.int32)
        return pairs, latency_ms


class ORBMethod:
    name = "orb"

    def __init__(self, top_k, max_hamming):
        self.extractor = cv2.ORB_create(
            nfeatures=top_k,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            fastThreshold=20,
        )
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.max_hamming = max_hamming

    def warmup(self, image):
        self.extract(image)

    def extract(self, image):
        start = time.perf_counter()
        keypoints, descriptors = self.extractor.detectAndCompute(image, None)
        latency_ms = (time.perf_counter() - start) * 1000.0
        points = np.asarray([keypoint.pt for keypoint in keypoints], dtype=np.float32)
        if len(points) == 0:
            points = np.empty((0, 2), dtype=np.float32)
        return {
            "points": points,
            "descriptors": descriptors,
            "scores": np.asarray([keypoint.response for keypoint in keypoints]),
            "latency_ms": latency_ms,
        }

    def match(self, features_a, features_b):
        start = time.perf_counter()
        descriptors_a = features_a["descriptors"]
        descriptors_b = features_b["descriptors"]
        if descriptors_a is None or descriptors_b is None:
            return np.empty((0, 2), dtype=np.int32), 0.0
        matches = self.matcher.match(descriptors_a, descriptors_b)
        matches = [match for match in matches if match.distance <= self.max_hamming]
        matches.sort(key=lambda match: match.distance)
        latency_ms = (time.perf_counter() - start) * 1000.0
        pairs = np.asarray(
            [(match.queryIdx, match.trainIdx) for match in matches], dtype=np.int32
        )
        if pairs.size == 0:
            pairs = np.empty((0, 2), dtype=np.int32)
        return pairs, latency_ms


def evaluate_pair(
    method,
    pair_type,
    features_a,
    features_b,
    width,
    height,
    frame_index,
    stamp_sec,
    bag_time_sec,
    delta_time,
    orb_slam_tracking_ok,
):
    pairs, match_latency_ms = method.match(features_a, features_b)
    points_a = features_a["points"][pairs[:, 0]] if len(pairs) else np.empty((0, 2))
    points_b = features_b["points"][pairs[:, 1]] if len(pairs) else np.empty((0, 2))
    ransac = fundamental_inliers(points_a, points_b)
    strict = ransac.copy()
    vertical_error = np.abs(points_a[:, 1] - points_b[:, 1]) if len(pairs) else np.asarray([])
    disparity = points_a[:, 0] - points_b[:, 0] if len(pairs) else np.asarray([])
    rectified = np.zeros(len(pairs), dtype=bool)
    if pair_type == "stereo" and len(pairs):
        rectified = (vertical_error <= 2.0) & (disparity >= 0.0) & (disparity <= 160.0)
        strict &= rectified

    match_count = len(pairs)
    ransac_count = int(ransac.sum())
    strict_count = int(strict.sum())
    record = {
        "method": method.name,
        "pair_type": pair_type,
        "frame_index": frame_index,
        "stamp_sec": stamp_sec,
        "bag_time_sec": bag_time_sec,
        "delta_time_sec": delta_time,
        "orb_slam_tracking_ok": orb_slam_tracking_ok,
        "keypoints_a": len(features_a["points"]),
        "keypoints_b": len(features_b["points"]),
        "matches": match_count,
        "ransac_inliers": ransac_count,
        "ransac_inlier_ratio": ransac_count / match_count if match_count else 0.0,
        "strict_inliers": strict_count,
        "strict_inlier_ratio": strict_count / match_count if match_count else 0.0,
        "strict_grid_coverage": grid_coverage(points_a[strict], width, height),
        "match_latency_ms": match_latency_ms,
        "median_vertical_error_px": float(np.median(vertical_error)) if len(vertical_error) else math.nan,
        "median_disparity_px": float(np.median(disparity[rectified])) if rectified.any() else math.nan,
        "rectified_inliers": int(rectified.sum()),
        "rectified_inlier_ratio": float(rectified.mean()) if len(rectified) else 0.0,
    }
    return record, pairs, strict


def summarize(records, feature_stats, bag_metadata):
    summary = {"bag": bag_metadata, "methods": {}}
    by_method_pair = defaultdict(list)
    for record in records:
        by_method_pair[(record["method"], record["pair_type"])].append(record)

    for method_name, stats in feature_stats.items():
        method_summary = {
            "extraction_latency_ms": distribution(stats["latency_ms"]),
            "detected_keypoints": distribution(stats["keypoints"]),
            "left_keypoint_grid_coverage": distribution(stats["coverage"]),
            "pairs": {},
        }
        for pair_type in ("stereo", "temporal"):
            group = by_method_pair.get((method_name, pair_type), [])
            pair_summary = {"pair_count": len(group)}
            for key in (
                "matches",
                "ransac_inliers",
                "ransac_inlier_ratio",
                "strict_inliers",
                "strict_inlier_ratio",
                "strict_grid_coverage",
                "match_latency_ms",
                "median_vertical_error_px",
                "median_disparity_px",
                "rectified_inliers",
                "rectified_inlier_ratio",
            ):
                pair_summary[key] = distribution(record[key] for record in group)
            pair_summary["robust_pair_rate"] = (
                sum(
                    record["strict_inliers"] >= 30
                    and record["strict_inlier_ratio"] >= 0.30
                    for record in group
                )
                / len(group)
                if group
                else 0.0
            )
            pair_summary["by_orb_slam_tracking_state"] = {}
            for state_name, state_value in (("tracking_ok", True), ("tracking_failed", False)):
                state_group = [
                    record for record in group if record["orb_slam_tracking_ok"] is state_value
                ]
                pair_summary["by_orb_slam_tracking_state"][state_name] = {
                    "pair_count": len(state_group),
                    "matches": distribution(record["matches"] for record in state_group),
                    "strict_inliers": distribution(
                        record["strict_inliers"] for record in state_group
                    ),
                    "strict_inlier_ratio": distribution(
                        record["strict_inlier_ratio"] for record in state_group
                    ),
                    "strict_grid_coverage": distribution(
                        record["strict_grid_coverage"] for record in state_group
                    ),
                }
            method_summary["pairs"][pair_type] = pair_summary
        summary["methods"][method_name] = method_summary

    if "xfeat" in summary["methods"] and "orb" in summary["methods"]:
        comparison = {}
        for pair_type in ("stereo", "temporal"):
            xfeat = summary["methods"]["xfeat"]["pairs"][pair_type]
            orb = summary["methods"]["orb"]["pairs"][pair_type]
            comparison[pair_type] = {
                "mean_strict_inlier_count_xfeat_over_orb": (
                    xfeat["strict_inliers"].get("mean", 0.0)
                    / orb["strict_inliers"].get("mean", 1.0)
                    if orb["strict_inliers"].get("mean", 0.0) > 0
                    else math.nan
                ),
                "mean_match_ratio_xfeat_over_orb": (
                    xfeat["matches"].get("mean", 0.0)
                    / orb["matches"].get("mean", 1.0)
                    if orb["matches"].get("mean", 0.0) > 0
                    else math.nan
                ),
            }
        comparison["extraction_latency_xfeat_over_orb"] = (
            summary["methods"]["xfeat"]["extraction_latency_ms"].get("mean", 0.0)
            / summary["methods"]["orb"]["extraction_latency_ms"].get("mean", 1.0)
        )
        summary["comparison"] = comparison
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--upstream", type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument("--top-k", type=int, default=1024)
    parser.add_argument("--anchor-stride", type=int, default=10)
    parser.add_argument("--temporal-gap", type=int, default=1)
    parser.add_argument("--max-anchors", type=int, default=0)
    parser.add_argument("--xfeat-min-cossim", type=float, default=0.82)
    parser.add_argument("--orb-max-hamming", type=float, default=64.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "xfeat_rdk_x5" / "artifacts" / "bag_2026-06-05-09-59-04",
    )
    args = parser.parse_args()
    if not 0 < args.temporal_gap < args.anchor_stride:
        raise ValueError("temporal-gap must be between 1 and anchor-stride - 1")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    frames, camera_info, scanned_left_frames, tracking_status = load_sampled_frames(
        args.bag, args.anchor_stride, args.temporal_gap, args.max_anchors
    )
    if len(frames) < 2:
        raise RuntimeError("Not enough synchronized frames were sampled")
    height, width = frames[0]["left"].shape
    right_projection = camera_info.get(RIGHT_INFO_TOPIC, {}).get("P", [0.0] * 12)
    focal_length = right_projection[0]
    baseline_m = abs(right_projection[3] / focal_length) if focal_length else math.nan

    methods = [
        XFeatMethod(args.upstream, args.top_k, args.xfeat_min_cossim),
        ORBMethod(args.top_k, args.orb_max_hamming),
    ]
    for method in methods:
        method.warmup(frames[0]["left"])

    records = []
    feature_stats = {
        method.name: {"latency_ms": [], "keypoints": [], "coverage": []}
        for method in methods
    }
    previous = {method.name: None for method in methods}
    temporal_frames = [
        frame for frame in frames if frame["frame_index"] % args.anchor_stride == args.temporal_gap
    ]
    visual_anchor_indices = {
        temporal_frames[len(temporal_frames) // 4]["frame_index"],
        temporal_frames[len(temporal_frames) // 2]["frame_index"],
        temporal_frames[(3 * len(temporal_frames)) // 4]["frame_index"],
    }
    failed_tracking_frames = [
        frame
        for frame in temporal_frames
        if frame.get("orb_slam_tracking_ok") is False
    ]
    if failed_tracking_frames:
        visual_anchor_indices.add(
            failed_tracking_frames[len(failed_tracking_frames) // 2]["frame_index"]
        )

    for sequence_index, frame in enumerate(frames):
        for method in methods:
            left_features = method.extract(frame["left"])
            right_features = method.extract(frame["right"])
            for features in (left_features, right_features):
                feature_stats[method.name]["latency_ms"].append(features["latency_ms"])
                feature_stats[method.name]["keypoints"].append(len(features["points"]))
            feature_stats[method.name]["coverage"].append(
                grid_coverage(left_features["points"], width, height)
            )

            stereo_record, stereo_pairs, stereo_inliers = evaluate_pair(
                method,
                "stereo",
                left_features,
                right_features,
                width,
                height,
                frame["frame_index"],
                frame["stamp_sec"],
                frame["bag_time_sec"],
                0.0,
                frame["orb_slam_tracking_ok"],
            )
            records.append(stereo_record)

            prior = previous[method.name]
            if prior and frame["frame_index"] - prior["frame_index"] == args.temporal_gap:
                temporal_record, temporal_pairs, temporal_inliers = evaluate_pair(
                    method,
                    "temporal",
                    prior["features"],
                    left_features,
                    width,
                    height,
                    frame["frame_index"],
                    frame["stamp_sec"],
                    frame["bag_time_sec"],
                    frame["stamp_sec"] - prior["stamp_sec"],
                    frame["orb_slam_tracking_ok"],
                )
                records.append(temporal_record)
                if frame["frame_index"] in visual_anchor_indices:
                    draw_matches(
                        prior["image"],
                        frame["left"],
                        prior["features"]["points"],
                        left_features["points"],
                        temporal_pairs,
                        temporal_inliers,
                        args.output_dir
                        / f"frame_{frame['frame_index']:04d}_temporal_{method.name}.png",
                    )

            if frame["frame_index"] in visual_anchor_indices:
                draw_matches(
                    frame["left"],
                    frame["right"],
                    left_features["points"],
                    right_features["points"],
                    stereo_pairs,
                    stereo_inliers,
                    args.output_dir
                    / f"frame_{frame['frame_index']:04d}_stereo_{method.name}.png",
                )

            previous[method.name] = {
                "frame_index": frame["frame_index"],
                "stamp_sec": frame["stamp_sec"],
                "image": frame["left"],
                "features": left_features,
            }

        if (sequence_index + 1) % 20 == 0 or sequence_index + 1 == len(frames):
            print(f"processed {sequence_index + 1}/{len(frames)} sampled stereo frames")

    bag_metadata = {
        "path": str(args.bag.resolve()),
        "left_topic": LEFT_TOPIC,
        "right_topic": RIGHT_TOPIC,
        "image_shape_hw": [height, width],
        "sampled_stereo_frames": len(frames),
        "sampled_temporal_pairs": sum(
            frames[index]["frame_index"] - frames[index - 1]["frame_index"]
            == args.temporal_gap
            for index in range(1, len(frames))
        ),
        "anchor_stride": args.anchor_stride,
        "temporal_gap_frames": args.temporal_gap,
        "scanned_left_frames": scanned_left_frames,
        "first_stamp_sec": frames[0]["stamp_sec"],
        "last_stamp_sec": frames[-1]["stamp_sec"],
        "first_bag_time_sec": frames[0]["bag_time_sec"],
        "last_bag_time_sec": frames[-1]["bag_time_sec"],
        "median_temporal_delta_sec": float(
            np.median(
                [record["delta_time_sec"] for record in records if record["pair_type"] == "temporal"]
            )
        ),
        "camera_info": camera_info,
        "stereo_baseline_m_from_projection": baseline_m,
        "xfeat_top_k": args.top_k,
        "xfeat_min_cossim": args.xfeat_min_cossim,
        "orb_max_hamming": args.orb_max_hamming,
        "orb_slam_tracking_status": {
            "total_messages": len(tracking_status),
            "true_messages": sum(value for _, value in tracking_status),
            "false_messages": sum(not value for _, value in tracking_status),
            "sampled_frames_true": sum(
                frame["orb_slam_tracking_ok"] is True for frame in frames
            ),
            "sampled_frames_false": sum(
                frame["orb_slam_tracking_ok"] is False for frame in frames
            ),
            "sampled_frames_unavailable": sum(
                frame["orb_slam_tracking_ok"] is None for frame in frames
            ),
        },
        "geometry": {
            "fundamental_ransac_threshold_px": 1.5,
            "stereo_vertical_threshold_px": 2.0,
            "stereo_disparity_range_px": [0.0, 160.0],
            "robust_pair_definition": "strict_inliers >= 30 and strict_inlier_ratio >= 0.30",
        },
    }
    summary = summarize(records, feature_stats, bag_metadata)

    csv_path = args.output_dir / "per_pair_metrics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(summary_path)
    print(json.dumps(summary["comparison"], ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
