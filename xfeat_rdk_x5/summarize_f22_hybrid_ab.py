#!/usr/bin/env python3
import argparse
import csv
import json
import math
import re
from pathlib import Path
from statistics import mean


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = round(fraction * (len(ordered) - 1))
    return ordered[index]


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
    quaternion = (x / norm, y / norm, z / norm, w / norm)
    x, y, z, w = quaternion
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    # The CSV stores Tcw. Convert to the camera center Cw = -Rcw^T tcw.
    center = tuple(
        -sum(rotation[row_index][column] * translation[row_index]
             for row_index in range(3))
        for column in range(3)
    )
    return center, quaternion


def distance(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def angle_degrees(left, right):
    dot = abs(sum(a * b for a, b in zip(left, right)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def summarize(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    states = [int(row["tracking_state"]) for row in rows]
    lost = [state in (3, 4) for state in states]
    segments = 0
    previous = False
    for value in lost:
        if value and not previous:
            segments += 1
        previous = value

    def floats(name):
        result = []
        for row in rows:
            value = float(row[name])
            if math.isfinite(value):
                result.append(value)
        return result

    def ints(name):
        return [int(row[name]) for row in rows]

    valid_depths = ints("valid_depths")
    inliers = ints("tracked_inliers")
    pose_valid = ints("pose_valid")
    poses = [pose(row) for row in rows]
    poses = [value for value in poses if value is not None]
    translation_steps = [
        distance(previous[0], current[0])
        for previous, current in zip(poses, poses[1:])
    ]
    rotation_steps = [
        angle_degrees(previous[1], current[1])
        for previous, current in zip(poses, poses[1:])
    ]
    summary = {
        "csv": str(path),
        "callbacks": len(rows),
        "state_counts": {str(state): states.count(state) for state in sorted(set(states))},
        "lost_frames": sum(lost),
        "lost_segments": segments,
        "pose_valid_frames": sum(pose_valid),
        "trajectory_path_m": sum(translation_steps),
        "translation_step_m_p90": percentile(translation_steps, 0.90),
        "translation_step_m_max": max(translation_steps, default=None),
        "rotation_step_deg_p90": percentile(rotation_steps, 0.90),
        "rotation_step_deg_max": max(rotation_steps, default=None),
        "valid_depths_mean": mean(valid_depths) if valid_depths else None,
        "valid_depths_p10": percentile(valid_depths, 0.10),
        "tracked_inliers_mean": mean(inliers) if inliers else None,
        "tracked_inliers_p10": percentile(inliers, 0.10),
        "track_total_ms_mean": mean(floats("track_total_ms")),
        "track_total_ms_p90": percentile(floats("track_total_ms"), 0.90),
        "hybrid_ms_mean": mean(floats("hybrid_ms")),
        "hybrid_added_mean": mean(ints("hybrid_added")),
        "hybrid_replaced_mean": mean(ints("hybrid_replaced")),
        "hybrid_accepted_mean": mean(ints("hybrid_accepted")),
    }
    return summary


def summarize_log(path):
    text = path.read_text(encoding="utf-8", errors="replace")
    diagnostics = []
    for line in text.splitlines():
        if "[STEREO_GATE_DIAG]" not in line:
            continue
        diagnostics.append(dict(re.findall(r"(\w+)=([^\s]+)", line)))

    def finite_values(name):
        values = []
        for row in diagnostics:
            try:
                value = float(row[name])
            except (KeyError, ValueError):
                continue
            if math.isfinite(value):
                values.append(value)
        return values

    reprojection_mean = finite_values("reproj_px_mean")
    reprojection_p90 = finite_values("reproj_px_p90")
    return {
        "map_creation_count": text.count("Creation of new map with id"),
        "diagnostic_rows": len(diagnostics),
        "diagnostic_gate_rejects": sum(
            row.get("gate_decision") == "reject" for row in diagnostics
        ),
        "diagnostic_reprojection_px_mean": (
            mean(reprojection_mean) if reprojection_mean else None
        ),
        "diagnostic_reprojection_px_p90_mean": (
            mean(reprojection_p90) if reprojection_p90 else None
        ),
        "diagnostic_reprojection_px_p90_max": (
            max(reprojection_p90, default=None)
        ),
        "runtime_error_markers": len(re.findall(
            r"fatal|exception|cannot initialize|segmentation|terminate called",
            text,
            re.IGNORECASE,
        )),
    }


def compare(orb_path, hybrid_path):
    def indexed(path):
        with path.open(newline="", encoding="utf-8") as stream:
            return {row["stamp_ns"]: row for row in csv.DictReader(stream)}

    orb = indexed(orb_path)
    hybrid = indexed(hybrid_path)
    stamps = sorted(set(orb) & set(hybrid), key=int)
    depth_deltas = [
        int(hybrid[stamp]["valid_depths"]) - int(orb[stamp]["valid_depths"])
        for stamp in stamps
    ]
    keypoint_deltas = [
        int(hybrid[stamp]["keypoints"]) - int(orb[stamp]["keypoints"])
        for stamp in stamps
    ]
    inlier_deltas = [
        int(hybrid[stamp]["tracked_inliers"]) - int(orb[stamp]["tracked_inliers"])
        for stamp in stamps
    ]
    position_deltas = []
    rotation_deltas = []
    for stamp in stamps:
        orb_pose = pose(orb[stamp])
        hybrid_pose = pose(hybrid[stamp])
        if orb_pose is None or hybrid_pose is None:
            continue
        position_deltas.append(distance(orb_pose[0], hybrid_pose[0]))
        rotation_deltas.append(angle_degrees(orb_pose[1], hybrid_pose[1]))
    return {
        "common_callbacks": len(stamps),
        "orb_only_callbacks": len(orb) - len(stamps),
        "hybrid_only_callbacks": len(hybrid) - len(stamps),
        "valid_depth_delta_mean": mean(depth_deltas),
        "valid_depth_delta_min": min(depth_deltas),
        "valid_depth_regression_frames": sum(value < 0 for value in depth_deltas),
        "keypoint_delta_mean": mean(keypoint_deltas),
        "keypoint_mismatch_frames": sum(value != 0 for value in keypoint_deltas),
        "tracked_inlier_delta_mean": mean(inlier_deltas),
        "tracked_inlier_delta_p10": percentile(inlier_deltas, 0.10),
        "position_delta_m_p90": percentile(position_deltas, 0.90),
        "position_delta_m_max": max(position_deltas, default=None),
        "rotation_delta_deg_p90": percentile(rotation_deltas, 0.90),
        "rotation_delta_deg_max": max(rotation_deltas, default=None),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    runs = {}
    paths = {}
    for path in sorted(args.artifact_dir.glob("f22_*.csv")):
        runs[path.stem] = summarize(path)
        paths[path.stem] = path
        log_path = path.with_suffix(".log")
        if log_path.exists():
            runs[path.stem].update(summarize_log(log_path))
    if not runs:
        raise SystemExit("no F22 CSV files found")
    comparisons = {}
    gates = {}
    for stem, orb_path in paths.items():
        if not stem.endswith("_orb"):
            continue
        base = stem[:-4]
        hybrid_path = paths.get(base + "_hybrid")
        if hybrid_path is not None:
            comparisons[base] = compare(orb_path, hybrid_path)
            orb_run = runs[stem]
            hybrid_run = runs[base + "_hybrid"]
            gates[base] = {
                "keypoints_preserved": comparisons[base]["keypoint_mismatch_frames"] == 0,
                "valid_depth_non_regression": comparisons[base]["valid_depth_regression_frames"] == 0,
                "tracking_loss_non_regression": hybrid_run["lost_frames"] <= orb_run["lost_frames"],
                "tracking_loss_segment_non_regression": hybrid_run["lost_segments"] <= orb_run["lost_segments"],
                "runtime_error_free": (
                    hybrid_run.get("runtime_error_markers", 0) == 0
                    and orb_run.get("runtime_error_markers", 0) == 0
                ),
            }
            gates[base]["passing"] = all(gates[base].values())
    overall_pass = bool(gates) and all(result["passing"] for result in gates.values())
    args.output.write_text(
        json.dumps({"comparisons": comparisons, "gates": gates,
                    "overall_pass": overall_pass, "runs": runs},
                   indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(args.output)


if __name__ == "__main__":
    main()
