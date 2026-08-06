#!/usr/bin/env python3
"""Compare ORB and XFeat tracking with one common stereo-depth/PnP estimator."""

import argparse
import csv
import hashlib
import json
import math
import subprocess
import tempfile
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from evaluate_fine_matcher import (
    XFeatMethod,
    grid_quota_order,
    guided_stereo_matches,
    normalized_patch_correlation,
    refine_target,
)
from evaluate_rosbag import distribution, fundamental_inliers, grid_coverage, load_sampled_frames


ROOT = Path(__file__).resolve().parents[1]


class ORBSlam3Helper:
    """Adapter around the real S316 ORBextractor + Frame stereo path."""

    def __init__(self, executable):
        self.executable = Path(executable).resolve()
        if not self.executable.is_file():
            raise FileNotFoundError(f"ORB-SLAM3 helper not found: {self.executable}")
        self.temporary = tempfile.TemporaryDirectory(prefix="f21-orb-slam3-")
        self.ordinal = 0

    def close(self):
        self.temporary.cleanup()

    def extract(self, left, right):
        prefix = Path(self.temporary.name) / f"frame_{self.ordinal:04d}"
        self.ordinal += 1
        left_path = prefix.with_name(prefix.name + "_left.png")
        right_path = prefix.with_name(prefix.name + "_right.png")
        csv_path = prefix.with_suffix(".csv")
        if not cv2.imwrite(str(left_path), left) or not cv2.imwrite(str(right_path), right):
            raise RuntimeError("failed to write temporary ORB-SLAM3 input images")
        start = time.perf_counter()
        completed = subprocess.run(
            [str(self.executable), str(left_path), str(right_path), str(csv_path)],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        latency_ms = (time.perf_counter() - start) * 1000.0
        with csv_path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        descriptors = np.asarray(
            [list(bytes.fromhex(row["descriptor_hex"])) for row in rows], np.uint8
        ).reshape(-1, 32)
        features = {
            "points": np.asarray(
                [[float(row["left_x"]), float(row["left_y"])] for row in rows],
                np.float32,
            ).reshape(-1, 2),
            "descriptors": descriptors,
            "scores": np.asarray([float(row["response"]) for row in rows], np.float32),
            "scales": np.asarray(
                [1.2 ** int(row["octave"]) for row in rows], np.float32
            ),
            "image_size": (left.shape[1], left.shape[0]),
        }
        valid_indices = np.asarray(
            [int(row["index"]) for row in rows if int(row["valid_depth"])], np.int32
        )
        right_points = np.asarray(
            [
                [float(row["right_x"]), float(row["left_y"])]
                for row in rows
                if int(row["valid_depth"])
            ],
            np.float32,
        ).reshape(-1, 2)
        depth_by_index = {
            int(row["index"]): float(row["depth"])
            for row in rows
            if int(row["valid_depth"])
        }
        return features, valid_indices, right_points, depth_by_index, latency_ms, completed.stdout.strip()


def orb_extract(extractor, image):
    keypoints, descriptors = extractor.detectAndCompute(image, None)
    return {
        "points": np.asarray([point.pt for point in keypoints], np.float32).reshape(-1, 2),
        "descriptors": descriptors,
        "scores": np.asarray([point.response for point in keypoints], np.float32),
        "scales": np.ones(len(keypoints), np.float32),
        "image_size": (image.shape[1], image.shape[0]),
    }


def orb_matches(a, b, maximum_hamming):
    if a["descriptors"] is None or b["descriptors"] is None:
        return np.empty((0,), np.int32), np.empty((0,), np.int32)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = [
        match for match in matcher.match(a["descriptors"], b["descriptors"])
        if match.distance <= maximum_hamming
    ]
    matches.sort(key=lambda match: (match.distance, match.queryIdx, match.trainIdx))
    return (
        np.asarray([match.queryIdx for match in matches], np.int32),
        np.asarray([match.trainIdx for match in matches], np.int32),
    )


def xfeat_matches(method, a, b, args, temporal):
    match_quota = getattr(
        method, "vo_grid_maximum_per_cell", args.grid_maximum_per_cell
    )
    match_limit = getattr(method, "vo_match_limit", args.fine_size)
    if temporal:
        torch = method.torch
        points_a = torch.from_numpy(a["points"])
        points_b = torch.from_numpy(b["points"])
        similarities = a["descriptors"].cpu() @ b["descriptors"].cpu().T
        dx = torch.abs(points_a[:, None, 0] - points_b[None, :, 0])
        dy = torch.abs(points_a[:, None, 1] - points_b[None, :, 1])
        similarities = similarities.masked_fill(
            (dx > args.temporal_radius) | (dy > args.temporal_radius), -2.0
        )
        values, indices = torch.topk(similarities, k=min(2, len(points_b)), dim=1)
        best = values[:, 0]
        train = indices[:, 0]
        second = values[:, 1] if values.shape[1] == 2 else torch.full_like(best, -2.0)
        reverse = torch.argmax(similarities, dim=0)
        query = torch.arange(len(points_a))
        ratio_ok = (second <= -1.0) | (
            (1.0 - best) < args.ratio * (1.0 - second)
        )
        keep = (
            (reverse[train] == query)
            & ratio_ok
            & (best >= args.minimum_cosine)
        )
        query = query[keep]
        train = train[keep]
        if len(query):
            ranking = (
                best[keep]
                * torch.from_numpy(a["scores"])[query]
                * torch.from_numpy(b["scores"])[train]
            )
            order = grid_quota_order(
                points_a[query], ranking, match_limit,
                a["image_size"][0], a["image_size"][1],
                args.grid_columns, args.grid_rows, match_quota,
            )
            query = query[order]
            train = train[order]
    else:
        query, train, _ = guided_stereo_matches(
            a,
            b,
            args.minimum_cosine,
            args.ratio,
            args.vertical_tolerance,
            args.maximum_disparity,
            match_limit,
            args.grid_columns,
            args.grid_rows,
            match_quota,
        )
    query_np = query.numpy()
    train_np = train.numpy()
    if getattr(method, "vo_use_fine", True):
        refined, confidence, accepted = refine_target(
            method, b, a, train, query, args.fine_confidence
        )
        if not temporal:
            refined[:, 1] = b["points"][train_np, 1]
    else:
        refined = b["points"][train_np].copy()
        confidence = np.ones(len(refined), dtype=np.float32)
        accepted = np.ones(len(refined), dtype=bool)
    width, height = a["image_size"]
    accepted &= (
        (refined[:, 0] >= 0.0)
        & (refined[:, 0] < width)
        & (refined[:, 1] >= 0.0)
        & (refined[:, 1] < height)
    )
    return query_np[accepted], train_np[accepted], refined[accepted], confidence


def stereo_depths(
    left_features,
    query_indices,
    right_points,
    left_image,
    right_image,
    args,
):
    left_points = left_features["points"][query_indices]
    disparity = left_points[:, 0] - right_points[:, 0]
    vertical = np.abs(left_points[:, 1] - right_points[:, 1])
    geometric = (
        (vertical <= args.vertical_tolerance)
        & (disparity >= args.minimum_disparity)
        & (disparity <= args.maximum_disparity)
    )
    ncc = normalized_patch_correlation(left_image, right_image, left_points, right_points)
    valid = geometric & np.isfinite(ncc) & (ncc >= args.minimum_ncc)
    depth_by_index = {
        int(index): float(args.fx * args.baseline / disparity[item])
        for item, index in enumerate(query_indices)
        if valid[item]
    }
    return depth_by_index, {
        "stereo_matches": int(len(query_indices)),
        "stereo_geometric_depths": int(geometric.sum()),
        "stereo_ncc_depths": int(valid.sum()),
        "stereo_ncc_ratio": float(valid.mean()) if len(valid) else 0.0,
        "stereo_ncc_coverage": grid_coverage(
            left_points[valid], left_image.shape[1], left_image.shape[0]
        ),
        "stereo_ncc_median": float(np.nanmedian(ncc)) if np.isfinite(ncc).any() else math.nan,
    }


def pnp_metrics(previous, current, query_indices, current_points, args):
    object_points = []
    image_points = []
    for item, query in enumerate(query_indices):
        depth = previous["depth_by_index"].get(int(query))
        if depth is None or not math.isfinite(depth):
            continue
        u, v = previous["left_features"]["points"][query]
        object_points.append(
            [(u - args.cx) * depth / args.fx, (v - args.cy) * depth / args.fy, depth]
        )
        image_points.append(current_points[item])
    object_points = np.asarray(object_points, np.float32).reshape(-1, 3)
    image_points = np.asarray(image_points, np.float32).reshape(-1, 2)
    start = time.perf_counter()
    success = False
    inliers = np.empty((0,), np.int32)
    reprojection = np.empty((0,), np.float32)
    translation = math.nan
    rotation = math.nan
    if len(object_points) >= 6:
        camera = np.asarray(
            [[args.fx, 0.0, args.cx], [0.0, args.fy, args.cy], [0.0, 0.0, 1.0]],
            np.float64,
        )
        cv2.setRNGSeed(0x58464541)
        solved, rvec, tvec, inlier_rows = cv2.solvePnPRansac(
            object_points,
            image_points,
            camera,
            None,
            iterationsCount=args.pnp_iterations,
            reprojectionError=args.pnp_reprojection,
            confidence=0.999,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if solved and inlier_rows is not None:
            inliers = inlier_rows.reshape(-1).astype(np.int32)
            success = len(inliers) >= args.minimum_pnp_inliers
            if len(inliers) >= 6:
                rvec, tvec = cv2.solvePnPRefineLM(
                    object_points[inliers], image_points[inliers], camera, None, rvec, tvec
                )
                projected, _ = cv2.projectPoints(
                    object_points[inliers], rvec, tvec, camera, None
                )
                reprojection = np.linalg.norm(
                    projected.reshape(-1, 2) - image_points[inliers], axis=1
                )
                translation = float(np.linalg.norm(tvec))
                rotation = float(np.linalg.norm(rvec))
    latency_ms = (time.perf_counter() - start) * 1000.0
    return {
        "pnp_candidates": int(len(object_points)),
        "pnp_success": int(success),
        "pnp_inliers": int(len(inliers)),
        "pnp_inlier_ratio": float(len(inliers) / len(object_points)) if len(object_points) else 0.0,
        "pnp_reprojection_median_px": float(np.median(reprojection)) if len(reprojection) else math.nan,
        "pnp_reprojection_p90_px": float(np.percentile(reprojection, 90)) if len(reprojection) else math.nan,
        "pnp_grid_coverage": grid_coverage(
            image_points[inliers], current["image"].shape[1], current["image"].shape[0]
        ) if len(inliers) else 0.0,
        "relative_translation_m": translation,
        "relative_rotation_rad": rotation,
        "pnp_ms": latency_ms,
    }


def summarize(rows):
    fields = [
        "extract_pair_ms", "stereo_matches", "stereo_geometric_depths",
        "stereo_ncc_depths", "stereo_ncc_ratio", "stereo_ncc_coverage",
        "stereo_ncc_median", "temporal_matches", "temporal_ransac_inliers",
        "temporal_ransac_ratio", "temporal_coverage", "pnp_candidates",
        "pnp_inliers", "pnp_inlier_ratio", "pnp_reprojection_median_px",
        "pnp_reprojection_p90_px", "pnp_grid_coverage", "pnp_ms",
    ]
    result = {}
    for method in sorted({row["method"] for row in rows}):
        selected = [row for row in rows if row["method"] == method]
        result[method] = {
            "pairs": len(selected),
            "pnp_success_rate": sum(row["pnp_success"] for row in selected) / len(selected),
            "fragile_pnp_pairs": sum(row["pnp_inliers"] < 30 for row in selected),
            **{field: distribution([row[field] for row in selected]) for field in fields},
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--top-k", type=int, default=600)
    parser.add_argument("--fine-size", type=int, default=384)
    parser.add_argument("--grid-columns", type=int, default=8)
    parser.add_argument("--grid-rows", type=int, default=6)
    parser.add_argument("--grid-maximum-per-cell", type=int, default=8)
    parser.add_argument("--minimum-cosine", type=float, default=0.82)
    parser.add_argument("--ratio", type=float, default=0.9)
    parser.add_argument("--fine-confidence", type=float, default=0.25)
    parser.add_argument("--vertical-tolerance", type=float, default=2.0)
    parser.add_argument("--maximum-disparity", type=float, default=160.0)
    parser.add_argument("--minimum-disparity", type=float, default=0.5)
    parser.add_argument("--temporal-radius", type=float, default=16.0)
    parser.add_argument("--minimum-ncc", type=float, default=0.5)
    parser.add_argument("--orb-maximum-hamming", type=float, default=64.0)
    parser.add_argument("--anchor-stride", type=int, default=10)
    parser.add_argument("--temporal-gap", type=int, default=1)
    parser.add_argument("--max-anchors", type=int, default=0)
    parser.add_argument("--fx", type=float, default=293.225983)
    parser.add_argument("--fy", type=float, default=293.225983)
    parser.add_argument("--cx", type=float, default=312.983093)
    parser.add_argument("--cy", type=float, default=274.322723)
    parser.add_argument("--baseline", type=float, default=0.060087482)
    parser.add_argument("--pnp-iterations", type=int, default=200)
    parser.add_argument("--pnp-reprojection", type=float, default=2.0)
    parser.add_argument("--minimum-pnp-inliers", type=int, default=30)
    parser.add_argument(
        "--orb-slam3-helper",
        type=Path,
        default=ROOT / "xfeat_rdk_x5" / "artifacts" / "orb_slam3_stereo_dump",
    )
    parser.add_argument(
        "--method",
        action="append",
        dest="selected_methods",
        help="Evaluate only this method (repeatable); the default evaluates all.",
    )
    args = parser.parse_args()

    frames, _, _, _ = load_sampled_frames(
        args.bag, args.anchor_stride, args.temporal_gap, args.max_anchors
    )
    xfeat_grid = XFeatMethod(
        ROOT / "Thirdparty" / "accelerated_features", args.top_k,
        "semi_dense_single", args.grid_columns, args.grid_rows, -1,
    )
    xfeat_grid.vo_grid_maximum_per_cell = args.grid_maximum_per_cell
    xfeat_global = XFeatMethod(
        ROOT / "Thirdparty" / "accelerated_features", args.top_k,
        "semi_dense_single", args.grid_columns, args.grid_rows, 0,
    )
    xfeat_global.vo_grid_maximum_per_cell = 0
    xfeat_extract_grid = XFeatMethod(
        ROOT / "Thirdparty" / "accelerated_features", args.top_k,
        "semi_dense_single", args.grid_columns, args.grid_rows, -1,
    )
    xfeat_extract_grid.vo_grid_maximum_per_cell = 0
    xfeat_grid16 = XFeatMethod(
        ROOT / "Thirdparty" / "accelerated_features", args.top_k,
        "semi_dense_single", args.grid_columns, args.grid_rows, -1,
    )
    xfeat_grid16.vo_grid_maximum_per_cell = 16
    xfeat_grid20 = XFeatMethod(
        ROOT / "Thirdparty" / "accelerated_features", args.top_k,
        "semi_dense_single", args.grid_columns, args.grid_rows, -1,
    )
    xfeat_grid20.vo_grid_maximum_per_cell = 20
    xfeat_sparse = XFeatMethod(
        ROOT / "Thirdparty" / "accelerated_features", args.top_k,
        "sparse", args.grid_columns, args.grid_rows, 0,
    )
    xfeat_sparse.vo_grid_maximum_per_cell = 0
    xfeat_sparse.vo_match_limit = args.top_k
    xfeat_sparse.vo_use_fine = False
    methods = {
        "xfeat_sparse_coarse": xfeat_sparse,
        "xfeat_grid_fine": xfeat_grid,
        "xfeat_global_fine": xfeat_global,
        "xfeat_extract_grid_fine": xfeat_extract_grid,
        "xfeat_grid16_fine": xfeat_grid16,
        "xfeat_grid20_fine": xfeat_grid20,
        "orb_control": cv2.ORB_create(
            nfeatures=args.top_k, scaleFactor=1.2, nlevels=5,
            edgeThreshold=31, fastThreshold=18,
        ),
        "orb_slam3_production": ORBSlam3Helper(args.orb_slam3_helper),
    }
    if args.selected_methods:
        unknown = sorted(set(args.selected_methods) - set(methods))
        if unknown:
            raise ValueError(f"unknown methods: {', '.join(unknown)}")
        methods = {name: methods[name] for name in args.selected_methods}
    for method in methods.values():
        if isinstance(method, XFeatMethod):
            method.extract(frames[0]["left"])
        elif isinstance(method, ORBSlam3Helper):
            method.extract(frames[0]["left"], frames[0]["right"])
        else:
            orb_extract(method, frames[0]["left"])

    previous = {}
    rows = []
    for ordinal, frame in enumerate(frames):
        phase = frame["frame_index"] % args.anchor_stride
        for name, method in methods.items():
            start = time.perf_counter()
            helper_result = None
            if isinstance(method, ORBSlam3Helper):
                helper_result = method.extract(frame["left"], frame["right"])
                left_features = helper_result[0]
            else:
                left_features = (
                    method.extract(frame["left"])
                    if isinstance(method, XFeatMethod)
                    else orb_extract(method, frame["left"])
                )
            right_features = None
            if phase == 0 and not isinstance(method, ORBSlam3Helper):
                right_features = (
                    method.extract(frame["right"])
                    if isinstance(method, XFeatMethod)
                    else orb_extract(method, frame["right"])
                )
            extract_ms = (time.perf_counter() - start) * 1000.0

            if phase == 0:
                if isinstance(method, ORBSlam3Helper):
                    query, right_points, helper_depths = (
                        helper_result[1], helper_result[2], helper_result[3]
                    )
                elif isinstance(method, XFeatMethod):
                    query, _, right_points, _ = xfeat_matches(
                        method, left_features, right_features, args, temporal=False
                    )
                else:
                    query, train = orb_matches(
                        left_features, right_features, args.orb_maximum_hamming
                    )
                    right_points = right_features["points"][train]
                depth_by_index, stereo = stereo_depths(
                    left_features, query, right_points, frame["left"], frame["right"], args
                )
                if isinstance(method, ORBSlam3Helper):
                    # Keep the exact depths selected by Frame::ComputeStereoMatches;
                    # the common NCC metrics remain independently recomputed above.
                    depth_by_index = {
                        index: helper_depths[index]
                        for index in depth_by_index
                        if index in helper_depths
                    }
                previous[name] = {
                    "frame_index": frame["frame_index"],
                    "left_features": left_features,
                    "depth_by_index": depth_by_index,
                    "image": frame["left"],
                    "extract_pair_ms": extract_ms,
                    **stereo,
                }
                continue

            prior = previous.get(name)
            if prior is None or frame["frame_index"] - prior["frame_index"] != args.temporal_gap:
                continue
            start = time.perf_counter()
            if isinstance(method, XFeatMethod):
                query, _, current_points, _ = xfeat_matches(
                    method, prior["left_features"], left_features, args, temporal=True
                )
            else:
                query, train = orb_matches(
                    prior["left_features"], left_features, args.orb_maximum_hamming
                )
                current_points = left_features["points"][train]
            temporal_ms = (time.perf_counter() - start) * 1000.0
            previous_points = prior["left_features"]["points"][query]
            ransac = fundamental_inliers(previous_points, current_points)
            current = {"image": frame["left"]}
            pnp = pnp_metrics(prior, current, query, current_points, args)
            row = {
                "method": name,
                "frame_index": frame["frame_index"],
                "stamp_ns": frame["stamp_ns"],
                "orb_tracking_ok": frame.get("orb_slam_tracking_ok"),
                "extract_pair_ms": prior["extract_pair_ms"] + extract_ms,
                "temporal_match_ms": temporal_ms,
                "temporal_matches": int(len(query)),
                "temporal_ransac_inliers": int(ransac.sum()),
                "temporal_ransac_ratio": float(ransac.mean()) if len(ransac) else 0.0,
                "temporal_coverage": grid_coverage(
                    previous_points[ransac], frame["left"].shape[1], frame["left"].shape[0]
                ),
                **{key: prior[key] for key in prior if key.startswith("stereo_")},
                **pnp,
            }
            rows.append(row)
        if (ordinal + 1) % 20 == 0:
            print(f"{ordinal + 1}/{len(frames)} sampled frames")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "per_pair.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    parameters = {
        key: str(value) if isinstance(value, Path) else value
        for key, value in vars(args).items()
    }
    summary = {
        "bag": str(args.bag.resolve()),
        "bag_sha256": hashlib.sha256(args.bag.read_bytes()).hexdigest(),
        "parameters": parameters,
        "methods": summarize(rows),
    }
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, default=str) + "\n")
    for method in methods.values():
        if isinstance(method, ORBSlam3Helper):
            method.close()
    print(json.dumps(summary["methods"], indent=2))


if __name__ == "__main__":
    main()
