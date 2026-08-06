#!/usr/bin/env python3
"""Evaluate geometry-guided sparse XFeat with optional Fine Matcher refinement."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from evaluate_rosbag import (
    DEFAULT_UPSTREAM,
    distribution,
    fundamental_inliers,
    grid_coverage,
    load_sampled_frames,
)


def grid_quota_order(
    points,
    rankings,
    limit,
    image_width,
    image_height,
    columns=8,
    rows=6,
    maximum_per_cell=8,
):
    """Return ranking indices after applying a deterministic hard grid quota."""
    import torch

    if limit < 0:
        raise ValueError("limit must be non-negative")
    if columns <= 0 or rows <= 0:
        raise ValueError("grid dimensions must be positive")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    order = torch.argsort(rankings, descending=True)
    if limit == 0 or maximum_per_cell <= 0:
        return order[:limit]

    cells = [[] for _ in range(columns * rows)]
    for index in order.tolist():
        x = float(points[index, 0])
        y = float(points[index, 1])
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        column = min(columns - 1, max(0, int(x * columns / image_width)))
        row = min(rows - 1, max(0, int(y * rows / image_height)))
        cell = row * columns + column
        if len(cells[cell]) < maximum_per_cell:
            cells[cell].append(index)

    selected = []
    for rank_in_cell in range(maximum_per_cell):
        round_indices = [
            cell[rank_in_cell] for cell in cells if len(cell) > rank_in_cell
        ]
        round_indices.sort(key=lambda index: (-float(rankings[index]), index))
        remaining = limit - len(selected)
        selected.extend(round_indices[:remaining])
        if len(selected) == limit:
            break
    return torch.tensor(selected, dtype=torch.long, device=order.device)


def guided_stereo_matches(
    features_a,
    features_b,
    minimum_cosine,
    ratio,
    vertical,
    disparity,
    limit,
    grid_columns=8,
    grid_rows=6,
    grid_maximum_per_cell=8,
):
    import torch

    points_a = torch.from_numpy(features_a["points"])
    points_b = torch.from_numpy(features_b["points"])
    descriptors_a = features_a["descriptors"].cpu()
    descriptors_b = features_b["descriptors"].cpu()
    similarities = descriptors_a @ descriptors_b.T
    dy = torch.abs(points_a[:, None, 1] - points_b[None, :, 1])
    dx = points_a[:, None, 0] - points_b[None, :, 0]
    allowed = (dy <= vertical) & (dx >= 0.0) & (dx <= disparity)
    similarities = similarities.masked_fill(~allowed, -2.0)

    count_b = similarities.shape[1]
    top_count = min(2, count_b)
    values, indices = torch.topk(similarities, k=top_count, dim=1)
    best = values[:, 0]
    best_indices = indices[:, 0]
    second = values[:, 1] if top_count == 2 else torch.full_like(best, -2.0)
    reverse = torch.argmax(similarities, dim=0)
    query_indices = torch.arange(len(points_a))
    mutual = reverse[best_indices] == query_indices
    best_distance = 1.0 - best
    second_distance = 1.0 - second
    ratio_ok = (second <= -1.0) | (best_distance < ratio * second_distance)
    keep = mutual & ratio_ok & (best >= minimum_cosine)

    query_indices = query_indices[keep]
    train_indices = best_indices[keep]
    cosines = best[keep]
    if len(query_indices):
        score_a = torch.from_numpy(features_a["scores"])[query_indices]
        score_b = torch.from_numpy(features_b["scores"])[train_indices]
        ranking = cosines * score_a * score_b
        image_width, image_height = features_a["image_size"]
        order = grid_quota_order(
            points_a[query_indices],
            ranking,
            limit,
            image_width,
            image_height,
            grid_columns,
            grid_rows,
            grid_maximum_per_cell,
        )
        query_indices = query_indices[order]
        train_indices = train_indices[order]
        cosines = cosines[order]
    return query_indices, train_indices, cosines


def refine_target(model, target, fixed, target_indices, fixed_indices, confidence):
    torch = model.torch
    if len(target_indices) == 0:
        return np.empty((0, 2), np.float32), np.empty((0,), np.float32), np.empty((0,), bool)
    target_desc = target["descriptors"][target_indices]
    fixed_desc = fixed["descriptors"][fixed_indices]
    descriptor_pairs = torch.cat([target_desc, fixed_desc], dim=-1)
    with torch.inference_mode():
        logits = model.model.net.fine_matcher(descriptor_pairs)
        probabilities = torch.softmax(logits * 3.0, dim=-1)
        fine_confidence = probabilities.max(dim=-1).values
        axis = torch.arange(8, device=logits.device, dtype=logits.dtype) - 4.0
        yy, xx = torch.meshgrid(axis, axis, indexing="ij")
        offsets_x = (probabilities * xx.reshape(1, 64)).sum(dim=-1)
        offsets_y = (probabilities * yy.reshape(1, 64)).sum(dim=-1)
        offsets = torch.stack([offsets_x, offsets_y], dim=-1)
    scales = target["scales"][target_indices.cpu().numpy(), None]
    refined = target["points"][target_indices.cpu().numpy()] + offsets.cpu().numpy() * scales
    confidence_values = fine_confidence.cpu().numpy()
    return refined.astype(np.float32), confidence_values, confidence_values > confidence


def official_refine_parity(model, target, fixed, target_indices, fixed_indices, confidence):
    """Compare the adapter against upstream refine_matches on identical tensors."""
    torch = model.torch
    device = model.model.dev
    d0 = {
        "descriptors": target["descriptors"].clone()[None],
        "keypoints": torch.from_numpy(target["points"].copy()).to(device)[None],
        "scales": torch.from_numpy(target["scales"].copy()).to(device)[None],
    }
    d1 = {
        "descriptors": fixed["descriptors"].clone()[None],
        "keypoints": torch.from_numpy(fixed["points"].copy()).to(device)[None],
        "scales": torch.from_numpy(fixed["scales"].copy()).to(device)[None],
    }
    with torch.inference_mode():
        upstream = model.model.refine_matches(
            d0,
            d1,
            matches=[(target_indices.to(device), fixed_indices.to(device))],
            batch_idx=0,
            fine_conf=confidence,
        ).cpu().numpy()
    adapted, _, accepted = refine_target(
        model, target, fixed, target_indices, fixed_indices, confidence
    )
    adapted = adapted[accepted]
    fixed_points = fixed["points"][fixed_indices.cpu().numpy()][accepted]
    if len(upstream) != len(adapted):
        raise RuntimeError(
            f"Official Fine parity count mismatch: upstream={len(upstream)} adapter={len(adapted)}"
        )
    target_error = float(np.max(np.abs(upstream[:, :2] - adapted))) if len(adapted) else 0.0
    fixed_error = float(np.max(np.abs(upstream[:, 2:] - fixed_points))) if len(adapted) else 0.0
    if target_error > 1e-4 or fixed_error > 1e-4:
        raise RuntimeError(
            f"Official Fine parity failed: target={target_error} fixed={fixed_error}"
        )
    return {
        "status": "passing",
        "input_matches": int(len(target_indices)),
        "accepted_matches": int(len(adapted)),
        "target_max_abs_error_px": target_error,
        "fixed_max_abs_error_px": fixed_error,
    }


class XFeatMethod:
    def __init__(
        self,
        upstream,
        top_k,
        feature_mode,
        grid_columns=8,
        grid_rows=6,
        extraction_grid_maximum_per_cell=-1,
    ):
        sys.path.insert(0, str(upstream.resolve()))
        import torch
        from modules.xfeat import XFeat

        self.torch = torch
        torch.set_num_threads(8)
        self.model = XFeat(top_k=top_k)
        self.top_k = top_k
        self.feature_mode = feature_mode
        self.grid_columns = grid_columns
        self.grid_rows = grid_rows
        self.extraction_grid_maximum_per_cell = extraction_grid_maximum_per_cell

    def extract(self, image):
        image_size = (image.shape[1], image.shape[0])
        if self.feature_mode == "sparse":
            output = self.model.detectAndCompute(image, top_k=self.top_k)[0]
            return {
                "points": output["keypoints"].cpu().numpy().astype(np.float32),
                "descriptors": output["descriptors"],
                "scores": output["scores"].cpu().numpy().astype(np.float32),
                "scales": np.ones(len(output["keypoints"]), dtype=np.float32),
                "image_size": image_size,
            }
        source = image[..., None] if image.ndim == 2 else image
        parsed = self.model.parse_input(source)
        use_grid_quota = self.extraction_grid_maximum_per_cell != 0
        output = self.model.detectAndComputeDense(
            parsed,
            top_k=0 if use_grid_quota else self.top_k,
            multiscale=self.feature_mode == "semi_dense_dual",
        )
        points_tensor = output["keypoints"][0]
        scores_tensor = output["scores"][0]
        if use_grid_quota:
            maximum_per_cell = self.extraction_grid_maximum_per_cell
            if maximum_per_cell < 0:
                maximum_per_cell = math.ceil(
                    self.top_k / (self.grid_columns * self.grid_rows)
                )
            selected = grid_quota_order(
                points_tensor,
                scores_tensor,
                self.top_k,
                image_size[0],
                image_size[1],
                self.grid_columns,
                self.grid_rows,
                maximum_per_cell,
            )
            points_tensor = points_tensor[selected]
            scores_tensor = scores_tensor[selected]
            descriptors = output["descriptors"][0][selected]
            scales = output["scales"][0][selected]
        else:
            descriptors = output["descriptors"][0]
            scales = output["scales"][0]
        points = points_tensor.cpu().numpy().astype(np.float32)
        return {
            "points": points,
            "descriptors": descriptors,
            "scores": scores_tensor.cpu().numpy().astype(np.float32),
            "scales": scales.cpu().numpy().astype(np.float32),
            "image_size": image_size,
        }


def normalized_patch_correlation(image_left, image_right, points_left, points_right, patch_size=9):
    half = patch_size // 2
    values = np.full(len(points_left), np.nan, dtype=np.float32)
    for index, (left, right) in enumerate(zip(points_left, points_right)):
        if (
            left[0] < half
            or left[0] >= image_left.shape[1] - half
            or left[1] < half
            or left[1] >= image_left.shape[0] - half
            or right[0] < half
            or right[0] >= image_right.shape[1] - half
            or right[1] < half
            or right[1] >= image_right.shape[0] - half
        ):
            continue
        patch_left = cv2.getRectSubPix(image_left, (patch_size, patch_size), tuple(left)).astype(np.float32)
        patch_right = cv2.getRectSubPix(image_right, (patch_size, patch_size), tuple(right)).astype(np.float32)
        patch_left -= patch_left.mean()
        patch_right -= patch_right.mean()
        denominator = float(np.linalg.norm(patch_left) * np.linalg.norm(patch_right))
        if denominator > 1e-6:
            values[index] = float(np.vdot(patch_left, patch_right) / denominator)
    return values


def pair_metrics(
    points_left,
    points_right,
    width,
    height,
    image_left=None,
    image_right=None,
    minimum_ncc=0.5,
):
    start = time.perf_counter()
    ransac = fundamental_inliers(points_left, points_right)
    vertical = np.abs(points_left[:, 1] - points_right[:, 1]) if len(points_left) else np.empty(0)
    disparity = points_left[:, 0] - points_right[:, 0] if len(points_left) else np.empty(0)
    strict = ransac & (vertical <= 2.0) & (disparity >= 0.0) & (disparity <= 160.0)
    ncc = (
        normalized_patch_correlation(image_left, image_right, points_left, points_right)
        if image_left is not None and image_right is not None and len(points_left)
        else np.empty(0, dtype=np.float32)
    )
    photometric = strict & np.isfinite(ncc) & (ncc >= minimum_ncc) if len(ncc) else np.zeros(len(strict), bool)
    finite_ncc = ncc[np.isfinite(ncc)]
    return {
        "matches": int(len(points_left)),
        "ransac_inliers": int(ransac.sum()),
        "strict_inliers": int(strict.sum()),
        "strict_inlier_ratio": float(strict.mean()) if len(strict) else 0.0,
        "strict_grid_coverage": grid_coverage(points_left[strict], width, height),
        "photometric_inliers": int(photometric.sum()),
        "photometric_inlier_ratio": float(photometric.mean()) if len(photometric) else 0.0,
        "photometric_grid_coverage": grid_coverage(points_left[photometric], width, height),
        "patch_ncc_median": float(np.median(finite_ncc)) if len(finite_ncc) else math.nan,
        "patch_ncc_p10": float(np.percentile(finite_ncc, 10)) if len(finite_ncc) else math.nan,
        "median_vertical_error_px": float(np.median(vertical)) if len(vertical) else math.nan,
        "geometry_ms": (time.perf_counter() - start) * 1000.0,
    }


def summarize(rows):
    fields = [
        "matches",
        "ransac_inliers",
        "strict_inliers",
        "strict_inlier_ratio",
        "strict_grid_coverage",
        "photometric_inliers",
        "photometric_inlier_ratio",
        "photometric_grid_coverage",
        "patch_ncc_median",
        "patch_ncc_p10",
        "median_vertical_error_px",
        "extract_left_ms",
        "extract_right_ms",
        "extract_pair_ms",
        "coarse_match_ms",
        "fine_match_ms",
        "geometry_ms",
    ]
    return {field: distribution([row[field] for row in rows]) for field in fields}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--upstream", type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument("--top-k", type=int, default=600)
    parser.add_argument(
        "--feature-mode",
        choices=("sparse", "semi_dense_single", "semi_dense_dual"),
        default="semi_dense_single",
    )
    parser.add_argument("--refine-target", choices=("left", "right"), default="right")
    parser.add_argument("--fine-size", type=int, default=384)
    parser.add_argument("--grid-columns", type=int, default=8)
    parser.add_argument("--grid-rows", type=int, default=6)
    parser.add_argument(
        "--extraction-grid-maximum-per-cell",
        type=int,
        default=-1,
        help="Semi-dense quota per cell; -1 is ceil(top-k/cells), 0 disables.",
    )
    parser.add_argument(
        "--grid-maximum-per-cell",
        type=int,
        default=8,
        help="Hard match quota per left-image grid cell; use 0 to disable.",
    )
    parser.add_argument("--min-cosine", type=float, default=0.82)
    parser.add_argument("--ratio", type=float, default=0.9)
    parser.add_argument("--fine-confidence", type=float, default=0.25)
    parser.add_argument(
        "--refine-y",
        action="store_true",
        help="Apply the learned vertical offset. Rectified stereo keeps y fixed by default.",
    )
    parser.add_argument("--vertical", type=float, default=2.0)
    parser.add_argument("--max-disparity", type=float, default=160.0)
    parser.add_argument("--minimum-ncc", type=float, default=0.5)
    parser.add_argument("--anchor-stride", type=int, default=10)
    parser.add_argument("--anchor-parity", choices=("all", "even", "odd"), default="all")
    parser.add_argument("--max-anchors", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("xfeat_rdk_x5/artifacts/fine_matcher_host.json"))
    parser.add_argument("--calibration-dir", type=Path)
    args = parser.parse_args()

    frames, _, _, _ = load_sampled_frames(args.bag, args.anchor_stride, 1, args.max_anchors)
    anchors = [frame for frame in frames if frame["frame_index"] % args.anchor_stride == 0]
    if args.anchor_parity != "all":
        parity = 0 if args.anchor_parity == "even" else 1
        anchors = [frame for ordinal, frame in enumerate(anchors) if ordinal % 2 == parity]
    if not anchors:
        raise SystemExit("No stereo anchors were selected")
    method = XFeatMethod(
        args.upstream,
        args.top_k,
        args.feature_mode,
        args.grid_columns,
        args.grid_rows,
        args.extraction_grid_maximum_per_cell,
    )
    method.extract(anchors[0]["left"])
    if args.calibration_dir:
        args.calibration_dir.mkdir(parents=True, exist_ok=True)

    coarse_rows = []
    fine_rows = []
    parity = None
    for ordinal, frame in enumerate(anchors):
        extract_start = time.perf_counter()
        left = method.extract(frame["left"])
        left_ms = (time.perf_counter() - extract_start) * 1000.0
        right_start = time.perf_counter()
        right = method.extract(frame["right"])
        right_ms = (time.perf_counter() - right_start) * 1000.0
        extract_pair_ms = (time.perf_counter() - extract_start) * 1000.0
        start = time.perf_counter()
        left_indices, right_indices, _ = guided_stereo_matches(
            left,
            right,
            args.min_cosine,
            args.ratio,
            args.vertical,
            args.max_disparity,
            args.fine_size,
            args.grid_columns,
            args.grid_rows,
            args.grid_maximum_per_cell,
        )
        coarse_ms = (time.perf_counter() - start) * 1000.0
        left_points = left["points"][left_indices.numpy()]
        right_points = right["points"][right_indices.numpy()]
        coarse = pair_metrics(
            left_points,
            right_points,
            frame["left"].shape[1],
            frame["left"].shape[0],
            frame["left"],
            frame["right"],
            args.minimum_ncc,
        )
        extraction = {
            "extract_left_ms": left_ms,
            "extract_right_ms": right_ms,
            "extract_pair_ms": extract_pair_ms,
        }
        coarse.update({**extraction, "coarse_match_ms": coarse_ms, "fine_match_ms": 0.0})
        coarse_rows.append(coarse)

        if parity is None and args.feature_mode != "sparse":
            if args.refine_target == "right":
                parity = official_refine_parity(
                    method, right, left, right_indices, left_indices, args.fine_confidence
                )
            else:
                parity = official_refine_parity(
                    method, left, right, left_indices, right_indices, args.fine_confidence
                )

        if args.calibration_dir:
            pairs = np.zeros((args.fine_size, 128), dtype=np.float32)
            valid = len(left_indices)
            if valid:
                if args.refine_target == "right":
                    target_desc = right["descriptors"][right_indices].cpu().numpy()
                    fixed_desc = left["descriptors"][left_indices].cpu().numpy()
                else:
                    target_desc = left["descriptors"][left_indices].cpu().numpy()
                    fixed_desc = right["descriptors"][right_indices].cpu().numpy()
                pairs[:valid] = np.concatenate([target_desc, fixed_desc], axis=1)
            pairs.tofile(str(args.calibration_dir / f"{ordinal:04d}.f32"))

        start = time.perf_counter()
        if args.refine_target == "right":
            refined_target, confidence_values, accepted = refine_target(
                method, right, left, right_indices, left_indices, args.fine_confidence
            )
            if not args.refine_y:
                refined_target[:, 1] = right_points[:, 1]
            refined_left = left_points
            refined_right = refined_target
        else:
            refined_target, confidence_values, accepted = refine_target(
                method, left, right, left_indices, right_indices, args.fine_confidence
            )
            if not args.refine_y:
                refined_target[:, 1] = left_points[:, 1]
            refined_left = refined_target
            refined_right = right_points
        fine_ms = (time.perf_counter() - start) * 1000.0
        in_bounds = (
            (refined_target[:, 0] >= 0.0)
            & (refined_target[:, 0] < frame["right"].shape[1])
            & (refined_target[:, 1] >= 0.0)
            & (refined_target[:, 1] < frame["right"].shape[0])
        )
        accepted &= in_bounds
        fine = pair_metrics(
            refined_left[accepted],
            refined_right[accepted],
            frame["left"].shape[1],
            frame["left"].shape[0],
            frame["left"],
            frame["right"],
            args.minimum_ncc,
        )
        fine.update(
            {
                "coarse_match_ms": coarse_ms,
                "fine_match_ms": fine_ms,
                "fine_confidence_mean": float(confidence_values.mean()) if len(confidence_values) else math.nan,
                **extraction,
            }
        )
        fine_rows.append(fine)
        print(
            f"{ordinal + 1}/{len(anchors)} frame={frame['frame_index']} "
            f"coarse={coarse['strict_inliers']} fine={fine['strict_inliers']}"
        )

    parameters = {}
    for key, value in vars(args).items():
        parameters[key] = str(value) if isinstance(value, Path) else value
    result = {
        "bag": str(args.bag.resolve()),
        "parameters": parameters,
        "pairs": len(anchors),
        "coarse": summarize(coarse_rows),
        "fine": summarize(fine_rows),
        "fine_confidence_mean": distribution([row["fine_confidence_mean"] for row in fine_rows]),
        "official_refine_parity": parity,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, default=str) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, default=str))


if __name__ == "__main__":
    main()
