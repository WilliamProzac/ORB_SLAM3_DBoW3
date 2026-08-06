#!/usr/bin/env python3
"""Visualize production ORB, XFeat* coarse, and XFeat* Fine stereo points."""

import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from evaluate_fine_matcher import (
    XFeatMethod,
    guided_stereo_matches,
    refine_target,
)
from evaluate_rosbag import fundamental_inliers, grid_coverage, load_sampled_frames


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def strict_mask(left, right, vertical=2.0, maximum_disparity=160.0):
    cv2.setRNGSeed(0x58464541)
    ransac = fundamental_inliers(left, right)
    if not len(left):
        return ransac
    dy = np.abs(left[:, 1] - right[:, 1])
    disparity = left[:, 0] - right[:, 0]
    return ransac & (dy <= vertical) & (disparity >= 0.0) & (
        disparity <= maximum_disparity
    )


def cell_counts(points, width, height, columns=8, rows=6):
    result = np.zeros((rows, columns), dtype=np.int32)
    if len(points):
        x = np.clip((points[:, 0] * columns / width).astype(int), 0, columns - 1)
        y = np.clip((points[:, 1] * rows / height).astype(int), 0, rows - 1)
        np.add.at(result, (y, x), 1)
    return result


def load_orb_points(csv_path):
    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    detected = len(rows)
    valid = [row for row in rows if int(row["valid_depth"]) == 1]
    left = np.asarray(
        [[float(row["left_x"]), float(row["left_y"])] for row in valid],
        dtype=np.float32,
    ).reshape(-1, 2)
    right = np.asarray(
        [[float(row["right_x"]), float(row["left_y"])] for row in valid],
        dtype=np.float32,
    ).reshape(-1, 2)
    return detected, left, right


def draw_panel(axis, image, points, disparities, title, color_map="plasma"):
    axis.imshow(image, cmap="gray", vmin=0, vmax=255)
    if len(points):
        artist = axis.scatter(
            points[:, 0], points[:, 1], c=disparities, s=11, cmap=color_map,
            vmin=0.0, vmax=20.0, linewidths=0.25, edgecolors="black", alpha=0.9,
        )
    else:
        artist = None
    for x in np.linspace(0, image.shape[1], 9):
        axis.axvline(x, color="white", alpha=0.28, linewidth=0.6)
    for y in np.linspace(0, image.shape[0], 7):
        axis.axhline(y, color="white", alpha=0.28, linewidth=0.6)
    axis.set_title(title, fontsize=11)
    axis.set_xlim(0, image.shape[1])
    axis.set_ylim(image.shape[0], 0)
    axis.set_xticks([])
    axis.set_yticks([])
    return artist


def draw_heatmap(axis, counts, title):
    axis.imshow(counts, cmap="magma", vmin=0)
    for row in range(counts.shape[0]):
        for column in range(counts.shape[1]):
            value = int(counts[row, column])
            axis.text(column, row, str(value), ha="center", va="center",
                      color="white" if value else "#bbbbbb", fontsize=8)
    axis.set_title(title, fontsize=10)
    axis.set_xticks(range(counts.shape[1]))
    axis.set_yticks(range(counts.shape[0]))
    axis.set_xlabel("8 columns")
    axis.set_ylabel("6 rows")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frame-index", type=int, default=320)
    parser.add_argument(
        "--orb-helper",
        type=Path,
        default=ROOT / "xfeat_rdk_x5" / "artifacts" / "orb_slam3_stereo_dump",
    )
    parser.add_argument("--top-k", type=int, default=600)
    parser.add_argument("--fine-size", type=int, default=384)
    parser.add_argument("--grid-columns", type=int, default=8)
    parser.add_argument("--grid-rows", type=int, default=6)
    parser.add_argument("--extraction-grid-maximum-per-cell", type=int, default=-1)
    parser.add_argument("--grid-maximum-per-cell", type=int, default=8)
    parser.add_argument("--min-cosine", type=float, default=0.82)
    parser.add_argument("--ratio", type=float, default=0.9)
    parser.add_argument("--fine-confidence", type=float, default=0.25)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "xfeat_rdk_x5" / "artifacts" / "feature_visualization",
    )
    args = parser.parse_args()

    stride = 10
    max_anchors = args.frame_index // stride + 1
    frames, _, _, _ = load_sampled_frames(args.bag, stride, 1, max_anchors)
    selected = [frame for frame in frames if frame["frame_index"] == args.frame_index]
    if len(selected) != 1:
        raise RuntimeError(f"frame {args.frame_index} was not found as a stride-10 anchor")
    frame = selected[0]
    left_image = frame["left"]
    right_image = frame["right"]
    height, width = left_image.shape

    args.output_dir.mkdir(parents=True, exist_ok=True)
    left_path = args.output_dir / f"frame_{args.frame_index:04d}_left.png"
    right_path = args.output_dir / f"frame_{args.frame_index:04d}_right.png"
    orb_csv = args.output_dir / f"frame_{args.frame_index:04d}_orb_slam3.csv"
    cv2.imwrite(str(left_path), left_image)
    cv2.imwrite(str(right_path), right_image)
    subprocess.run(
        [str(args.orb_helper), str(left_path), str(right_path), str(orb_csv)],
        check=True,
    )
    orb_detected, orb_left, orb_right = load_orb_points(orb_csv)

    method = XFeatMethod(
        ROOT / "Thirdparty" / "accelerated_features",
        args.top_k,
        "semi_dense_single",
        args.grid_columns,
        args.grid_rows,
        args.extraction_grid_maximum_per_cell,
    )
    method.extract(left_image)  # warmup
    left = method.extract(left_image)
    right = method.extract(right_image)
    left_indices, right_indices, coarse_cosine = guided_stereo_matches(
        left,
        right,
        args.min_cosine,
        args.ratio,
        2.0,
        160.0,
        args.fine_size,
        args.grid_columns,
        args.grid_rows,
        args.grid_maximum_per_cell,
    )
    coarse_left_all = left["points"][left_indices.numpy()]
    coarse_right_all = right["points"][right_indices.numpy()]
    coarse_keep = strict_mask(coarse_left_all, coarse_right_all)
    coarse_left = coarse_left_all[coarse_keep]
    coarse_right = coarse_right_all[coarse_keep]

    refined_right, confidence, accepted = refine_target(
        method, right, left, right_indices, left_indices, args.fine_confidence
    )
    # The deployed rectified-stereo path refines horizontal position only.
    refined_right[:, 1] = coarse_right_all[:, 1]
    accepted &= (
        (refined_right[:, 0] >= 0.0)
        & (refined_right[:, 0] < width)
        & (refined_right[:, 1] >= 0.0)
        & (refined_right[:, 1] < height)
    )
    fine_left_all = coarse_left_all[accepted]
    fine_right_all = refined_right[accepted]
    fine_keep = strict_mask(fine_left_all, fine_right_all)
    fine_left = fine_left_all[fine_keep]
    fine_right = fine_right_all[fine_keep]

    groups = [
        ("ORB-SLAM3", orb_detected, orb_left, orb_right),
        ("XFeat* Coarse", len(left["points"]), coarse_left, coarse_right),
        ("XFeat* + Fine", len(left["points"]), fine_left, fine_right),
    ]
    metrics = {}
    figure, axes = plt.subplots(
        2, 3, figsize=(18, 10), gridspec_kw={"height_ratios": [2.2, 1.0]},
        constrained_layout=True,
    )
    scatter = None
    for column, (name, detected, points_left, points_right) in enumerate(groups):
        coverage = grid_coverage(points_left, width, height)
        disparities = points_left[:, 0] - points_right[:, 0]
        counts = cell_counts(points_left, width, height)
        occupied = int(np.count_nonzero(counts))
        title = (
            f"{name}\nraw detected={detected}, stereo points={len(points_left)}, "
            f"coverage={occupied}/48 ({coverage:.1%})"
        )
        current = draw_panel(axes[0, column], left_image, points_left, disparities, title)
        scatter = current if current is not None else scatter
        draw_heatmap(axes[1, column], counts, f"Points per occupied grid cell: {occupied}/48")
        metrics[name] = {
            "raw_detected_left": int(detected),
            "strict_stereo_points": int(len(points_left)),
            "occupied_grid_cells": occupied,
            "grid_coverage": float(coverage),
            "median_disparity_px": float(np.median(disparities)) if len(disparities) else None,
        }
    figure.suptitle(
        f"Bag frame {args.frame_index}: strict stereo-point count and spatial distribution\n"
        "Dots are left-image observations; color is horizontal disparity (px)",
        fontsize=15,
    )
    if scatter is not None:
        colorbar = figure.colorbar(scatter, ax=axes[0, :], shrink=0.72, pad=0.01)
        colorbar.set_label("Horizontal disparity (px), clipped to [0, 20]")
    output_png = args.output_dir / f"frame_{args.frame_index:04d}_orb_xfeat_coarse_fine.png"
    figure.savefig(output_png, dpi=160)
    plt.close(figure)

    result = {
        "bag": str(args.bag.resolve()),
        "bag_sha256": sha256(args.bag),
        "frame_index": args.frame_index,
        "stamp_ns": int(frame["stamp_ns"]),
        "image_shape_hw": [height, width],
        "definition": (
            "Top row shows left-image observations with a valid/strict stereo "
            "correspondence. Bottom row is the matching 8x6 cell population."
        ),
        "parameters": {
            "orb": "ORB-SLAM3 Frame/ComputeStereoMatches, S316 600 features, 5 levels",
            "xfeat_mode": "official XFeat* semi_dense_single",
            "top_k": args.top_k,
            "fine_size": args.fine_size,
            "grid_columns": args.grid_columns,
            "grid_rows": args.grid_rows,
            "extraction_grid_maximum_per_cell": (
                int(np.ceil(args.top_k / (args.grid_columns * args.grid_rows)))
                if args.extraction_grid_maximum_per_cell < 0
                else args.extraction_grid_maximum_per_cell
            ),
            "grid_maximum_per_cell": args.grid_maximum_per_cell,
            "minimum_cosine": args.min_cosine,
            "ratio": args.ratio,
            "fine_confidence": args.fine_confidence,
            "vertical_tolerance_px": 2.0,
            "maximum_disparity_px": 160.0,
            "fine_refine_y": False,
        },
        "coarse_candidates": int(len(left_indices)),
        "left_reliability": {
            "minimum": float(left["scores"].min()) if len(left["scores"]) else None,
            "mean": float(left["scores"].mean()) if len(left["scores"]) else None,
            "maximum": float(left["scores"].max()) if len(left["scores"]) else None,
        },
        "raw_xfeat_grid": {
            "occupied_cells": int(
                np.count_nonzero(cell_counts(left["points"], width, height))
            ),
            "maximum_points_per_cell": int(
                cell_counts(left["points"], width, height).max()
            ),
        },
        "fine_confidence_accepted": int(accepted.sum()),
        "fine_confidence_mean": float(confidence.mean()) if len(confidence) else None,
        "metrics": metrics,
        "outputs": {
            "visualization": str(output_png.resolve()),
            "left_image": str(left_path.resolve()),
            "right_image": str(right_path.resolve()),
            "orb_dump": str(orb_csv.resolve()),
        },
    }
    output_json = args.output_dir / f"frame_{args.frame_index:04d}_metrics.json"
    output_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
