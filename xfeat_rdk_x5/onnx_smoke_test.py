#!/usr/bin/env python3
"""Validate the ONNX-core/CPU-postprocess split against official XFeat."""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UPSTREAM = ROOT / "Thirdparty" / "accelerated_features"
DEFAULT_MODEL = ROOT / "xfeat_rdk_x5" / "artifacts" / "xfeat_backbone_480x640_opset11.onnx"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--top-k", type=int, default=1024)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "xfeat_rdk_x5" / "artifacts" / "onnx_pipeline_smoke.json",
    )
    args = parser.parse_args()

    sys.path.insert(0, str(args.upstream.resolve()))

    import cv2
    import numpy as np
    import onnxruntime as ort
    import torch
    import torch.nn.functional as F
    from modules.interpolator import InterpolateSparse2d
    from modules.xfeat import XFeat

    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    extractor = XFeat(top_k=args.top_k)
    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = args.threads
    session_options.inter_op_num_threads = 1
    session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    session = ort.InferenceSession(
        str(args.model), sess_options=session_options, providers=["CPUExecutionProvider"]
    )
    nearest = InterpolateSparse2d("nearest")
    bilinear = InterpolateSparse2d("bilinear")
    stage_timings = []

    def infer_split(image, record_stages=False):
        start_preprocess = time.perf_counter()
        tensor = torch.from_numpy(image).permute(2, 0, 1)[None].float()
        gray = tensor.mean(dim=1, keepdim=True)
        gray = (gray - gray.mean(dim=(2, 3), keepdim=True)) / (
            gray.var(dim=(2, 3), unbiased=False, keepdim=True) + 1e-5
        ).sqrt()
        start_onnx = time.perf_counter()
        dense_np, logits_np, reliability_np = session.run(None, {"image": gray.numpy()})
        start_nms = time.perf_counter()
        dense = F.normalize(torch.from_numpy(dense_np), dim=1)
        logits = torch.from_numpy(logits_np)
        reliability = torch.from_numpy(reliability_np)

        heatmap = extractor.get_kpts_heatmap(logits)
        keypoints = extractor.NMS(
            heatmap, threshold=extractor.detection_threshold, kernel_size=5
        )
        start_selection = time.perf_counter()
        scores = (
            nearest(heatmap, keypoints, args.height, args.width)
            * bilinear(reliability, keypoints, args.height, args.width)
        ).squeeze(-1)
        scores[torch.all(keypoints == 0, dim=-1)] = -1
        indices = torch.argsort(-scores)
        keypoints = torch.gather(
            keypoints, 1, indices[..., None].expand(-1, -1, 2)
        )[:, : args.top_k]
        scores = torch.gather(scores, -1, indices)[:, : args.top_k]
        start_descriptors = time.perf_counter()
        descriptors = extractor.interpolator(
            dense, keypoints, H=args.height, W=args.width
        )
        descriptors = F.normalize(descriptors, dim=-1)
        valid = scores[0] > 0
        end = time.perf_counter()
        if record_stages:
            stage_timings.append(
                {
                    "preprocess": (start_onnx - start_preprocess) * 1000.0,
                    "onnx_core": (start_nms - start_onnx) * 1000.0,
                    "heatmap_nms": (start_selection - start_nms) * 1000.0,
                    "score_topk": (start_descriptors - start_selection) * 1000.0,
                    "descriptor_sample": (end - start_descriptors) * 1000.0,
                }
            )
        return {
            "keypoints": keypoints[0][valid],
            "scores": scores[0][valid],
            "descriptors": descriptors[0][valid],
        }

    image_paths = [args.upstream / "assets" / "ref.png", args.upstream / "assets" / "tgt.png"]
    images = []
    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(path)
        images.append(cv2.resize(image, (args.width, args.height), interpolation=cv2.INTER_AREA))

    reference_outputs = [
        extractor.detectAndCompute(image, top_k=args.top_k)[0] for image in images
    ]
    split_outputs = [infer_split(image) for image in images]

    parity = []
    for expected, actual in zip(reference_outputs, split_outputs):
        if expected["keypoints"].shape != actual["keypoints"].shape:
            raise RuntimeError(
                f"Feature count changed: {expected['keypoints'].shape} vs {actual['keypoints'].shape}"
            )
        expected_coords = [tuple(point.tolist()) for point in expected["keypoints"]]
        actual_coords = [tuple(point.tolist()) for point in actual["keypoints"]]
        actual_by_coord = {coord: index for index, coord in enumerate(actual_coords)}
        set_equal = set(expected_coords) == set(actual_coords)
        if not set_equal:
            raise RuntimeError("ONNX split changed the selected keypoint coordinate set")
        actual_indices = torch.tensor(
            [actual_by_coord[coord] for coord in expected_coords], dtype=torch.long
        )
        aligned_scores = actual["scores"][actual_indices]
        aligned_descriptors = actual["descriptors"][actual_indices]
        parity.append(
            {
                "feature_count": len(actual["keypoints"]),
                "keypoint_set_equal": set_equal,
                "keypoint_order_identical": expected_coords == actual_coords,
                "score_max_abs_after_coordinate_alignment": float(
                    (expected["scores"] - aligned_scores).abs().max()
                ),
                "descriptor_max_abs_after_coordinate_alignment": float(
                    (expected["descriptors"] - aligned_descriptors).abs().max()
                ),
            }
        )

    reference_matches = extractor.match(
        reference_outputs[0]["descriptors"], reference_outputs[1]["descriptors"]
    )
    split_matches = extractor.match(
        split_outputs[0]["descriptors"], split_outputs[1]["descriptors"]
    )

    def matched_coordinate_pairs(outputs, matches):
        left, right = matches
        return {
            (
                tuple(outputs[0]["keypoints"][index0].tolist()),
                tuple(outputs[1]["keypoints"][index1].tolist()),
            )
            for index0, index1 in zip(left.tolist(), right.tolist())
        }

    reference_match_pairs = matched_coordinate_pairs(reference_outputs, reference_matches)
    split_match_pairs = matched_coordinate_pairs(split_outputs, split_matches)

    for _ in range(2):
        infer_split(images[0])
    timings_ms = []
    for _ in range(args.repeats):
        start = time.perf_counter()
        infer_split(images[0], record_stages=True)
        timings_ms.append((time.perf_counter() - start) * 1000.0)

    checks = {
        "keypoint_sets_equal": all(item["keypoint_set_equal"] for item in parity),
        "scores_close_after_alignment": max(
            item["score_max_abs_after_coordinate_alignment"] for item in parity
        ) < 1e-5,
        "descriptors_close_after_alignment": max(
            item["descriptor_max_abs_after_coordinate_alignment"] for item in parity
        ) < 1e-4,
        "match_coordinate_pairs_equal": reference_match_pairs == split_match_pairs,
    }
    if not all(checks.values()):
        raise RuntimeError(f"Split-pipeline checks failed: {checks}, parity={parity}")

    report = {
        "status": "passing",
        "model": str(args.model.relative_to(ROOT)),
        "input_shape_nchw": [1, 1, args.height, args.width],
        "top_k": args.top_k,
        "threads": args.threads,
        "parity": parity,
        "reference_match_count": len(reference_matches[0]),
        "split_match_count": len(split_matches[0]),
        "end_to_end_onnx_cpu_ms": {
            "samples": len(timings_ms),
            "mean": statistics.mean(timings_ms),
            "median": statistics.median(timings_ms),
            "min": min(timings_ms),
            "max": max(timings_ms),
        },
        "stage_mean_ms": {
            name: statistics.mean(sample[name] for sample in stage_timings)
            for name in stage_timings[0]
        },
        "checks": checks,
        "note": "Timing includes grayscale/InstanceNorm preprocessing, ONNX Runtime core, and sparse CPU postprocessing; matching is excluded.",
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
