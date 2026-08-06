#!/usr/bin/env python3
"""Export the fixed-shape XFeat CNN core and verify PyTorch/ONNX parity.

The exported model expects an already instance-normalized grayscale float32
tensor. Sparse NMS, keypoint selection, descriptor sampling, and matching stay
outside the graph so the BPU graph remains fixed-shape and convolution-heavy.
"""

import argparse
import copy
import hashlib
import json
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UPSTREAM = ROOT / "Thirdparty" / "accelerated_features"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def revision(path: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()


def normalize_like_instance_norm(rgb, eps=1e-5):
    gray = rgb.mean(dim=1, keepdim=True)
    mean = gray.mean(dim=(2, 3), keepdim=True)
    variance = gray.var(dim=(2, 3), unbiased=False, keepdim=True)
    return (gray - mean) / (variance + eps).sqrt()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--opset", type=int, default=11)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "xfeat_rdk_x5" / "artifacts" / "xfeat_backbone_480x640_opset11.onnx",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=ROOT / "xfeat_rdk_x5" / "artifacts" / "onnx_validation.json",
    )
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.report = args.report.resolve()

    upstream = args.upstream.resolve()
    sys.path.insert(0, str(upstream))

    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    from modules.model import XFeatModel

    class XFeatBPUCore(nn.Module):
        """Fixed-shape core with deployment-friendly keypoint stem.

        The upstream 8x8 unfold followed by a 1x1 convolution is exactly
        equivalent to one 8x8 stride-8 convolution. Folding those operations
        avoids hundreds of Slice/Transpose nodes in the exported ONNX graph.
        """

        def __init__(self, source):
            super().__init__()
            self.skip1 = copy.deepcopy(source.skip1)
            self.block1 = copy.deepcopy(source.block1)
            self.block2 = copy.deepcopy(source.block2)
            self.block3 = copy.deepcopy(source.block3)
            self.block4 = copy.deepcopy(source.block4)
            self.block5 = copy.deepcopy(source.block5)
            self.block_fusion = copy.deepcopy(source.block_fusion)
            self.heatmap_head = copy.deepcopy(source.heatmap_head)

            upstream_stem = source.keypoint_head[0].layer
            pointwise = upstream_stem[0]
            fused = nn.Conv2d(1, pointwise.out_channels, 8, stride=8, bias=False)
            with torch.no_grad():
                fused.weight.copy_(pointwise.weight[:, :, 0, 0].reshape(-1, 1, 8, 8))
            self.keypoint_stem = nn.Sequential(
                fused,
                copy.deepcopy(upstream_stem[1]),
                copy.deepcopy(upstream_stem[2]),
            )
            self.keypoint_tail = copy.deepcopy(source.keypoint_head[1:])

        def forward(self, x):
            x1 = self.block1(x)
            x2 = self.block2(x1 + self.skip1(x))
            x3 = self.block3(x2)
            x4 = self.block4(x3)
            x5 = self.block5(x4)

            x4 = F.interpolate(x4, x3.shape[-2:], mode="bilinear")
            x5 = F.interpolate(x5, x3.shape[-2:], mode="bilinear")
            features = self.block_fusion(x3 + x4 + x5)
            reliability = self.heatmap_head(features)
            keypoints = self.keypoint_tail(self.keypoint_stem(x))
            return features, keypoints, reliability

    torch.manual_seed(0)
    torch.set_num_threads(8)

    weights = upstream / "weights" / "xfeat.pt"
    reference = XFeatModel().eval()
    reference.load_state_dict(torch.load(weights, map_location="cpu"))

    bpu_core = XFeatBPUCore(reference).eval()

    rgb = torch.randn(1, 3, args.height, args.width, dtype=torch.float32)
    normalized_gray = normalize_like_instance_norm(rgb)
    with torch.inference_mode():
        reference_outputs = reference(rgb)
        split_outputs = bpu_core(normalized_gray)

    split_max_abs = [
        float((expected - actual).abs().max())
        for expected, actual in zip(reference_outputs, split_outputs)
    ]
    if max(split_max_abs) > 2e-5:
        raise RuntimeError(f"External normalization changed outputs: {split_max_abs}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        bpu_core,
        normalized_gray,
        str(args.output),
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=["image"],
        output_names=["dense_descriptors", "keypoint_logits", "reliability"],
        dynamic_axes=None,
    )

    model = onnx.load(str(args.output))
    onnx.checker.check_model(model)
    operator_counts = Counter(node.op_type for node in model.graph.node)

    session = ort.InferenceSession(str(args.output), providers=["CPUExecutionProvider"])
    ort_input = normalized_gray.numpy()
    ort_outputs = session.run(None, {"image": ort_input})
    ort_max_abs = [
        float(np.max(np.abs(expected.detach().numpy() - actual)))
        for expected, actual in zip(split_outputs, ort_outputs)
    ]
    if max(ort_max_abs) > 1e-4:
        raise RuntimeError(f"ONNX Runtime parity failed: {ort_max_abs}")

    for _ in range(2):
        session.run(None, {"image": ort_input})
    timings_ms = []
    for _ in range(10):
        start = time.perf_counter()
        session.run(None, {"image": ort_input})
        timings_ms.append((time.perf_counter() - start) * 1000.0)

    report = {
        "status": "passing",
        "upstream_revision": revision(upstream),
        "weights_sha256": sha256(weights),
        "onnx_sha256": sha256(args.output),
        "onnx_path": str(args.output.relative_to(ROOT)),
        "opset": args.opset,
        "input_contract": {
            "name": "image",
            "dtype": "float32",
            "layout": "NCHW",
            "shape": [1, 1, args.height, args.width],
            "preprocessing": "RGB/BGR mean to gray, then per-image InstanceNorm(eps=1e-5) on CPU",
        },
        "outputs": {
            name: list(value.shape)
            for name, value in zip(
                ["dense_descriptors", "keypoint_logits", "reliability"], ort_outputs
            )
        },
        "external_normalization_max_abs": split_max_abs,
        "onnxruntime_max_abs": ort_max_abs,
        "onnxruntime_latency_ms": {
            "samples": len(timings_ms),
            "mean": sum(timings_ms) / len(timings_ms),
            "min": min(timings_ms),
            "max": max(timings_ms),
        },
        "operator_counts": dict(sorted(operator_counts.items())),
        "parameter_count_full_module": sum(p.numel() for p in reference.parameters()),
        "parameter_count_exported_core": sum(p.numel() for p in bpu_core.parameters()),
        "note": "The 8x8 unfold plus first keypoint 1x1 convolution is fused into one 8x8 stride-8 convolution; fine_matcher is outside this graph",
    }

    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
