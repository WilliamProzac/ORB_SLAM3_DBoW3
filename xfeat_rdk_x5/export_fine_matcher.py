#!/usr/bin/env python3
"""Export the official XFeat Fine Matcher as fixed-shape ONNX models.

The upstream module applies BatchNorm1d to a flattened list of descriptor
pairs.  Deployment uses [1, M, 128], so this exporter folds every BatchNorm
into its preceding Linear layer and wraps the M dimension explicitly.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch import nn


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UPSTREAM = ROOT / "Thirdparty" / "accelerated_features"


class FixedFineMatcher(nn.Module):
    def __init__(self, matcher: nn.Module, match_count: int):
        super().__init__()
        self.matcher = matcher
        self.match_count = int(match_count)

    def forward(self, descriptor_pairs):
        flat = descriptor_pairs.reshape(self.match_count, 128)
        logits = self.matcher(flat)
        return logits.reshape(1, self.match_count, 64)


def fold_linear_batch_norm(linear: nn.Linear, batch_norm: nn.BatchNorm1d):
    if batch_norm.training:
        raise RuntimeError("BatchNorm must be in evaluation mode before folding")
    scale = torch.rsqrt(batch_norm.running_var.detach() + batch_norm.eps)
    if batch_norm.affine:
        scale = scale * batch_norm.weight.detach()
        shift = batch_norm.bias.detach()
    else:
        shift = torch.zeros_like(scale)

    folded = nn.Linear(linear.in_features, linear.out_features, bias=True)
    bias = linear.bias.detach() if linear.bias is not None else torch.zeros_like(scale)
    with torch.no_grad():
        folded.weight.copy_(linear.weight.detach() * scale[:, None])
        folded.bias.copy_((bias - batch_norm.running_mean.detach()) * scale + shift)
    return folded


def fold_official_matcher(module: nn.Sequential):
    layers = list(module.children())
    folded = []
    index = 0
    while index < len(layers):
        layer = layers[index]
        if isinstance(layer, nn.Linear) and index + 1 < len(layers) and isinstance(
            layers[index + 1], nn.BatchNorm1d
        ):
            folded.append(fold_linear_batch_norm(layer, layers[index + 1]))
            index += 2
            continue
        if isinstance(layer, nn.Linear):
            clone = nn.Linear(layer.in_features, layer.out_features, layer.bias is not None)
            clone.load_state_dict(layer.state_dict())
            folded.append(clone)
        elif isinstance(layer, nn.ReLU):
            folded.append(nn.ReLU(inplace=False))
        else:
            raise RuntimeError(f"Unsupported Fine Matcher layer: {type(layer).__name__}")
        index += 1
    return nn.Sequential(*folded).eval()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument(
        "--output-dir", type=Path, default=Path(__file__).resolve().parent / "models"
    )
    parser.add_argument("--sizes", type=int, nargs="+", default=[256, 384, 512])
    parser.add_argument("--opset", type=int, default=11)
    parser.add_argument("--seed", type=int, default=0x58464541)
    args = parser.parse_args()

    if any(size <= 0 for size in args.sizes):
        raise SystemExit("All fixed match counts must be positive")
    sys.path.insert(0, str(args.upstream.resolve()))
    from modules.xfeat import XFeat

    torch.manual_seed(args.seed)
    torch.set_num_threads(1)
    xfeat = XFeat().to("cpu")
    official = xfeat.net.fine_matcher.eval()
    folded = fold_official_matcher(official)

    parity_input = torch.randn(max(args.sizes), 128)
    with torch.inference_mode():
        official_output = official(parity_input)
        folded_output = folded(parity_input)
    fold_error = float(torch.max(torch.abs(official_output - folded_output)))
    if fold_error > 2e-5:
        raise RuntimeError(f"BatchNorm folding parity failed: {fold_error}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = []
    for size in sorted(set(args.sizes)):
        model = FixedFineMatcher(folded, size).eval()
        sample = torch.randn(1, size, 128)
        output = args.output_dir / f"xfeat_fine_matcher_m{size}.onnx"
        torch.onnx.export(
            model,
            sample,
            str(output),
            input_names=["descriptor_pairs"],
            output_names=["fine_logits"],
            opset_version=args.opset,
            do_constant_folding=True,
            dynamic_axes=None,
        )
        graph = onnx.load(str(output))
        onnx.checker.check_model(graph)

        session = ort.InferenceSession(str(output), providers=["CPUExecutionProvider"])
        onnx_output = session.run(None, {"descriptor_pairs": sample.numpy()})[0]
        with torch.inference_mode():
            torch_output = model(sample).numpy()
        onnx_error = float(np.max(np.abs(onnx_output - torch_output)))
        if onnx_error > 2e-5:
            raise RuntimeError(f"ONNX parity failed for M={size}: {onnx_error}")

        input_shape = list(session.get_inputs()[0].shape)
        output_shape = list(session.get_outputs()[0].shape)
        expected_input = [1, size, 128]
        expected_output = [1, size, 64]
        if input_shape != expected_input or output_shape != expected_output:
            raise RuntimeError(
                f"Unexpected fixed shapes for M={size}: {input_shape} -> {output_shape}"
            )
        records.append(
            {
                "match_count": size,
                "path": str(output),
                "sha256": sha256(output),
                "bytes": output.stat().st_size,
                "input_shape": input_shape,
                "output_shape": output_shape,
                "onnx_max_abs_error": onnx_error,
            }
        )

    metadata = {
        "upstream": str(args.upstream.resolve()),
        "checkpoint": str((args.upstream / "weights" / "xfeat.pt").resolve()),
        "checkpoint_sha256": sha256(args.upstream / "weights" / "xfeat.pt"),
        "parameters": sum(parameter.numel() for parameter in official.parameters()),
        "fold_max_abs_error": fold_error,
        "opset": args.opset,
        "models": records,
    }
    metadata_path = args.output_dir / "xfeat_fine_matcher_export.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
