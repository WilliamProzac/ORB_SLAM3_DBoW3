#!/usr/bin/env python3
"""Run a deterministic CPU smoke test of the official XFeat sparse pipeline."""

import argparse
import hashlib
import json
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UPSTREAM = ROOT / "Thirdparty" / "accelerated_features"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_revision(path: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()


def percentile(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, round((len(ordered) - 1) * fraction))
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--top-k", type=int, default=1024)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    upstream = args.upstream.resolve()
    sys.path.insert(0, str(upstream))

    import cv2
    import numpy as np
    import torch
    from modules.xfeat import XFeat

    torch.manual_seed(0)
    torch.set_num_threads(args.threads)

    image_paths = [upstream / "assets" / "ref.png", upstream / "assets" / "tgt.png"]
    images = []
    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(path)
        images.append(cv2.resize(image, (args.width, args.height), interpolation=cv2.INTER_AREA))

    extractor = XFeat(top_k=args.top_k)
    for _ in range(args.warmup):
        extractor.detectAndCompute(images[0], top_k=args.top_k)

    timings_ms = []
    first_output = None
    for _ in range(args.repeats):
        start = time.perf_counter()
        output = extractor.detectAndCompute(images[0], top_k=args.top_k)[0]
        timings_ms.append((time.perf_counter() - start) * 1000.0)
        if first_output is None:
            first_output = output

    second_output = extractor.detectAndCompute(images[1], top_k=args.top_k)[0]
    match0, match1 = extractor.match(
        first_output["descriptors"], second_output["descriptors"]
    )

    descriptor_norms = first_output["descriptors"].norm(dim=1)
    checks = {
        "finite_keypoints": bool(torch.isfinite(first_output["keypoints"]).all()),
        "finite_descriptors": bool(torch.isfinite(first_output["descriptors"]).all()),
        "finite_scores": bool(torch.isfinite(first_output["scores"]).all()),
        "descriptor_width_is_64": first_output["descriptors"].shape[1] == 64,
        "descriptor_norm_close_to_one": bool(
            torch.allclose(descriptor_norms, torch.ones_like(descriptor_norms), atol=1e-5)
        ),
        "both_images_have_features": min(
            len(first_output["keypoints"]), len(second_output["keypoints"])
        ) > 0,
        "mutual_matches_found": len(match0) > 0 and len(match0) == len(match1),
    }
    if not all(checks.values()):
        raise RuntimeError(f"XFeat smoke checks failed: {checks}")

    result = {
        "status": "passing",
        "upstream_revision": git_revision(upstream),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "torch": torch.__version__,
        "opencv": cv2.__version__,
        "numpy": np.__version__,
        "cuda_available": torch.cuda.is_available(),
        "threads": args.threads,
        "input_shape_nchw": [1, 3, args.height, args.width],
        "top_k": args.top_k,
        "image_sha256": {path.name: sha256(path) for path in image_paths},
        "features": [len(first_output["keypoints"]), len(second_output["keypoints"])],
        "mutual_matches_min_cossim_0_82": len(match0),
        "descriptor_norm_mean": float(descriptor_norms.mean()),
        "latency_ms": {
            "samples": len(timings_ms),
            "mean": statistics.mean(timings_ms),
            "median": statistics.median(timings_ms),
            "p95": percentile(timings_ms, 0.95),
            "min": min(timings_ms),
            "max": max(timings_ms),
        },
        "checks": checks,
    }

    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
