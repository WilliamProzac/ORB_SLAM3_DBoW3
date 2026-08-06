#!/usr/bin/env python3
"""Prepare normalized float32 featuremap inputs for OpenExplorer PTQ.

The input can be either a directory of ordinary images or a ROS1 bag image
topic.  Bag sampling is spread across the full recording so calibration does
not over-represent one short part of a camera run.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np


SUPPORTED_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff"}


def normalize_image(image, width, height):
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    if image.ndim == 2:
        gray = image.astype(np.float32)
    elif image.ndim == 3:
        gray = image.astype(np.float32).mean(axis=2)
    else:
        raise RuntimeError(f"Unsupported image rank: {image.ndim}")
    gray = (gray - gray.mean()) / np.sqrt(gray.var() + 1e-5)
    return np.ascontiguousarray(gray[None, None, :, :], dtype=np.float32)


def image_directory_samples(input_dir, limit):
    paths = sorted(
        path
        for path in input_dir.rglob("*")
        if path.is_file() and path.suffix.lower() in SUPPORTED_SUFFIXES
    )[:limit]
    if not paths:
        raise RuntimeError(f"No supported images found under {input_dir}")

    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Failed to read {path}")
        yield image, {"source": str(path.resolve())}


def decode_ros_image(message):
    if message.encoding not in ("mono8", "8UC1"):
        raise RuntimeError(
            f"ROS bag calibration currently requires mono8/8UC1, got "
            f"{message.encoding!r}"
        )
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    return np.ascontiguousarray(rows[:, : message.width])


def rosbag_samples(bag_path, topic, limit):
    try:
        import rosbag
    except ModuleNotFoundError:
        noetic_python = Path("/opt/ros/noetic/lib/python3/dist-packages")
        if noetic_python.is_dir():
            sys.path.append(str(noetic_python))
        import rosbag

    with rosbag.Bag(str(bag_path), "r") as bag:
        total = int(bag.get_message_count(topic_filters=[topic]))
        if total == 0:
            raise RuntimeError(f"No messages found on {topic} in {bag_path}")
        sample_count = min(limit, total)
        selected = np.linspace(0, total - 1, sample_count, dtype=np.int64)
        selected_set = set(int(index) for index in selected)
        for frame_index, (_, message, bag_time) in enumerate(
            bag.read_messages(topics=[topic])
        ):
            if frame_index not in selected_set:
                continue
            yield decode_ros_image(message), {
                "source": str(bag_path.resolve()),
                "topic": topic,
                "frame_index": frame_index,
                "stamp_ns": int(message.header.stamp.to_nsec()),
                "bag_time_ns": int(bag_time.to_nsec()),
                "source_shape": [int(message.height), int(message.width)],
                "source_encoding": message.encoding,
            }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_path", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--limit", type=int, default=100)
    parser.add_argument(
        "--rosbag-topic",
        help="Read evenly spaced mono8 frames from this ROS1 bag topic",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        help="Manifest path (default: a sibling of the tensor directory)",
    )
    args = parser.parse_args()

    if args.limit <= 0:
        raise RuntimeError("--limit must be positive")
    if args.width <= 0 or args.height <= 0:
        raise RuntimeError("--width and --height must be positive")
    if args.rosbag_topic:
        if not args.input_path.is_file():
            raise RuntimeError(f"ROS bag not found: {args.input_path}")
        samples = rosbag_samples(args.input_path, args.rosbag_topic, args.limit)
        source_type = "rosbag"
    else:
        if not args.input_path.is_dir():
            raise RuntimeError(f"Image directory not found: {args.input_path}")
        samples = image_directory_samples(args.input_path, args.limit)
        source_type = "image_directory"

    args.output_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for index, (image, source_metadata) in enumerate(samples):
        tensor = normalize_image(image, args.width, args.height)
        output = args.output_dir / f"{index:04d}.f32"
        tensor.tofile(output)
        entry = dict(source_metadata)
        entry.update(
            {
                "output": output.name,
                "shape": list(tensor.shape),
                "dtype": str(tensor.dtype),
                "mean": float(tensor.mean()),
                "variance": float(tensor.var()),
                "byte_size": output.stat().st_size,
                "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
            }
        )
        entries.append(entry)

    manifest = {
        "source_type": source_type,
        "count": len(entries),
        "recommended_minimum": 100,
        "sufficient_for_representative_ptq": len(entries) >= 100,
        "input_contract": {
            "shape": [1, 1, args.height, args.width],
            "dtype": "float32",
            "layout": "NCHW",
            "normalization": "per-image mean/variance, epsilon=1e-5",
        },
        "entries": entries,
    }
    manifest_path = args.manifest or (
        args.output_dir.parent / f"{args.output_dir.name}_manifest.json"
    )
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(manifest_path)
    if len(entries) < 100:
        print(
            f"WARNING: only {len(entries)} samples were generated; use about 100 "
            "representative camera images before PTQ."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
