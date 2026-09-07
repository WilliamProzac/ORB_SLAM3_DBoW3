#!/usr/bin/env python3
"""Analyze the F35 host-bag/RDK-X5 ultra-long RGB-D OOM run."""

import argparse
import bisect
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path
from statistics import mean

import rosbag


RGB_TOPIC = "/camera/infra1/image_raw"
DEPTH_TOPIC = "/camera/depth/image_raw"


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def stats(values):
    return {
        "mean": mean(values) if values else None,
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else None,
    }


def source_pairs(path):
    stamps = {RGB_TOPIC: {}, DEPTH_TOPIC: {}}
    with rosbag.Bag(path) as bag:
        duration = bag.get_end_time() - bag.get_start_time()
        for topic, message, _ in bag.read_messages(topics=[RGB_TOPIC, DEPTH_TOPIC]):
            stamps[topic][message.header.stamp.to_nsec()] = int(message.header.seq)
    common = sorted(set(stamps[RGB_TOPIC]) & set(stamps[DEPTH_TOPIC]))
    exact = [stamps[RGB_TOPIC][stamp] for stamp in common]
    return {
        "duration_s": duration,
        "message_count": len(stamps[RGB_TOPIC]) + len(stamps[DEPTH_TOPIC]),
        "rgb_frames": len(stamps[RGB_TOPIC]),
        "depth_frames": len(stamps[DEPTH_TOPIC]),
        "exact_pairs": len(common),
        "exact_rgb_sequences": exact,
    }


def parse_meminfo(path):
    values = {}
    for line in path.read_text(errors="replace").splitlines():
        match = re.match(r"^(\w+):\s+(\d+) kB$", line)
        if match and match.group(1) not in values:
            values[match.group(1)] = int(match.group(2))
    return values


def runs(values, predicate):
    result = []
    start = None
    for index, value in enumerate(values):
        if predicate(value) and start is None:
            start = index
        if start is not None and (not predicate(value) or index == len(values) - 1):
            end = index if predicate(value) else index - 1
            result.append([start, end])
            start = None
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact_root", type=Path)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    root = args.artifact_root.resolve()
    bag_path = args.bag.resolve()
    run = next((root / "runs").glob("f35_long154955_f700_l5_r1"))

    timing = read_rows(run / "timing" / "rgbd_frame_timing.csv")
    mapping = read_rows(run / "timing" / "local_mapping_timing.csv")
    telemetry = read_rows(run / "telemetry.csv")
    log = (run / "slam.log").read_text(errors="replace")
    kernel_before = (run / "board_system_before.txt").read_text(errors="replace")
    kernel_after = (run / "board_system_after.txt").read_text(errors="replace")
    before_mem = parse_meminfo(run / "board_system_before.txt")
    after_mem = parse_meminfo(run / "board_system_after.txt")
    source = source_pairs(bag_path)

    callback_sequences = [int(row["rgb_seq"]) for row in timing]
    callback_set = set(callback_sequences)
    source_set = set(source.pop("exact_rgb_sequences"))
    missing_sequences = sorted(source_set - callback_set)
    unexpected_sequences = sorted(callback_set - source_set)
    states = Counter(int(row["state_after"]) for row in timing)
    non_ok_runs = runs([int(row["state_after"]) for row in timing], lambda value: value != 2)
    non_ok_episodes = [
        {
            "callback_first": first,
            "callback_last": last,
            "rgb_seq_first": callback_sequences[first],
            "rgb_seq_last": callback_sequences[last],
            "states": dict(Counter(int(timing[i]["state_after"]) for i in range(first, last + 1))),
        }
        for first, last in non_ok_runs
    ]

    floats = lambda rows_, key: [float(row[key]) for row in rows_]
    rss = [value / 1024 for value in floats(telemetry, "node_rss_kb")]
    vsz = [value / 1024 for value in floats(telemetry, "node_vsz_kb")]
    node_swap = [value / 1024 for value in floats(telemetry, "node_vmswap_kb")]
    available = [value / 1024 for value in floats(telemetry, "mem_available_kb")]
    swap_free = [value / 1024 for value in floats(telemetry, "swap_free_kb")]
    committed = [value / 1024 for value in floats(telemetry, "committed_as_kb")]
    commit_limit = [value / 1024 for value in floats(telemetry, "commit_limit_kb")]
    callback_ms = floats(timing, "callback_total_ms")

    oom_pattern = re.compile(
        r"out of memory|oom-kill|killed process|memory cgroup out of memory|std::bad_alloc|segfault",
        re.IGNORECASE,
    )
    kernel_new_text = kernel_after[len(kernel_before) :] if kernel_after.startswith(kernel_before) else kernel_after
    kernel_oom_lines = [line for line in kernel_new_text.splitlines() if oom_pattern.search(line)]
    log_failure_lines = [line for line in log.splitlines() if oom_pattern.search(line)]

    map_count = log.count("New Map created with")
    keyframes = len(mapping)
    rss_growth = rss[-1] - rss[0]
    result = {
        "source": {
            "bag": str(bag_path),
            "sha256": "e28da82eb7231ddfde67661e01cea6306e3eb374e32bc163fd6deba6b3efc475",
            **source,
        },
        "configuration": {
            "orb_features": 700,
            "orb_levels": 5,
            "orb_scale_factor": 1.2,
            "fast_initial": 20,
            "fast_min": 7,
            "extractor_workers": 3,
            "playback_rate": 1.0,
            "bag_location": "host 192.168.1.3",
            "slam_location": "RDK-X5 192.168.1.10",
        },
        "delivery": {
            "callbacks": len(timing),
            "source_exact_pairs": source["exact_pairs"],
            "delivery_percent": len(timing) / source["exact_pairs"] * 100,
            "missing_exact_pair_count": len(missing_sequences),
            "missing_exact_pair_sequences": missing_sequences,
            "unexpected_callback_sequences": unexpected_sequences,
        },
        "tracking_and_map": {
            "state_after_counts": dict(states),
            "non_ok_episodes": non_ok_episodes,
            "inserted_keyframes": sum(int(row["inserted_keyframe"]) for row in timing),
            "processed_keyframes": keyframes,
            "last_keyframe_id": max(int(row["keyframe_id"]) for row in mapping),
            "atlas_maps_created": map_count,
            "loop_detections": log.count("*Loop detected"),
            "clean_shutdown_marker": "Shutdown: All threads finished" in log,
        },
        "timing_ms": {
            "callback_total": stats(callback_ms),
            "callback_over_90_ms": sum(value > 90 for value in callback_ms),
            "orb_extract": stats(floats(timing, "orb_extract_ms")),
            "local_mapping_total": stats(floats(mapping, "total_ms")),
            "header_age_growth": float(timing[-1]["header_age_ms"]) - float(timing[0]["header_age_ms"]),
        },
        "memory": {
            "samples": len(telemetry),
            "rss_mib": {"start": rss[0], "end": rss[-1], "peak": max(rss), "growth": rss_growth},
            "vsz_mib": {"start": vsz[0], "end": vsz[-1], "peak": max(vsz)},
            "node_swap_mib": {"start": node_swap[0], "end": node_swap[-1], "peak": max(node_swap)},
            "system_mem_available_mib": {"start": available[0], "end": available[-1], "minimum": min(available)},
            "system_swap_consumed_mib_peak": swap_free[0] - min(swap_free),
            "committed_as_mib": {"start": committed[0], "end": committed[-1], "peak": max(committed)},
            "commit_limit_mib": commit_limit[0],
            "minimum_commit_headroom_mib": commit_limit[0] - max(committed),
            "minor_fault_delta": int(telemetry[-1]["node_minflt"]) - int(telemetry[0]["node_minflt"]),
            "major_fault_delta": int(telemetry[-1]["node_majflt"]) - int(telemetry[0]["node_majflt"]),
            "observed_growth_mib_per_processed_keyframe": rss_growth / keyframes,
            "board_meminfo_kb_before": {key: before_mem.get(key) for key in ("MemAvailable", "SwapFree", "Committed_AS")},
            "board_meminfo_kb_after": {key: after_mem.get(key) for key in ("MemAvailable", "SwapFree", "Committed_AS")},
        },
        "oom_verdict": {
            "rosbag_exit_code": int((run / "rosbag_play_exit_code.txt").read_text()),
            "slam_exit_code": int((run / "slam_exit_code.txt").read_text()),
            "kernel_oom_lines": kernel_oom_lines,
            "slam_allocation_or_segfault_lines": log_failure_lines,
            "oom_observed": bool(kernel_oom_lines or log_failure_lines),
            "global_ba_exercised": log.count("*Loop detected") > 0,
        },
    }

    (root / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        wall = floats(telemetry, "timestamp_s")
        elapsed = [value - wall[0] for value in wall]
        figure, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        axes[0].plot(elapsed, rss, label="node RSS")
        axes[0].plot(elapsed, available, label="system MemAvailable")
        axes[0].plot(elapsed, node_swap, label="node VmSwap")
        axes[0].set_ylabel("MiB")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[1].plot(elapsed, committed, label="Committed_AS")
        axes[1].axhline(commit_limit[0], color="red", linestyle="--", label="CommitLimit")
        axes[1].set_xlabel("elapsed wall time [s]")
        axes[1].set_ylabel("MiB")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        figure.tight_layout()
        figure.savefig(root / "memory_pressure.png", dpi=160)
        plt.close(figure)
    except ImportError:
        pass

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
