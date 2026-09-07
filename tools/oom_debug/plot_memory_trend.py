#!/usr/bin/env python3
"""Plot board memory pressure and keyframe growth for an OOM debug run."""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def as_float(row, key, default=0.0):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return default


def loss_spans(times, states):
    spans = []
    start = None
    for timestamp, state in zip(times, states):
        is_lost = state != 2
        if is_lost and start is None:
            start = timestamp
        elif not is_lost and start is not None:
            spans.append((start, timestamp))
            start = None
    if start is not None:
        spans.append((start, times[-1]))
    return spans


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    telemetry_path = args.run_dir / "board" / "telemetry.csv"
    keyframes_path = args.run_dir / "board" / "keyframe_timeline.csv"
    telemetry = read_csv(telemetry_path)
    keyframes = read_csv(keyframes_path)
    if not telemetry or not keyframes:
        raise SystemExit("telemetry or keyframe timeline is empty")

    t0 = as_float(telemetry[0], "epoch_s")
    tx = [(as_float(row, "epoch_s") - t0) / 60.0 for row in telemetry]
    rss = [as_float(row, "vmrss_kb") / 1024.0 for row in telemetry]
    process_swap = [as_float(row, "vmswap_kb") / 1024.0 for row in telemetry]
    footprint = [a + b for a, b in zip(rss, process_swap)]
    mem_available = [as_float(row, "mem_available_kb") / 1024.0 for row in telemetry]
    swap_free = [as_float(row, "swap_free_kb") / 1024.0 for row in telemetry]

    kx = [(as_float(row, "image_timestamp") - t0) / 60.0 for row in keyframes]
    cumulative_kf = [int(as_float(row, "cumulative_inserted_keyframes")) for row in keyframes]
    states = [int(as_float(row, "state_after")) for row in keyframes]
    spans = loss_spans(kx, states)

    plt.rcParams["axes.unicode_minus"] = False
    cjk_font = font_manager.FontProperties(
        fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    )
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True,
                             gridspec_kw={"height_ratios": [1.35, 1.0, 1.0]})

    ax = axes[0]
    ax.plot(tx, footprint, color="#c62828", linewidth=2.2, label="RSS + 进程 Swap")
    ax.plot(tx, rss, color="#1565c0", linewidth=1.8, label="RSS")
    ax.plot(tx, process_swap, color="#ef6c00", linewidth=1.8, label="进程 Swap")
    peak = max(range(len(footprint)), key=footprint.__getitem__)
    ax.scatter([tx[peak]], [footprint[peak]], color="#c62828", s=35, zorder=5)
    ax.annotate(f"峰值 {footprint[peak]:.0f} MiB",
                (tx[peak], footprint[peak]), xytext=(-92, 16),
                textcoords="offset points", arrowprops={"arrowstyle": "->"},
                fontproperties=cjk_font)
    ax.set_ylabel("进程内存 / MiB", fontproperties=cjk_font)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", ncol=3, prop=cjk_font)

    # Add a second horizontal scale using the actual time at which each
    # keyframe milestone was reached. This preserves non-uniform insertion
    # rates instead of assuming that keyframes grow linearly with time.
    max_kf = cumulative_kf[-1]
    milestones = list(range(0, (max_kf // 250) * 250 + 1, 250))
    if not milestones or milestones[-1] != max_kf:
        milestones.append(max_kf)
    milestone_times = []
    for target in milestones:
        index = next(
            (i for i, count in enumerate(cumulative_kf) if count >= target),
            len(cumulative_kf) - 1,
        )
        milestone_times.append(kx[index])
    kf_axis = ax.twiny()
    kf_axis.set_xlim(ax.get_xlim())
    kf_axis.set_xticks(milestone_times)
    kf_axis.set_xticklabels([str(value) for value in milestones])
    kf_axis.set_xlabel("对应的累计关键帧数量", fontproperties=cjk_font, labelpad=7)
    kf_axis.tick_params(axis="x", colors="#37474f")

    ax = axes[1]
    ax.plot(tx, mem_available, color="#2e7d32", linewidth=2.0, label="系统 MemAvailable")
    ax.plot(tx, swap_free, color="#6a1b9a", linewidth=1.8, label="系统 SwapFree")
    minimum = min(range(len(mem_available)), key=mem_available.__getitem__)
    ax.scatter([tx[minimum]], [mem_available[minimum]], color="#2e7d32", s=35, zorder=5)
    ax.annotate(f"最低 {mem_available[minimum]:.0f} MiB",
                (tx[minimum], mem_available[minimum]), xytext=(10, 20),
                textcoords="offset points", arrowprops={"arrowstyle": "->"},
                fontproperties=cjk_font)
    ax.set_ylabel("系统剩余内存 / MiB", fontproperties=cjk_font)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="center left", prop=cjk_font)

    ax = axes[2]
    ax.plot(kx, cumulative_kf, color="#37474f", linewidth=2.0,
            label="累计插入关键帧")
    for index, (start, end) in enumerate(spans):
        ax.axvspan(start, end, color="#d32f2f", alpha=0.20,
                   label="跟踪非 OK" if index == 0 else None)
    ax.set_ylabel("关键帧数量", fontproperties=cjk_font)
    ax.set_xlabel("从监控启动开始的时间 / min", fontproperties=cjk_font)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", prop=cjk_font)

    fig.suptitle(
        f"RDK-X5 手持 RGB-D 建图内存趋势\n{args.run_dir.name}（700 特征，5 层）",
        fontsize=15,
        fontproperties=cjk_font,
    )
    fig.text(
        0.995,
        0.006,
        "红色阴影：Tracking state != OK；RSS 平台期由 Swap 增长掩盖",
        ha="right",
        fontsize=9,
        color="#555555",
        fontproperties=cjk_font,
    )
    fig.tight_layout(rect=(0, 0.025, 1, 0.94))

    output = args.output or args.run_dir / "memory_trend.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    print(output.resolve())


if __name__ == "__main__":
    main()
