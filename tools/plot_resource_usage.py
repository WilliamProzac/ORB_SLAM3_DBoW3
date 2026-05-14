#!/usr/bin/env python3
import argparse
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TIME_RE = re.compile(r"^(\d{2})时(\d{2})分(\d{2})秒$")


def parse_time_seconds(time_str: str):
    m = TIME_RE.match(time_str)
    if not m:
        return None
    h, mi, s = map(int, m.groups())
    return h * 3600 + mi * 60 + s


def parse_log(path: Path):
    cpu_points = []  # (t, cpu_percent)
    ram_points = []  # (t, rss_mb)
    mode = None

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            if "UID" in line and "%CPU" in line:
                mode = "cpu"
                continue
            if "UID" in line and "RSS" in line and "%MEM" in line:
                mode = "mem"
                continue

            parts = line.split()
            if len(parts) < 9:
                continue

            t = parse_time_seconds(parts[0])
            if t is None:
                continue

            try:
                if mode == "cpu":
                    cpu_percent = float(parts[7])
                    cpu_points.append((t, cpu_percent))
                elif mode == "mem":
                    rss_kb = float(parts[6])
                    rss_mb = rss_kb / 1024.0
                    ram_points.append((t, rss_mb))
            except (ValueError, IndexError):
                continue

    cpu_points = normalize_time(cpu_points)
    ram_points = normalize_time(ram_points)
    return cpu_points, ram_points


def normalize_time(points):
    if not points:
        return []
    t0 = points[0][0]
    normalized = []
    day = 24 * 3600
    last_t = t0
    offset = 0

    for t, v in points:
        if t < last_t:
            offset += day
        tt = t + offset - t0
        normalized.append((tt, v))
        last_t = t

    return normalized


def plot_metric(series_dict, ylabel, title, output_path):
    plt.figure(figsize=(12, 6))

    for name, points in series_dict.items():
        if not points:
            continue
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        plt.plot(xs, ys, linewidth=1.4, label=name)

    plt.xlabel("Elapsed Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True, linestyle="--", alpha=0.35)
    plt.legend(fontsize=9)
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()


def plot_single_log(cpu_points, ram_points, title, output_path):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    if cpu_points:
        xs = [p[0] for p in cpu_points]
        ys = [p[1] for p in cpu_points]
        axes[0].plot(xs, ys, linewidth=1.5, color="#1f77b4")
    axes[0].set_ylabel("%CPU")
    axes[0].set_title(f"{title} - CPU")
    axes[0].grid(True, linestyle="--", alpha=0.35)

    if ram_points:
        xs = [p[0] for p in ram_points]
        ys = [p[1] for p in ram_points]
        axes[1].plot(xs, ys, linewidth=1.5, color="#d62728")
    axes[1].set_xlabel("Elapsed Time (s)")
    axes[1].set_ylabel("RAM RSS (MB)")
    axes[1].set_title(f"{title} - RAM")
    axes[1].grid(True, linestyle="--", alpha=0.35)

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Plot CPU and RAM usage from pidstat-style resource logs."
    )
    parser.add_argument("logs", nargs="+", help="Log file paths")
    parser.add_argument(
        "--out-dir",
        default="logs/plots",
        help="Output directory for generated figures",
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cpu_series = {}
    ram_series = {}

    for log_path in args.logs:
        path = Path(log_path)
        cpu_points, ram_points = parse_log(path)
        label = path.name.replace("resource_usage_", "")
        cpu_series[label] = cpu_points
        ram_series[label] = ram_points

        per_log_output = out_dir / f"{path.stem}.png"
        plot_single_log(cpu_points, ram_points, path.name, per_log_output)
        print(f"Per-log chart: {per_log_output}")

    cpu_output = out_dir / "cpu_usage_lines.png"
    ram_output = out_dir / "ram_usage_lines.png"

    plot_metric(cpu_series, "%CPU", "CPU Usage Comparison", cpu_output)
    plot_metric(ram_series, "RAM RSS (MB)", "RAM Usage Comparison", ram_output)

    print(f"CPU chart: {cpu_output}")
    print(f"RAM chart: {ram_output}")


if __name__ == "__main__":
    main()
