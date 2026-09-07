# RDK-X5 RGB-D hand-held OOM capture

This tooling runs the validated `production_w3` RGB-D executable with the
`S316_f700_l5.yaml` settings and captures enough evidence to distinguish a
kernel OOM kill, an uncaught allocation failure, a signal crash, and a board
reboot.

Run the preflight from the laptop:

```bash
tools/oom_debug/run_handheld_oom_test.sh --preflight
```

Start a real hand-held session from a terminal that will remain open:

```bash
tools/oom_debug/run_handheld_oom_test.sh handheld_$(date +%Y%m%d_%H%M%S)
```

板卡上的 `orb_slam3` 标准输出和错误输出会实时显示在当前终端，
同时完整保存到 `handheld_oom_runs/RUN_ID/board_stream.log`。如需从另一个
终端旁路观察，可对该文件执行 `tail -f`。

Press Ctrl+C once for a graceful stop. From another terminal, the same action
can be requested with:

```bash
tools/oom_debug/run_handheld_oom_test.sh --stop
```

If the laptop controller was interrupted or the board rebooted, retrieve a run
later with:

```bash
tools/oom_debug/run_handheld_oom_test.sh --collect RUN_ID
```

Classify a collected run with:

```bash
tools/oom_debug/analyze_oom_run.sh handheld_oom_runs/RUN_ID
```

Raw camera images are not recorded by these scripts. The laptop records only
poses, tracking state, ROS logs and TF in a small diagnostic bag. Board-side
telemetry is sampled every 0.5 seconds and includes RSS, virtual/swap memory,
commit accounting, page faults, memory PSI, cgroup OOM counters, disk space,
temperature and CPU frequency.

`keyframe_timeline.csv` is derived from the production timing CSV after node
exit. Its counter is the number of `inserted_keyframe` events and does not count
the initial map keyframe. `local_map_timeline.csv` reports keyframe IDs and the
number of map points participating in local BA; it is not the exact number of
currently retained global map points. Exact live map totals would require a
separately validated diagnostic wrapper or a core-library code change.

Core files are limited to 4 GiB per process and written on the board's root
filesystem under `/var/lib/orb-slam-coredump`. Kernel OOM uses `SIGKILL` and
therefore cannot produce a core; use the kernel journal and exit code 137 for
that case.
