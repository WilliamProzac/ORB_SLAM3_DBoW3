#!/usr/bin/env python3
from pathlib import Path
import csv
import os
import sys
import time

proc = Path("/proc") / sys.argv[1]
cgroup = Path(sys.argv[2]) if len(sys.argv) > 2 else None


def kv(path):
    result = {}
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) > 1 and fields[1].isdigit():
            result[fields[0].rstrip(":")] = int(fields[1])
    return result


writer = csv.writer(sys.stdout)
ticks_per_second=os.sysconf('SC_CLK_TCK')
frequency=Path('/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq')
temperature=Path('/sys/class/thermal/thermal_zone0/temp')
writer.writerow(("steady_s", "rss_kib", "pss_kib", "swap_kib",
                 "available_kib", "majflt", "pswpin", "pswpout",
                 "cgroup_current", "cgroup_swap_current", "cgroup_oom",
                 "cgroup_oom_kill", "cgroup_high", "cgroup_max",
                 "process_cpu_s", "threads", "cpu_frequency_khz", "temperature_milli_c", "oom_kill"))
while True:
    try:
        status = kv(proc / "status")
        smaps = kv(proc / "smaps_rollup")
        meminfo = kv(Path("/proc/meminfo"))
        vmstat = kv(Path("/proc/vmstat"))
        stat = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        if stat[0] == "Z":
            break
        cg_current = cg_swap = cg_oom = cg_oom_kill = cg_high = cg_max = ""
        if cgroup is not None and cgroup.exists():
            cg_current = (cgroup / "memory.current").read_text().strip()
            cg_swap = (cgroup / "memory.swap.current").read_text().strip()
            events = kv(cgroup / "memory.events")
            cg_oom = events.get("oom", 0)
            cg_oom_kill = events.get("oom_kill", 0)
            cg_high = events.get("high", 0)
            cg_max = events.get("max", 0)
        writer.writerow((time.monotonic(), status.get("VmRSS"),
                         smaps.get("Pss"), status.get("VmSwap"),
                         meminfo["MemAvailable"], stat[9],
                         vmstat["pswpin"], vmstat["pswpout"], cg_current,
                         cg_swap, cg_oom, cg_oom_kill, cg_high, cg_max,
                         (int(stat[11])+int(stat[12]))/ticks_per_second,int(stat[17]),
                         frequency.read_text().strip() if frequency.exists() else '',
                         temperature.read_text().strip() if temperature.exists() else '',vmstat.get('oom_kill',0)))
        sys.stdout.flush()
    except (FileNotFoundError, ProcessLookupError):
        break
    time.sleep(1)
