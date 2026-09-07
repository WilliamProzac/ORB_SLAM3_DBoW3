#!/usr/bin/env bash

set -Eeuo pipefail

if [ "$#" -ne 1 ] || [ ! -d "$1" ]; then
  echo "usage: $0 HANDHELD_OOM_RUN_DIR" >&2
  exit 2
fi

RUN_DIR="$(cd "$1" && pwd)"
BOARD_DIR="${RUN_DIR}/board"
EXIT_CODE="unknown"
[ -s "${BOARD_DIR}/slam_exit_code.txt" ] && EXIT_CODE="$(cat "${BOARD_DIR}/slam_exit_code.txt")"

kernel_oom=false
bad_alloc=false
segfault=false
core_count=0

if grep -Eiq 'Out of memory|oom-kill|Killed process .*ros_rgbd' \
    "${BOARD_DIR}"/kernel*.log "${BOARD_DIR}"/dmesg*.txt 2>/dev/null; then
  kernel_oom=true
fi
if grep -Eiq 'std::bad_alloc|terminate called after throwing|cannot allocate memory' \
    "${BOARD_DIR}/slam.log" "${RUN_DIR}/board_stream.log" 2>/dev/null; then
  bad_alloc=true
fi
if grep -Eiq 'segfault|segmentation fault' \
    "${BOARD_DIR}"/kernel*.log "${BOARD_DIR}"/dmesg*.txt \
    "${BOARD_DIR}/slam.log" 2>/dev/null; then
  segfault=true
fi
if [ -d "${RUN_DIR}/cores" ]; then
  core_count="$(find "${RUN_DIR}/cores" -maxdepth 1 -type f -name 'core-*' | wc -l)"
fi

classification="UNKNOWN_OR_CLEAN_EXIT"
if [ "$kernel_oom" = true ]; then
  classification="KERNEL_OOM_KILL"
elif [ "$bad_alloc" = true ]; then
  classification="CPP_BAD_ALLOC_OR_ALLOCATION_FAILURE"
elif [ "$segfault" = true ] || [ "$EXIT_CODE" = 139 ]; then
  classification="SIGSEGV"
elif [ "$EXIT_CODE" = 134 ]; then
  classification="SIGABRT"
elif [ "$EXIT_CODE" = 137 ]; then
  classification="SIGKILL_WITHOUT_KERNEL_OOM_EVIDENCE"
elif [ "$EXIT_CODE" = 0 ]; then
  classification="CLEAN_SLAM_EXIT"
fi

echo "classification=${classification}"
echo "slam_exit_code=${EXIT_CODE}"
echo "kernel_oom_evidence=${kernel_oom}"
echo "bad_alloc_evidence=${bad_alloc}"
echo "segfault_evidence=${segfault}"
echo "core_count=${core_count}"

if [ -s "${BOARD_DIR}/telemetry.csv" ]; then
  awk -F, '
    NR==2 {min_mem=$14; min_swap=$15}
    NR>1 {
      if($8+0>max_rss) max_rss=$8+0;
      if($9+0>max_vmswap) max_vmswap=$9+0;
      if($14+0<min_mem) min_mem=$14+0;
      if($15+0<min_swap) min_swap=$15+0;
      if($17+0>max_committed) max_committed=$17+0;
      rows++
    }
    END {
      printf "telemetry_rows=%d\npeak_rss_kb=%.0f\npeak_vmswap_kb=%.0f\nminimum_mem_available_kb=%.0f\nminimum_swap_free_kb=%.0f\npeak_committed_as_kb=%.0f\n",rows,max_rss,max_vmswap,min_mem,min_swap,max_committed
    }
  ' "${BOARD_DIR}/telemetry.csv"
fi

if [ -s "${BOARD_DIR}/keyframe_timeline.csv" ]; then
  awk -F, 'NR>1 {last=$3; rows++} END {printf "frame_rows=%d\ninserted_keyframe_events=%d\n",rows,last+0}' \
    "${BOARD_DIR}/keyframe_timeline.csv"
fi
if [ -s "${BOARD_DIR}/local_map_timeline.csv" ]; then
  awk -F, 'NR>1 {kf=$1; mp=$8; rows++} END {printf "local_mapping_rows=%d\nlast_keyframe_id=%s\nlast_lba_map_points=%s\n",rows,kf,mp}' \
    "${BOARD_DIR}/local_map_timeline.csv"
fi

if [ -s "${RUN_DIR}/heartbeat.csv" ]; then
  awk -F, 'NR>1 && $3==1 {boot[$5]=1; ok++} NR>1 && $3==0 {failed++} END {for(b in boot)n++; printf "heartbeat_ok=%d\nheartbeat_failed=%d\nobserved_board_boot_ids=%d\n",ok,failed,n}' \
    "${RUN_DIR}/heartbeat.csv"
fi
