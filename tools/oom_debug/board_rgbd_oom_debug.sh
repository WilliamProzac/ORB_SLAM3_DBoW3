#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="/userdata/orb_slam"
DEPLOY_DIR="${ROOT_DIR}/rgbd_orb_optimization_20260817/production_w3"
SLAM_BIN="${DEPLOY_DIR}/ros_rgbd"
SLAM_LIB="${DEPLOY_DIR}/liborb_slam3_ros.so"
VOC_FILE="${ROOT_DIR}/src/orb_slam3_ros-master/orb_slam3/Vocabulary/orbvoc.dbow3"
SETTINGS_FILE="${ROOT_DIR}/rgbd_long_154955_20260829/configs/S316_f700_l5.yaml"
RESULT_ROOT="/var/log/orb-slam-oom"
CORE_DIR="/var/lib/orb-slam-coredump"
PID_FILE="/run/orb_slam_oom_debug.pid"
LOCK_FILE="/run/orb_slam_oom_debug.lock"
RGB_TOPIC="/camera/infra1/image_raw"
DEPTH_TOPIC="/camera/depth/image_raw"
EXPECTED_BIN_SHA="a41fd8359313a0c78bb690743c08b964a18491b94c21d797019986666a99cc7b"
EXPECTED_LIB_SHA="51c3e4d720e5e8c22e63405b44c2b39a782e55464347016ff2bf1db9987a2860"
EXPECTED_CONFIG_SHA="8cd62c08617a05e7b1a2d38a6d9bfce8cd2afe1fdb17086958cfd0183f75f3c8"

source_ros_environment() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/devel/setup.bash"
  set -u
}

usage() {
  cat <<'EOF'
Usage:
  board_rgbd_oom_debug.sh RUN_ID
  board_rgbd_oom_debug.sh --preflight
  board_rgbd_oom_debug.sh --heartbeat RUN_ID
  board_rgbd_oom_debug.sh --stop

Runs the validated RGB-D production binary against the live S316 topics and
stores persistent OOM/crash evidence below /var/log/orb-slam-oom/RUN_ID.
EOF
}

heartbeat_sample() {
  local run_id="$1"
  local run_dir="${RESULT_ROOT}/${run_id}"
  local board_epoch boot pid alive rss mem swap
  board_epoch="$(date +%s.%N)"
  boot="$(cat /proc/sys/kernel/random/boot_id)"
  pid="$(cat "${run_dir}/slam.pid" 2>/dev/null || true)"
  alive=0
  rss=""
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    alive=1
    rss="$(awk '/^VmRSS:/ {print $2}' "/proc/${pid}/status" 2>/dev/null || true)"
  fi
  mem="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
  swap="$(awk '/^SwapFree:/ {print $2}' /proc/meminfo)"
  printf '%s,%s,%s,%s,%s,%s,%s\n' \
    "$board_epoch" "$boot" "$pid" "$alive" "$rss" "$mem" "$swap"
}

is_expected_process() {
  local pid="$1"
  [ -r "/proc/${pid}/cmdline" ] &&
    tr '\0' ' ' <"/proc/${pid}/cmdline" | grep -Fq "board_rgbd_oom_debug.sh"
}

stop_active_run() {
  if [ ! -s "$PID_FILE" ]; then
    echo "No active OOM debug run."
    return 0
  fi
  local pid
  pid="$(cat "$PID_FILE")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null && is_expected_process "$pid"; then
    kill -INT "$pid"
    echo "Requested graceful stop for collector PID ${pid}."
  else
    echo "Removing stale PID file: ${PID_FILE}" >&2
    rm -f -- "$PID_FILE"
  fi
}

require_file_sha() {
  local path="$1"
  local expected="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [ "$actual" != "$expected" ]; then
    echo "SHA256 mismatch: ${path}" >&2
    echo "expected=${expected}" >&2
    echo "actual=${actual}" >&2
    return 1
  fi
}

preflight() {
  local failed=0 root_avail_kb userdata_avail_kb
  for command_name in flock journalctl stdbuf tail sha256sum; do
    command -v "$command_name" >/dev/null 2>&1 || {
      echo "FAIL command missing: ${command_name}"; failed=1;
    }
  done
  for path in "$SLAM_BIN" "$SLAM_LIB" "$VOC_FILE" "$SETTINGS_FILE"; do
    if [ -r "$path" ]; then
      echo "OK readable ${path}"
    else
      echo "FAIL missing ${path}"
      failed=1
    fi
  done
  require_file_sha "$SLAM_BIN" "$EXPECTED_BIN_SHA" || failed=1
  require_file_sha "$SLAM_LIB" "$EXPECTED_LIB_SHA" || failed=1
  require_file_sha "$SETTINGS_FILE" "$EXPECTED_CONFIG_SHA" || failed=1
  grep -qE '^ORBextractor\.nFeatures:[[:space:]]*700([[:space:]]|$)' "$SETTINGS_FILE" || {
    echo "FAIL settings are not nFeatures=700"; failed=1;
  }
  grep -qE '^ORBextractor\.nLevels:[[:space:]]*5([[:space:]]|$)' "$SETTINGS_FILE" || {
    echo "FAIL settings are not nLevels=5"; failed=1;
  }
  [ "$(cat /proc/sys/kernel/core_pattern)" = "${CORE_DIR}/core-%e-%p-%t" ] || {
    echo "FAIL core_pattern=$(cat /proc/sys/kernel/core_pattern)"; failed=1;
  }
  [ -d "$CORE_DIR" ] || { echo "FAIL missing ${CORE_DIR}"; failed=1; }
  root_avail_kb="$(df -Pk / | awk 'NR==2 {print $4}')"
  userdata_avail_kb="$(df -Pk /userdata | awk 'NR==2 {print $4}')"
  [ "$root_avail_kb" -ge 6291456 ] || {
    echo "FAIL root filesystem has less than 6 GiB free"; failed=1;
  }
  [ "$userdata_avail_kb" -ge 4194304 ] || {
    echo "FAIL /userdata has less than 4 GiB free"; failed=1;
  }
  source_ros_environment
  export ROS_MASTER_URI="http://192.168.1.10:11311"
  export ROS_IP="192.168.1.10"
  rosnode list >/dev/null 2>&1 || { echo "FAIL ROS master unreachable"; failed=1; }
  for topic in "$RGB_TOPIC" "$DEPTH_TOPIC"; do
    rostopic info "$topic" >/dev/null 2>&1 && echo "OK ROS topic ${topic}" || {
      echo "FAIL ROS topic ${topic}"; failed=1;
    }
  done
  if pgrep -af "^${SLAM_BIN} " >/dev/null; then
    echo "FAIL production RGB-D node already running"
    failed=1
  fi
  echo "boot_id=$(cat /proc/sys/kernel/random/boot_id)"
  echo "board_time=$(date --iso-8601=ns)"
  df -h / /userdata
  free -h
  return "$failed"
}

if [ "${1:-}" = "--preflight" ]; then
  preflight
  exit $?
fi
if [ "${1:-}" = "--stop" ]; then
  stop_active_run
  exit $?
fi
if [ "${1:-}" = "--heartbeat" ]; then
  [ "$#" -eq 2 ] || { usage >&2; exit 2; }
  heartbeat_sample "$2"
  exit $?
fi
if [ "$#" -ne 1 ]; then
  usage >&2
  exit 2
fi

RUN_ID="$1"
if [[ ! "$RUN_ID" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "Invalid RUN_ID: ${RUN_ID}" >&2
  exit 2
fi

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "Another OOM debug run is active." >&2
  exit 3
fi

preflight

RUN_DIR="${RESULT_ROOT}/${RUN_ID}"
TIMING_DIR="${RUN_DIR}/timing"
mkdir -p "$TIMING_DIR" "$CORE_DIR"
chmod 0750 "$RESULT_ROOT" "$RUN_DIR" "$TIMING_DIR" "$CORE_DIR"
touch "${RUN_DIR}/start.marker"
echo "$$" >"$PID_FILE"

source_ros_environment
export ROS_MASTER_URI="http://192.168.1.10:11311"
export ROS_IP="192.168.1.10"
export ROS_PACKAGE_PATH="${ROOT_DIR}/src${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
export LD_LIBRARY_PATH="${DEPLOY_DIR}:${ROOT_DIR}/src/orb_slam3_ros-master/orb_slam3/Thirdparty/DBoW3/lib:${ROOT_DIR}/src/orb_slam3_ros-master/orb_slam3/Thirdparty/g2o/lib:/usr/local/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu:/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# Limit a single diagnostic core to 4 GiB and exclude file-backed mappings.
ulimit -c 4194304

SLAM_PID=""
KERNEL_PID=""
TELEMETRY_PID=""
FRAME_STATS_PID=""
MAP_STATS_PID=""
CLEANED_UP=false
STOP_REASON="natural_exit"

stop_child() {
  local pid="${1:-}"
  local signal="${2:-TERM}"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill "-${signal}" "$pid" 2>/dev/null || true
  fi
}

capture_after() {
  if [ -s "${TIMING_DIR}/rgbd_frame_timing.csv" ]; then
    awk -F, '
      BEGIN {print "image_timestamp,frame_id,cumulative_inserted_keyframes,state_after,lm_queue_after"}
      NR>1 && NF>=13 {if($13==1)n++; printf "%s,%s,%d,%s,%s\n",$4,$5,n,$7,$12}
    ' "${TIMING_DIR}/rgbd_frame_timing.csv" >"${RUN_DIR}/keyframe_timeline.csv"
  fi
  if [ -s "${TIMING_DIR}/local_mapping_timing.csv" ]; then
    awk -F, '
      BEGIN {print "keyframe_id,image_timestamp,lba_executed,lba_aborted,lba_edges,lba_optimized_keyframes,lba_fixed_keyframes,lba_map_points"}
      NR>1 && NF>=17 {printf "%s,%s,%s,%s,%s,%s,%s,%s\n",$1,$2,$12,$13,$14,$15,$16,$17}
    ' "${TIMING_DIR}/local_mapping_timing.csv" >"${RUN_DIR}/local_map_timeline.csv"
  fi
  date --iso-8601=ns >"${RUN_DIR}/finished_at.txt"
  cat /proc/meminfo >"${RUN_DIR}/meminfo.after.txt" 2>&1 || true
  ps -eo pid,ppid,stat,pcpu,pmem,rss,vsz,nlwp,etime,comm,args \
    >"${RUN_DIR}/processes.after.txt" 2>&1 || true
  dmesg -T | tail -n 2000 >"${RUN_DIR}/dmesg.after.txt" 2>&1 || true
  journalctl -k -b -n 2000 --no-pager >"${RUN_DIR}/kernel.after.log" 2>&1 || true
  find "$CORE_DIR" -maxdepth 1 -type f -newer "${RUN_DIR}/start.marker" \
    -printf '%s\t%TY-%Tm-%TdT%TH:%TM:%TS%TZ\t%p\n' \
    >"${RUN_DIR}/core_files.txt" 2>&1 || true
  find "$RUN_DIR" -maxdepth 2 -type f -printf '%P\t%s bytes\n' | sort \
    >"${RUN_DIR}/result_files.txt" 2>&1 || true
  sync -d "$RUN_DIR" 2>/dev/null || sync
}

cleanup() {
  local rc=$?
  local slam_rc=""
  if [ "$CLEANED_UP" = true ]; then
    return
  fi
  CLEANED_UP=true
  trap - EXIT INT TERM

  if [ -n "$SLAM_PID" ]; then
    if kill -0 "$SLAM_PID" 2>/dev/null; then
      stop_child "$SLAM_PID" INT
      for _ in $(seq 1 30); do
        kill -0 "$SLAM_PID" 2>/dev/null || break
        sleep 1
      done
      stop_child "$SLAM_PID" TERM
    fi
    set +e
    wait "$SLAM_PID" 2>/dev/null
    slam_rc=$?
    set -e
    if [ ! -s "${RUN_DIR}/slam_exit_code.txt" ]; then
      echo "$slam_rc" >"${RUN_DIR}/slam_exit_code.txt"
    fi
    SLAM_PID=""
  fi
  stop_child "$TELEMETRY_PID" TERM
  stop_child "$FRAME_STATS_PID" TERM
  stop_child "$MAP_STATS_PID" TERM
  stop_child "$KERNEL_PID" TERM
  rm -f -- "$PID_FILE"
  printf 'collector_exit_code=%s\nstop_reason=%s\n' "$rc" "$STOP_REASON" \
    >"${RUN_DIR}/collector_exit.txt"
  capture_after
  exit "$rc"
}

on_interrupt() {
  STOP_REASON="operator_interrupt"
  exit 130
}

on_terminate() {
  STOP_REASON="operator_terminate"
  exit 143
}

trap cleanup EXIT
trap on_interrupt INT
trap on_terminate TERM

{
  echo "run_id=${RUN_ID}"
  echo "started_at=$(date --iso-8601=ns)"
  echo "boot_id=$(cat /proc/sys/kernel/random/boot_id)"
  echo "ros_master_uri=${ROS_MASTER_URI}"
  echo "ros_ip=${ROS_IP}"
  echo "rgb_topic=${RGB_TOPIC}"
  echo "depth_topic=${DEPTH_TOPIC}"
  echo "slam_bin=${SLAM_BIN}"
  echo "slam_lib=${SLAM_LIB}"
  echo "settings=${SETTINGS_FILE}"
  echo "core_pattern=$(cat /proc/sys/kernel/core_pattern)"
  echo "core_limit_blocks=$(ulimit -c)"
  echo "coredump_filter=0x23"
  uname -a
  uptime
  free -h
  df -h / /userdata
  cat /proc/swaps
  sysctl vm.overcommit_memory vm.overcommit_ratio vm.panic_on_oom \
    vm.oom_kill_allocating_task vm.oom_dump_tasks 2>/dev/null || true
} >"${RUN_DIR}/metadata.txt"

sha256sum "$SLAM_BIN" "$SLAM_LIB" "$VOC_FILE" "$SETTINGS_FILE" \
  >"${RUN_DIR}/sha256.txt"
cp -a "$SETTINGS_FILE" "${RUN_DIR}/S316_f700_l5.yaml"
env | sort >"${RUN_DIR}/environment.txt"
cat /proc/meminfo >"${RUN_DIR}/meminfo.before.txt"
ps -eo pid,ppid,stat,pcpu,pmem,rss,vsz,nlwp,etime,comm,args \
  >"${RUN_DIR}/processes.before.txt"
dmesg -T | tail -n 2000 >"${RUN_DIR}/dmesg.before.txt" 2>&1 || true
journalctl -k -b -n 2000 --no-pager >"${RUN_DIR}/kernel.before.log" 2>&1 || true
rosnode list >"${RUN_DIR}/rosnodes.before.txt" 2>&1 || true
rostopic list -v >"${RUN_DIR}/rostopics.before.txt" 2>&1 || true
rosparam dump "${RUN_DIR}/rosparams.before.yaml" >/dev/null 2>&1 || true

stdbuf -oL -eL journalctl -kf -o short-precise --no-pager \
  >"${RUN_DIR}/kernel.follow.log" 2>&1 &
KERNEL_PID=$!

NODE_NAME="orb_slam3_oom_${RUN_ID//[^A-Za-z0-9_]/_}"
SLAM_CMD=(
  stdbuf -oL -eL "$SLAM_BIN"
  "_voc_file:=${VOC_FILE}"
  "_settings_file:=${SETTINGS_FILE}"
  "_rgb_topic:=${RGB_TOPIC}"
  "_depth_topic:=${DEPTH_TOPIC}"
  "_enable_pangolin:=false"
  "_timing_enabled:=true"
  "_timing_output_dir:=${TIMING_DIR}"
  "__name:=${NODE_NAME}"
)

printf '%q ' "${SLAM_CMD[@]}" >"${RUN_DIR}/slam_command.txt"
printf '\n' >>"${RUN_DIR}/slam_command.txt"
"${SLAM_CMD[@]}" > >(tee -a "${RUN_DIR}/slam.log") 2>&1 &
SLAM_PID=$!
echo "$SLAM_PID" >"${RUN_DIR}/slam.pid"
echo 0x23 >"/proc/${SLAM_PID}/coredump_filter" 2>/dev/null || true

(
  echo "epoch_s,iso_time,pid,cpu_pct,vmpeak_kb,vmsize_kb,vmhwm_kb,vmrss_kb,vmswap_kb,threads,fd_count,minflt,majflt,mem_available_kb,swap_free_kb,commit_limit_kb,committed_as_kb,pswpin,pswpout,pgmajfault,psi_some_avg10,psi_full_avg10,cgroup_oom,cgroup_oom_kill,root_avail_kb,userdata_avail_kb,temp_mC,max_freq_kHz"
  sample=0
  while kill -0 "$SLAM_PID" 2>/dev/null; do
    epoch="$(date +%s.%N)"
    iso="$(date --iso-8601=ns | tr ',' '.')"
    cpu="$(ps -p "$SLAM_PID" -o pcpu= 2>/dev/null | tr -d ' ' || true)"
    status="$(awk '
      /^(VmPeak|VmSize|VmHWM|VmRSS|VmSwap|Threads):/ {v[$1]=$2}
      END {printf "%s,%s,%s,%s,%s,%s",v["VmPeak:"],v["VmSize:"],v["VmHWM:"],v["VmRSS:"],v["VmSwap:"],v["Threads:"]}
    ' "/proc/${SLAM_PID}/status" 2>/dev/null || true)"
    [ -n "$status" ] || status=",,,,,"
    fds="$(find "/proc/${SLAM_PID}/fd" -mindepth 1 -maxdepth 1 2>/dev/null | wc -l)"
    faults="$(awk '{printf "%s,%s",$10,$12}' "/proc/${SLAM_PID}/stat" 2>/dev/null || true)"
    [ -n "$faults" ] || faults=","
    mem="$(awk '
      /^(MemAvailable|SwapFree|CommitLimit|Committed_AS):/ {v[$1]=$2}
      END {printf "%s,%s,%s,%s",v["MemAvailable:"],v["SwapFree:"],v["CommitLimit:"],v["Committed_AS:"]}
    ' /proc/meminfo)"
    vm="$(awk '
      /^(pswpin|pswpout|pgmajfault) / {v[$1]=$2}
      END {printf "%s,%s,%s",v["pswpin"],v["pswpout"],v["pgmajfault"]}
    ' /proc/vmstat)"
    psi="$(awk '
      /^some / {for(i=1;i<=NF;i++) if($i ~ /^avg10=/){split($i,a,"="); some=a[2]}}
      /^full / {for(i=1;i<=NF;i++) if($i ~ /^avg10=/){split($i,a,"="); full=a[2]}}
      END {printf "%s,%s",some,full}
    ' /proc/pressure/memory 2>/dev/null || printf ',')"
    cg_rel="$(awk -F: '$1=="0" {print $3}' "/proc/${SLAM_PID}/cgroup" 2>/dev/null || true)"
    cg_events="/sys/fs/cgroup${cg_rel}/memory.events"
    cg="$(awk '
      $1=="oom" {oom=$2} $1=="oom_kill" {kill=$2}
      END {printf "%s,%s",oom,kill}
    ' "$cg_events" 2>/dev/null || printf ',')"
    root_avail="$(df -Pk / | awk 'NR==2 {print $4}')"
    userdata_avail="$(df -Pk /userdata | awk 'NR==2 {print $4}')"
    temp="$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -nr | head -n 1 || true)"
    freq="$(cat /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq 2>/dev/null | sort -nr | head -n 1 || true)"
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
      "$epoch" "$iso" "$SLAM_PID" "${cpu:-}" "$status" "$fds" "$faults" "$mem" "$vm" "$psi" "$cg" "${root_avail},${userdata_avail},${temp:-},${freq:-}"
    sample=$((sample + 1))
    if [ $((sample % 20)) -eq 0 ]; then
      sync -d "${RUN_DIR}/telemetry.csv" 2>/dev/null || true
    fi
    sleep 0.5
  done
) >"${RUN_DIR}/telemetry.csv" &
TELEMETRY_PID=$!

(
  timing_file="${TIMING_DIR}/rgbd_frame_timing.csv"
  while [ ! -s "$timing_file" ] && kill -0 "$SLAM_PID" 2>/dev/null; do sleep 0.2; done
  echo "receipt_epoch_s,image_timestamp,frame_id,cumulative_inserted_keyframes,state_after,lm_queue_after"
  [ -s "$timing_file" ] || exit 0
  tail --pid="$SLAM_PID" -n +2 -F "$timing_file" 2>"${RUN_DIR}/frame_stats.stderr" |
    stdbuf -oL awk -F, 'NF>=13 {if($13==1)n++; printf "%d,%s,%s,%d,%s,%s\n",systime(),$4,$5,n,$7,$12; fflush()}'
) >"${RUN_DIR}/keyframe_timeline.csv" &
FRAME_STATS_PID=$!

(
  timing_file="${TIMING_DIR}/local_mapping_timing.csv"
  while [ ! -s "$timing_file" ] && kill -0 "$SLAM_PID" 2>/dev/null; do sleep 0.2; done
  echo "receipt_epoch_s,keyframe_id,image_timestamp,lba_executed,lba_aborted,lba_edges,lba_optimized_keyframes,lba_fixed_keyframes,lba_map_points"
  [ -s "$timing_file" ] || exit 0
  tail --pid="$SLAM_PID" -n +2 -F "$timing_file" 2>"${RUN_DIR}/map_stats.stderr" |
    stdbuf -oL awk -F, 'NF>=17 {printf "%d,%s,%s,%s,%s,%s,%s,%s,%s\n",systime(),$1,$2,$12,$13,$14,$15,$16,$17; fflush()}'
) >"${RUN_DIR}/local_map_timeline.csv" &
MAP_STATS_PID=$!

ready=false
for _ in $(seq 1 180); do
  if ! kill -0 "$SLAM_PID" 2>/dev/null; then
    break
  fi
  if rosnode info "/${NODE_NAME}" >"${RUN_DIR}/node_info.ready.txt" 2>&1 &&
     grep -Fq "$RGB_TOPIC" "${RUN_DIR}/node_info.ready.txt" &&
     grep -Fq "$DEPTH_TOPIC" "${RUN_DIR}/node_info.ready.txt"; then
    ready=true
    break
  fi
  sleep 1
done
echo "ready=${ready}" >"${RUN_DIR}/readiness.txt"

set +e
wait "$SLAM_PID"
SLAM_EXIT=$?
set -e
echo "$SLAM_EXIT" >"${RUN_DIR}/slam_exit_code.txt"
SLAM_PID=""
if [ "$SLAM_EXIT" -eq 137 ]; then
  STOP_REASON="slam_sigkill_possible_oom"
elif [ "$SLAM_EXIT" -eq 134 ]; then
  STOP_REASON="slam_sigabrt_possible_bad_alloc"
elif [ "$SLAM_EXIT" -eq 139 ]; then
  STOP_REASON="slam_sigsegv"
else
  STOP_REASON="slam_exit_${SLAM_EXIT}"
fi
exit "$SLAM_EXIT"
