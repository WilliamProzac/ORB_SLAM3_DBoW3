#!/usr/bin/env bash

set -Eeuo pipefail

BOARD="${BOARD:-root@192.168.1.10}"
BOARD_HOST="${BOARD#*@}"
HOST_ROS_IP="${HOST_ROS_IP:-192.168.1.3}"
BOARD_ROS_MASTER_URI="${BOARD_ROS_MASTER_URI:-http://192.168.1.10:11311}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
GDB_MULTIARCH="${GDB_MULTIARCH:-${SCRIPT_DIR}/gdb-multiarch-root/usr/bin/gdb-multiarch}"
REMOTE_SCRIPT="/usr/local/sbin/board_rgbd_oom_debug.sh"
REMOTE_RESULT_ROOT="/var/log/orb-slam-oom"
REMOTE_CORE_ROOT="/var/lib/orb-slam-coredump"
LOCAL_RESULT_ROOT="${LOCAL_RESULT_ROOT:-${REPO_ROOT}/handheld_oom_runs}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5 "$BOARD")

usage() {
  cat <<'EOF'
Usage:
  run_handheld_oom_test.sh [RUN_ID]
  run_handheld_oom_test.sh --preflight
  run_handheld_oom_test.sh --collect RUN_ID
  run_handheld_oom_test.sh --stop

The default mode starts a live hand-held RGB-D mapping session. It records only
lightweight ROS diagnostics on the laptop; raw camera images are never written
to the board by this script.
EOF
}

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  set -u
  export ROS_MASTER_URI="$BOARD_ROS_MASTER_URI"
  export ROS_IP="$HOST_ROS_IP"
  export NO_PROXY="localhost,127.0.0.1,${HOST_ROS_IP},${BOARD_HOST}${NO_PROXY:+,${NO_PROXY}}"
}

collect_run() {
  local run_id="$1"
  local out_dir="${LOCAL_RESULT_ROOT}/${run_id}"
  mkdir -p "$out_dir/board" "$out_dir/cores"
  if rsync -a --partial -e 'ssh -o BatchMode=yes -o ConnectTimeout=5' \
      "${BOARD}:${REMOTE_RESULT_ROOT}/${run_id}/" "$out_dir/board/"; then
    echo "board_logs_collected=true" >>"$out_dir/controller_status.txt"
  else
    echo "board_logs_collected=false" >>"$out_dir/controller_status.txt"
  fi
  if rsync -a --partial -e 'ssh -o BatchMode=yes -o ConnectTimeout=5' \
      "${BOARD}:${REMOTE_CORE_ROOT}/" "$out_dir/cores/"; then
    echo "cores_collected=true" >>"$out_dir/controller_status.txt"
  else
    echo "cores_collected=false" >>"$out_dir/controller_status.txt"
  fi
  find "$out_dir" -maxdepth 3 -type f -printf '%P\t%s bytes\n' | sort \
    >"$out_dir/result_files.txt"
  echo "Collected results: ${out_dir}"
}

preflight() {
  source_ros
  local failed=0
  local sync_epoch hz_output
  sync_epoch="$(date +%s)"
  "${SSH[@]}" "date -s @${sync_epoch} >/dev/null" || {
    echo "FAIL could not synchronize board clock"; failed=1;
  }
  "${SSH[@]}" "$REMOTE_SCRIPT --preflight" || failed=1
  for topic in /camera/infra1/image_raw /camera/depth/image_raw; do
    rostopic info "$topic" >/dev/null 2>&1 && echo "OK laptop can resolve ${topic}" || {
      echo "FAIL laptop cannot resolve ${topic}"; failed=1;
    }
    hz_output="$(timeout 6 rostopic hz -w 10 "$topic" 2>&1 || true)"
    if grep -q 'average rate:' <<<"$hz_output"; then
      printf 'OK topic rate %s: %s\n' "$topic" "$(grep 'average rate:' <<<"$hz_output" | tail -n 1)"
    else
      echo "FAIL no messages received from ${topic}"
      failed=1
    fi
  done
  if command -v gdb-multiarch >/dev/null 2>&1; then
    echo "OK laptop debugger $(command -v gdb-multiarch)"
  elif [ -x "$GDB_MULTIARCH" ] &&
       "$GDB_MULTIARCH" -q -batch -ex 'set architecture aarch64' >/dev/null 2>&1; then
    echo "OK bundled laptop debugger ${GDB_MULTIARCH}"
  else
    echo "FAIL laptop gdb-multiarch missing"
    failed=1
  fi
  local host_epoch board_epoch delta
  host_epoch="$(date +%s)"
  board_epoch="$("${SSH[@]}" 'date +%s')" || board_epoch=0
  delta=$((host_epoch - board_epoch))
  [ "$delta" -lt 0 ] && delta=$((-delta))
  if [ "$delta" -le 2 ]; then
    echo "OK clock_delta_seconds=${delta}"
  else
    echo "FAIL clock_delta_seconds=${delta}"
    failed=1
  fi
  df -h "$LOCAL_RESULT_ROOT" 2>/dev/null || df -h "$(dirname "$LOCAL_RESULT_ROOT")"
  return "$failed"
}

if [ "${1:-}" = "--preflight" ]; then
  preflight
  exit $?
fi
if [ "${1:-}" = "--collect" ]; then
  [ "$#" -eq 2 ] || { usage >&2; exit 2; }
  collect_run "$2"
  exit $?
fi
if [ "${1:-}" = "--stop" ]; then
  "${SSH[@]}" "$REMOTE_SCRIPT --stop"
  exit $?
fi
if [ "$#" -gt 1 ]; then
  usage >&2
  exit 2
fi

RUN_ID="${1:-handheld_$(date +%Y%m%d_%H%M%S)}"
if [[ ! "$RUN_ID" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "Invalid RUN_ID: ${RUN_ID}" >&2
  exit 2
fi

preflight
source_ros

LOCAL_DIR="${LOCAL_RESULT_ROOT}/${RUN_ID}"
mkdir -p "$LOCAL_DIR"

REMOTE_SSH_PID=""
HEARTBEAT_PID=""
TRACK_PID=""
POSE_PID=""
POSE_SLAM_PID=""
ROSOUT_PID=""
BAG_PID=""
STOP_REQUESTED=false

stop_local_pid() {
  local pid="${1:-}"
  local signal="${2:-TERM}"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill "-${signal}" "$pid" 2>/dev/null || true
  fi
}

on_signal() {
  STOP_REQUESTED=true
  "${SSH[@]}" "$REMOTE_SCRIPT --stop" >/dev/null 2>&1 || true
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  stop_local_pid "$TRACK_PID"
  stop_local_pid "$POSE_PID"
  stop_local_pid "$POSE_SLAM_PID"
  stop_local_pid "$ROSOUT_PID"
  stop_local_pid "$BAG_PID" INT
  stop_local_pid "$HEARTBEAT_PID"
  if [ -n "$REMOTE_SSH_PID" ] && kill -0 "$REMOTE_SSH_PID" 2>/dev/null; then
    "${SSH[@]}" "$REMOTE_SCRIPT --stop" >/dev/null 2>&1 || true
    wait "$REMOTE_SSH_PID" 2>/dev/null || true
  fi
  exit "$rc"
}

trap cleanup EXIT
trap on_signal INT TERM

{
  echo "run_id=${RUN_ID}"
  echo "host_started_at=$(date --iso-8601=ns)"
  echo "board=${BOARD}"
  echo "host_ros_ip=${HOST_ROS_IP}"
  echo "ros_master_uri=${BOARD_ROS_MASTER_URI}"
  echo "raw_images_recorded=false"
  echo "host_boot_id=$(cat /proc/sys/kernel/random/boot_id)"
  echo "board_boot_id=$("${SSH[@]}" 'cat /proc/sys/kernel/random/boot_id')"
  uname -a
  df -h "$LOCAL_DIR"
} >"${LOCAL_DIR}/metadata.txt"

# Show the line-buffered board output in this terminal while retaining the
# complete laptop-side copy needed for post-mortem analysis.
"${SSH[@]}" "$REMOTE_SCRIPT $RUN_ID" \
  > >(tee -a "${LOCAL_DIR}/board_stream.log") 2>&1 &
REMOTE_SSH_PID=$!

(
  echo "local_epoch_s,local_iso_time,ssh_ok,board_epoch_s,board_boot_id,slam_pid,slam_alive,slam_rss_kb,mem_available_kb,swap_free_kb"
  while true; do
    local_epoch="$(date +%s.%N)"
    local_iso="$(date --iso-8601=ns | tr ',' '.')"
    if sample="$("${SSH[@]}" "$REMOTE_SCRIPT --heartbeat $RUN_ID")"; then
      printf '%s,%s,1,%s\n' "$local_epoch" "$local_iso" "$sample"
    else
      printf '%s,%s,0,,,,,,\n' "$local_epoch" "$local_iso"
    fi
    sleep 5
  done
) >"${LOCAL_DIR}/heartbeat.csv" 2>"${LOCAL_DIR}/heartbeat.stderr" &
HEARTBEAT_PID=$!

ready=false
for _ in $(seq 1 180); do
  [ "$STOP_REQUESTED" = false ] || break
  if ! kill -0 "$REMOTE_SSH_PID" 2>/dev/null; then
    break
  fi
  if rosnode list 2>/dev/null | grep -q '^/orb_slam3_oom_'; then
    rosnode list >"${LOCAL_DIR}/rosnodes.ready.txt"
    ready=true
    break
  fi
  sleep 1
done
echo "ready=${ready}" >"${LOCAL_DIR}/readiness.txt"
if [ "$ready" != true ] && [ "$STOP_REQUESTED" = false ]; then
  echo "SLAM did not become ready; see board_stream.log" >&2
fi

if [ "$STOP_REQUESTED" = false ]; then
  rostopic echo -p /robot_pose_tracking_ok >"${LOCAL_DIR}/tracking_ok.csv" 2>"${LOCAL_DIR}/tracking_ok.stderr" &
  TRACK_PID=$!
  rostopic echo -p /robot_pose >"${LOCAL_DIR}/robot_pose.csv" 2>"${LOCAL_DIR}/robot_pose.stderr" &
  POSE_PID=$!
  rostopic echo -p /robot_pose_slam >"${LOCAL_DIR}/robot_pose_slam.csv" 2>"${LOCAL_DIR}/robot_pose_slam.stderr" &
  POSE_SLAM_PID=$!
  rostopic echo /rosout_agg >"${LOCAL_DIR}/rosout_agg.log" 2>"${LOCAL_DIR}/rosout_agg.stderr" &
  ROSOUT_PID=$!

  rosbag record --lz4 -O "${LOCAL_DIR}/diagnostics" \
    /rosout_agg /robot_pose /robot_pose_slam /robot_pose_tracking_ok /tf /tf_static \
    >"${LOCAL_DIR}/rosbag_record.log" 2>&1 &
  BAG_PID=$!

  echo "Hand-held OOM test is running: ${RUN_ID}"
  echo "Logs: ${LOCAL_DIR}"
  echo "Press Ctrl+C once for a graceful stop."
fi

set +e
wait "$REMOTE_SSH_PID"
REMOTE_SSH_EXIT=$?
if [ "$STOP_REQUESTED" = true ] && kill -0 "$REMOTE_SSH_PID" 2>/dev/null; then
  wait "$REMOTE_SSH_PID"
  REMOTE_SSH_EXIT=$?
fi
set -e
REMOTE_SSH_PID=""
echo "$REMOTE_SSH_EXIT" >"${LOCAL_DIR}/remote_ssh_exit_code.txt"

if [ "$STOP_REQUESTED" = false ] && [ "$REMOTE_SSH_EXIT" -eq 255 ]; then
  echo "Remote SSH disconnected; waiting up to 5 minutes for board recovery."
  for _ in $(seq 1 60); do
    if "${SSH[@]}" 'printf "board_time=%s\nboot_id=%s\n" "$(date --iso-8601=ns)" "$(cat /proc/sys/kernel/random/boot_id)"' \
        >"${LOCAL_DIR}/post_disconnect_board.txt" 2>"${LOCAL_DIR}/post_disconnect_board.stderr"; then
      break
    fi
    sleep 5
  done
fi

stop_local_pid "$TRACK_PID"
stop_local_pid "$POSE_PID"
stop_local_pid "$POSE_SLAM_PID"
stop_local_pid "$ROSOUT_PID"
stop_local_pid "$BAG_PID" INT
stop_local_pid "$HEARTBEAT_PID"
sleep 2
TRACK_PID=""
POSE_PID=""
POSE_SLAM_PID=""
ROSOUT_PID=""
BAG_PID=""
HEARTBEAT_PID=""

collect_run "$RUN_ID"
echo "host_finished_at=$(date --iso-8601=ns)" >>"${LOCAL_DIR}/metadata.txt"
trap - EXIT INT TERM

if [ -s "${LOCAL_DIR}/board/slam_exit_code.txt" ]; then
  echo "SLAM exit code: $(cat "${LOCAL_DIR}/board/slam_exit_code.txt")"
else
  echo "SLAM exit code was not recovered; inspect heartbeat and previous boot journal."
fi
