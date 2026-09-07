#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <f600_l5|f700_l5|f800_l5|f800_l6|f1000_l6|f1200_l8> <repeat>" >&2
  exit 2
fi

CONFIG_ID="$1"
REPEAT="$2"
case "${CONFIG_ID}" in
  f600_l5|f700_l5|f800_l5|f800_l6|f1000_l6|f1200_l8) ;;
  *) echo "Unknown configuration: ${CONFIG_ID}" >&2; exit 2 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG_ROOT="${SLAM_DATA_ROOT:-${REPO_DIR}}"
BOARD="${RDK_X5_SSH:-root@192.168.1.10}"
HOST_ROS_IP="${HOST_ROS_IP:-192.168.1.3}"
ROS_PORT="${F33_ROS_PORT:-11312}"
RATE="${F33_PLAYBACK_RATE:-1.0}"
BAG="${F33_BAG:-${BAG_ROOT}/2026-08-25-09-56-06.bag}"
PRODUCTION_STAGE="${F33_PRODUCTION_STAGE:-/userdata/orb_slam/rgbd_orb_optimization_20260817/production_w3}"
REMOTE_ROOT="${F33_REMOTE_ROOT:-/userdata/orb_slam/rgbd_orb_matrix_20260825}"
ARTIFACT_ROOT="${F33_ARTIFACTS:-${SCRIPT_DIR}/artifacts/f33_rgbd_orb_matrix}"
CONFIG_ROOT="${F33_CONFIG_ROOT:-${ARTIFACT_ROOT}/configs}"
LOCAL_CONFIG="${CONFIG_ROOT}/S316_${CONFIG_ID}.yaml"
RUN_PREFIX="${F33_RUN_PREFIX:-f33}"
PREFIX="${RUN_PREFIX}_${CONFIG_ID}_r${REPEAT}"
NODE_NAME="/${PREFIX}"
TOPIC_ROOT="/${PREFIX}"
RGB_TOPIC="${TOPIC_ROOT}/camera/infra1/image_raw"
DEPTH_TOPIC="${TOPIC_ROOT}/camera/depth/image_raw"
IMU_TOPIC="${TOPIC_ROOT}/camera/imu/data"
LOCAL_RESULT="${ARTIFACT_ROOT}/runs/${PREFIX}"
REMOTE_CONFIG="${REMOTE_ROOT}/configs/S316_${CONFIG_ID}.yaml"
REMOTE_RESULT="${REMOTE_ROOT}/results/${PREFIX}"
REMOTE_TIMING="${REMOTE_RESULT}/timing"
VOCAB="/userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3/Vocabulary/orbvoc.dbow3"
BINARY="${PRODUCTION_STAGE}/ros_rgbd"

[[ -f "${BAG}" ]] || { echo "Missing bag: ${BAG}" >&2; exit 2; }
[[ -f "${LOCAL_CONFIG}" ]] || { echo "Missing config: ${LOCAL_CONFIG}" >&2; exit 2; }
mkdir -p "${LOCAL_RESULT}"

. /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://${HOST_ROS_IP}:${ROS_PORT}"
export ROS_IP="${HOST_ROS_IP}"
export NO_PROXY="localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10"
export no_proxy="${NO_PROXY}"
export ROS_HOME="${ROS_HOME:-${REPO_DIR}/.ros_runtime_11312/f33_rgbd_orb_matrix}"
mkdir -p "${ROS_HOME}"

if ! rosnode list >/dev/null 2>&1; then
  echo "ROS master is not reachable at ${ROS_MASTER_URI}" >&2
  exit 2
fi

ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "mkdir -p ${REMOTE_ROOT}/configs ${REMOTE_TIMING}; test ! -f ${REMOTE_ROOT}/*.bag; pkill -INT -f '^${BINARY} ' 2>/dev/null || true"
scp -o BatchMode=yes -o ConnectTimeout=5 "${LOCAL_CONFIG}" "${BOARD}:${REMOTE_CONFIG}"

{
  echo "run_id=${PREFIX}"
  echo "bag=${BAG}"
  echo "bag_location=host_${HOST_ROS_IP}"
  echo "board=192.168.1.10"
  echo "ros_master=${ROS_MASTER_URI}"
  echo "playback_rate=${RATE}"
  echo "settings=${REMOTE_CONFIG}"
  echo "slam_cpu_set=unrestricted"
  date --iso-8601=seconds
  sha256sum "${BAG}" "${LOCAL_CONFIG}"
  ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
    "sha256sum ${BINARY} ${PRODUCTION_STAGE}/liborb_slam3_ros.so ${VOCAB} ${REMOTE_CONFIG}; df -h /userdata"
} > "${LOCAL_RESULT}/metadata.txt"

ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "date --iso-8601=seconds; cat /proc/meminfo; echo process_snapshot; ps -eo pid,ppid,stat,%cpu,%mem,vsz,rss,nlwp,comm,args; echo kernel_tail; dmesg -T | tail -n 400" \
  > "${LOCAL_RESULT}/board_system_before.txt" 2>&1

remote_ssh_pid=""
telemetry_ssh_pid=""
track_pid=""
pose_pid=""

cleanup() {
  rosnode kill "${NODE_NAME}" >/dev/null 2>&1 || true
  [[ -z "${track_pid}" ]] || kill "${track_pid}" >/dev/null 2>&1 || true
  [[ -z "${pose_pid}" ]] || kill "${pose_pid}" >/dev/null 2>&1 || true
  [[ -z "${telemetry_ssh_pid}" ]] || kill "${telemetry_ssh_pid}" >/dev/null 2>&1 || true
  [[ -z "${remote_ssh_pid}" ]] || wait "${remote_ssh_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ssh -T -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "source /opt/ros/noetic/setup.bash; source /userdata/orb_slam/devel/setup.bash; export ROS_MASTER_URI=http://${HOST_ROS_IP}:${ROS_PORT}; export ROS_IP=192.168.1.10; export NO_PROXY=localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10; export ORB_ROOT=/userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3; export LD_LIBRARY_PATH=${PRODUCTION_STAGE}:\${ORB_ROOT}/Thirdparty/DBoW3/lib:\${ORB_ROOT}/Thirdparty/g2o/lib:/usr/local/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu:/opt/ros/noetic/lib; cd ${REMOTE_RESULT}; exec ${BINARY} _voc_file:=${VOCAB} _settings_file:=${REMOTE_CONFIG} _rgb_topic:=${RGB_TOPIC} _depth_topic:=${DEPTH_TOPIC} _enable_pangolin:=false _timing_enabled:=true _timing_output_dir:=${REMOTE_TIMING} __name:=${PREFIX}" \
  > "${LOCAL_RESULT}/slam.log" 2>&1 &
remote_ssh_pid="$!"

ready=0
for _ in $(seq 1 180); do
  if rosnode info "${NODE_NAME}" > "${LOCAL_RESULT}/node_info.ready.txt" 2>&1 && \
     grep -Fq "${RGB_TOPIC}" "${LOCAL_RESULT}/node_info.ready.txt" && \
     grep -Fq "${DEPTH_TOPIC}" "${LOCAL_RESULT}/node_info.ready.txt"; then
    ready=1
    break
  fi
  sleep 1
done
[[ "${ready}" == 1 ]] || { echo "Timed out waiting for ${NODE_NAME}" >&2; exit 10; }

remote_node_pid="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" "pgrep -n -f '^${BINARY} .*__name:=${PREFIX}'")"
echo 'timestamp_s,node_cpu_pct,node_rss_kb,node_threads,node_vsz_kb,temp_mC,max_freq_kHz,node_vmswap_kb,node_minflt,node_majflt,mem_available_kb,swap_free_kb,commit_limit_kb,committed_as_kb' > "${LOCAL_RESULT}/telemetry.csv"
ssh -T -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "while kill -0 ${remote_node_pid} 2>/dev/null; do ts=\$(date +%s.%N); node=\$(ps -p ${remote_node_pid} -o %cpu=,rss=,nlwp=,vsz= | tr -s ' ' | sed 's/^ //'); temp=\$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -nr | head -n1); freq=\$(cat /sys/devices/system/cpu/cpu[0-7]/cpufreq/scaling_cur_freq 2>/dev/null | sort -nr | head -n1); vmswap=\$(awk '/^VmSwap:/ {print \$2}' /proc/${remote_node_pid}/status 2>/dev/null); faults=\$(awk '{print \$10 \",\" \$12}' /proc/${remote_node_pid}/stat 2>/dev/null); mem=\$(awk '/^(MemAvailable|SwapFree|CommitLimit|Committed_AS):/ {v[\$1]=\$2} END {print v[\"MemAvailable:\"] \",\" v[\"SwapFree:\"] \",\" v[\"CommitLimit:\"] \",\" v[\"Committed_AS:\"]}' /proc/meminfo); echo \"\${ts},\$(echo \${node} | tr ' ' ','),\${temp:-0},\${freq:-0},\${vmswap:-0},\${faults:-0,0},\${mem}\"; sleep 0.5; done" \
  >> "${LOCAL_RESULT}/telemetry.csv" 2>/dev/null &
telemetry_ssh_pid="$!"

rostopic echo -p /robot_pose_tracking_ok > "${LOCAL_RESULT}/tracking_ok.csv" 2> "${LOCAL_RESULT}/tracking_ok.stderr" &
track_pid="$!"
rostopic echo -p /robot_pose > "${LOCAL_RESULT}/robot_pose.csv" 2> "${LOCAL_RESULT}/robot_pose.stderr" &
pose_pid="$!"
sleep 2

set +e
rosbag play --quiet --delay=1.0 --rate "${RATE}" "${BAG}" \
  "/camera/infra1/image_raw:=${RGB_TOPIC}" \
  "/camera/depth/image_raw:=${DEPTH_TOPIC}" \
  "/camera/imu/data:=${IMU_TOPIC}" > "${LOCAL_RESULT}/rosbag_play.log" 2>&1
play_exit="$?"
set -e
echo "${play_exit}" > "${LOCAL_RESULT}/rosbag_play_exit_code.txt"

# Let subscriber, Local Mapping and Loop Closing queues settle before shutdown.
sleep 20
rosnode info "${NODE_NAME}" > "${LOCAL_RESULT}/node_info.after_play.txt" 2>&1 || true
rosnode kill "${NODE_NAME}" > "${LOCAL_RESULT}/rosnode_kill.txt" 2>&1 || true
set +e
wait "${remote_ssh_pid}" 2>/dev/null
node_exit="$?"
set -e
remote_ssh_pid=""
echo "${node_exit}" > "${LOCAL_RESULT}/slam_exit_code.txt"

kill "${track_pid}" >/dev/null 2>&1 || true
kill "${pose_pid}" >/dev/null 2>&1 || true
track_pid=""
pose_pid=""
wait "${telemetry_ssh_pid}" 2>/dev/null || true
telemetry_ssh_pid=""

scp -r -o BatchMode=yes -o ConnectTimeout=5 \
  "${BOARD}:${REMOTE_TIMING}" "${LOCAL_RESULT}/"
ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "test ! -f ${REMOTE_ROOT}/*.bag; pgrep -af '^${BINARY} ' || true" \
  > "${LOCAL_RESULT}/board_after.txt"
ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "date --iso-8601=seconds; cat /proc/meminfo; echo process_snapshot; ps -eo pid,ppid,stat,%cpu,%mem,vsz,rss,nlwp,comm,args; echo kernel_tail; dmesg -T | tail -n 400" \
  > "${LOCAL_RESULT}/board_system_after.txt" 2>&1

find "${LOCAL_RESULT}" -maxdepth 2 -type f -printf '%P %s bytes\n' | sort \
  > "${LOCAL_RESULT}/result_files.txt"
sha256sum "${LOCAL_RESULT}/timing/rgbd_frame_timing.csv" \
  "${LOCAL_RESULT}/timing/local_mapping_timing.csv" \
  "${LOCAL_RESULT}/slam.log" >> "${LOCAL_RESULT}/metadata.txt"
trap - EXIT INT TERM

echo "${PREFIX}: callbacks=$(( $(wc -l < "${LOCAL_RESULT}/timing/rgbd_frame_timing.csv") - 1 )) play_exit=${play_exit} node_exit=${node_exit}"
