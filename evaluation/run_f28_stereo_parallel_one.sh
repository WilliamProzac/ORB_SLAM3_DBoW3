#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "Usage: $0 <stereo|stereo_inertial> <workers:1-4> <094639|094814|095014> <repeat>" >&2
  exit 2
fi

MODE="$1"
WORKERS="$2"
BAG_ID="$3"
REPEAT="$4"
case "${MODE}" in stereo|stereo_inertial) ;; *) echo "Invalid mode" >&2; exit 2;; esac
case "${WORKERS}" in 1|2|3|4) ;; *) echo "Invalid worker count" >&2; exit 2;; esac
case "${BAG_ID}" in 094639|094814|095014) ;; *) echo "Invalid bag id" >&2; exit 2;; esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG_ROOT="${SLAM_DATA_ROOT:-${REPO_DIR}}"
BOARD="${RDK_X5_SSH:-root@192.168.1.10}"
HOST_ROS_IP="${HOST_ROS_IP:-192.168.1.3}"
ROS_PORT="${F28_ROS_PORT:-11312}"
STAGE="${F28_STAGE:-/userdata/orb_slam/f28_stereo_parallel_tuning_20260820}"
ARTIFACTS="${F28_ARTIFACTS:-${SCRIPT_DIR}/artifacts/f28_stereo_parallel_tuning}"
RATE="${F28_PLAYBACK_RATE:-1.0}"
DURATION="${F28_DURATION:-}"
PREFIX="f28_${BAG_ID}_${MODE}_w${WORKERS}_${REPEAT}"

case "${BAG_ID}" in
  094639) BAG="${BAG_ROOT}/2026-08-06-09-46-39.bag" ;;
  094814) BAG="${BAG_ROOT}/2026-08-06-09-48-14.bag" ;;
  095014) BAG="/tmp/orb_bag_analysis_20260818/2026-08-06-09-50-14.bag" ;;
esac
[[ -f "${BAG}" ]] || { echo "Missing bag: ${BAG}" >&2; exit 2; }

. /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://${HOST_ROS_IP}:${ROS_PORT}"
export ROS_IP="${HOST_ROS_IP}"
export NO_PROXY="localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10"
export no_proxy="${NO_PROXY}"
export ROS_HOME="${ROS_HOME:-${REPO_DIR}/.ros_runtime_11312/f28_stereo_parallel}"
mkdir -p "${ROS_HOME}" "${ARTIFACTS}"

NODE_NAME="/${PREFIX}"
REMOTE_CSV="${STAGE}/${PREFIX}.csv"
LOCAL_CSV="${ARTIFACTS}/${PREFIX}.csv"
LOG="${ARTIFACTS}/${PREFIX}.log"
TELEMETRY="${ARTIFACTS}/${PREFIX}_telemetry.csv"
META="${ARTIFACTS}/${PREFIX}_metadata.txt"

if [[ "${MODE}" == "stereo" ]]; then
  BINARY="${STAGE}/ros_stereo"
  SETTINGS="${STAGE}/S316_ORB_w${WORKERS}.yaml"
else
  BINARY="${STAGE}/ros_stereo_inertial"
  SETTINGS="${STAGE}/S316_Inertial_w${WORKERS}.yaml"
fi

if ! rosnode list >/dev/null 2>&1; then
  echo "ROS master is not reachable at ${ROS_MASTER_URI}" >&2
  exit 2
fi

# The live camera runs against its own board-local ROS master. It supplies the
# real camera/driver CPU load without mixing live timestamps into bag replay.
ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "source /opt/ros/noetic/setup.bash; ROS_MASTER_URI=http://127.0.0.1:11311 rosnode list | grep -qx /s316_camera_node"

{
  echo "prefix=${PREFIX}"
  echo "mode=${MODE}"
  echo "parallel_workers=${WORKERS}"
  echo "bag=${BAG}"
  echo "playback_rate=${RATE}"
  echo "duration_s=${DURATION:-full}"
  echo "slam_cpu_set=2-7"
  echo "camera_cpu_set=0-1"
  date --iso-8601=seconds
  sha256sum "${BAG}"
  ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
    "sha256sum ${BINARY} ${STAGE}/liborb_slam3_ros.so ${SETTINGS}; pgrep -n -f '^/ros1_project/camera_ws/devel/lib/s316_camera/s316_camera_node ' | xargs -r taskset -pc"
} > "${META}"

rosparam set "${NODE_NAME}/voc_file" \
  /userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3/Vocabulary/orbvoc.dbow3
rosparam set "${NODE_NAME}/settings_file" "${SETTINGS}"
rosparam set "${NODE_NAME}/enable_pangolin" false
if [[ "${MODE}" == "stereo" ]]; then
  rosparam set "${NODE_NAME}/frontend_stats_csv" "${REMOTE_CSV}"
else
  rosparam set "${NODE_NAME}/left_topic" /camera/infra1/image_raw
  rosparam set "${NODE_NAME}/right_topic" /camera/infra2/image_raw
  rosparam set "${NODE_NAME}/imu_topic" /camera/imu/data
  rosparam set "${NODE_NAME}/imu_time_shift" 0.016355351
  rosparam set "${NODE_NAME}/maximum_stereo_delta" 0.005
  rosparam set "${NODE_NAME}/image_queue_capacity" 2
  rosparam set "${NODE_NAME}/imu_queue_capacity" 4000
  rosparam set "${NODE_NAME}/sync_stats_csv" "${REMOTE_CSV}"
fi

remote_ssh_pid=""
telemetry_ssh_pid=""
cleanup() {
  rosnode kill "${NODE_NAME}" >/dev/null 2>&1 || true
  [[ -z "${telemetry_ssh_pid}" ]] || kill "${telemetry_ssh_pid}" >/dev/null 2>&1 || true
  [[ -z "${remote_ssh_pid}" ]] || wait "${remote_ssh_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ssh -T -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "source /opt/ros/noetic/setup.bash; source /userdata/orb_slam/devel/setup.bash; export ROS_MASTER_URI=http://${HOST_ROS_IP}:${ROS_PORT}; export ROS_IP=192.168.1.10; export NO_PROXY=localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10; export ORB_ROOT=/userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3; export LD_LIBRARY_PATH=${STAGE}:\${ORB_ROOT}/Thirdparty/DBoW3/lib:\${ORB_ROOT}/Thirdparty/g2o/lib:/usr/local/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu:/opt/ros/noetic/lib; exec taskset -c 2-7 ${BINARY} __name:=${PREFIX}" \
  >"${LOG}" 2>&1 &
remote_ssh_pid="$!"

ready=0
for attempt in $(seq 1 120); do
  if rosnode info "${NODE_NAME}" 2>/dev/null | grep -q '/camera/infra1/image_raw'; then
    ready=1
    break
  fi
  sleep 1
done
[[ "${ready}" == 1 ]] || { echo "Timed out waiting for ${NODE_NAME}" >&2; exit 1; }

REMOTE_NODE_PID="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" "pgrep -n -f '^${BINARY} __name:=${PREFIX}'")"
echo 'timestamp_s,node_cpu_pct,node_rss_kb,node_threads,camera_cpu_pct,camera_rss_kb,camera_threads,temp_mC,max_freq_kHz' > "${TELEMETRY}"
ssh -T -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "while kill -0 ${REMOTE_NODE_PID} 2>/dev/null; do ts=\$(date +%s.%N); node=\$(ps -p ${REMOTE_NODE_PID} -o %cpu=,rss=,nlwp= | tr -s ' ' | sed 's/^ //'); camera_pid=\$(pgrep -n -f '^/ros1_project/camera_ws/devel/lib/s316_camera/s316_camera_node ' || true); if test -n \"\${camera_pid}\"; then camera=\$(ps -p \${camera_pid} -o %cpu=,rss=,nlwp= | tr -s ' ' | sed 's/^ //'); else camera='0 0 0'; fi; temp=\$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -nr | head -n1); freq=\$(cat /sys/devices/system/cpu/cpu[0-7]/cpufreq/scaling_cur_freq 2>/dev/null | sort -nr | head -n1); echo \"\${ts},\$(echo \${node} | tr ' ' ','),\$(echo \${camera} | tr ' ' ','),\${temp:-0},\${freq:-0}\"; sleep 1; done" \
  >> "${TELEMETRY}" 2>/dev/null &
telemetry_ssh_pid="$!"

play_args=("${BAG}" --quiet --delay=1.0 --rate "${RATE}")
if [[ -n "${DURATION}" ]]; then
  play_args+=(--duration "${DURATION}")
fi
rosbag play "${play_args[@]}" --topics \
  /camera/infra1/image_raw /camera/infra2/image_raw /camera/imu/data
sleep 5
rosnode kill "${NODE_NAME}" >/dev/null 2>&1 || true
wait "${remote_ssh_pid}" 2>/dev/null || true
remote_ssh_pid=""
wait "${telemetry_ssh_pid}" 2>/dev/null || true
telemetry_ssh_pid=""

scp -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}:${REMOTE_CSV}" "${LOCAL_CSV}"
echo "${PREFIX}: $(( $(wc -l < "${LOCAL_CSV}") - 1 )) callbacks"
sha256sum "${LOCAL_CSV}" "${LOG}" "${TELEMETRY}" >> "${META}"
trap - EXIT INT TERM
