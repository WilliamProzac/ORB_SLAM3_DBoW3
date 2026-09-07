#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BOARD="${RDK_X5_SSH:-root@192.168.1.10}"
HOST_ROS_IP="${HOST_ROS_IP:-192.168.1.3}"
STAGE="${F23_STAGE:-/userdata/orb_slam/f23_xfeat_20260810}"
BAG="${1:-${REPO_DIR}/2026-08-06-09-46-39.bag}"
RATE="${F23_PLAYBACK_RATE:-0.5}"
DURATION="${F23_DURATION:-8}"
ARTIFACTS="${F23_ARTIFACTS:-${SCRIPT_DIR}/artifacts/f23_xfeat_smoke}"
PREFIX="f23_xfeat_smoke"
REMOTE_CSV="${STAGE}/smoke.csv"
LOCAL_CSV="${ARTIFACTS}/smoke.csv"
LOG="${ARTIFACTS}/smoke.log"

[[ -f "${BAG}" ]] || { echo "Missing bag: ${BAG}" >&2; exit 2; }

. /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://${HOST_ROS_IP}:11311"
export ROS_IP="${HOST_ROS_IP}"
export NO_PROXY="localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10"
export no_proxy="${NO_PROXY}"
export ROS_HOME="${ROS_HOME:-${REPO_DIR}/.ros_runtime/f23_xfeat_smoke}"
mkdir -p "${ROS_HOME}" "${ARTIFACTS}"

remote_pid=""
cleanup() {
  rosnode kill /orb_slam3 >/dev/null 2>&1 || true
  if [[ -n "${remote_pid}" ]]; then
    wait "${remote_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ! rosnode list >/dev/null 2>&1; then
  echo "ROS master is not reachable at ${ROS_MASTER_URI}" >&2
  exit 2
fi

rosparam set /use_sim_time false
rosparam set /orb_slam3/voc_file \
  /userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3/Vocabulary/orbvoc.dbow3
rosparam set /orb_slam3/settings_file "${STAGE}/S316_XFeat.yaml"
rosparam set /orb_slam3/enable_pangolin false
rosparam set /orb_slam3/frontend_stats_csv "${REMOTE_CSV}"

ssh -T -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "source /opt/ros/noetic/setup.bash; source /userdata/orb_slam/devel/setup.bash; export ROS_MASTER_URI=http://${HOST_ROS_IP}:11311; export ROS_IP=192.168.1.10; export NO_PROXY=localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10; export ORB_ROOT=/userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3; export LD_LIBRARY_PATH=${STAGE}:\${ORB_ROOT}/lib:\${ORB_ROOT}/Thirdparty/DBoW3/lib:\${ORB_ROOT}/Thirdparty/g2o/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu:/opt/ros/noetic/lib; exec taskset -c 2-7 ${STAGE}/ros_stereo __name:=orb_slam3 /camera/infra1/image_raw:=/${PREFIX}/infra1 /camera/infra2/image_raw:=/${PREFIX}/infra2" \
  >"${LOG}" 2>&1 &
remote_pid="$!"

for _ in $(seq 1 90); do
  if rosnode ping -c 1 /orb_slam3 >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
if ! rosnode ping -c 1 /orb_slam3 >/dev/null 2>&1; then
  echo "Timed out waiting for the F23 node; see ${LOG}" >&2
  exit 3
fi

# Node registration precedes vocabulary and BPU model initialization.
sleep 20
rosbag play "${BAG}" --quiet --delay=1.0 --rate "${RATE}" \
  --duration "${DURATION}" --topics \
  /camera/infra1/image_raw /camera/infra2/image_raw \
  /camera/infra1/image_raw:=/${PREFIX}/infra1 \
  /camera/infra2/image_raw:=/${PREFIX}/infra2

rosnode kill /orb_slam3 >/dev/null 2>&1 || true
wait "${remote_pid}" 2>/dev/null || true
remote_pid=""
scp -o BatchMode=yes -o ConnectTimeout=5 \
  "${BOARD}:${REMOTE_CSV}" "${LOCAL_CSV}"

rows=$(( $(wc -l < "${LOCAL_CSV}") - 1 ))
echo "F23 XFeat smoke callbacks: ${rows}"
sha256sum "${LOCAL_CSV}" "${LOG}"
trap - EXIT INT TERM
