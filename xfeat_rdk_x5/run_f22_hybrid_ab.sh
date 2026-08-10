#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BOARD="${RDK_X5_SSH:-root@192.168.1.10}"
HOST_ROS_IP="${HOST_ROS_IP:-192.168.1.3}"
STAGE="${F22_STAGE:-/userdata/orb_slam/f22_hybrid_20260810}"
RATE="${F22_PLAYBACK_RATE:-0.5}"
ARTIFACTS="${F22_ARTIFACTS:-${SCRIPT_DIR}/artifacts/f22_hybrid_ab}"
BAGS=(
  "${REPO_DIR}/2026-08-06-09-46-39.bag"
  "${REPO_DIR}/2026-08-06-09-48-14.bag"
  "/tmp/f22_2026-08-06-09-50-14.bag"
)

. /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://${HOST_ROS_IP}:11311"
export ROS_IP="${HOST_ROS_IP}"
export NO_PROXY="localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10"
export no_proxy="${NO_PROXY}"
export ROS_HOME="${ROS_HOME:-${REPO_DIR}/.ros_runtime/f22_hybrid_ab}"
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

wait_for_node() {
  local attempt
  for attempt in $(seq 1 90); do
    if rosnode ping -c 1 /orb_slam3 >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

run_one() {
  local mode="$1"
  local bag="$2"
  local bag_id
  local mode_lower
  bag_id="$(basename "${bag}" .bag | sed 's/^f22_//')"
  mode_lower="$(printf '%s' "${mode}" | tr '[:upper:]' '[:lower:]')"
  local prefix="f22_${bag_id//-/_}_${mode_lower}"
  local remote_csv="${STAGE}/${prefix}.csv"
  local local_csv="${ARTIFACTS}/${prefix}.csv"
  local log="${ARTIFACTS}/${prefix}.log"
  local settings="${STAGE}/S316_${mode}.yaml"

  rosparam set /use_sim_time false
  rosparam set /orb_slam3/voc_file \
    /userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3/Vocabulary/orbvoc.dbow3
  rosparam set /orb_slam3/settings_file "${settings}"
  rosparam set /orb_slam3/enable_pangolin false
  rosparam set /orb_slam3/frontend_stats_csv "${remote_csv}"

  ssh -T -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
    "source /opt/ros/noetic/setup.bash; source /userdata/orb_slam/devel/setup.bash; export ROS_MASTER_URI=http://${HOST_ROS_IP}:11311; export ROS_IP=192.168.1.10; export NO_PROXY=localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10; export ORB_ROOT=/userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3; export LD_LIBRARY_PATH=${STAGE}:\${ORB_ROOT}/lib:\${ORB_ROOT}/Thirdparty/DBoW3/lib:\${ORB_ROOT}/Thirdparty/g2o/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu:/opt/ros/noetic/lib; exec taskset -c 2-7 ${STAGE}/ros_stereo __name:=orb_slam3 /camera/infra1/image_raw:=/${prefix}/infra1 /camera/infra2/image_raw:=/${prefix}/infra2" \
    >"${log}" 2>&1 &
  remote_pid="$!"
  if ! wait_for_node; then
    echo "Timed out waiting for ${mode} node; see ${log}" >&2
    return 1
  fi
  # ros::init registers the name before the large vocabulary and BPU models
  # are ready. Keep bag publication away from that initialization window.
  sleep 20
  rosbag play "${bag}" --quiet --delay=1.0 --rate "${RATE}" --topics \
    /camera/infra1/image_raw /camera/infra2/image_raw \
    /camera/infra1/image_raw:=/${prefix}/infra1 \
    /camera/infra2/image_raw:=/${prefix}/infra2
  rosnode kill /orb_slam3 >/dev/null 2>&1 || true
  wait "${remote_pid}" 2>/dev/null || true
  remote_pid=""
  scp -o BatchMode=yes -o ConnectTimeout=5 \
    "${BOARD}:${remote_csv}" "${local_csv}"
  echo "${mode} ${bag_id}: $(( $(wc -l < "${local_csv}") - 1 )) callbacks"
  sha256sum "${local_csv}" "${log}"
}

for bag in "${BAGS[@]}"; do
  [[ -f "${bag}" ]] || { echo "Missing bag: ${bag}" >&2; exit 2; }
  run_one ORB "${bag}"
  run_one Hybrid "${bag}"
done

python3 "${SCRIPT_DIR}/summarize_f22_hybrid_ab.py" \
  "${ARTIFACTS}" --output "${ARTIFACTS}/summary.json"
trap - EXIT INT TERM
