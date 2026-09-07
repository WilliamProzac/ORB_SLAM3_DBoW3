#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG_ROOT="${SLAM_DATA_ROOT:-${REPO_DIR}}"
BOARD="${RDK_X5_SSH:-root@192.168.1.10}"
HOST_ROS_IP="${HOST_ROS_IP:-192.168.1.3}"
ROS_PORT="${F27_ROS_PORT:-11312}"
STAGE="${F27_STAGE:-/userdata/orb_slam/f27_stereo_vs_imu_rate1_20260818}"
ARTIFACTS="${F27_ARTIFACTS:-${SCRIPT_DIR}/artifacts/f27_stereo_vs_imu_rate1}"
RATE="${F27_PLAYBACK_RATE:-1.0}"
BAGS=(
  "${BAG_ROOT}/2026-08-06-09-46-39.bag"
  "${BAG_ROOT}/2026-08-06-09-48-14.bag"
  "/tmp/orb_bag_analysis_20260818/2026-08-06-09-50-14.bag"
)

. /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://${HOST_ROS_IP}:${ROS_PORT}"
export ROS_IP="${HOST_ROS_IP}"
export NO_PROXY="localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10"
export no_proxy="${NO_PROXY}"
export ROS_HOME="${ROS_HOME:-${REPO_DIR}/.ros_runtime_11312/f27_stereo_vs_imu}"
mkdir -p "${ROS_HOME}" "${ARTIFACTS}"

remote_pid=""
current_node=""
cleanup() {
  if [[ -n "${current_node}" ]]; then
    rosnode kill "${current_node}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${remote_pid}" ]]; then
    wait "${remote_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ! rosnode list >/dev/null 2>&1; then
  echo "ROS master is not reachable at ${ROS_MASTER_URI}" >&2
  exit 2
fi

wait_for_subscriptions() {
  local node_name="$1"
  local attempt
  for attempt in $(seq 1 120); do
    if rosnode info "${node_name}" 2>/dev/null |
       grep -q '/camera/infra1/image_raw'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

run_one() {
  local mode="$1"
  local bag="$2"
  local bag_base bag_id prefix node_name binary settings remote_csv local_csv log
  bag_base="$(basename "${bag}" .bag)"
  bag_id="${bag_base#2026-08-06-}"
  bag_id="${bag_id//-/}"
  prefix="f27_${bag_id}_${mode}"
  node_name="/${prefix}"
  remote_csv="${STAGE}/${prefix}.csv"
  local_csv="${ARTIFACTS}/${prefix}.csv"
  log="${ARTIFACTS}/${prefix}.log"

  if [[ "${mode}" == "stereo" ]]; then
    binary="${STAGE}/ros_stereo"
    settings="${STAGE}/S316_ORB.yaml"
  else
    binary="${STAGE}/ros_stereo_inertial"
    settings="${STAGE}/S316_Inertial.yaml"
  fi

  rosparam set "${node_name}/voc_file" \
    /userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3/Vocabulary/orbvoc.dbow3
  rosparam set "${node_name}/settings_file" "${settings}"
  rosparam set "${node_name}/enable_pangolin" false
  if [[ "${mode}" == "stereo" ]]; then
    rosparam set "${node_name}/frontend_stats_csv" "${remote_csv}"
  else
    rosparam set "${node_name}/left_topic" /camera/infra1/image_raw
    rosparam set "${node_name}/right_topic" /camera/infra2/image_raw
    rosparam set "${node_name}/imu_topic" /camera/imu/data
    rosparam set "${node_name}/imu_time_shift" 0.016355351
    rosparam set "${node_name}/maximum_stereo_delta" 0.005
    rosparam set "${node_name}/image_queue_capacity" 2
    rosparam set "${node_name}/imu_queue_capacity" 4000
    rosparam set "${node_name}/sync_stats_csv" "${remote_csv}"
  fi

  ssh -T -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
    "source /opt/ros/noetic/setup.bash; source /userdata/orb_slam/devel/setup.bash; export ROS_MASTER_URI=http://${HOST_ROS_IP}:${ROS_PORT}; export ROS_IP=192.168.1.10; export NO_PROXY=localhost,127.0.0.1,${HOST_ROS_IP},192.168.1.10; export ORB_ROOT=/userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3; export LD_LIBRARY_PATH=${STAGE}:\${ORB_ROOT}/Thirdparty/DBoW3/lib:\${ORB_ROOT}/Thirdparty/g2o/lib:/usr/local/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu:/opt/ros/noetic/lib; exec taskset -c 2-7 ${binary} __name:=${prefix}" \
    >"${log}" 2>&1 &
  remote_pid="$!"
  current_node="${node_name}"

  if ! wait_for_subscriptions "${node_name}"; then
    echo "Timed out waiting for ${node_name}; see ${log}" >&2
    return 1
  fi

  rosbag play "${bag}" --quiet --delay=1.0 --rate "${RATE}" --topics \
    /camera/infra1/image_raw /camera/infra2/image_raw /camera/imu/data
  sleep 5
  rosnode kill "${node_name}" >/dev/null 2>&1 || true
  wait "${remote_pid}" 2>/dev/null || true
  remote_pid=""
  current_node=""

  scp -o BatchMode=yes -o ConnectTimeout=5 \
    "${BOARD}:${remote_csv}" "${local_csv}"
  echo "${mode} ${bag_base}: $(( $(wc -l < "${local_csv}") - 1 )) callbacks"
  sha256sum "${local_csv}" "${log}"
}

for index in "${!BAGS[@]}"; do
  bag="${BAGS[$index]}"
  [[ -f "${bag}" ]] || { echo "Missing bag: ${bag}" >&2; exit 2; }
  if (( index % 2 == 0 )); then
    run_one stereo "${bag}"
    run_one stereo_inertial "${bag}"
  else
    run_one stereo_inertial "${bag}"
    run_one stereo "${bag}"
  fi
done

python3 "${SCRIPT_DIR}/summarize_f27_stereo_vs_imu.py" \
  "${ARTIFACTS}" --output "${ARTIFACTS}/summary.json"
trap - EXIT INT TERM
