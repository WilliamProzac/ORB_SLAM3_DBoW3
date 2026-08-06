#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
. /opt/ros/noetic/setup.bash
set -u

HOST_ROS_IP="${HOST_ROS_IP:-192.168.1.3}"
export ROS_MASTER_URI="http://${HOST_ROS_IP}:11311"
export ROS_IP="${HOST_ROS_IP}"
export ROS_HOME="${ROS_HOME:-${REPO_DIR}/.ros_runtime/lan}"
mkdir -p "${ROS_HOME}"

usage() {
  echo "Usage: $0 master | check | play BAG [rosbag-play-options...]" >&2
}

case "${1:-}" in
  master)
    exec roscore
    ;;
  check)
    echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
    echo "ROS_IP=${ROS_IP}"
    exec rosnode list
    ;;
  play)
    if [[ $# -lt 2 ]]; then
      usage
      exit 2
    fi
    bag="$2"
    shift 2
    exec rosbag play "${bag}" "$@" --topics \
      /camera/infra1/image_raw \
      /camera/infra2/image_raw \
      /robot_pose_tracking_ok
    ;;
  *)
    usage
    exit 2
    ;;
esac
