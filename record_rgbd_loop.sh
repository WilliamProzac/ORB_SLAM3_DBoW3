#!/usr/bin/env bash
# Run on the laptop. All images are recorded locally on the board.
set -eo pipefail
ARGS=("$@")
# ROS/catkin setup scripts inspect positional arguments themselves.
set --
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${LOCAL_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
if [[ ! -r "$ROS_SETUP" ]]; then
  echo "找不到本机 ROS 环境: $ROS_SETUP；可设置 LOCAL_ROS_SETUP。" >&2
  exit 1
fi
source "$ROS_SETUP"
exec /usr/bin/python3 "$SCRIPT_DIR/evaluation/rgbd_recorder.py" host "${ARGS[@]}"
