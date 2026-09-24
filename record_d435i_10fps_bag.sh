#!/usr/bin/env bash
# Run on the laptop. The board starts D435i 30 FPS capture, publishes exact-pair
# 10 FPS RGB-D, and records the 10 FPS product locally on the board.
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
exec /usr/bin/python3 "$SCRIPT_DIR/evaluation/rgbd_recorder.py" host \
  --board sunrise@192.168.1.11 \
  --remote-root /userdata/d435i_10fps_bags \
  --ros-setup /opt/ros/noetic/setup.bash \
  --camera-setup /userdata/ros1_realsense_ws/devel/setup.bash \
  --camera-start-script /userdata/ros1_realsense_ws/start_rgbd10.sh \
  --busy-process realsense2_camera_node \
  --busy-process realsense2_camera_manager \
  --busy-process rgbd_decimator_node \
  --rgb-topic /camera_10fps/color/image_raw \
  --depth-topic /camera_10fps/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera_10fps/color/camera_info \
  --camera-info-topic /camera_10fps/aligned_depth_to_color/camera_info \
  --snapshot /userdata/ros1_realsense_ws/src/rgbd_rate_adapter/launch/d435i_rgbd_30_to_10.launch=d435i_rgbd_30_to_10.launch \
  --snapshot /userdata/ros1_realsense_ws/src/rgbd_rate_adapter/src/rgbd_decimator_node.cpp=rgbd_decimator_node.cpp \
  --min-hz 9.5 \
  --sync-ms 0.1 \
  --warmup 2 \
  --run-prefix d435i10 \
  --wrapper ./record_d435i_10fps_bag.sh \
  "${ARGS[@]}"
