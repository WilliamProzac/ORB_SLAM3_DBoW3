#!/usr/bin/env bash
set -eo pipefail

. /opt/ros/noetic/setup.bash
set -u

MASTER_IP="${ROS_MASTER_IP:-192.168.1.3}"
BOARD_IP="${RDK_X5_ROS_IP:-192.168.1.10}"
REMOTE_ROOT="${RDK_X5_ROOT:-/userdata/xfeat_rdk_x5}"
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="${BOARD_IP}"
export LD_LIBRARY_PATH="/usr/hobot/lib:/usr/lib/aarch64-linux-gnu:/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

exec "${REMOTE_ROOT}/benchmark_build/xfeat_ros_benchmark" \
  _model_path:="${REMOTE_ROOT}/xfeat_backbone_544x640_bayes_e.bin" \
  _output_csv:="${REMOTE_ROOT}/benchmark_full_544x640.csv" \
  _top_k:=600 \
  _anchor_stride:=10 \
  _temporal_gap:=1 \
  _max_anchors:=0 \
  _cpu_threads:=8 \
  _xfeat_min_cosine:=0.82 \
  _xfeat_grid_columns:=8 \
  _xfeat_grid_rows:=6 \
  _xfeat_fixed_matches:=600 \
  _xfeat_semidense_single:=false \
  _fine_model_path:="" \
  _xfeat_extraction_grid_maximum_per_cell:=0 \
  _xfeat_grid_maximum_per_cell:=0 \
  _orb_max_hamming:=64.0 \
  "$@"
