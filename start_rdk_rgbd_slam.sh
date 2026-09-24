#!/usr/bin/env bash
# Run on the laptop; ROS and SLAM run only on the RDK X5.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 -u "$script_dir/evaluation/rdk_rgbd_live.py" host "$@"
