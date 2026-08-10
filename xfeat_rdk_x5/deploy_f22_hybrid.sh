#!/usr/bin/env bash
set -euo pipefail

BOARD="${RDK_X5_SSH:-root@192.168.1.10}"
RDK_WORKSPACE="${RDK_WORKSPACE:-/home/ywl/Project/RDK_X5_orb_slam}"
STAGE="${F22_STAGE:-/userdata/orb_slam/f22_hybrid_20260810}"
PACKAGE="${RDK_WORKSPACE}/src/orb_slam3_ros-master"

binary="${RDK_WORKSPACE}/devel/lib/orb_slam3_ros/ros_stereo"
library="${PACKAGE}/orb_slam3/lib/liborb_slam3_ros.so"
settings="${PACKAGE}/config/Stereo/S316.yaml"
for path in "${binary}" "${library}" "${settings}"; do
  if [[ ! -f "${path}" ]]; then
    echo "Missing F22 deployment input: ${path}" >&2
    exit 2
  fi
done

ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" "mkdir -p '${STAGE}'"
scp -o BatchMode=yes -o ConnectTimeout=5 \
  "${binary}" "${BOARD}:${STAGE}/ros_stereo"
scp -o BatchMode=yes -o ConnectTimeout=5 \
  "${library}" "${BOARD}:${STAGE}/liborb_slam3_ros.so"
scp -o BatchMode=yes -o ConnectTimeout=5 \
  "${settings}" "${BOARD}:${STAGE}/S316_Hybrid.yaml"
ssh -o BatchMode=yes -o ConnectTimeout=5 "${BOARD}" \
  "sed 's/^Feature.Type:.*/Feature.Type: \"ORB\"/' '${STAGE}/S316_Hybrid.yaml' > '${STAGE}/S316_ORB.yaml'; chmod +x '${STAGE}/ros_stereo'; sha256sum '${STAGE}/ros_stereo' '${STAGE}/liborb_slam3_ros.so' '${STAGE}/S316_Hybrid.yaml' '${STAGE}/S316_ORB.yaml'"
