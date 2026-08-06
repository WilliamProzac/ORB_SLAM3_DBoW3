#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOARD="${RDK_X5_SSH:-root@192.168.1.10}"
REMOTE_ROOT="${RDK_X5_ROOT:-/userdata/xfeat_rdk_x5}"
MODEL="${SCRIPT_DIR}/artifacts/model_output/xfeat_backbone_544x640_bayes_e/xfeat_backbone_544x640_bayes_e.bin"
FINE_MODELS=(
  "${SCRIPT_DIR}/artifacts/model_output/xfeat_fine_matcher_m256_bayes_e/xfeat_fine_matcher_m256_bayes_e.bin"
  "${SCRIPT_DIR}/artifacts/model_output/xfeat_fine_matcher_m384_bayes_e/xfeat_fine_matcher_m384_bayes_e.bin"
  "${SCRIPT_DIR}/artifacts/model_output/xfeat_fine_matcher_m512_bayes_e/xfeat_fine_matcher_m512_bayes_e.bin"
)

if [[ ! -f "${MODEL}" ]]; then
  echo "Missing generated model: ${MODEL}" >&2
  exit 1
fi
for fine_model in "${FINE_MODELS[@]}"; do
  if [[ ! -f "${fine_model}" ]]; then
    echo "Missing Fine Matcher model: ${fine_model}" >&2
    exit 1
  fi
done

ssh -o BatchMode=yes "${BOARD}" \
  "mkdir -p '${REMOTE_ROOT}/benchmark_runner' '${REMOTE_ROOT}/benchmark_build'"
scp "${SCRIPT_DIR}/rdk_x5/CMakeLists.txt" \
    "${SCRIPT_DIR}/rdk_x5/dnn_runner.cpp" \
    "${SCRIPT_DIR}/rdk_x5/xfeat_frontend.hpp" \
    "${SCRIPT_DIR}/rdk_x5/xfeat_frontend.cpp" \
    "${SCRIPT_DIR}/rdk_x5/ros_benchmark.cpp" \
    "${SCRIPT_DIR}/rdk_x5/run_ros_benchmark.sh" \
    "${BOARD}:${REMOTE_ROOT}/benchmark_runner/"
scp "${MODEL}" "${BOARD}:${REMOTE_ROOT}/xfeat_backbone_544x640_bayes_e.bin"
scp "${FINE_MODELS[@]}" "${BOARD}:${REMOTE_ROOT}/"
ssh -o BatchMode=yes "${BOARD}" \
  "cd '${REMOTE_ROOT}/benchmark_build' && . /opt/ros/noetic/setup.sh && cmake '../benchmark_runner' -DCMAKE_BUILD_TYPE=Release && cmake --build . -j1"

echo "Deployed to ${BOARD}:${REMOTE_ROOT}"
