#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${SCRIPT_DIR}/artifacts/orb_slam3_stereo_dump"

mkdir -p "${SCRIPT_DIR}/artifacts"
g++ -std=c++14 -O2 "${SCRIPT_DIR}/orb_slam3_stereo_dump.cpp" \
  -o "${OUTPUT}" \
  -I"${REPO_ROOT}/include" -I"${REPO_ROOT}" \
  -I"${REPO_ROOT}/Thirdparty/Sophus" -I/usr/include/eigen3 \
  -L"${REPO_ROOT}/lib" \
  -L"${REPO_ROOT}/Thirdparty/DBoW3/lib" \
  -L"${REPO_ROOT}/Thirdparty/g2o/lib" \
  -lORB_SLAM3 -lDBoW3 -lg2o \
  $(pkg-config --cflags --libs opencv4) \
  -Wl,-rpath,"${REPO_ROOT}/lib" \
  -Wl,-rpath,"${REPO_ROOT}/Thirdparty/DBoW3/lib" \
  -Wl,-rpath,"${REPO_ROOT}/Thirdparty/g2o/lib"

echo "built ${OUTPUT}"
