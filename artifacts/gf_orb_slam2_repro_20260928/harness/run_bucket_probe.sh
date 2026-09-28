#!/usr/bin/env bash
set -euo pipefail
repro_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
read -r -a opencv_flags <<< "$(pkg-config --cflags opencv4)"
g++ -std=c++14 -DCOMPILEDWITHC11 \
  -I"$repro_root/upstream" -I"$repro_root/upstream/include" \
  -I/usr/include/eigen3 -I/opt/ros/noetic/include \
  "${opencv_flags[@]}" -include "$repro_root/harness/compat.h" \
  "$repro_root/harness/test_bucket.cc" \
  -L"$repro_root/build" -Wl,-rpath,"$repro_root/build" -lgf_baseline \
  -o "$repro_root/build/bucket_probe"
"$repro_root/build/bucket_probe"
