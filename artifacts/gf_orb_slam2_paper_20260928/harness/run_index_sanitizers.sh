#!/usr/bin/env bash
set -euo pipefail
repro_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
read -r -a opencv_flags <<< "$(pkg-config --cflags opencv4)"
for mode in asan tsan; do
  if [[ $mode == asan ]]; then flags=(-fsanitize=address,undefined); else flags=(-fsanitize=thread -fno-pie -no-pie); fi
  g++ -std=c++14 -O1 -g -fno-omit-frame-pointer "${flags[@]}" \
    -DGF_PAPER_BUCKETS -DCOMPILEDWITHC11 \
    -I"$repro_root/upstream" -I"$repro_root/upstream/include" -I/usr/include/eigen3 \
    "${opencv_flags[@]}" -include "$repro_root/harness/compat.h" \
    "$repro_root/upstream/src/Hashing.cc" "$repro_root/harness/test_hash_index.cc" "$repro_root/harness/test_hash_stubs.cc" \
    -lopencv_core -pthread -o "$repro_root/build/index_$mode"
  ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 TSAN_OPTIONS=halt_on_error=1 \
    "$repro_root/build/index_$mode" > "$repro_root/logs/index_$mode.log" 2>&1
  echo "$mode index fixture passed"
done
