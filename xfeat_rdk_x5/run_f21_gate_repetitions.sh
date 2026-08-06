#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BOARD="${RDK_X5_SSH:-root@192.168.1.10}"
BAG="${F21_BAG:-${REPO_DIR}/2026-06-05-09-59-04.bag}"
ARTIFACTS="${SCRIPT_DIR}/artifacts"
START_RUN="${1:-2}"
END_RUN="${2:-5}"

. /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://192.168.1.3:11311"
export ROS_IP="192.168.1.3"
export NO_PROXY="localhost,127.0.0.1,192.168.1.3,192.168.1.10"
export no_proxy="${NO_PROXY}"

remote_pid=""
cleanup() {
  if [[ -n "${remote_pid}" ]] && kill -0 "${remote_pid}" 2>/dev/null; then
    kill -INT "${remote_pid}" 2>/dev/null || true
    wait "${remote_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

wait_for_node() {
  local node="$1"
  local attempt
  for attempt in $(seq 1 90); do
    if rosnode ping -c 1 "${node}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${node}" >&2
  return 1
}

stop_node() {
  local node="$1"
  rosnode kill "${node}" >/dev/null 2>&1 || true
  if [[ -n "${remote_pid}" ]]; then
    wait "${remote_pid}" 2>/dev/null || true
    remote_pid=""
  fi
}

play_bag() {
  "${SCRIPT_DIR}/host_ros_lan.sh" play "${BAG}" --quiet --delay=1.0
}

run_orb() {
  local run="$1"
  local remote_csv="/userdata/orb_slam/f21_gate_close_20260803/orb_run${run}.csv"
  local local_csv="${ARTIFACTS}/f21_gate_orb_run${run}.csv"
  local log="${ARTIFACTS}/f21_gate_orb_run${run}.log"
  rosparam set /use_sim_time false
  rosparam set /orb_slam3/voc_file \
    /userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3/Vocabulary/orbvoc.dbow3
  rosparam set /orb_slam3/settings_file \
    /userdata/orb_slam/src/orb_slam3_ros-master/config/Stereo/S316.yaml
  rosparam set /orb_slam3/enable_pangolin false
  rosparam set /orb_slam3/frontend_stats_csv "${remote_csv}"
  ssh -T "${BOARD}" \
    'source /opt/ros/noetic/setup.bash; source /userdata/orb_slam/devel/setup.bash; export ROS_MASTER_URI=http://192.168.1.3:11311; export ROS_IP=192.168.1.10; export NO_PROXY=localhost,127.0.0.1,192.168.1.3,192.168.1.10; export ORB_ROOT=/userdata/orb_slam/src/orb_slam3_ros-master/orb_slam3; export LD_LIBRARY_PATH=/userdata/orb_slam/f21_gate_close_20260803:$ORB_ROOT/lib:$ORB_ROOT/Thirdparty/DBoW3/lib:$ORB_ROOT/Thirdparty/g2o/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu:/opt/ros/noetic/lib; exec /userdata/orb_slam/f21_gate_close_20260803/ros_stereo __name:=orb_slam3' \
    >"${log}" 2>&1 &
  remote_pid="$!"
  wait_for_node /orb_slam3
  # The ROS node registers before the large DBoW3 vocabulary finishes loading.
  # Do not publish the first bag frame until System initialization is complete.
  sleep 15
  play_bag
  stop_node /orb_slam3
  scp "${BOARD}:${remote_csv}" "${local_csv}"
  local lines
  lines="$(wc -l < "${local_csv}")"
  # The production queue is intentionally depth one. A dropped callback is a
  # real 10 Hz baseline outcome, so retain and report it instead of retrying
  # until a perfect run appears. Require at least 99% of 644 source frames.
  [[ "${lines}" -ge 641 && "${lines}" -le 645 ]]
  echo "ORB run ${run}: $((lines - 1))/644 callbacks recorded"
  sha256sum "${local_csv}"
}

run_xfeat() {
  local run="$1"
  local remote_csv="/userdata/xfeat_rdk_x5/f21_gate_xfeat_run${run}.csv"
  local local_csv="${ARTIFACTS}/f21_gate_xfeat_run${run}.csv"
  local log="${ARTIFACTS}/f21_gate_xfeat_run${run}.log"
  ssh -T "${BOARD}" \
    "/userdata/xfeat_rdk_x5/benchmark_runner/run_ros_benchmark.sh _output_csv:=${remote_csv} _top_k:=600 _anchor_stride:=10 _temporal_gap:=1 _max_anchors:=0 _cpu_threads:=8 _xfeat_min_cosine:=0.82 _xfeat_semidense_single:=true _fine_model_path:=/userdata/xfeat_rdk_x5/xfeat_fine_matcher_m384_bayes_e.bin _xfeat_fixed_matches:=384 _xfeat_grid_columns:=8 _xfeat_grid_rows:=6 _xfeat_extraction_grid_maximum_per_cell:=-1 _xfeat_grid_maximum_per_cell:=16 _xfeat_fine_confidence:=0.20 _minimum_patch_ncc:=0.5" \
    >"${log}" 2>&1 &
  remote_pid="$!"
  wait_for_node /xfeat_orb_benchmark
  sleep 2
  play_bag
  stop_node /xfeat_orb_benchmark
  scp "${BOARD}:${remote_csv}" "${local_csv}"
  [[ "$(wc -l < "${local_csv}")" -eq 521 ]]
  sha256sum "${local_csv}"
}

mkdir -p "${ARTIFACTS}"
for run in $(seq "${START_RUN}" "${END_RUN}"); do
  echo "F21 run ${run}: ORB"
  run_orb "${run}"
  echo "F21 run ${run}: XFeat"
  run_xfeat "${run}"
done

trap - EXIT INT TERM
