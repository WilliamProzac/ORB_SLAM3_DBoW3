#!/usr/bin/env bash

set -Eeuo pipefail

MODE="attach"
WORKSPACE="/userdata/orb_slam"
OUT_ROOT=""
RUN_ID=""
PACKAGE="orb_slam3_ros"
LAUNCH_FILE="kitti_stereo.launch"
BAG_SPLIT_DURATION="60"
SNAPSHOT_INTERVAL="30"
RECORD_IMAGES=true
RECORD_GRID=true
RECORD_FUTURE_DEBUG=true
NO_BAG=false
COMPRESS=false

SCRIPT_NAME="$(basename "$0")"
declare -a COLLECTOR_PIDS=()
SLAM_PID=""
DBG=""

usage() {
  cat <<USAGE
Usage:
  $SCRIPT_NAME [attach|launch-slam] [options]

Modes:
  attach       Collect diagnostics from an already-running ROS/SLAM session.
  launch-slam  Launch "roslaunch <package> <launch-file>" and collect diagnostics.

Options:
  --workspace PATH       Catkin workspace on the RDK board. Default: $WORKSPACE
  --out-root PATH        Output root. Default: <workspace>/debug_runs
  --run-id ID            Run directory name. Default: timestamp
  --package NAME         ROS package for launch-slam. Default: $PACKAGE
  --launch FILE          Launch file for launch-slam. Default: $LAUNCH_FILE
  --bag-split SEC        rosbag split duration. Default: $BAG_SPLIT_DURATION
  --snapshot-interval SEC
                         Refresh ROS node/topic/param snapshots. Default: $SNAPSHOT_INTERVAL
  --no-images            Do not record image or pointcloud topics.
  --no-grid              Do not record /ppseg_gridmap.
  --no-future-debug      Do not record optional future debug topics.
  --no-bag               Skip rosbag recording; collect logs and snapshots only.
  --compress             Create a .tar.gz archive at shutdown.
  -h, --help             Show this help.

Examples:
  $SCRIPT_NAME attach
  $SCRIPT_NAME attach --no-images
  $SCRIPT_NAME launch-slam --launch kitti_stereo.launch
USAGE
}

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

sanitize_topic() {
  local topic="$1"
  topic="${topic#/}"
  topic="${topic//\//_}"
  printf '%s' "$topic"
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

run_capture() {
  local label="$1"
  shift
  {
    printf '$'
    printf ' %q' "$@"
    printf '\n'
    "$@"
  } >"$label" 2>&1 || true
}

source_if_present() {
  local file="$1"
  if [ -f "$file" ]; then
    local nounset_was_enabled=false
    case "$-" in
      *u*) nounset_was_enabled=true ;;
    esac

    set +u
    # shellcheck disable=SC1090
    source "$file"
    if [ "$nounset_was_enabled" = true ]; then
      set -u
    fi
  fi
}

add_collector_pid() {
  COLLECTOR_PIDS+=("$1")
}

stop_pid() {
  local pid="$1"
  local sig="${2:-INT}"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill "-$sig" "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [ -n "${DBG:-}" ] && [ -d "$DBG" ]; then
    log "Stopping debug collectors..."
    for pid in "${COLLECTOR_PIDS[@]}"; do
      stop_pid "$pid" INT
    done

    if [ -n "$SLAM_PID" ]; then
      log "Stopping roslaunch started by this script..."
      stop_pid "$SLAM_PID" INT
    fi

    sleep 3

    for pid in "${COLLECTOR_PIDS[@]}"; do
      stop_pid "$pid" TERM
    done
    if [ -n "$SLAM_PID" ]; then
      stop_pid "$SLAM_PID" TERM
    fi

    dmesg >"$DBG/dmesg.after.txt" 2>&1 || true
    if [ -e /root/.ros/log/latest ]; then
      cp -a /root/.ros/log/latest "$DBG/ros_latest_log" 2>/dev/null || true
    fi

    {
      echo "finished_at=$(date -Is)"
      echo "exit_code=$exit_code"
      echo "run_dir=$DBG"
    } >>"$DBG/meta.txt"

    if [ "$COMPRESS" = true ]; then
      local archive="${DBG}.tar.gz"
      tar -C "$(dirname "$DBG")" -czf "$archive" "$(basename "$DBG")" 2>"$DBG/archive.stderr.log" || true
      [ -f "$archive" ] && log "Archive created: $archive"
    fi

    log "Debug run saved: $DBG"
    log "Pull to laptop with:"
    log "  rsync -avz root@192.168.1.10:$DBG/ ./rdk_debug_runs/$(basename "$DBG")/"
  fi

  exit "$exit_code"
}

parse_args() {
  if [ "${1:-}" = "attach" ] || [ "${1:-}" = "launch-slam" ]; then
    MODE="$1"
    shift
  fi

  while [ "$#" -gt 0 ]; do
    case "$1" in
      --workspace)
        WORKSPACE="${2:?missing value for --workspace}"
        shift 2
        ;;
      --out-root)
        OUT_ROOT="${2:?missing value for --out-root}"
        shift 2
        ;;
      --run-id)
        RUN_ID="${2:?missing value for --run-id}"
        shift 2
        ;;
      --package)
        PACKAGE="${2:?missing value for --package}"
        shift 2
        ;;
      --launch)
        LAUNCH_FILE="${2:?missing value for --launch}"
        shift 2
        ;;
      --bag-split)
        BAG_SPLIT_DURATION="${2:?missing value for --bag-split}"
        shift 2
        ;;
      --snapshot-interval)
        SNAPSHOT_INTERVAL="${2:?missing value for --snapshot-interval}"
        shift 2
        ;;
      --no-images)
        RECORD_IMAGES=false
        shift
        ;;
      --no-grid)
        RECORD_GRID=false
        shift
        ;;
      --no-future-debug)
        RECORD_FUTURE_DEBUG=false
        shift
        ;;
      --no-bag)
        NO_BAG=true
        shift
        ;;
      --compress)
        COMPRESS=true
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "Unknown argument: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done
}

setup_environment() {
  source_if_present /opt/ros/noetic/setup.bash
  source_if_present /ros1_project/deps_ws/devel/setup.bash
  source_if_present /ros1_project/camera_ws/devel/setup.bash
  source_if_present "$WORKSPACE/devel/setup.bash"

  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.1.10:11311}"
  export ROS_IP="${ROS_IP:-192.168.1.10}"
}

make_run_dir() {
  OUT_ROOT="${OUT_ROOT:-$WORKSPACE/debug_runs}"
  RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  DBG="$OUT_ROOT/$RUN_ID"

  mkdir -p "$DBG"/{bags,hz,samples,node_info}
}

write_meta() {
  {
    echo "run_id=$RUN_ID"
    echo "started_at=$(date -Is)"
    echo "mode=$MODE"
    echo "workspace=$WORKSPACE"
    echo "package=$PACKAGE"
    echo "launch_file=$LAUNCH_FILE"
    echo "record_images=$RECORD_IMAGES"
    echo "record_grid=$RECORD_GRID"
    echo "record_future_debug=$RECORD_FUTURE_DEBUG"
    echo "no_bag=$NO_BAG"
    echo "bag_split_duration=$BAG_SPLIT_DURATION"
    echo "snapshot_interval=$SNAPSHOT_INTERVAL"
    echo "compress=$COMPRESS"
    echo "ros_master_uri=${ROS_MASTER_URI:-}"
    echo "ros_ip=${ROS_IP:-}"
    echo "hostname=$(hostname 2>/dev/null || true)"
    echo "kernel=$(uname -a 2>/dev/null || true)"
  } >"$DBG/meta.txt"

  env | sort >"$DBG/env.txt" 2>&1 || true
  dmesg >"$DBG/dmesg.before.txt" 2>&1 || true
  run_capture "$DBG/df_h.txt" df -h
  run_capture "$DBG/free_h.txt" free -h
  run_capture "$DBG/uptime.txt" uptime
}

wait_for_master() {
  local max_wait="$1"
  local waited=0
  while [ "$waited" -lt "$max_wait" ]; do
    if rostopic list >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

start_slam_if_requested() {
  if [ "$MODE" != "launch-slam" ]; then
    echo "attach mode: no roslaunch started by collect_slam_debug.sh" >"$DBG/slam.launch.log"
    return
  fi

  log "Starting roslaunch $PACKAGE $LAUNCH_FILE"
  (
    cd "$WORKSPACE"
    roslaunch "$PACKAGE" "$LAUNCH_FILE"
  ) > >(tee "$DBG/slam.launch.log") 2>&1 &
  SLAM_PID=$!
}

snapshot_ros_state() {
  if wait_for_master 10; then
    run_capture "$DBG/topics.txt" rostopic list -v
    run_capture "$DBG/nodes.txt" rosnode list
    rosparam dump "$DBG/params.yaml" >/dev/null 2>&1 || true
  else
    echo "ROS master was not reachable within 10 seconds." >"$DBG/topics.txt"
    echo "ROS master was not reachable within 10 seconds." >"$DBG/nodes.txt"
  fi

  for node in /s316_camera_node /orb_slam3 /occupancy_grid_publisher; do
    if rosnode list 2>/dev/null | grep -qx "$node"; then
      run_capture "$DBG/node_info/$(sanitize_topic "$node").txt" rosnode info "$node"
    fi
  done
}

record_topics() {
  local topics=(
    /rosout_agg
    /robot_pose
    /robot_pose_slam
    /robot_pose_tracking_ok
    /odometry
    /tf
    /tf_static
  )

  if [ "$RECORD_IMAGES" = true ]; then
    topics+=(
      /camera/infra1/image_raw
      /camera/infra2/image_raw
      /camera/depth/cloud_colored
    )
  fi

  if [ "$RECORD_GRID" = true ]; then
    topics+=(/ppseg_gridmap)
  fi

  if [ "$RECORD_FUTURE_DEBUG" = true ]; then
    topics+=(
      /orb_slam3/debug_frame
      /orb_slam3/debug_keypoints_image
      /orb_slam3/debug_matches_image
    )
  fi

  printf '%s\n' "${topics[@]}" >"$DBG/bag_topics.txt"

  if [ "$NO_BAG" = true ]; then
    log "Skipping rosbag recording because --no-bag was supplied."
    return
  fi

  if ! have_cmd rosbag; then
    echo "rosbag command not found" >"$DBG/bags/rosbag.not_found.txt"
    return
  fi

  log "Starting rosbag record"
  rosbag record --lz4 --split --duration="$BAG_SPLIT_DURATION" \
    -O "$DBG/bags/slam_debug" "${topics[@]}" \
    >"$DBG/rosbag.record.log" 2>&1 &
  add_collector_pid "$!"
}

start_hz_collectors() {
  local topics=(
    /camera/infra1/image_raw
    /camera/infra2/image_raw
    /robot_pose
    /robot_pose_slam
    /robot_pose_tracking_ok
    /odometry
    /ppseg_gridmap
  )

  if ! have_cmd rostopic; then
    return
  fi

  for topic in "${topics[@]}"; do
    rostopic hz "$topic" >"$DBG/hz/$(sanitize_topic "$topic").hz.log" 2>&1 &
    add_collector_pid "$!"
  done
}

start_sample_collectors() {
  if ! have_cmd rostopic; then
    return
  fi

  rostopic echo -n 50 /robot_pose_tracking_ok >"$DBG/samples/robot_pose_tracking_ok.txt" 2>&1 &
  add_collector_pid "$!"

  rostopic echo -n 20 /robot_pose >"$DBG/samples/robot_pose.txt" 2>&1 &
  add_collector_pid "$!"

  rostopic echo -n 20 /robot_pose_slam >"$DBG/samples/robot_pose_slam.txt" 2>&1 &
  add_collector_pid "$!"

  rostopic echo -n 20 /odometry >"$DBG/samples/odometry.txt" 2>&1 &
  add_collector_pid "$!"
}

start_resource_collectors() {
  if have_cmd pidstat; then
    pidstat -u -r -d 2 >"$DBG/resource_pidstat.log" 2>&1 &
    add_collector_pid "$!"
  else
    echo "pidstat command not found" >"$DBG/resource_pidstat.log"
  fi

  if have_cmd top; then
    top -b -d 2 >"$DBG/top.log" 2>&1 &
    add_collector_pid "$!"
  fi
}

main() {
  parse_args "$@"
  setup_environment
  make_run_dir
  trap cleanup EXIT INT TERM

  log "Creating debug run at $DBG"
  write_meta
  start_slam_if_requested

  if ! wait_for_master 20; then
    log "ROS master is not reachable; continuing with non-ROS snapshots."
  fi

  snapshot_ros_state
  record_topics
  start_hz_collectors
  start_sample_collectors
  start_resource_collectors

  log "Collectors are running. Press Ctrl+C to finish and save the run."
  while true; do
    sleep "$SNAPSHOT_INTERVAL"
    snapshot_ros_state
  done
}

main "$@"
