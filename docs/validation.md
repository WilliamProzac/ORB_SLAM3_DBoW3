# Validation

Source basis: `README.md`, `build.sh`, `build_ros.sh`,
`run_stereo_inertial.sh`, and repository tree scan on `master`.

## First-Party Validation Entry Points Found

- `./build.sh`
- `./build_ros.sh`
- `./run_stereo_inertial.sh <mode> <sensor> [--log]`
- `./collect_slam_debug.sh [attach|launch-slam] [options]` for collecting
  board-side ROS runtime evidence from the RDK X5 catkin deployment.
- `./record_rgbd_loop.sh` for standalone S316 RGB-D capture from the laptop;
  see `docs/rgbd-loop-recording.md`.
- `./record_d435i_10fps_bag.sh` for ROS1 D435i 30 FPS capture and synchronized
  10 FPS RGB-D recording; see `docs/d435i-10fps-recording.md`.

## REC01 Standalone RGB-D Recording

- Syntax: `bash -n record_rgbd_loop.sh` and
  `/usr/bin/python3 -m py_compile evaluation/rgbd_recorder.py evaluation/test_rgbd_recorder.py`.
- Offline checks: source `/opt/ros/noetic/setup.bash`, then run
  `/usr/bin/python3 evaluation/test_rgbd_recorder.py`. Uses real LZ4 bags to
  cover valid data, missing depth, zero depth, non-monotonic stamps, malformed
  payloads, mismatched RGB-D stamps, truncation and unfinished `.active` files.
- Board: `./record_rgbd_loop.sh --duration 6 --output artifacts/rgbd_recorder_validation`;
  repeat without `--duration` and press Ctrl+C after recording begins. Require
  successful preflight, finalized bag, matching SHA-256, a full message read,
  valid RGB-D pairs and no remaining session camera/ROS processes.
- Retry: use `--fetch <run_id>` with the same `--output`, and require another
  successful verification without starting camera processes. After the
  user-requested cleanup extension, successful verification deletes only the
  corresponding board `rgbd.bag`; verify the local copy and board logs remain.
- Cleanup regression: the same offline test script now has 6 test groups,
  including successful deletion/retry, failure preservation for mismatched
  hashes or invalid data/recording status, and refusal to delete changed
  remote files or sessions with `.active` bags.
- Passed on 2026-09-18: runs `rgbd_loop_20260918_145800_5dc9d3` and
  `rgbd_loop_20260918_150021_125c59` under
  `artifacts/rgbd_recorder_validation/`. The latter manually stopped via Ctrl+C:
  84 exact-stamp RGB-D pairs at 11.11 Hz, 1688 total messages, 172202854 bytes.
  `board_result.json` and `bag_check.json` record completion and validation;
  `fetch_test.log` records recovery and `final_board_processes.txt` is empty.
- Cleanup extension passed real `--fetch` on the first run: local 134217703-byte
  bag and 1311-message full read remained valid, the selected board bag was
  removed, and its logs plus the other session's bag remained. Evidence:
  `fetch_cleanup_test.log` and `remote_cleanup_check.txt` in the same directory.
- Not exercised: actual network failure, disk exhaustion, power loss, or a
  physical closed-loop walk. Basic data checks do not establish SLAM loop
  detection or geometric calibration accuracy. No C++ build is needed.

## REC02 D435i 10 FPS RGB-D Recording

- Syntax/regression: run `bash -n record_d435i_10fps_bag.sh`, compile
  `evaluation/rgbd_recorder.py`, and run
  `/usr/bin/python3 evaluation/test_rgbd_recorder.py` after sourcing Noetic.
- Hardware: run `./record_d435i_10fps_bag.sh --duration 8 --output
  artifacts/d435i_10fps_recorder_validation`. Require a stable post-warmup
  preflight above 9.5 Hz, exact RGB/depth pairing, finalized LZ4 bag, matching
  SHA-256, complete sequential read, verified board-bag cleanup, retained logs,
  and no remaining camera/ROS/recorder process.
- Passed 2026-09-24 with `d435i10_20260924_111100_130a26`: preflight RGB and
  depth were both 9.99 Hz with 100% pairing. The 7.906-second, 48,284,291-byte
  bag contains 79 RGB, 79 aligned-depth, 79 color-info, 79 depth-info and one
  TF-static message. Full read covered all 317 indexed messages; both image
  streams were 9.9929 Hz, 79/79 paired within 0.1 ms, with 87.94% sampled valid
  depth. SHA-256 is `e34f0df7f7583f7da17c51f254a59cbeb6cf14867c57e4205a1a79f41343f1c4`.
- The first hardware attempt correctly stopped before recording because the
  initial five-second header-rate window was 9.477 Hz. The recorder now waits
  for both topics, discards a configurable two-second D435i startup warmup,
  then measures a fresh five-second window. Entire recorded bags remain fully
  checked without discarding startup frames.
- Evidence is under `artifacts/d435i_10fps_recorder_validation/`. Physical
  closed-loop coverage, network failure, disk exhaustion and power loss were
  not exercised.

## RGBD01 D435i + MEM18/perf20 Integration

- Syntax: `bash -n run_d435i_slam.sh`; compile `evaluation/d435i_slam.py`.
- Offline: `/usr/bin/python3 evaluation/test_d435i_slam.py -v` checks calibration,
  depth units, rejected models, sequence gaps and histogram summaries.
- Hardware: `./run_d435i_slam.sh 120 off`, then `./run_d435i_slam.sh 60 on`.
  Require live CameraInfo, matched binary SHA256/file/ldd/loaded core, complete
  10 FPS input, initialized and valid tracking, no resets, latency distribution,
  memory/paging evidence, graceful owned-process cleanup and transfer hashes.
- Static-only scope. Do not mark tracking passed when the scene fails the
  unchanged initialization requirements. Timing-on is diagnostic only.
- Instructions: `docs/d435i-slam.md`; evidence: `artifacts/d435i_slam_20260924/`.

## RGBD02 host live launcher

- Run `bash -n start_rdk_rgbd_slam.sh` and
  `/usr/bin/python3 evaluation/test_rdk_rgbd_live.py -v`.
- Launch `./start_rdk_rgbd_slam.sh`, verify actual SLAM stdout/stderr appears
  before shutdown, then press Ctrl+C. Require owned camera/SLAM/master cleanup,
  host log preservation and board-log fetch. Optional `--duration 60` is a
  convenience timeout; this launcher is not a tracking acceptance test.
- EOF, explicit stop and heartbeat timeout have offline control tests. Actual
  LAN packet loss, power failure and forced process kills are not asserted tested.

## Build Validation

- `F01`: run `./build.sh` from repo root.
- Success means the script completes its third-party builds, extracts
  `Vocabulary/ORBvoc.txt.tar.gz`, and finishes the top-level CMake build.
- `F02`: export `ROS_PACKAGE_PATH` as documented in `README.md`, then run
  `./build_ros.sh`.
- Success means the rosbuild package under `Examples_old/ROS/ORB_SLAM3/`
  finishes building.

## Runtime Validation With `run_stereo_inertial.sh`

- Valid modes on this branch are `mapping`, `mapping_save`, and
  `localization`.
- Valid sensors on this branch are `stereo_inertial`, `rgbd`, and `stereo`.
- The wrapper writes terminal logs to `logs/` only when `--log` is used.
- The wrapper always starts `pidstat` monitoring and `rosbag record`.
- No checked-in `--no-bag` switch exists on this branch.
- Before treating a runtime attempt as meaningful, confirm the required ROS
  inputs are already live for the selected sensor, because the wrapper records
  bags but does not replay them.
- If ROS, camera topics, IMU topics, `pidstat`, or bag-recording prerequisites
  are missing, record the validation as `blocked` or `skipped` instead of
  inventing an unsupported workaround.

## RDK X5 Runtime Evidence Collection

- `collect_slam_debug.sh` is an auxiliary board-side collection script. Copy it
  to the RDK X5 deployment, usually `/userdata/orb_slam/`, and run it from the
  board.
- Use `./collect_slam_debug.sh attach` when `s316_camera camera.launch` and
  `orb_slam3_ros kitti_stereo.launch` are already running.
- Use `./collect_slam_debug.sh launch-slam` to let the script start
  `roslaunch orb_slam3_ros kitti_stereo.launch` and save its stdout/stderr to
  the run directory.
- The script records per-run evidence under
  `/userdata/orb_slam/debug_runs/<run_id>/`, including metadata, environment,
  ROS topic/node/param snapshots, `dmesg` before/after, ROS latest logs, topic
  frequency logs, selected message samples, resource logs, and optional rosbag
  files.
- Use `--no-images` for lower disk and network cost when image frames are not
  required. Use `--no-bag` for log-only diagnosis.
- A passing collection run proves only that evidence was captured. It does not
  prove SLAM runtime correctness unless the captured ROS topics and logs are
  inspected against the feature-specific criteria below.

## Feature-Specific Runtime Notes

- `F03`: run `./run_stereo_inertial.sh mapping stereo_inertial --log`.
- During `F03`, confirm the script prints the selected mapping YAML, creates a
  log file under `logs/`, creates a resource log under `logs/`, starts bag
  recording under `bags/`, and keeps the ROS node alive until the session is
  intentionally stopped or a real runtime error occurs.
- `F05`: run `./run_stereo_inertial.sh localization <sensor> [--log]`.
- During `F05`, confirm the script prints the expected `*_load.yaml` path for
  the chosen sensor and launches the matching ROS executable without extra
  undocumented flags.
- `F11`: first run `./build_ros.sh`, then run either
  `./run_stereo_inertial.sh mapping stereo --log` or
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F11`, confirm `/odometry` is advertised as `nav_msgs/Odometry`,
  confirm `header.frame_id` is `odom`, confirm `child_frame_id` is
  `left_camera_link`, confirm the first published pose after successful
  tracking is near a fresh local origin instead of inheriting the loaded map
  origin, and confirm `twist` is derived from consecutive fresh tracked poses
  instead of staying permanently zero once the robot is moving.
- For `F11`, low-confidence twist is acceptable, but the implementation should
  make that uncertainty explicit through covariance or another documented
  contract instead of pretending the differential estimate is high confidence.
- For `F11`, also confirm held-pose samples during tracking loss use unknown
  twist covariance instead of reporting a confident zero velocity.
- For `F11`, also confirm the node broadcasts a TF chain that makes
  `continuous_map`, `odom`, `left_camera_link`, and `left_camera` visible to
  RViz. The 2026-06-05 Scheme B follow-up changed the root from raw `map` to
  external `continuous_map`.
- The 2026-05-29 implementation session reran `./build_ros.sh` successfully
  after the local-origin odometry change, but host-side runtime validation was
  blocked because `rosnode list` and `rostopic list` could not communicate
  with a ROS master before `run_stereo_inertial.sh` could be exercised.
- The 2026-06-05 Scheme B implementation reran `./build.sh` and
  `./build_ros.sh` successfully after changing the root frame to
  `continuous_map`, but runtime validation still requires a live or replayed
  stereo session that exercises tracking loss and new-map recovery.
- `F12`: first run `./build_ros.sh`, then run
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F12`, confirm `/relocalization_status` is advertised at
  approximately `10 Hz`, confirm the built message carries only
  `timestamp_ns` and `status`, and confirm the runtime output can emit exactly
  `RelocalizationRunning`, `RelocalizationSucceed`, and
  `RelocalizationFailed`.
- For `F12`, `grid_map` already provides a migration template, but validation
  must reject a straight port if it still emits `RelocalizationSucceeded`
  instead of the requested `RelocalizationSucceed`.
- The 2026-05-25 validation session measured `/relocalization_status` at about
  `10.0 Hz` and directly sampled `RelocalizationSucceed`; because the
  localization session converged quickly, the short topic captures did not
  separately observe `RelocalizationRunning` or `RelocalizationFailed` before
  the steady-state publish loop resumed.
- `F13`: first run `./build_ros.sh`, then run either
  `./run_stereo_inertial.sh mapping stereo --log` or
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F13`, confirm `/robot_pose` is advertised as
  `geometry_msgs/PoseStamped`, confirm `header.frame_id` is
  `continuous_map`, confirm `/robot_pose_slam` is advertised as
  `geometry_msgs/PoseStamped`, confirm `/robot_pose_slam.header.frame_id` is
  `map`, confirm `/robot_pose_tracking_ok` is advertised as `std_msgs/Bool`,
  and confirm the published pose follows the live tracked position during
  successful tracking.
- During `F13`, force or capture a tracking-loss interval after the first
  valid pose and confirm `/robot_pose` continues publishing at image callback
  cadence with the last valid continuous pose while `/robot_pose_tracking_ok`
  is `false`.
- During `F13`, confirm `Fail to track local map!` and Atlas new-map recovery
  do not produce a visible reset to the raw internal map origin or optimized
  map frame in `/robot_pose`. Ordinary `map_change` increments should not spam
  correction warnings, and later loop-closure or merge events should not start
  a `/robot_pose` convergence window.
- During `F13`, if the active map id changes during recovery, confirm the log
  uses the map-switch continuity warning rather than restarting any
  convergence window on every routine update.
- During `F13`, inspect `/robot_pose_slam` separately around map merge, loop
  closure, and return-to-start. This topic is allowed to jump with ORB-SLAM3
  optimization; use it to evaluate the optimized SLAM map pose while keeping
  `/robot_pose` reserved for downstream continuous control feedback.
- `F14`: first run `./build_ros.sh`, then run either
  `./run_stereo_inertial.sh mapping stereo --log` or
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F14`, confirm `/robot_pose_map` is advertised as
  `geometry_msgs/PoseStamped`, confirm `header.frame_id` is `planning_map`,
  confirm `/pose_correction/set_enabled` is advertised as `std_srvs/SetBool`,
  and
  confirm both `MyD435i_stereo.yaml` and `MyD435i_stereo_load.yaml` carry the
  documented `PoseCorrection.EnableCorrectedMapPose`,
  `PoseCorrection.CorrectionHorizonFrames`, and
  `PoseCorrection.AutoEnableOnMapEvent` keys.
- During `F14`, ordinary `map_change` increments should not repeatedly restart
  correction. After a tracking-loss interval starts, confirm `/robot_pose_map`
  keeps publishing but does not converge toward raw SLAM again until the
  subsequent merge-equivalent big map event. Then confirm the runtime log emits
  `Starting gradual /robot_pose_map planning_map correction after ORB-SLAM big map event`.
- During `F14`, confirm `/robot_pose_map` transitions over the configured
  `PoseCorrection.CorrectionHorizonFrames` instead of stepping immediately to
  the new raw SLAM pose, confirm `/robot_pose_slam` remains the unblended
  raw/optimized `map` pose for comparison, and confirm `planning_map -> map`
  stays visible in TF for downstream planning consumers.
- During `F14`, test the runtime toggle with
  `rosservice call /pose_correction/set_enabled "data: true"` and
  `rosservice call /pose_correction/set_enabled "data: false"`, then confirm
  the response text and the observed output behavior match the requested state.
- For `F14`, also validate the one-shot path explicitly: with
  `PoseCorrection.EnableCorrectedMapPose` false and
  `PoseCorrection.AutoEnableOnMapEvent` true, a loop-closure or merge event
  should still start and complete one correction window.
- `F04`: validate by reading the documented entrypoints and confirming this
  branch's governance docs do not claim unsupported wrapper options or
  branch-only runtime topics.
- `F10`: validate by confirming `docs/grid-map-port-plan.md` is clearly marked
  as historical reference material rather than current `master` behavior.

## Validation Done Criteria

- Record the exact command that was run.
- Record whether the validation covered the native build, ROS build, or ROS
  runtime path.
- If `run_stereo_inertial.sh` was used, record the selected mode and sensor.
- Record whether live ROS inputs were present.
- If validation was not run, say so explicitly and explain why.

## Known Gaps

- No top-level unit-test entrypoint was found.
- No repository CI workflow file was found.
- No checked-in automated replay wrapper was found for the modified ROS flow.
- `run_stereo_inertial.sh` couples runtime launch with `pidstat` and bag
  recording, so runtime validation currently depends on those external tools
  being available.

## XFeat / RDK X5 Validation

- `F15` official local path:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps python3 xfeat_rdk_x5/smoke_test.py --output xfeat_rdk_x5/artifacts/local_smoke.json`.
- `F15` fixed-shape export:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps python3 xfeat_rdk_x5/export_onnx.py`.
- `F15` split-pipeline parity:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 xfeat_rdk_x5/onnx_smoke_test.py --threads 1`.
- Success requires finite 64D descriptors, non-empty mutual matches, valid
  opset-11 ONNX, raw-output error below `1e-4`, equal selected keypoint sets,
  coordinate-aligned descriptor error below `1e-4`, and equal match-coordinate
  pairs.
- The local checks do not prove BPU execution. BPU completion additionally
  requires `hb_mapper checker`, PTQ/makertbin accuracy evidence, `.bin`
  verification, and a real RDK X5 runtime measurement.

### F16 bag evaluation

- Run from the repository root:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 python3 xfeat_rdk_x5/evaluate_rosbag.py 2026-06-05-09-59-04.bag`.
- The validation reads the bag directly and does not require a ROS master.
- Success means both methods process identical sampled stereo and temporal
  pairs, all aggregate metrics are finite, `summary.json` and
  `per_pair_metrics.csv` are produced, and representative match images exist.
- Verified on 2026-07-30: 130 stereo pairs, 65 temporal pairs, and a 100%
  robust-pair rate for both XFeat and the OpenCV ORB comparison.
- This validates feature/matching behavior only. It does not validate an
  XFeat-integrated ORB-SLAM3 trajectory, BPU execution, or ATE/RPE.

### F18 RDK X5 INT8 deployment

- Generate 100 calibration tensors from the bag with
  `xfeat_rdk_x5/prepare_calibration.py`, passing
  `--rosbag-topic /camera/infra1/image_raw` and keeping `--manifest` outside
  the tensor directory.
- In `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.6`, run
  `hb_mapper checker --model-type onnx --march bayes-e` on the exported core,
  then run `hb_mapper makertbin --model-type onnx --config
  config/xfeat_backbone_480x640_bayes_e.yaml` from `/data`.
- Success requires all reported core nodes in BPU subgraphs, a generated
  Bayes-e `.bin`, final-output cosine similarity of at least 0.99, successful
  loading by board DNN 1.23.10 / HBRT 3.15.54, finite board outputs, and a
  recorded warmed inference latency.
- Build the board runner from `xfeat_rdk_x5/rdk_x5/` against the installed DNN
  headers/libraries, then run it with `LD_LIBRARY_PATH=/usr/hobot/lib`.
- Run `xfeat_rdk_x5/compare_ptq_outputs.py` on float, quantized, and returned
  board outputs. The board-vs-quantized values must match and every
  board-vs-float output cosine must be at least 0.99.
- Verified 2026-07-31: all 33 reported nodes were on one BPU subgraph; the
  920,712-byte model loaded on `root@192.168.1.10`; 50 warmed BPU0 runs averaged
  13.473 ms; all three board outputs exactly matched the quantized simulation;
  the minimum board-vs-float cosine was 0.990869. Evidence is in
  `xfeat_rdk_x5/artifacts/ptq_board_validation_480x640.json` and
  `docs/xfeat-rdk-x5-research.md`.

### F19 complete frontend and ROS LAN replay

- Export and calibrate the native bag shape with
  `config/xfeat_backbone_544x640_bayes_e.yaml`; checker success requires one
  BPU subgraph with no CPU fallback and PTQ output cosine above 0.99.
- Deploy and compile with `./xfeat_rdk_x5/deploy_board_benchmark.sh`.
- Start `./xfeat_rdk_x5/host_ros_lan.sh master`, run the deployed board
  `run_ros_benchmark.sh`, and verify `rosnode info /xfeat_orb_benchmark`
  reports `192.168.1.10` plus both infrared subscriptions before replay.
- Replay with `./xfeat_rdk_x5/host_ros_lan.sh play
  2026-06-05-09-59-04.bag --delay=1.0`, copy the CSV back, and run
  `xfeat_rdk_x5/summarize_board_benchmark.py`.
- Success requires the expected original indices across 130 stereo pairs and
  65 temporal pairs per method, finite latency/matching/geometry aggregates,
  and no missing sampled anchors. This is a feature frontend test; do not claim
  trajectory accuracy or ORB-SLAM3 integration from it.
- Verified 2026-07-31: 390 result rows covered the expected pairs. XFeat median
  extraction was 60.00 ms/image versus ORB 66.34 ms, while means were 64.85
  and 68.67 ms. Mean stereo end-to-end latency was 185.63 versus 161.98 ms.
  Both methods had 100% robust-pair rate; XFeat had wider inlier coverage but
  fewer strict stereo and temporal inliers. Evidence:
  `xfeat_rdk_x5/artifacts/board_ros_benchmark_544x640/summary.json` and
  `docs/xfeat-rdk-x5-board-benchmark.md`.

### F20 single-BPU stereo/CPU parallelization

- Run `xfeat_dnn_runner` with CLI core `1`; success criteria for BPU1 require a
  completed inference, not merely the presence of `HB_BPU_CORE_1` in headers.
  Also inspect `/dev/bpu_core*` and `/sys/devices/system/bpu/`.
- This board correctly fails the BPU1 probe with `-6000001` and exposes only
  BPU0. The implementation must use separate left/right tensor buffers and
  serialize the BPU0 critical section; concurrent calls on shared buffers are
  invalid.
- Build on board with `cmake --build
  /userdata/xfeat_rdk_x5/benchmark_build -j1`. Replay two anchors using
  `_cpu_threads:=1`, `4`, and `8`; select the measured fastest configuration.
- For the final run, explicitly pass `_max_anchors:=0` because private ROS
  parameters survive node restarts on a persistent master. Replay with the
  one-second connection delay and require 390 result rows over frames 0–641.
- Compare all parallel CSV quality fields with F19. Success requires zero
  mismatches for timestamps, keypoint/match counts, RANSAC/strict inliers,
  ratios, coverage and residual metrics.
- Verified 2026-07-31: CPU=8 was fastest; XFeat stereo extraction wall time
  was 57.78 ms versus parallel ORB 74.48 ms. XFeat stereo end-to-end was
  125.46 ms versus ORB 99.38 ms, a 32.4% improvement over serial XFeat. All
  390 quality rows matched F19 exactly. Evidence:
  `docs/xfeat-rdk-x5-parallel-benchmark.md` and
  `xfeat_rdk_x5/artifacts/board_ros_benchmark_parallel_544x640/`.

### F21 XFeat* semi-dense and fixed-size Fine Matcher

- Export the official `128 -> 512 -> 512 -> 512 -> 512 -> 64` Fine Matcher
  with `xfeat_rdk_x5/export_fine_matcher.py` for `M=256/384/512`. Validate
  fixed input `1xMx128`, output `1xMx64`, and float parity before conversion.
- Convert each model with OpenExplore v1.2.6 / `hb_mapper 1.23.8`. Checker
  success requires a single BPU subgraph and no CPU fallback. Compare float,
  quantized simulation, and board outputs with
  `xfeat_rdk_x5/compare_fine_ptq_outputs.py`; then run 1000 board iterations
  to check determinism, memory stability, and task release.
- The correctness evaluator must use upstream `detectAndComputeDense`, retain
  the per-point scale, concatenate descriptors in the upstream target/fixed
  order, and apply `offset * scale` to the target point. Its single- and
  dual-scale refinement outputs must agree with upstream within `1e-4 px`.
- Do not use rectified RANSAC/vertical error alone to judge XFeat* stereo
  quality. Single-scale dense points share their source-grid rows, so wrong
  disparity identities can still appear as perfect epipolar inliers. Record a
  normalized 9x9 patch NCC side metric and require the geometry test as well.
- Current offline command family:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages python3 xfeat_rdk_x5/evaluate_fine_matcher.py 2026-06-05-09-59-04.bag --feature-mode semi_dense_single --fine-size <256|384|512> --top-k 600 --anchor-parity all --minimum-ncc 0.5 --output <json>`.
- Board M384 full replay uses `_top_k:=600`, `_anchor_stride:=10`,
  `_temporal_gap:=1`, `_max_anchors:=0`, `_cpu_threads:=8`,
  `_xfeat_semidense_single:=true`, the M384 Fine `.bin`, confidence `0.25`,
  and patch NCC threshold `0.5`. Success of this component run requires 520
  rows: 130 each for XFeat* coarse/Fine and ORB stereo, plus 65 each for XFeat
  and ORB temporal controls, spanning source frames 0 through 641.
- Verified 2026-08-03: the full M384 board replay produced all 520 rows. Fine
  raised mean NCC-qualified stereo inliers from `248.29` to `289.42`
  (`+16.6%`) and their ratio from `0.650` to `0.767`; mean patch-NCC median
  rose from `0.654` to `0.792`. Fine stereo latency was `54.89 ms` mean and
  `59.37 ms` P90; its minimum strict and photometric inliers were `330` and
  `176`. The same harness's OpenCV ORB control produced `286.28` photometric
  inliers, `0.383` photometric grid coverage, and `88.70/101.43 ms` mean/P90.
  Evidence: `xfeat_rdk_x5/artifacts/f21_semidense_m384_board_full.csv` and
  `xfeat_rdk_x5/artifacts/f21_semidense_m384_board_full_summary.json`.
- Cross-build the real S316 baseline from
  `/home/ywl/Project/RDK_X5_orb_slam` with
  `RDK_BUILD_JOBS=1 ./tools/build_rdk_arm64_docker.sh`. The local derived image
  `rdk-orb-slam3-arm64-build:v3` is based on the supplied complete
  `ros1_arm64_ubuntu22.04:v3` archive and adds the missing `libepoxy-dev`.
  Stage `ros_stereo` and `liborb_slam3_ros.so` on the board, require zero
  missing `ldd` entries, and verify the uploaded SHA-256 values before replay.
- Verified one production run on 2026-08-03. The instrumented node processed
  all 644 bag callbacks with S316 `600` features and `5` pyramid levels. After
  discarding 30 warmups and one invalid default-Frame statistics row at an
  Atlas map reset, ORB extraction/stereo/frontend/TrackStereo means were
  `57.73/13.24/70.96/88.15 ms`; frontend P90 was `81.76 ms`, mean valid depths
  `215.51`, and mean depth coverage `0.755`. Tracking had one 30-frame,
  `2.97 s` loss segment.
- The run-1 comparison uses `end_to_end_pair_ms` for XFeat Fine, including its
  BPU Fine stage. XFeat mean/P90 was `55.05/59.37 ms`, satisfying the speed
  gate against this run, and mean strict count was `375.4`; strict grid
  coverage was only `0.580` versus ORB `0.755`, so the preliminary frontend
  gate fails and promotion remains false. Evidence:
  `xfeat_rdk_x5/artifacts/f21_orb_production_run1.csv`,
  `f21_preliminary_production_gate_run1.json`, and
  `f21_orb_crossbuild_run1_manifest.json`.
- Superseded by the formal result below. The preliminary comparison mixed all
  ORB depths with strict-filtered XFeat pairs and therefore did not own the
  final coverage decision. EuRoC trajectory gates belong to F23/F24 after
  integration, not to this pre-integration F21 gate.
- Same-frame visualization was verified on bag frame `320` with the real
  ORB-SLAM3 `Frame::ComputeStereoMatches` and official host XFeat* M384 path.
  ORB detected `603` points and returned `270` valid depths over `37/48` cells.
  XFeat* coarse returned `384` strict candidates over `29/48` cells; Fine
  retained `369` strict candidates and the same `29/48` coverage while changing
  median disparity from `0.0` to `3.00 px`. Evidence is under
  `xfeat_rdk_x5/artifacts/feature_visualization/`.
- A second representative-frame check used frame `440` of
  `2026-06-05-09-56-06.bag`. ORB detected `605` left points and returned `224`
  valid depths over `43/48` cells. XFeat* coarse produced `384` candidates over
  `20/48` cells and Fine retained `371` over the same cells; median disparity
  changed from `0.0` to `2.45 px`. The rendered PNG and metrics JSON are under
  `xfeat_rdk_x5/artifacts/feature_visualization_bag_2026-06-05-09-56-06/`.
- Reliability/grid-quota regression verified 2026-08-03. Run
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages python3 xfeat_rdk_x5/test_grid_quota.py -v`; all three tests must pass,
  including dense reliability shape/order, deterministic hard quota, and
  reliability-aware guided matching.
- With extraction grid `8x6`, automatic per-cell maximum `13`, match maximum
  `8`, top-k `600`, and M384, bag frame `440` produced raw/coarse/Fine coverage
  `48/48`, `47/48`, and `47/48`, with coarse/Fine strict counts `325/285`.
  This replaces neither the full board replay nor the formal F21 gate.
- Five-anchor smoke evidence in
  `xfeat_rdk_x5/artifacts/f21_grid_quota_host_smoke_09-56-06.json` requires
  Fine parity `passing`, no empty pair, mean Fine strict count `298.2`, and
  mean/min Fine coverage `0.971/0.938`. The RDK-X5 CMake build reached 100% for
  `xfeat_ros_benchmark`; a runtime smoke was not completed because no ROS
  master was listening at the test URI.
- The completed hard-grid board replay is
  `xfeat_rdk_x5/artifacts/f21_grid_quota_m384_board_full_20260803.csv`.
  It must contain 520 data rows with exactly 130 stereo rows per method and 65
  temporal rows per method, span source frames 0 through 641, and align by
  `(frame_index, stamp_ns)` with the pre-grid replay. Its CSV SHA-256 is
  `eb59d7d63ed450a5742c9904b7e5727b0d28e7f8f655def3e93aec68c18850c2`.
- Run the common stereo-depth/temporal-Fine/PnP diagnostic with:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages python3 xfeat_rdk_x5/evaluate_stereo_vo.py 2026-06-05-09-59-04.bag --output-dir xfeat_rdk_x5/artifacts/f21_stereo_vo_full`.
  It samples the same 65 `(anchor, anchor+1)` pairs for hard-grid XFeat+Fine,
  no-hard-grid XFeat+Fine, and an OpenCV ORB control; forms previous-frame 3D
  points from NCC-qualified stereo depth; refines current XFeat observations
  with the official Fine Matcher; and sends all methods through identical
  `solvePnPRansac` settings. Require 65 rows per method, finite metrics, no
  fewer-than-30-inlier pair, and 65/65 PnP success before treating a candidate
  as tracking-robust.
- The host evaluator's `extract_pair_ms` is diagnostic only: it runs FP32
  Python inference and includes different selection overheads. Use the board
  replay for latency gates. Its ORB rows are also a common-PnP OpenCV control,
  not the production ORB-SLAM3 trajectory baseline.
- Verified 2026-08-03: hard-grid XFeat+Fine failed the tracking diagnostic
  with `99.38` mean PnP inliers (`49.6%` of the ORB control), `98.5%` success,
  and one 24-inlier pair at frame 561. No-hard-grid XFeat+Fine returned `213.0`
  mean PnP inliers (`106.4%` of ORB), 65/65 success, and no fragile pair.
  This rejects the current hard quota but does not reject Fine Matcher. A
  softened minimum-per-cell plus global reliability fill must be evaluated
  before another board promotion attempt.
- The four-way extension additionally enables `xfeat_sparse_coarse`: official
  sparse XFeat with 600 features, no Fine Matcher, no quota, MNN/ratio-tested
  rectified stereo matching, and 16-pixel local temporal matching. The full
  result under `xfeat_rdk_x5/artifacts/f21_stereo_vo_fourway/` must contain 65
  aligned finite rows for each of the four methods (260 data rows total).
  Verified 2026-08-03: sparse XFeat averaged `83.0` PnP inliers (`41.4%` of
  ORB), succeeded on 63/65 pairs, and had 9/6 inliers at fragile frames
  511/561. No-hard-grid XFeat+Fine remained the only XFeat variant with 65/65
  success, no fragile pair, and mean PnP inliers above the ORB control.
- Deployed sparse-600 validation uses the board runner defaults: `_top_k:=600`,
  `_xfeat_fixed_matches:=600`, `_xfeat_semidense_single:=false`, empty
  `_fine_model_path`, and both quota parameters zero. The startup log must say
  `top_k=600 semidense_single=0 ... max_per_cell=0
  extraction_max_per_cell=0 fine=disabled`. Replay one anchor and require both
  XFeat keypoint columns to equal 600 and every Fine timing column to remain
  zero. Verified 2026-08-03 in
  `xfeat_rdk_x5/artifacts/f21_sparse600_board_smoke.csv`: both stereo rows and
  the temporal row contain `600/600` XFeat points; the temporal row has 429
  matches and 383 RANSAC inliers.
- Formal F21 verification passed 2026-08-03 with the frozen configuration in
  `xfeat_rdk_x5/config/f21_xfeat_star_m384.yaml`: extraction quota `-1`, loose
  match quota `16`, M384, and Fine confidence `0.20`. Five alternating runs
  each produced 644 ORB callbacks and 520 XFeat records. Combined mean/P90
  frontend latency was `70.505/81.827 ms` for ORB and `54.633/59.402 ms` for
  XFeat. The common real-S316-ORB/XFeat estimator passed NCC count/coverage,
  PnP count/coverage, 65/65 success, and zero-fragile-pair gates. The M384
  threshold-0.20 PTQ check passed, and two sequential 1000-iteration board
  runs returned identical output hashes and stable peak RSS. Authoritative
  evidence: `docs/f21-gate-report.md` and
  `xfeat_rdk_x5/artifacts/f21_gate_final.json`.

## F25 RDK X5 stereo gate diagnostic (2026-08-06)

- Cross-build command: `RDK_BUILD_JOBS=4 ./tools/build_rdk_arm64_docker.sh`
  from `/home/ywl/Project/RDK_X5_orb_slam`. The ARM64 node/library hashes were
  `d6df0d5e...f71ccb` and `3f4de91f...e085c54f`. They were copied to the
  independent board directory
  `/userdata/orb_slam/f25_stereo_gate25_sparse_20260806`; the existing staging
  and devel artifacts were not replaced.
- Runtime controls: live S316 camera threads on CPUs 0-1, ORB-SLAM3 on CPUs
  2-7, Pangolin viewer disabled, host bag playback at 1.0x, and image topics
  remapped under a unique `/baggate25_sparse_*` prefix so the node could not
  subscribe to the live camera image topics.
- `2026-06-05-09-59-04.bag`: 643 callbacks, all tracking state 2, one Atlas
  map with 135 keyframes and 2822 saved map points. Frame 523 reached exactly
  25 local-map inliers and was accepted; its pose step was 0.062627 m and
  1.613700 deg, with inlier reprojection mean/P90 1.264135/2.374674 px and
  chi-square mean/P90 1.390776/2.646152. The former threshold-30 run entered
  state 3 at frame 528 and created a new map after 30 lost frames.
- `2026-06-05-09-56-06.bag`: 883 callbacks, 30 state-3 frames, then a new map.
  Failure began at frame 432 when `TrackReferenceKeyFrame` found only 9 BoW
  matches. The preceding frame had 50 local-map inliers, and no 25-29-inlier
  diagnostic occurred before failure, so changing the 30-inlier local-map gate
  could not have caused or prevented this loss. The threshold-30 CPU-isolated
  run had remained state 2 throughout, exposing asynchronous LocalMapping/map
  state repeatability as a remaining runtime variable.
- Mean callback time changed from 84.002 to 84.565 ms on `09-59-04` and from
  96.592 to 97.366 ms on `09-56-06` versus the threshold-30 CPU-isolated runs.
  The sparse diagnostic overhead is therefore below 1%, but repeat runs are
  still required before promoting 25 as a production threshold.
- Host evidence and SHA-256:
  `/tmp/orb_slam3_095606_rate1_camera_cpuiso_gate25_sparse.csv`
  (`1dff179a...91650bf`),
  `/tmp/orb_slam3_095606_gate25_sparse_diag.log`
  (`a2ce0700...65d6a9e`),
  `/tmp/orb_slam3_095904_rate1_camera_cpuiso_gate25_sparse.csv`
  (`d9f51beb...ebb8d50`), and
  `/tmp/orb_slam3_095904_gate25_sparse_diag.log`
  (`cecd50cd...7b74e5d`).

## F22 RDK X5 Hybrid stereo replay (2026-08-10)

- Host core build: `./build.sh` passed after adding the provider seam and
  frontend statistics. Standalone `-std=c++17 -Wall -Wextra` compilation also
  passed for `xfeat_frontend.cpp` and `xfeat_hybrid_depth_provider.cpp`.
- RDK cross-build: `RDK_BUILD_JOBS=1
  /home/ywl/Project/RDK_X5_orb_slam/tools/build_rdk_arm64_docker.sh` passed.
  The first `-j4` attempt was rejected as evidence because `cc1plus` was killed
  while compiling `Optimizer.cc`; the retained single-job retry completed both
  catkin packages. Node/library hashes are `5a507a46...1eaf9b` and
  `56987b52...7766f9`; board `ldd` reported no missing dependency.
- Board smoke: an 8-second slice of `09-46-39` produced 89/89 state-2 and
  pose-valid callbacks. XFeat Fine added 14-17 depths per sampled frame and
  initialized both BPU models without error.
- Full replay command: `./xfeat_rdk_x5/run_f22_hybrid_ab.sh`, using unique ROS
  topic prefixes, viewer disabled, CPU affinity 2-7, and 0.5x playback. The
  slower rate is required because the validation Hybrid intentionally computes
  both complete ORB and XFeat frontends; it prevents queue drops from becoming
  a confounder. Source image-pair counts are 996, 811 and 715, while recorded
  ORB/Hybrid callbacks are `994/994`, `803/810`, and `704/713`.
- Common-timestamp verification covers 2,496 frames. ORB keypoint mismatch is
  zero, valid-depth regression count is zero, and mean Hybrid depth deltas are
  `+18.39/+17.12/+16.13`. Mean tracked-inlier deltas are
  `+18.79/+13.88/+17.82`.
- Tracking-loss frames/segments are ORB versus Hybrid `0/0` and `0/0` for
  `09-46-39`, `34/1` and `34/1` for `09-48-14`, and `34/1` and `0/0` for
  `09-50-14`. Map counts are `1/1`, `2/2`, and `2/1`. No runtime error marker
  was found in any log.
- Trajectory evidence: first-bag maximum consecutive translation/rotation is
  ORB `0.064 m/2.91 deg` and Hybrid `0.069 m/3.40 deg`. Both second-bag runs
  reset maps. On the third bag ORB reaches `2.945 m/95.25 deg` across its reset,
  while Hybrid remains at `0.100 m/3.14 deg` and stays in one map.
- Reprojection evidence uses the existing sparse near-gate diagnostic and is
  conditional on its trigger. Mean of per-row reprojection mean/P90 is
  ORB/Hybrid `1.24/2.30` versus `1.45/2.76` px, `1.32/2.45` versus
  `1.27/2.40` px, and `1.35/2.53` versus `1.24/2.34` px for the three bags.
- Timing: Hybrid itself averages `53.42/53.26/53.34 ms`; total callback mean is
  `161.85/162.41/155.92 ms`, versus ORB `106.46/107.11/100.72 ms`. F22 passes
  the specified loss/observation gate, but this dual-frontend bridge is not an
  11 Hz real-time production mode.
- Machine-readable evidence:
  `xfeat_rdk_x5/artifacts/f22_hybrid_ab/summary.json` plus the six paired CSV
  and log files. The original unindexed `2026-08-06-09-50-14.bag` was not
  modified; the tested indexed copy is `/tmp/f22_2026-08-06-09-50-14.bag`.

## F23 Full XFeat frontend validation (2026-08-10)

- Descriptor unit check:
  `g++ -std=c++17 -Wall -Wextra -Werror -Iinclude
  xfeat_rdk_x5/test_feature_descriptor.cpp src/FeatureDescriptor.cc
  $(pkg-config --cflags --libs opencv4) -o
  /tmp/f23_test_feature_descriptor && /tmp/f23_test_feature_descriptor`.
  Success requires exact ORB Hamming checks, finite float cosine distances,
  descriptor-family rejection, and the expected float medoid.
- Host build check: `cmake --build build --target ORB_SLAM3 -j1` from the
  repository root. Script check: `bash -n
  xfeat_rdk_x5/run_f23_xfeat_smoke.sh`. Also require `git diff --check`.
- RDK cross-build check: from `/home/ywl/Project/RDK_X5_orb_slam`, run
  `RDK_BUILD_JOBS=1 ./tools/build_rdk_arm64_docker.sh`. The package must use
  `-DORB_SLAM3_SOURCE_DIR=/ros1_project/orb_slam3_dbow3` so the tested core is
  this repository rather than a stale mirror.
- Board smoke/full replay helper:
  `F23_ARTIFACTS=<output> F23_DURATION=<seconds>
  ./xfeat_rdk_x5/run_f23_xfeat_smoke.sh <bag>`. It uses the isolated stage
  `/userdata/orb_slam/f23_xfeat_20260810`, 0.5x playback, CPUs 2-7, a unique
  topic prefix, Pangolin disabled, and copies the per-callback CSV back.
- The 8-second temporal-Fine smoke passed 87/87 state-2 callbacks with no
  `Fail to track local map!`. Evidence:
  `xfeat_rdk_x5/artifacts/f23_xfeat_smoke_temporal_fine/`.
- The complete `2026-08-06-09-46-39.bag` temporal-Fine run is failing
  evidence, not a pass: 981 callbacks, 682 state-2, 299 state-3, 29 loss
  segments, 2.97 s maximum loss and eight maps. Evidence:
  `xfeat_rdk_x5/artifacts/f23_lawn_full_094639_temporal_fine/`.
- The authoritative F23 gate remains five runs each of MH_01, MH_03 and MH_05,
  requiring 5/5 initialization, ATE/RPE no greater than 1.05x ORB, and no
  worse loss duration. No matching local dataset directories were found on
  2026-08-10, so that gate is blocked and F23 must not be marked passing.

## BR01 branch and archive validation (2026-09-07)

The separation check compares inherited XFeat source bytes against `419878d`,
the four runtime core modifications against their pre-migration backup, and all
54 saved source/config files against `backup_manifest.json`. The archived 7,626
file/link records were verified after relocation against SHA-256, size, mode or
symlink target. Existing branch refs are compared with the saved ref inventory.

Run the existing F23 descriptor test on its branch:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude \
  xfeat_rdk_x5/test_feature_descriptor.cpp src/FeatureDescriptor.cc \
  $(pkg-config --cflags --libs opencv4) -o /tmp/f23_branch_descriptor_check
/tmp/f23_branch_descriptor_check
```

Run `bash -n` on migrated shell scripts and use Python `ast.parse` on migrated
Python files. Check `git diff --check`, the source/artifact ownership, unique
feature IDs, expected branch ancestry, ignore rules and final clean statuses in
both core worktrees and the separate ROS/build repository. Validate build-script
default and override paths without starting Docker or ROS.

Results are recorded in the local archive's `validation.json`; branch/source
routing is documented in `docs/branch-layout.md`.

Full host/ARM builds and board replays were not rerun: this task preserves the
existing C++ source bytes and organizes branch ownership. Only runner data-root
selection and build-script default workspace/core routing were adjusted. The
F23 long-run and EuRoC limitations remain blocked; imported runtime passing
states describe the earlier recorded experiments, not new runtime verification.

Verified 2026-09-07: 79 inherited source files match `419878d`; all four pending
runtime core files and four unchanged ROS/build files match their saved bytes.
The descriptor test passed, 9 shell scripts and 10 Python modules passed syntax
checks, and build defaults/overrides, feature ownership and ignore rules passed.
The new local branch commit IDs and clean statuses are recorded in the archive
after committing. Original branch refs are retained.

## LM01 Active local map (runtime worktree)

The implementation lives in `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_memory_lifecycle/core`,
not this IDE checkout's XFeat core. Run `bash memory/tools/build_localmap_host.sh c1`
from the runtime outer repository; the fixture is `memory/devel-host-lm01-c1/lib/orb_slam3_ros/local_map_budget_probe`.
The runtime `memory/LM01.md` records contracts, stage gates and board artifact ownership.
Source/binary provenance and results are archived here under `artifacts/local_map_budget_20260924`.
C1 native/ARM probes and transaction fixtures pass; sanitizer qualifications are recorded in c1/checks.json. Production integration and later quality/realtime gates remain in progress.


## LM01 C2 — functional gate only (2026-09-24)

Runtime RGB-D full/KF/shadow transaction integration, true KF cap and legacy
counters now pass native full/KF80/shadow 120-frame tests, original 27 ARM probes,
new cap/competition/invalidation fixtures, TSan races/small-lock checks and
ASan/UBSan/LSan fixture with the recorded C1 dependency exceptions. A narrow
recursive identity barrier serializes retirement/fusion/transfer with active
transactions; ordinary metadata updates remain concurrent. This fixes actual
extra non-OK frames found during integration. Full-System LSan remains open.

Board full and KF80 short runs are 120/120 OK, but P95 112.448/115.138 ms
versus perf20 prefix 90.928 ms: performance FAIL, feature default OFF. Next is
C2.1 transaction overhead reduction, then C3 event-based controller, C4 selector
comparison, C5 MP/statistics/cohorts and C6 formal matrix. Preserve all failures.
Evidence: IDE artifacts/local_map_budget_20260924/c2/checks.json and runs/.
Runtime memory/LM01.md specifies the refined synchronization contract.

## GF01 upstream local-map hashing reproduction (2026-09-28)

- Independent source/build/inputs are under
  `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_repro_20260928`.
  Run commands and exact upstream/local hashes are in the archived
  `artifacts/gf_orb_slam2_repro_20260928/README.md` and `REPORT.md`.
- Four CPU targets built; 3 accuracy evaluator tests pass; input regeneration
  matches frozen manifests; bucket-container diagnostic verifies the current
  unbounded storage / bounded query behavior.
- Completed 15 EuRoC + 9 corrected-input TUM rounds, all source frames consumed.
  One narrow-OTS EuRoC run has 68 LOST frames. Failed measurements are retained;
  speed/robustness claims require the full configuration to pass, not just
  selected successful runs. No stable end-to-end speed benefit was found.
- `validation.json`, `comparison_summary.json`, per-matrix `analysis.json`,
  `run_summary.csv`, plots and SHA256 raw indices are archived. TUM accuracy
  only covers 24.3% of online poses because of a long ground-truth gap.
- No ARM/ROS callback gate, GF concurrent-lifecycle sanitizer gate, full-paper
  benchmark or combined Good Feature Matching/Good Graph gate was run.
  GF01 passing refers to reproducible evaluation delivery only.

## GF02: paper-described MIH policy experiment

Independent root: `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_paper_20260928`; source `33e5064`, GF01 parent `39e794a`. Exact commands are archived in `artifacts/gf_orb_slam2_paper_20260928/README.md`.

- Four CPU modes built; release plus focused ASan/UBSan/LSan and TSan fixtures passed for capacity/refresh/eviction/migration/partial membership/concurrent query/reset. These fixtures do not certify the full System/MapPoint lifecycle. Three accuracy-tool tests passed.
- Four 400-frame smoke runs passed, but did not yet activate querying. Formal EuRoC and TUM matrices each completed 12 rounds, 95,628 input frames in total. All records are contiguous/finite, timestamps increase, and frozen source/binary/config/driver/input-list hashes match.
- 23/24 rounds remained OK after initialization. TUM `1_paper_ots` lost from index 28 through 5056, with zero hash queries throughout; its fast LOST frames are retained and excluded from acceleration claims.
- EuRoC baseline / bounded buckets / bounded+all-descriptor API P95: 35.609 / 63.127 / 64.263 ms. TUM: 27.550 / 52.368 / 59.477 ms; bounded-only has a failed repeat. Production promotion fails. GF02 `passing` means the implementation/evaluation/evidence task is delivered, not performance or full-paper acceptance.
- Report, complete per-run JSON/CSV, raw-file SHA256 indices, plots, patches and provenance are in `artifacts/gf_orb_slam2_paper_20260928`. `harness/audit_freeze.py` reproduces the final integrity audit.
- Not executed: full SLAM sanitizers, ARM/ROS timing, loop closure, map merge/save/reload, long-duration NewCollege or all EuRoC sequences. TUM GT covers only 1227/5057 poses. Production ORB3/LM01 and board files are unchanged.

## GF03: module factorial evaluation

Independent root `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_modules_20260928`. Verification: build all eight replay targets; run `harness/run_matrix.py` on V1_01_easy and tum_large with three repeats; use `harness/analyze.py` and a frozen-artifact audit. Confirm actual F/H/G execution and preserve failed runs. GF03 passing will mean evaluation delivered, not automatically algorithm acceptance.
