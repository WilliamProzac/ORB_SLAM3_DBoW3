# Validation

Source basis: `README.md`, `build.sh`, `build_ros.sh`,
`run_stereo_inertial.sh`, and repository tree scan on `master`.

## First-Party Validation Entry Points Found

- `./build.sh`
- `./build_ros.sh`
- `./run_stereo_inertial.sh <mode> <sensor> [--log]`
- `./collect_slam_debug.sh [attach|launch-slam] [options]` for collecting
  board-side ROS runtime evidence from the RDK X5 catkin deployment.

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
