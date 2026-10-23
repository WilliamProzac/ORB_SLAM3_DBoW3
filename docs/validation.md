## RGBD27 latest RGB-D source publication (2026-10-23)

The exact atomic_02 C++ implementation and headers are published at the repository
root with their matching wrapper in runtime/rgbd. The original 106 core files
and 116 ARM compilation inputs are hash-bound; only ROS build/package
declarations are adapted for this single-repository layout.

Read-only check: python3 -B runtime/rgbd/verify_source.py.
Build and fixture commands: [runtime/rgbd/README.md](../runtime/rgbd/README.md).
Original long-bag results, rejected candidates and acceptance limits:
[runtime/rgbd/VALIDATION.md](../runtime/rgbd/VALIDATION.md).

Do not use the old root build_ros.sh to infer validation of this matching
catkin workspace. Publication neither deploys a board binary nor reruns the
long bag. The candidate still fails the complete-input/90ms/workload gates.

# Validation

Latest memory-task state: [current session summary](../../memory/SESSION_SUMMARY_20260920_MEM18.md).
Current frozen/source candidate is the active MEM17/MEM18 repair worktree; see [current progress](../../memory/MEM17_CANDIDATE_SCORE_PROGRESS.md).
Native/ASan paired builds and106 source/14 binary hashes pass. Real candidate API
regression changes history-dependent B/A to A/A while preserving3 positive
controls.26+26 legal checks and normal native955 allOK/save/GBA join pass.
Old-map native330 still fails305(map9<10), recovering306; this cause remains open.
MEM16→MEM17 ASan330 observed101→1 nonOK; MEM18 retains only305 nonOK in the
fixed-map330 replay, with LSan3,379,036B/7,205 allocations still failing.
Full ARM17 build/hash verification is complete; direct board link is unavailable.
No realtime/board runtime acceptance yet.
MP physical release exists; MLPnPsolver RAII is now applied and passes targeted
native/ASan fixtures, while full-System LSan remains failed at3,379,036B/7,205
allocations. KF/Map deletion remains disabled. Joint memory, tracking and realtime
acceptance remains open; no repair commit/production update.

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
- This branch snapshot did not contain `docs/repo-map.md`; repository routing
  therefore followed `AGENTS.md` and direct source/build-script inspection.
- `run_stereo_inertial.sh` couples runtime launch with `pidstat` and bag
  recording, so runtime validation currently depends on those external tools
  being available.

## F26 RGB-D detailed timing validation (2026-08-13)

- The four-job build was killed at `Optimizer.cc` by the 9 GiB container
  memory limit. The single-job incremental retry completed successfully.
- Static checks cover `git diff --check`, both CSV schemas, and shutdown-only
  file writes. The output binaries are aarch64.
- The original board-replay TODO was completed on 2026-08-14; see the runtime
  validation below.

## F26 board runtime validation (2026-08-14)

- The first `13-56-52 @ 1.0x` replay exposed a validation gap: new ORB/depth
  fields and the LocalMapping row append were guarded by the disabled legacy
  `REGISTER_TIMES` macro. The run was retained but excluded from timing
  comparison.
- The new RGB-D and LocalMapping measurements were made independent of the
  legacy macro. A single-job ARM64 rebuild passed in 1 h 22 min. Corrected
  core SHA-256: `6df2983c49023752c301b714da2a8f46f9b1c1d76d20b25244fb62541efe0d9b`.
- Corrected board runs all exited cleanly and populated both CSVs:
  `13-54-55 @ 1.0x` 477 frame / 93 LocalMapping rows;
  `13-56-52 @ 1.0x` 597 / 123; and
  `13-56-52 @ 0.5x` 624 / 134.
- At 1.0x, 36.5%--55.1% of callbacks exceeded the 90 ms input period and
  13/490 or 27/624 synchronized pairs were dropped. At 0.5x no pair was
  dropped and tracking stayed OK for all 624 frames.
- Mean ORB extraction was 49.5--52.5 ms; depth association was 0.13 ms. Local
  BA abort ratio was 74.7% and 59.6% at 1.0x versus 33.3% at 0.5x.
- Evidence:
  `/home/ywl/Project/ORB_SLAM3_DBoW3/board_rgbd_timing_test_20260814/REPORT.md`
  and its `rgbd_timing_test_results_20260814/` subdirectory.

### F26 ORB six-stage extension (2026-08-14)

- A two-job ARM64 Docker cross-build completed successfully. Node SHA-256 is
  `35332603c704cb12a5054df701d1ecab83c32e99d4cb55e245dd0f90608c4494`;
  core SHA-256 is
  `3a45dc2f83cc7c79f728ff5ad5ddae64acae6a407314a85a3e4935a35040f81a`.
- A separate board deployment replayed `13-54-55` at 1.0x. rosbag and SLAM
  both exited 0, shutdown was clean, and the CSV contained 470 frame rows and
  108 LocalMapping rows. There were no lost/recently-lost frames.
- On 469 steady OK frames, ORB mean/P95 was 52.017/58.820 ms. Mean component
  costs were FAST/OctTree 26.856 ms, descriptor 9.475 ms, blur 7.590 ms,
  pyramid 4.347 ms, orientation 2.842 ms and assembly 0.793 ms. The unscoped
  difference was 0.114 ms mean, confirming measurement closure.
- Evidence:
  `/home/ywl/Project/ORB_SLAM3_DBoW3/board_rgbd_orb_stage_timing_20260814/`.

## F27 RDK-X5 RGB-D ORB extraction optimization

- Build the modified core and ROS wrapper through the existing ARM64 Docker
  workflow used by F26; confirm `ros_rgbd` and the linked core are aarch64.
- Compare the legacy clone blur with the reusable standalone-copy path and the
  ROI `BORDER_ISOLATED` candidate on synthetic and real pyramid images. Full
  output and the three-pixel boundary band must be byte-identical before the
  direct ROI path may be enabled.
- Replay `2026-08-13-13-54-55.bag` and
  `2026-08-13-13-56-52.bag` at 0.5x. Baseline and optimized rows must have
  identical ORB hashes for all 490 and 624 input frames, including keypoint
  order and fields, descriptor bytes, lapping placement and `monoIndex`.
- Screen `ORBextractor.parallelWorkers=2/3/4` with at least two 1.0x runs per
  bag. Select the lowest callback P95; if results are within 1 ms, select the
  smaller worker count.
- Final verification is five 1.0x runs per bag with CPU affinity unrestricted.
  Every run must process all expected frames, remain in one map without
  `RECENTLY_LOST`, `LOST`, reset or `Fail to track local map`, keep ORB
  mean/P95 at or below 38/43 ms, and keep callback P95 below 90 ms.
- Before each board run execute `/home/metoak/mo_init.sh`. Save hashes,
  telemetry and reports under the parent project's
  `board_rgbd_orb_optimization_20260817/` directory.

Verified 2026-08-17:

- ARM64 core/ROS builds and the three validation executables passed. Board
  FAST process-CPU/wall ratio was 0.984735, validating level parallelism.
- Legacy versus optimized, workers 2/3/4 versus serial, and three worker-3
  repetitions had zero hash mismatches over both bags' 1114 frames.
- The OpenCV 4.5.4 blur test passed 50 synthetic/ROI cases over all five
  pyramid sizes, including the entire three-pixel boundary band. The optional
  direct isolated-ROI path also matched all 1114 real-frame hashes, but stays
  disabled by default because its measured benefit was below 0.7 ms.
- Worker 3 production completed five 1.0x runs per bag: every run processed
  490/490 or 624/624 frames with zero gaps, state 2 throughout and one map.
  Pooled ORB mean/P95 was 28.915/34.707 ms and 27.650/33.530 ms; callback P95
  was 85.256 and 80.427 ms. Core SHA-256:
  `51c3e4d720e5e8c22e63405b44c2b39a782e55464347016ff2bf1db9987a2860`.
- Evidence and raw logs:
  `/home/ywl/Project/ORB_SLAM3_DBoW3/board_rgbd_orb_optimization_20260817/`.

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

## RD26 RDK X5 Stereo-Inertial Replay (2026-08-18)

- ARM64 cross-build completed through
  `/home/ywl/Project/RDK_X5_orb_slam/tools/build_rdk_arm64_docker.sh`.
- The deployed `ros_stereo_inertial` and `liborb_slam3_ros.so` SHA-256 values
  are `d3a6019e...5e28` and `518ec0df...00bfe`; the board launch selected
  `Stereo-Inertial`, loaded the S316 IMU calibration, and had no missing
  shared-library error.
- Complete 0.5x replays processed `996`, `811`, and `715` stereo pairs and
  consumed `17933`, `14609`, and `12863` corrected IMU messages. All three
  runs reported inertial initialization, ended in tracking state 2 with valid
  poses, and contained zero state-3 CSV rows.
- The first two bags produced early `Not enough motion for initializing`
  resets inside ORB-SLAM3, then recovered and reinitialized. This is retained
  as data-quality/initial-motion evidence rather than hidden as an integration
  failure.
- Evidence and hashes are in
  `evaluation/artifacts/f26_stereo_inertial_rdk_x5/summary.md`. The original
  unindexed third bag was not changed; only its reindexed `/tmp` copy was
  replayed.

## RD27 RDK X5 Stereo versus Stereo-Inertial at 1.0x (2026-08-18)

- The ARM64 cross-build passed for a shared core and both ROS nodes. Board
  hashes matched the host staging hashes; `ldd` reported no missing library.
  The isolated board stage was
  `/userdata/orb_slam/f27_stereo_vs_imu_rate1_20260818`.
- Both modes used explicit ORB, 600 features, five pyramid levels, identical
  S316 camera/baseline/vocabulary/tracking settings, Pangolin disabled and CPU
  affinity 2-7. Runs were cold-started and their order alternated by bag.
- All six `rosbag play --rate 1.0` runs produced a per-frame CSV and complete
  node log without a runtime error. The source had 2522 complete image pairs.
  Pure stereo processed 2150 (85.25%); stereo-inertial processed 2016
  (79.94%).
- Pure stereo had loss frames/segments/duration of `0/0/0.00 s`,
  `26/1/3.24 s`, and `55/2/6.39 s`, ending with `1/2/3` Atlas maps.
  Stereo-inertial had zero tracking-loss rows and one final Atlas map in every
  run. Its final active maps had `188/155/130` keyframes and
  `4276/3447/2588` MapPoints, versus pure stereo's `169/78/16` and
  `4069/1886/590`.
- Weighted mean callback cost was 104.8 ms for stereo and 112.5 ms for
  stereo-inertial. Weighted tracked inliers were 111.1 and 120.0. Both exceed
  the 90.9 ms image period often enough to drop source frames; this throughput
  limitation is retained in the result rather than hidden with slow playback.
- Every inertial run had one early insufficient-motion active-map reset and
  later reinitialized. Stable-segment time-normalized P90 motion closely
  matched stereo, separating the startup reset from sustained pose behavior.
- Evidence:
  `evaluation/artifacts/f27_stereo_vs_imu_rate1/report.md`, `summary.json`,
  six CSV/log pairs, `mapping_health.png`, and `trajectories_xz.png`.
  No ATE/RPE claim is made because the bags have no external ground truth.

## RD31 RDK X5 RGB-D versus RGB-D-Inertial (2026-08-21)

- Added and ARM64-cross-built `ros_rgbd_inertial`, using exact RGB/depth
  synchronization, an independent IMU queue, the validated S316 camera/IMU
  clock correction, and `System::TrackRGBD` with inertial samples. The final
  hashes are `31f511fa...fbdc36b` for the inertial node,
  `4962aece...41f684` for the fair queue-20 RGB-D node, and
  `852a8b19...9c1e` for their shared core library.
- Shell syntax checks passed for the cross-build and board runner; Python
  compilation passed for `evaluation/analyze_f31_rgbd_imu_ab.py`; both the
  RDK workspace and current repository passed `git diff --check`. The complete
  ARM64 cross-build succeeded before deployment.
- Four formal 1.0x runs used SLAM CPUs 2-7, no live camera process, and
  independent maps per mode. Pure RGB-D processed 670/670 mapping and 803/803
  localization pairs, all state 2. RGB-D+IMU processed 670 mapping pairs
  (669 state 2 after the initial frame) and 802/803 localization pairs (771
  state 2, 31 startup state 3); all rosbag/node exits were zero.
- The inertial map initialized at frame 34/2.70 s and saved 117 KFs and 6254
  MapPoints. Localization loaded the same Atlas hash, added zero keyframes,
  produced zero Local Mapping rows and no new map. The 31 RECENTLY_LOST rows
  were confined to processed frames 2-47; frame 48 onward stayed OK.
- Callback mean/P95 was RGB-D versus RGB-D+IMU 57.68/70.87 versus
  65.44/85.80 ms in mapping, and 69.43/81.87 versus 88.02/107.39 ms in
  localization. ORB changed by at most 0.50 ms mean; the main increment was
  Local Map tracking. IMU localization exceeded 90.909 ms on 33.79% of rows
  and accumulated 995.9 ms median / 2785.3 ms P95 queue delay.
- The bags have no external pose truth. SE(3)-aligned common-time trajectories
  and localization-to-own-map nearest-path repeatability were recorded only as
  relative proxies. Pure RGB-D nearest-path mean/P95 was 8.84/23.12 cm versus
  9.71/23.49 cm for IMU OK frames, so the evidence does not show an inertial
  accuracy gain.
- Evidence: `evaluation/artifacts/f31_rgbd_imu_ab/report.md`, `summary.json`,
  `trajectory_comparison.png`, `board_results/`, `configs/`, and `deploy/`.
  The isolated board stage is
  `/userdata/orb_slam/f31_rgbd_imu_ab_20260821/`; its test roscore and all
  SLAM/rosbag processes were stopped after validation.

## RD32 RDK X5 RGB-D Host-to-Board LAN Replay (2026-08-25)

- The host at `192.168.1.3` ran ROS master on port 11312 and replayed
  `2026-08-25-09-56-06.bag` at 1.0x. The RDK X5 at `192.168.1.10` ran the
  verified `production_w3` RGB-D node. The interrupted preliminary board bag
  copy was deleted; the formal run used LAN transport only.
- The source has 888 exact RGB/depth pairs. The node recorded 888 callbacks,
  zero sequence gaps, 888 state-2 results, 888 true tracking-status messages,
  and 888 poses. It created only map 0 and reached final KF ID 190.
- Callback mean/P95/P99/max was `61.46/77.27/84.99/89.10 ms`; no callback
  exceeded the 90 ms input period. ORB extraction mean/P95 was
  `27.72/30.30 ms` and mean tracked inliers were 170.15.
- No loop/merge/correction or runtime-error line occurred. Frames 199 and 642
  nevertheless form a strong 39.87-second revisit with 159 ratio matches,
  135 fundamental-matrix inliers, and 121 homography inliers.
- Shutdown reported that all threads finished. The deployed YAML has no Atlas
  save path, so the map was not persisted. No ATE/RPE claim is made because
  the bag has no external trajectory truth; the live camera was also absent.
- Evidence: `evaluation/artifacts/rgbd_20260825_lan/report.md`, `summary.json`,
  the raw timing/pose/tracking/telemetry/log files, trajectory plot, and repeat
  match visualization.

## RD33 RGB-D ORB Feature/Scale Matrix (2026-08-25)

- Use the exact RD32 production node/library/vocabulary and change only
  `ORBextractor.nFeatures` and `ORBextractor.nLevels` in isolated YAML files.
- Keep `2026-08-25-09-56-06.bag` on host `192.168.1.3`; run the ROS master and
  `rosbag play --rate 1.0` on the host while the RDK X5 at `192.168.1.10` runs
  only the SLAM node. Do not copy the bag to the board.
- Cold-start and screen `600x5`, `800x5`, `800x6`, `1000x6`, and `1200x8`.
  Refine the feasible 600--800 boundary with `700x5`. Retain all
  callback/local-mapping CSVs, logs, pose/tracking topics, telemetry,
  config and binary hashes, playback/node exit status, and exact delivered
  counts. Repeat the best feasible candidate before selecting it.
- A feasible setting must keep one map and continuous state-2 tracking, avoid
  input sequence gaps, and avoid sustained queue/header-age growth at the 11 Hz
  source rate. Compare loop events, ORB/callback P95 and maximum, 90 ms
  overruns, inliers, keyframes, path continuity, memory, and temperature.
- The bag has no external trajectory ground truth, so the matrix can select a
  real-time/robustness operating point but cannot establish ATE/RPE superiority.
- Verified in eight cold-start runs. Both `700x5` runs delivered all 888 pairs,
  stayed state 2 in one map, and had zero sequence gaps. Their averaged callback
  mean/P95 was `65.87/83.44 ms`, averaged ORB mean was `28.48 ms`, and averaged
  tracked inliers were `188.8`, versus `166.2` for the matrix `600x5` run.
- `800x5` repeatably lost `2/3` pairs; `800x6`, `1000x6`, and `1200x8` lost
  `4/18/83` pairs and their callback P95 reached `90.89/104.37/124.22 ms`.
  All runs stayed state 2 for every delivered callback and shut down cleanly,
  but none logged a loop event. Select `700x5`; do not attribute the missing
  loop solely to feature count or pyramid depth.
- Evidence: `evaluation/artifacts/f33_rgbd_orb_matrix/report.md`,
  `summary.json`, `parameter_matrix.png`, `trajectory_overlay.png`, isolated
  YAMLs, and all raw timing/log/pose/tracking/telemetry files under `runs/`.

## RD34 New RGB-D Bag LAN Mapping (2026-08-25)

- Keep `2026-08-25-13-12-57.bag` on host `192.168.1.3`; use ROS master port
  11312 and replay at `1.0x` to the RDK X5 at `192.168.1.10`.
- Use the exact production RD32/RD33 node, library and vocabulary with the RD33
  selected `700x5`, scale factor 1.2, FAST 20/7 and parallel workers 3 config.
- The source gate is 1247 exact RGB/depth timestamp pairs with no source topic
  sequence gap. Record delivered callbacks/gaps, states, tracking status,
  callback/ORB latency, header age, inliers, keyframes/maps, loop/merge logs,
  trajectory continuity, resources, hashes and clean exits.
- Do not copy the bag to the board. Without external trajectory truth, report
  only internal consistency and do not claim ATE/RPE.
- Verified in one complete cold-start run. All 1247 exact RGB-D pairs were
  delivered with zero sequence gaps, every callback ended in state 2, and all
  1247 tracking-status messages were true. Only map 0 was created, through
  KF158; rosbag and node exited zero and shutdown was clean.
- Explicit loops occurred at KF80/source frame 498 and KF141/source frame 985.
  Published map-frame poses showed 0.1479 m and 0.1338 m correction jumps five
  frames later. These exceed trajectory step P99 0.0259 m and confirm applied
  map correction, not merely place-recognition candidates.
- Callback mean/P95/max was 64.46/81.29/100.25 ms; 11 callbacks exceeded 90 ms
  but no input was lost and header-age growth was only 14.78 ms. Mean/P95 node
  CPU was 186.4/218.0%, maximum RSS 661.0 MiB, and maximum temperature 55.19 C.
- Evidence: `evaluation/artifacts/f34_rgbd_20260825_131257/report.md`,
  `summary.json`, `trajectory_loop_events.png`, and the raw run directory.

## RD35 Ultra-long RGB-D OOM Replay (2026-08-29)

- Keep `2026-08-25-15-49-55.bag` on host `192.168.1.3`; run ROS master on
  port 11312 and replay at `1.0x` to the RDK X5 at `192.168.1.10` using the
  selected `700x5`, scale 1.2, FAST 20/7, workers 3 production configuration.
- The source gate is 3163 exact RGB/depth pairs. Capture process RSS, VSZ,
  VmSwap, page faults, system MemAvailable, SwapFree, CommitLimit and
  Committed_AS at 0.5 s intervals, plus before/after process and kernel
  snapshots. Require node/playback exit codes and a clean-board check.
- Verified in one full run: 3136 callbacks, 741 processed KFs through ID 740,
  five Atlas maps, zero loops, playback/node exit code 0 and clean SLAM thread
  shutdown. Twenty-seven exact source pairs were lost in two local overload
  clusters; callback mean/P95/max was 64.46/82.58/169.72 ms.
- Peak RSS was 923.6 MiB, node VmSwap stayed at 0 MiB, minimum MemAvailable was
  140.7 MiB, peak Committed_AS was 1463.7 MiB against a 2701.8 MiB
  CommitLimit, and no OOM/allocation-failure signature occurred. Memory
  returned to its pre-run level after exit. No bag or SLAM process remained on
  the board.
- This verifies only the observed fragmented-map/no-loop workload. Because no
  loop or GBA ran and no individual map reached 1000 KFs, it does not disprove
  the separately identified large `BundleAdjustment` reserve risk.
- Evidence: `evaluation/artifacts/f35_rgbd_oom_20260825_154955/report.md`,
  `summary.json`, `memory_pressure.png`, and the raw run directory.

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


## RD36 — recovered RGB-D queue study (2026-09-08)

Completed 18 full-bag board replays with frozen production core. Evidence lives in
outer `experiments/rgbd_queue_20260908_1518/`, including `conclusion.md`,
`report.md`, `run_manifest.json`, `validation.json` and `provenance/`.
`python3 experiments/rgbd_queue_20260908_1518/validate_results.py` validates
pair accounting, effective configurations, binary provenance, clean SLAM/player
exits and absence of new OOM. All 18 passed these measurement checks; this is
not a stable-tracking or complete-input acceptance pass. The first controller
collection error was recovered and remains explicitly recorded.

The wrapper-only Docker build passed; frozen-library ABI symbol/layout checks
passed. Source bag and production binary hashes matched at completion. Temporary
arm64 binfmt was removed, experiment processes/master11332 ended, and unrelated
master11311 was retained. Source patches are supplied without a Git commit.
No further parameter runs are pending: the user requested stopping based on
existing evidence. One favorable 8/8/20 result is insufficient to recommend
production deployment. See the conclusion for synchronization eviction proof,
complete-input tracking failures and latency-reference limitations.


## RD37 analysis only — 2026-09-08

`python3 experiments/rgbd_callback_review_20260908/analyze_existing.py` from
outer workspace recomputes statistics on 18 existing RD36 CSVs (37,905 rows).
The new callback mean/P95/P99 and counts were cross-checked against the previous
matrix. SHA-256 identifies every input CSV and inspected source/frozen library.
Host llvm-objdump-10 inspected ARM64 code without executing it. See
`docs/rgbd-callback-optimization-review.md` and outer analysis artifacts.
No optimized SLAM build, component-equivalence test, board replay or deployment
was run; implementation and runtime acceptance remain not_started. The
arithmetic sensitivity scenarios are not measurements of optimized code.

## RD37–RD41 exact RGB-D implementation (2026-09-08)

Pre-change commits: outer b838bcb, core eb7aee3. Docker ARM64 build and
board component parity passed (6,000 distance, 49,344 candidate queries,
96 matcher and exact pose cases). Extractor and optimizer sources unchanged.
Ten complete benchmark runs and one separate main-TID profile were collected.
Same-build L0/L4 three-run median mean/P95/P99:76.59/95.90/110.81 vs
70.20/89.35/103.20 ms. L4 lost9/24/15 input pairs; loss episodes0/0/2,
map counts1/1/2. No complete-input acceptance and no P99<90ms acceptance.
No ground-truth ATE/RPE claim. All experiment exit/provenance checks passed.
See outer experiments/rgbd_exact_20260908_1901/{conclusion,report,implementation}.md
and component_parity.json, validation.json, run_manifest.json. New changes
remain uncommitted for review; production is frozen and experiment processes
have ended. Temporary ARM64 binfmt was unregistered after compilation.

## RD42 optimized feature/queue tuning (2026-09-09)

Existing level4 node/core reused, no algorithm edits or rebuild. Nine host bag 1x
replays:700/600/500 queue 1 screening and 700 queue 4/8, then interleaved 700
queue 1/8 repeats. Queue8 three runs all 2128/2128,no loss,one map; queue1
missing 4/21/36, loss 0/1/1. Queue8 median P95/P99=89.45/103.76ms andmaximum
relative schedule backlog 674.30 ms: input continuity gate passed on this bag;
strict low-latency gate did not. All run provenance/exit checks passed.
Full evidence: outer experiments/rgbd_tuning_20260909_0938/conclusion.md,
report.md, validation.json and run_manifest.json. Production unchanged,
original bag unchanged, experiment processes ended. No Git commit.

## RD44–RD46 independent RGB-D input (2026-09-11, completed)

Baseline outer18ba684/core4d6df6a committed before implementation. Actual
state-machine ASan/UBSan tests pass including100 randomized seeds, payload
budget, duplicate/late/reset stamps, shutdown draining and ownership release.
Host ROS syntax and native Docker-to-ARM64 build passed without binfmt.
Same-source headers and existing core objects audited; only LocalMapping
diagnostic storage cap changed in core. Board component parity passed6000
distance,49344 query,96 matcher/pose cases. Smoke processed89 pairs,all OK,
separate receiver/worker TIDs,empty queues,saved map,exit0.

Full-bag screening and interleaved repeats in outer experiments/rgbd_fifo_20260911.
C1(FIFO8) received all2910 per stream but FIFO capacity dropped6 complete pairs.
C16_1 received all2910 per stream but1500ms pair timeout dropped9 consecutive
pairs7238–7246,followed by35 non-OK andtwo maps; measured delivery skew1510–1616ms.
C16_2 processed2910,all OK,one map,but max FIFO wait1254ms. C16_3 also processed2910,all OK,one map; FIFO max wait1418ms.
No production acceptance; no timeout or feature/algorithm tuning to hide failure.
Nine full-bag runs completed; final conclusions are in the experiment report/conclusion.
Lifecycle services0/1/2/1 during a separate167-pair replay passed,all frames OK,
receiver kept running during save; explicit drain followed bySIGINT exited0.

Additional active-stream SIGINT test passed:publisher was still replaying,
node finished35 received pairs,all worker/core threads exited,remaining image
bytes/pair/FIFO counts0,exit0. See outer runs/interrupt/. No full input count
is claimed for this deliberately interrupted short replay.

## RD47 transport timing — 2026-09-12

Wrapper-only Docker ARM64 cross build and independent deployment passed; no
binfmt changes, core binary/hash unchanged. Full same-C++ receive-only2910/2910,
full FIFO SLAM received2910/processed2899 (11 capacity_fifo pairs), and direct8s
89/89. All delivered messages retain valid ROS receipt timestamps; steady entry
matches existing dequeue event exactly. Clock decomposition, ordering, bounded
rows/no overflow, loaded hashes, FIFO thread separation, drain and exit0 checks
passed. Both actual SLAM runs non-OK0/map1/save success, no new OOM; maps copied
and checksummed before removing only new board copies, no reload verification.
New 502ms skew located predominantly before ROS subscription queue. This is
diagnostic validation, not final TCP root cause or production input acceptance.
Evidence: outer experiments/rgbd_transport_timing_20260912/report.md, runs/*/summary.json.


## RD48 TCP detailed measurement — 2026-09-12

Full capture reproduced 1900.420ms skew with depth advertised window5888B, rcv_ssthresh5792B and12.884s rwnd-limited/12.900s; direct transport mechanism identified, memory-cgroup trigger unproven. slam_2 teardown failed after full playback; not normal-exit acceptance. Normal slam_3 processed2910/2910,non-OK0,map1,save/exit0; receive_only2910/2910,max30.458ms,exit0. Host wall-clock adjustment invalidates precise cross-host alignment in slam_3; same-board timing remains valid. No repair or parameter changes. Evidence: outer experiments/rgbd_tcp_detail_20260912/report.md.


## RD49 window cause validation — 2026-09-12

Same-board monotonic TCP/reclaim/perf observation: full SLAM received2910,processed2909,one FIFO eviction,non-OK0,map1,save/exit0,max1139.887ms; small depth thresholds during background kswapd reclaim, no swap I/O in those episodes. Same-sampling receive-only2910,exit0,max48.081ms,no small thresholds or pgscan/pgsteal. Vendor public source matches pressure path, exact running commit unknown. FTRACE/KPROBES/BPF_SYSCALL compiled out; perf samples do not prove socket_pressure write. Strong memcg-pressure evidence, not branch-level root-cause proof or repair. Evidence: outer experiments/rgbd_window_cause_20260912/report.md.


## RD50 memory.low candidate validation — 2026-09-13

ABBA full1x experiment, fresh root-leaf cgroup before node exec/socket creation; low0 vs900MiB only, min0/high-max unchanged, no global or parent changes. All4 received/processed2910,non-OK0,map1,drain/save/exit0,OOM0. A max358/522ms,P9973/192ms; B max733/768ms,P99270/414ms. B early reclaim suppressed but late working set>900MiB still reclaims; candidate not accepted. All temporary groups removed; all maps checksummed/archived. Evidence: outer experiments/rgbd_memory_low_20260913/report.md.

## RD51 — RGB-D稳定基线轻量故障记录器（2026-09-13）

本节是当前独立feature/rgbd-field-recorder工作树的交付；之前RD44–RD50保留为实验记录，
其FIFO/网络/cgroup实验不进入本次源码。outer18ba684/core4d6df6a，direct8/8/10不变。
定长纯数字core观测、默认关闭的有界双缓冲记录器、分析器、独立配置/构建部署取回工具已实现。
node6db74402与core4ea31242必须成套使用；旧重型日志在轻量及关闭对照配置中关闭。

真实记录器ASan/UBSan及ARM64自测、exact match/pose parity、独立烟雾/SIGINT/写失败通过。
原包+关闭/开启/开启/关闭全回放均exit0，处理2897/2910/2900/2899/2895；
后四轮非OK0/0/35/43，活动地图1/1/2/2。CPU开启-关闭均值-0.070个百分点，
小于重复噪声；不宣称算法输出一致或零开销。ARM64两bank实测保守预算1768KiB<2MiB。

验收和源哈希详见外层diagnostics/validation_report.md、memory_budget.md、comparison.json，
最终真实回放事件见verified_first_failure.json，完整原始数据在diagnostics/runs/verified_*。
10分钟数字压力结果见board_stress.txt。地图已取回校验SHA后只删除对应轮次板端副本，生产哈希不变。
没有验证地图重载。现场驱动源码/另一块板/实体相机仍缺；driver明确unsupported，
本地跨主机bag事件不代表现场根因。core采用定长事件sink而非轮询末帧getter；缺失阶段为null。

## RD52 — 地图支持分层观测（2026-09-14）

在RD51纯观测接口上补充参考KF、局部KF/点池、可投影点、已关联/新增匹配、
优化前后关联/外点、原地图版本变化；以KF ID和创建frame ID关联入队、处理、
视觉局部BA开始/返回、建图周期完成。默认关闭、direct8/8/10和全部算法判断不变。

详细字段见外层diagnostics/map_support/design.md，验证及交付范围见validation.md。
记录器双bank容量/线程/内存布局不变，新增行缩短实际窗口，不宣称仍完整保留30+10秒。
旧RD51数据缺少新增字段，分析器标unsupported；不能补造历史故障的地图支持链。
BA返回不表示收敛，未执行的可选阶段时长为invalid。描述子过滤细项、BA精确写回、
位姿漂移及更早的地图演化仍非完整观测；相邻建图事件不构成因果证明。
RD51的ABBA CPU数值仅为历史证据，不作为RD52新增字段的开销验收。

### RD52 同包追加四轮回放（2026-09-14）

固定最终3f6c core的发布包，4轮诊断开启/空地图完整回放：处理2907/2892/2876/2902，非OK 0/34/34/0，活动地图1/2/2/1。两次异常前分别有450 ms和2070 ms实际处理源缺口，中间4/22个源序号两路回调及ExactTime drop均可见。前者在运动模型/参考KF阶段失败；后者优化前158关联、剔除140、18内点低于原30阈值。不是地图点总数逐步耗尽的证明；第四轮630 ms间隔后70内点仍跟踪成功。详见外层 `diagnostics/map_support/repeats_20260914/report.md`、`results.json`、`failure_evidence.json`，原始数据 `diagnostics/runs/rd52_repeat_0N_20260914`。四轮地图取回SHA校验后删除对应板端副本，生产/发布二进制未变。正常轮没有覆盖异常源位置的完整支持链，驱动/网络/调度根因仍缺观测。此次仅追加验证记录，不变更RD52实现或发布包。

## RD53 — S316驱动至SLAM交付观测（2026-09-14）

驱动SDK边界、三路原始身份、左右转换/publish、深度转换/publish、release阶段通过固定共享数字区交给SLAM记录器；默认关闭，无图像引用/驱动写盘线程，原core与Recorder不变。真实相机最终开启1024帧、关闭358帧均OK；成功关联168帧，能够分解驱动发布差和发布后新增错位。完整实现/限制见外层diagnostics/camera_delivery/design.md、validation.md、README.md。ARM64满记录区保守组件预算1708KiB，CPU完整ABBA尚未验收。SDK内部和ROS队列/反序列化细分仍未知，不声称严重迟到根因已复现。

RD53补充：默认不限时运行，Ctrl+C/stop_board.sh正常保存；补齐未初始化RGB-D地图的保存保护。node e07edfde...，core不变。验证与失败修复记录见外层diagnostics/camera_delivery/manual_validation。

## MEM01–MEM04 RGB-D memory work (2026-09-18)

Authoritative implementation is the independent `rgbd_memory_lifecycle` outer
workspace and this core on `fix/rgbd-memory-lifecycle`. Snapshot/provenance and
all commands/results live in `../../memory/README.md` and `../../memory/evidence/`.
Host: `bash memory/tools/build_host.sh` from the outer root. ARM64 full paired
build: `bash memory/tools/build_arm.sh`; baseline: add `--baseline` after restoring
the instrumentation-only source described by evidence/stats-baseline-core.patch.
These full builds do not reuse cached core/node objects. Runtime dependencies
and compiler options are recorded; a copied native g2o without matching Eigen
alignment caused initial failures, fixed by a matching full dependency rebuild.

Completed gates so far: atomic concurrent lifetime counter probes (ASan/UBSan),
actual KF/MP repeated retirement and replacement, fresh bag extraction SHA256,
2910/2910 native baseline states OK, four fixed graph BA comparisons (mono,
stereo, second camera, visual local) with identical graph sizes and pose/point
text outputs, checked capacity growth/overflow (ASan/UBSan), repeated/legacy/
growing/multimap/empty Atlas round-trips with observation identity checks,
300-frame saved-map relocalization (300 OK), 180-frame reset (180 OK), and
controlled mid-write failure preserving the existing map hash then successful retry.

Not equivalent to completion: baseline full run overlapped compilation, so its
resource numbers are exploratory. No ground truth for this bag. Synthetic GBA
exercises the actual optimizer but not loop detection or delayed GBA writeback.
General concurrent save, inertial/real merge, and physical reclamation are NOT
accepted by these tests. Object counters intentionally show retired objects
remaining live; this patch has no automatic object deletion.

### Final verification update

Full paired ARM64 baseline/candidate core and node were built from all current
source units, uploaded after explicit session authorization, and verified by
SHA256 and actual process library maps. Deployment is isolated under
`/userdata/rgbd_memory_lifecycle_20260918`; production and field binaries remain
unchanged. No test node/probe/camera or independent ROS master remains running.
Build/probe/source checks: `../../memory/evidence/arm_build_verified.json` and
`final_verification.json`. Board runtime probes include mono/stereo/body/global,
local and merge-local BA, retirement, and all five quiescent save fixtures.

Formal native ABBA completed without concurrent builds: each 2910/2910, non-OK0,
one map. Mean peak PSS A1262.59/B1210.65MiB; mean TrackRGBD P99 A26.19/B30.79ms.
The memory observation improved; the native tail target failed. No process swap
or major faults occurred, although system swap-in was nonzero. The early
compilation-overlapped baseline is excluded. Off/on/on/off 120-frame diagnostic
screening found no stable additional tail, not proof of zero overhead or ARM
overhead acceptance. Output SE3 disagreement is not ground-truth accuracy.

Board ABBA at 1x processed A1/B1/B2/A2=2894/2906/2904/2891 of2910, non-OK0,
one map, exit0. All four fail complete-input acceptance. Mean peak VmSwap
A236.42/B176.41MiB; P99 bins overlap and no stable tail improvement is proved.
About265k retired MPs remain live; object reclamation is not implemented.
Save-to-exit still incurred31.9k–33.1k major faults. All four map hashes verified;
only B2 received an actual board reload/localization test. At1x it processed
313/334, while the explicitly functional0.5x repeat processed334/334, non-OK0,
one map, relocalized with290 inliers. The slower repeat is excluded from ABBA.
Full tables, samples and exact limits: `../../memory/REPORT.md`, board_report.json,
and board_runs/*/pressure_1s.csv. The user has no additional known loop/GBA/merge
bag and explicitly requested these real paths remain marked unverified.

Complete-core ASan/LSan is NOT passed. Mono global/local BA and real replay fail
on a pre-existing Eigen temporary-expression lifetime bug; actual replay stopped
after28 completed frames. Loaded Pinhole cleanup exposes an uninitialized `tvr`;
the final save fixtures retain that camera and LSan reports64 bytes. Stereo/body
BA, retirement and empty-map sanitizer fixtures pass within their stated scope.
See `../../memory/SANITIZERS.md`; none of these failures is suppressed or called
third-party noise. No ThreadSanitizer, delayed-reader/GBA, full merge or physical
reclamation acceptance. Features therefore remain blocked at their unmet gates,
not marked passing merely because paired builds and ordinary probes succeeded.


## MEM05–MEM08 continuation with the supplied loop bag (2026-09-18)

The earlier MEM01–MEM04 results above are historical. See
[CONTINUATION.md](../../memory/CONTINUATION.md) for current frozen builds, failed
runs and gates. Input955 exact RGB-D pairs, source identity and frames preserved.
MEM05 fixes Eigen temporary lifetime and archive-camera initialization; MEM06
owns/cancels/joins GBA and uses atomic optimizer cancellation. Native loop,
GBA-reset and GBA-shutdown execute real paths. Full MEM06 ASan:955/955, non-OK0,
no runtime access error; LSan still fails55090111 bytes/181860 allocations.

MEM07 migrates all MP readers to shared handles, preserves raw-pointer archive
format through an Atlas-wide adoption table, drops retired-KF point slots and
releases quiescent save snapshots. Delayed-reader, cycle, reset, transfer and
cache-reset tests pass with ASan/LSan. Native full loop physically destroys74512
retired points in the first version. KF/Map collection is explicitly disabled.
Shared MP entries cost16 rather than8 bytes; BA statistics append element widths.

Do not hide the failed full MEM07 ASan replay:35 non-OK frames, then automatic
map creation/merge; heap-buffer-overflow at490 flushed frames in the merge BA
stereo diagnostic loop. This is a pre-existing wrong companion-vector index,
also present in global BA's large-correction branch. MEM08's three regression
fixtures reproduce failures before repair. After the two-index fix,24 native
and24 ASan/LSan fixtures pass; six pre-existing graph outputs remain identical.
Current real-loop/merge/saved-map replays and ARM verification are recorded in
`../../memory/evidence/continuation/`, never substituted for the failed evidence.
Both control and candidate controlled-merge runs warn about a reference KF from
another map; path execution alone is not complete merge-quality acceptance.


Current MEM09 closeout: see ../../memory/MEM09_STAGE_REPORT.md and
../../memory/evidence/continuation/board_archive_arm/results.json. Native original
2910/2910 allOK, actual955 loop/reset/load/controlled-merge and26+26 fixtures pass
their scoped gates. Complete ASan controlled merge has one known injected-new-map
LOST frame and3,841,727 bytes reported leaked; not all sanitizer gates pass.
Board formal loop B2 has35 nonOK and3 missing; both long candidates lose inputs
and regress P99. All8 archives load with unique object/control-block identity,
while alias/orphan/reciprocal warnings remain on controls and candidates.
A short diagnostic accounting error was corrected from header to record-time
bounds:40s means444 pairs here, both controls complete; full-bag results unchanged.
MEM10 copy-cost optimization is now under separate validation; do not infer
passing status from compilation or from MEM09 evidence.


MEM10 measured closeout: three runtime files only, no changes to point ownership,
selection thresholds or queues.26 native+26 ASan/LSan+26 ARM scoped fixtures pass,
six graph numeric texts unchanged per architecture/compiler. Native exact parity
6000/49344/96/96 passes; five955-frame loop/merge/reset/load runs complete allOK.
Full ASan955 executes GBA/merge without access error but repeats the exact known
new-map BoW defect atordinal702/seq758/state4; LSan fails39,024bytes/959 allocations.
The smaller unreachable-leak report does not establish full System cleanup.
Strict functional failure and narrow access-stage exception are separate records.

Eight quiet-host1x X5 incremental runs use A=MEM09/B=MEM10, A1/B1/B2/A2 per bag.
Loop A1/A2 P99[101,102)/[102,103), nonOK3/0; B1/B2 P99[94,95)/[90,91), nonOK0/0;
all955 inputs. Long A1/A2 processed2703/2707, B1/B2 processed2772/2780 of2910,
all processed framesOK; P99 A[144,145)/[141,142), B[135,136)/[132,133).
This is repeated relative improvement, not complete-input or production acceptance.
See ../../memory/MEM10_STAGE_REPORT.md, versioned JSONs and raw board_runs/mem10_*.


MEM10 board reload closeout: loop archive1x955/955, zero nonOK, relocalized/map1.
Long archive first30s1x269/334 and nonOK3;0.5x334/334 and zero nonOK. Both
relocalize with one map. The half-rate result is functional only; failed1x remains
in mem10_board_reload.json and MEM10_STAGE_REPORT.md. Six board production/library
hashes match pre-test capture and no own test nodes/probes remain. Final identity
and original-worktree checks are separate from acceptance in
../../memory/evidence/continuation/mem10_final_verification.json.


MEM11 joint acceptance review (tooling only): user requires memory improvement
and no real-time degradation. Added ../../memory/tools/realtime_guard.py and
REALTIME_ACCEPTANCE.md. Seven specification scenarios cover complete improvement,
dropped-input false speedup, cross-batch mismatch, worst-callback regression,
a worse repeat hidden by a better mean, overlapping histogram uncertainty, and
tracking failure. All behaved as required. Five actual reviews (MEM09->MEM10 for
both bags; historical MEM06->MEM10 for both; initial baseline->MEM10 long bag)
return1/REJECTED; those are expected gate failures, not failed tool execution.
Evidence: ../../memory/evidence/realtime_gate_v1/. No source/binary change or new
board trial; historical comparisons cannot attribute all observed differences
to ownership or constitute a new ABBA. Existing complete-input failures already
prevent current acceptance. Prioritize MP overhead before expanding KF deletion.


MEM12 evidence review: analyze_tracking_latency.py reconciles16 formal board runs
and283 common diagnostic frames. Exact source-loss partitions, histogram counts,
source SHA256 and80 unchanged runtime/build source files verified. MEM09 loop B2
has lower P95 than B1 yet35nonOK, max989ms and map counts1->2->1. MEM10 long B1
has fewer total gaps but six consecutive missing pairs (seq7540-7545),630ms max
source gap; all processed states remainOK. About45-47% of MEM10 long callbacks
are>=90ms. No full pipeline per-frame timing exists for those formal failures.
Correction: the generic Fail-to-track-local-map log can be printed when upstream
bOK is alreadyfalse and TrackLocalMap did not run. Earlier prose using this as
a specific failing-stage label is not supported. First trigger remainsunknown.
See ../../memory/TRACKING_LATENCY_ANALYSIS.md; no runtime change or new replay.

## LM01 C1 transaction foundation

From the runtime outer repository: `bash memory/tools/build_localmap_host.sh c1`.
Run `memory/devel-host-lm01-c1/lib/orb_slam3_ros/local_map_budget_probe`.
Full cross build: `bash memory/tools/build_arm.sh --output build-arm-lm01-c1 --localmap`
(the output directory must not already exist). Board check runner:
`bash memory/tools/run_localmap_board_probes.sh c1` (unique package/round).
Native/ARM 27 probes, new parity/concurrent geometry fixture and native 120-frame
S316 pass. Exact logs, source/binary manifest and sanitizer exceptions are in
IDE `artifacts/local_map_budget_20260924/c1/`; see outer `memory/LM01.md` for limits.
ASan/UBSan/LSan pass in a non-sandbox run, with two documented Boost UBSan
header exclusions. TSan data-race and bounded lock-order fixtures pass with
explicit tool-limit/GDAL qualifications. Full-System LSan remains open.
Budgets and production transaction integration remain C2 work.


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


## LM01 C2.1 — reduce transaction collection overhead (2026-09-24)

Replace per-KF repeated weak ownership promotions with attempt-private identity
hint/exact deduplication and retained owners. Snapshot deltas use a reserved
contiguous vector and an identity index; projection pointers are sealed only
after all candidates are known. Batched snapshot timing removes per-point clock
calls. Commit avoids redundant map locks under the existing identity barrier,
but retains all-before-any-write feature/position validation and counter semantics.

Native full/KF80/shadow 120-frame tests, 27 original ARM probes, new frozen
parity/cap/competition/invalidation fixtures, TSan race/small-lock fixtures and
ASan/UBSan/LSan fixture pass with previously documented dependency exceptions.
The private collector fixture evicts hints and checks exact dedup/order without
shared marker writes. Board full/KF80 short tests both process120/120, non-OK0;
Callback P95 is100.200/99.795ms, Local Map P95 is43.001/41.127ms. This improves
C2's112.448/115.138ms callback, but DOES NOT meet90ms and is not a promotion.
Long2910 screening is running; its results are not included in this commit gate.
Default staysOFF. C3–C6 remain required. Evidence: IDE
artifacts/local_map_budget_20260924/c21/r2/checks.json and immutable runs/.

Tools now freeze native binary pairs before replay, record governor/frequency,
CPU time and temperature, report full-path residence including budget recovery,
and provide a TUM PNG replay driver (20/20 reader smoke only, no precision claim).


## LM01 C3 — typed events and adaptive probes (2026-09-24)

Map event generations/versions are runtime-only. Loop correction, committed
GBA, merge, active-map switch and reset publish explicit types; ordinary
IncreaseChangeIndex does not. Controller separates budget and final health,
uses undistorted8x6 relative/absolute coverage, keeps one retry, escalates budgets,
and requires actual5-frame bounded probes after recovery. Startup/event safety
is separate from budget-induced cooldown, so fallback accounting cannot hide
budget recovery. Both health records and event/controller deadlines are logged.

Native120 and2910 KF80 runs process all inputs in stateOK. Original27 ARM probes
and new transaction/controller/event/grid fixtures pass (graph delta<=7.1e-15).
Controller event concurrency passes TSan and ASan/UBSan/LSan without suppressions.
The inherited full-System LSan issue is unchanged. C3r1's stale stationary-view
inlier reference caused repeated healthy probes to fail; negative evidence is
retained. C3r2 starts a fresh budget-health reference after entering recovery,
then still requires20 protected frames and5 real bounded successes. Full retry
inliers never establish budget health. Scene-change regression tests pass.

First boardKF80 long screen:2450/2910, non-OK0, callbackP95=136.285ms,
fixedlast25P95=145.490ms. OrdinaryOK full/retry fraction13.54%, last25=19.01%:
REALTIME,INPUT and RESIDENCE gates FAIL. DefaultOFF; no production promotion.
KF96/KF64 screening remains running on the immutable C3r2 ARM pair. C4–C6 remain.
C2.1 fixedKF80 long also failed:2490/2910,non-OK0,P95=123.895ms,last25=126.206ms;
cap violations/retries/commit failures0. Do not infer a win from smaller maps.
Evidence: IDE artifacts/local_map_budget_20260924/c3/r2/checks.json and runs/.


## LM01 C4a — experimental coverage and association/geometry diagnostics

Votes stays default. Coverage uses entry matched-point/grid bitsets, normalized
marginals, one pose read per candidate and incremental minimum diversity. It
never scans candidate KF point slots to score; chosen KFs retain original gather
order. Logs distinguish mandatory coverage, first saturation and post-saturation
vote/diversity/tie decisions. Optional shadow diagnostics compare actual feature
associations, optimized pose delta, residual and6x6 information spectrum. Their
cost is included in tracking; they do not change Tracking success criteria.

Native120 coverage/shadow,27 ARM probes, saturation/order and controlled frozen
transaction tests, geometry-rank fixture and ASan/UBSan/LSan fixture pass. The
sanitizer builds now read a frozen source snapshot, so later stages cannot alter
an in-flight build. C1 Boost UBSan-only exceptions remain; full-SystemLSan stays
open. Live shadow is separately labeled from exact frozen fixture parity.

First native2910 pair is2910/2910 OK in both modes. New-match recall votes99.833%
versus coverage99.885% (+0.052pp), new-recallP5 98.571/98.750%. Tail new recall
99.757/99.817%. SelectionP95 .229/1.883ms. Coverage saturates2879/2909 frames,
1456 at the mandatory set. This does not justify promotion. Second native pair
and interleaved board pairs remain running on immutable binaries/configs; C4b
will retain final decision evidence. C4a is an implementation checkpoint, not
final performance acceptance. First boardvotes2612/2910,non-OK35,P95=120.518ms.

Completed C3 adaptive tier screen also rejects every tier: KF96 processed2495,
non-OK70,P95=143.660ms; KF80 processed2450,non-OK0,P95=136.285ms; KF64 processed2577,
non-OK35,P95=129.535ms. KF96's first failure is before Local Map execution, after
a successful full-path frame and an input-sequence gap; cause remains unproven.
No candidate is eligible for production. C5 MP/statistics/cohort work and C6
formal verification/delivery remain. Evidence: IDE artifacts/local_map_budget_20260924/c4/.

## LM01 C4b — completed strategy decision

Two interleaved board pairs and two native shadow pairs completed on frozen C4a
binaries. Votes callback P95=120.518/123.913 ms; coverage=142.096/141.289 ms.
Processed counts were votes 2612/2547 and coverage 2279/2413 of 2910.
Non-OK counts were votes 35/0 and coverage 0/34. All board rounds FAIL.
Native four runs processed 2910/2910, all OK. New-inlier recall improvement was
only 0.029–0.052 percentage points (<0.5 pp promotion threshold), with selection
P95 about 1.9 ms vs votes 0.23 ms. Coverage remains experimental; votes remains
default. These results establish no realtime or tracking-quality acceptance.
Ordinary rounds generated no Atlas files; manifests and SHA checks were retained.
C5 implementation/validation is in progress, C6 remains.
Evidence: artifacts/local_map_budget_20260924/c4/{board,native}_comparison.json.
Core implementation b7ebd840; outer 6c495b5; evidence checkpoint bf514150.

## LM01 C5a — MP budgets/statistics/lifetime checkpoint

KF/MP transaction fixtures, normalized unique-KF scoring, grid/depth allocation,
young-reservation tests, lifetime concurrency and27 ARM probes pass.
ASan/UBSan/LSan fixtures and TSan full/lifetime/small-lock checks pass with
previously documented Boost/GDAL exceptions; inherited full-SystemLSan open.
Four native2910 runs are complete/allOK. MP6500 never binds (eligible max1439);
shadow new recall99.877%, association99.936%. Newborn gate worst drop0.122pp.
First boardC5legacy80 fails2436/2910,nonOK35,P95133.027ms. Six board ablations
and C6 formal/TUM/fault/save-reload matrix remain. DefaultOFF, votes default,
coverage experimental. See memory/LM01_C5_CONTRACT.md and IDE artifacts/c5.
Parent core64d073fd / outerc0ddab1 / IDE39ccbf6c; manifests archive dirty-build
source hashes and paired binaries. This is functional acceptance only.

## LM01 C6 diagnostics and formal matrix

Local BA now appends calling-thread CPU milliseconds to its existing timing
CSV, using CLOCK_THREAD_CPUTIME_ID (unknown=-1). Algorithm, graph selection,
iterations, thresholds and queue limits are unchanged. Native replay drivers
accept LM01_DETAILED_TIMING=1 to export the same backend timing after shutdown.
Resource sampling additionally records the system OOM-kill counter; reporting
separates process CPU, callback-span CPU, BA CPU/wall time and queue occupancy.
Native off/candidate120 both allOK,64 BAs each, CPU positive and<=wall; full
transaction probe passes. ARM node/core/probes completely rebuilt in c6cpu.
The ARM manifest now records the new MP fixture header; the intervening c6
rebuild is byte-identical to c5 in all four binaries, confirming provenance.

Initial TUM5057 six runs are archived as initial_tum, not discarded: candidate
runs1/3 split maps; off has one isolated nonOK in run2. Initial blackout/gap
fixture: candidate15 intentional nonOK then relocalized, single map; off
recovered but split map. Reset and map-switch regressions recovered in both.
Ground truth contains a129.865s gap; typical20ms associations are1227/5057.

The final CPU-instrumented native and ARM pairs are running the required
matrices. No candidate has passed admission. Final C6 evidence remains pending.


## LM01 C6 final — implemented, production gate blocked

All planned experiment groups were executed and negative outcomes retained.
Final ARM/core/node/probe sources are the126-file snapshot for core05e83e55
and outer9feff3c; later changes are analysis/tools/docs. Full six-binary native
reproduction freeze matches the tested manifest. Default LocalMapMode remains off.

Formal long A-B/B-A/A-B: A processed2752/2825/2831, P95=113.661/110.157/108.010ms;
B processed2282/2316/2431, P95=148.547/146.167/146.365ms, nonOK=0/36/111.
Fixed-tail B P95=153.786/172.585/148.022ms; ordinary tail full/retry=18.51/12.38/7.41%.
Formal loop: A953/955/955, B900/930/907 of955; B P95=123.417/110.799/116.878ms.
All six candidate board rounds fail input/P95/P99. All six loop rounds retain
one known loop and complete GBA writeback/finish/join. Pose topic integrity
checks pass for all processed OK callbacks, not ground-truth accuracy.

Final TUM: six attempts; five5057-pair completions, off1 interrupted after
merge cleanup stalled (4693 complete flushed rows). candidate1/3 allOK;
candidate2 and off2/3 split maps, corrected trajectory timestamps invalid.
No valid three-run ATE/RPE comparison; precision gate FAIL. Initial six TUM
runs are also retained. GT has129.865s gap; typically1227/5057 associations.
Final blackout+5-pair gap: both recover but split maps, candidate295 inputs
with52 nonOK; reset and intentional map-switch A/B240 each allOK. Save120/120
OK, loaded existing map101/120 OK; reload input-completeness FAIL. The8914885B
map was fetched/SHA-verified before manifest-owned deletion. Final board free
9775378432B; no experiment SLAM process or registered map remains.

Scoped transaction/statistics/KF/MP/controller/concurrency fixtures,27 ARM
probes and numeric max7.1e-15 remain PASS. C5 sanitizer exceptions/limits apply;
whole-System LSan remains inherited open. A merge progress failure is open,
not fixed or attributed solely to this feature. C5/C6 MP caps never bind;
no MP-cut speedup claim. All newborn gates within5pp. Coverage not promoted.

Analyzer audits: relative callback timestamps cannot align exactly to absolute
resource CPU samples; report whole-process and BA thread CPU. The inherited
visual BA num_MPs output is unpopulated; use separately scoped MemoryStats
candidate MPs and actual graph vertices/edges, never interpret timer zeros as
zero landmarks. No raw evidence was rewritten. Runtime algorithms were frozen
throughout formal testing. Follow-up MIH/BA and diagnostic-overhead work remains
conditional and unimplemented; see memory/LM01_FOLLOWUP.md.

Primary delivery: /home/ywl/Project/ORB_SLAM3_DBoW3/artifacts/local_map_budget_20260924/REPORT.md
Machine gates: c6/acceptance.json (production_pass=false); full data:
c6/formal_details.json, c6/final_tum, c6/final_faults, c6/save_reload,
c6/pose_sidecars/analysis.json, c6/board_final_audit.json.

Reproduction: memory/tools/build_localmap_host.sh + freeze_localmap_host_build.py,
memory/tools/build_arm.sh --localmap, run_localmap_board_matrix.py,
run_localmap_tum_matrix.py (bounded timeout/resume preserves failures),
run_localmap_fault_matrix.py, run_localmap_save_reload.py. Final evaluation:
evaluate_localmap_acceptance.py BOARD TUM OUT --supplementary EVIDENCE.json.
collect_localmap_delivery.py and analyze_localmap_pose_monitor.py reproduce
cost/resource/pose summaries; actual final datasets exercise the analyzers.
