# Progress Log

## Current Active Feature

- Feature ID: F26
- Current state: passing
- Owner/session: isolated RGB-D `TrackRGBD` and LocalMapping timing build and
  RDK-X5 bag replay on 2026-08-13--14
- Blockers: none for F26. Repeated optimization trials remain follow-up work,
  not part of the instrumentation feature.

## Command Log

| Date | Command | Result | Evidence |
|---|---|---|---|
| 2026-08-14 | deploy corrected core and run `13-54-55 @ 1.0x`, `13-56-52 @ 1.0x`, and `13-56-52 @ 0.5x` with `run_board_rgbd_timing_test.sh` | passed | All runs exited 0 and populated both timing CSVs. Report: `/home/ywl/Project/ORB_SLAM3_DBoW3/board_rgbd_timing_test_20260814/REPORT.md`. |
| 2026-08-14 | `RDK_BUILD_JOBS=1 ... ./tools/build_rdk_arm64_docker.sh` after decoupling new timing from `REGISTER_TIMES` | passed | Both catkin packages succeeded in 1 h 22 min; corrected core SHA-256 `6df2983c...efe0d9b`. |
| 2026-08-13 | `RDK_BUILD_JOBS=1 RDK_WORKSPACE=.../rgbd_track_timing RDK_CORE_SOURCE=.../rgbd_track_timing/core ./tools/build_rdk_arm64_docker.sh` | passed | Both catkin packages succeeded; aarch64 `ros_rgbd` SHA-256 `8a5d230d...7c09c`, core library `31e050f8...1836b`. |
| 2026-06-05 | `sed -n '1,240p' docs/features.md`, `sed -n '1,260p' docs/progress.md`, `sed -n '1,260p' docs/session-handoff.md`, `sed -n '1,260p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `sed -n '260,620p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, and `sed -n '620,860p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | reviewed the current pose-correction implementation before narrowing the trigger semantics to big map events only |
| 2026-06-05 | `./build_ros.sh` | passed | rosbuild rebuilt and linked `Stereo` successfully after restricting gradual pose correction to big map events and fixing the one-shot auto-enable path |
| 2026-06-06 | `git status --short`, `sed -n '1,220p' README.md`, `sed -n '1,260p' docs/features.md`, `sed -n '1,220p' docs/validation.md`, `nl -ba Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc | sed -n '300,700p'`, `nl -ba Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc | sed -n '700,860p'`, `sed -n '110,155p' Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml`, and `sed -n '110,155p' Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml` | passed | completed the governance-mandated initialization read and isolated the current F13/F14 pose-correction surface before adding the tracking-loss unstable window |
| 2026-06-06 | `./build_ros.sh` | passed | rosbuild rebuilt and linked `Stereo` successfully after adding the tracking-loss unstable window that suppresses `/robot_pose` and `/robot_pose_map` convergence until the subsequent merge-equivalent big map event |
| 2026-06-06 | `rg -n "continuous_pose_correction|robot_pose correction|converg|unstable window|robot_pose_map|robot_pose_slam" README.md docs Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml` and `./build_ros.sh` | passed | removed the `/robot_pose` gradual convergence path, kept `/robot_pose_map` correction intact, updated the published semantics in docs/YAML comments, and rebuilt the ROS wrapper successfully |
| 2026-06-06 | `sed -n '700,780p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `git diff -- Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, two one-off `python3` assertions for topic-param coverage before/after the patch, and `./build_ros.sh` | passed | added private ROS params `~left_image_topic` and `~right_image_topic` to `Stereo`, preserving the existing `image_rect_raw` defaults while allowing bag replay to override them to `image_raw` without rewriting the bag |
| 2026-06-05 | `sed -n '1,220p' README.md`, `sed -n '1,240p' docs/repo-map.md`, `sed -n '1,260p' docs/validation.md`, `sed -n '1,260p' docs/features.md`, and `git status --short` | passed | completed the governance-mandated initialization read before documenting the pose-correction addition |
| 2026-06-05 | `rg -n "robot_pose_map|pose_correction|PoseCorrection|map_anchor|GetCorrectedMapPose|BlendPose|AutoEnableOnMapEvent|CorrectionHorizonFrames|EnableCorrectedMapPose" Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml README.md docs/features.md docs/validation.md docs/progress.md docs/session-handoff.md` | passed | isolated the pose-correction surface and confirmed both mapping and localization stereo YAMLs now carry `PoseCorrection.*` keys |
| 2026-06-05 | `sed -n '1,320p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `sed -n '320,620p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `sed -n '620,860p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `sed -n '1,240p' include/System.h`, and `sed -n '330,430p' src/System.cc` | passed | confirmed `/robot_pose_map`, `map -> map_anchor`, `/pose_correction/set_enabled`, and the `System` map-change accessors; also found that `AutoEnableOnMapEvent` currently arms correction state but does not by itself bypass the corrected-output enable gate |
| 2026-06-05 | `./build.sh` | passed | rebuilt `libORB_SLAM3.so` after adding non-consuming System accessors for the active map id and big-change counter |
| 2026-06-05 | `./build_ros.sh` | passed | rosbuild rebuilt and linked `Stereo` successfully after the Scheme B `/robot_pose`, `/robot_pose_tracking_ok`, and `continuous_map` TF changes |
| 2026-06-05 | `./build_ros.sh` before rebuilding core | failed | expected transient link failure: `undefined reference to ORB_SLAM3::System::GetCurrentMapId()` because `libORB_SLAM3.so` did not yet include the new accessor |
| 2026-05-29 | `./build_ros.sh` after adding TF broadcasting in `ros_stereo.cc` | passed | rosbuild rebuilt the ROS package and relinked `Stereo` successfully after adding `map -> odom` and `odom -> left_camera` TF publication |
| 2026-05-29 | `sed -n '1,320p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `sed -n '320,460p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, and `rg -n "odometry|robot_pose|twist|TrackStereo|tracking|odom" Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | isolated the F11 change surface to the stereo ROS wrapper and confirmed the existing twist-guard rails before editing |
| 2026-05-29 | `./build_ros.sh` | passed | rosbuild rebuilt the ROS package and relinked `Stereo` successfully after the local-origin odometry change |
| 2026-05-29 | `which pidstat`, `which rosbag`, `rosnode list`, and `rostopic list` | partial | confirmed `pidstat` and `rosbag` exist locally, but both ROS graph checks failed with `ERROR: Unable to communicate with master!`, so runtime validation stayed blocked |
| 2026-05-29 | `git status --short`, `sed -n '1,260p' README.md`, `docs/repo-map.md`, `docs/features.md`, `docs/validation.md`, `docs/progress.md`, and `docs/session-handoff.md` | passed | completed the governance-mandated initialization read for the docs-only F11 follow-up |
| 2026-05-29 | `sed -n '1,260p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `sed -n '240,380p' src/System.cc`, `sed -n '1350,1395p' src/Tracking.cc`, and `sed -n '3600,3725p' src/Tracking.cc` | passed | confirmed the current localization startup flow and why `/odometry` begins in the loaded `map` frame instead of a local zeroed frame |
| 2026-05-25 | `git status --short` | passed | confirmed the worktree already contained unrelated modified and untracked runtime artifacts before implementation |
| 2026-05-25 | `sed -n '1,260p' README.md`, `docs/repo-map.md`, `docs/features.md`, `docs/validation.md`, `docs/progress.md`, and `docs/session-handoff.md` | passed | completed the governance-mandated initialization read before code edits |
| 2026-05-25 | `sed -n '1,320p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `Examples_old/ROS/ORB_SLAM3/CMakeLists.txt`, `Examples_old/ROS/ORB_SLAM3/manifest.xml`, `include/Tracking.h`, `src/Tracking.cc`, `include/System.h`, and `src/System.cc` | passed | captured the `master` branch baseline for the ROS stereo wrapper and relocalization bridge |
| 2026-05-25 | `git show grid_map:Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `.../CMakeLists.txt`, `.../manifest.xml`, `.../msg/RelocalizationStatus.msg`, `include/Tracking.h`, `src/Tracking.cc`, `include/System.h`, and `src/System.cc` | passed | extracted the minimum reusable migration surface from `grid_map` without bringing back occupancy-grid code |
| 2026-05-25 | `./build_ros.sh` | passed | rosbuild regenerated `ORB_SLAM3/RelocalizationStatus` and linked the updated `Stereo` node successfully |
| 2026-05-25 | `which pidstat` and `which rosbag` | passed | confirmed wrapper prerequisites were present on the host |
| 2026-05-25 | `rosnode list`, `rostopic list`, and `rostopic list | rg '^/camera/(infra1/image_rect_raw|infra2/image_rect_raw)$'` | passed | confirmed a host ROS master and live stereo image topics were available for runtime validation |
| 2026-05-25 | `./run_stereo_inertial.sh localization stereo --log` | passed | launched `/Stereo` against live stereo inputs, printed `MyD435i_stereo_load.yaml`, and reached relocalization success in the node log |
| 2026-05-25 | `rostopic info /robot_pose`, `rostopic type /robot_pose`, and `rostopic echo -n 1 /robot_pose` | passed | verified `geometry_msgs/PoseStamped` on `/robot_pose` with `header.frame_id: "map"` |
| 2026-05-25 | `rostopic info /odometry`, `rostopic type /odometry`, and `rostopic echo -n 3 /odometry` | passed | verified `nav_msgs/Odometry` on `/odometry`, `map -> left_camera` framing, and non-zero differential `twist` with conservative covariance |
| 2026-05-25 | `rostopic info /relocalization_status`, `rostopic type /relocalization_status`, `rostopic hz /relocalization_status`, and `rostopic echo -n 1 /relocalization_status` | passed | verified package-local status message on `/relocalization_status`, observed `RelocalizationSucceed`, and measured about `10.0 Hz` publish rate |
| 2026-05-25 | `timeout 10s rostopic echo /relocalization_status` during a second localization run | partial | captured the steady-state status stream repeatedly publishing `RelocalizationSucceed`, but did not individually catch `RelocalizationRunning` or `RelocalizationFailed` before convergence |

## Notes

### 2026-07-30 — F17 Git object-store cleanup and ignore rules

- Added explicit ignore rules for ROS bags, runtime logs/state, generated
  trajectories and reports, ROS message bindings, temporary dependency
  checkouts, downloaded DBoW3 assets, and editor/build metadata.
- Large-object attribution showed that normal local/remote branches retained
  only 0.467 GiB of blobs, while six internal `refs/codex` worktree snapshots
  retained 30.405 GiB, dominated by 2-5 GiB rosbag files.
- `git fsck --connectivity-only --no-progress` exited 0 before cleanup. The
  final `git fsck --full --no-progress` also exited 0 and reported only known
  dangling blobs/trees, with no missing or corrupt object.
- Deleted the six internal snapshot refs, ran default-safe `git gc`, and
  removed five `tmp_pack_*` files that `git count-objects -vH` explicitly
  classified as garbage. `.git` decreased from about 87 GiB to 20 GiB;
  garbage decreased from 7.26 GiB initially to zero.
- Codex subsequently created one new current-session recovery checkpoint. It
  contains about 483 MiB of blobs, shares most objects with normal history,
  and does not include the newly ignored rosbag/runtime paths.
- Immediate `git gc --prune=now` was not performed because it would
  irreversibly remove every unreachable local object. The remaining 19.07 GiB
  of loose objects are unreachable and retained by Git's default recovery
  grace period; the reachable pack is 221.86 MiB.
- During maintenance, the ignored `bags/` worktree files unexpectedly
  disappeared even though normal `git gc` should not remove untracked files.
  They were restored from the already validated dangling tree
  `61fa312b8ed3d13510eab1e40632d397f2a94e57`. A filename/byte-size diff
  against the tree passed for all 15 files, `bags/` returned to 28 GiB, and
  `git status --ignored --short bags` reports `!! bags/`.
- Final filesystem reading: `/home` uses 159 GiB of 298 GiB, with 124 GiB
  available (57% used). Branch heads and `origin/*` refs remained unchanged;
  `master` and `origin/master` both remained at `7f22eb93...`.

### 2026-07-30 — F15 XFeat local reproduction and RDK X5 assessment

- Cloned the official XFeat repository into
  `Thirdparty/accelerated_features/` at commit `e92685f`.
- Added `xfeat_rdk_x5/` with pinned local dependencies, official sparse CPU
  smoke testing, fixed-shape ONNX export, split-pipeline parity testing,
  calibration-data preparation, and a Bayes-e PTQ YAML template.
- Official CPU inference passed with 1024 finite 64D features per sample and 63
  matches. The optimized ONNX split preserved keypoint sets and match pairs;
  details are recorded under `xfeat_rdk_x5/artifacts/`.
- The exporter replaced the direct `_unfold2d` export (more than 140
  Slice/Transpose nodes) with an exact fusion into an 8x8 stride-8 convolution.
- Local OpenExplorer conversion was blocked: no `hb_mapper`, inaccessible
  Docker daemon socket, and SSH timeout to `192.168.1.10`. No `.bin` or BPU
  runtime claim was made.
- Full analysis: `docs/xfeat-rdk-x5-research.md`.

### 2026-07-30 — F16 XFeat evaluation on the recorded stereo bag

- Added `xfeat_rdk_x5/evaluate_rosbag.py` and evaluated 130 synchronized
  stereo pairs plus 65 adjacent temporal pairs across
  `2026-06-05-09-59-04.bag`.
- Compared the official XFeat sparse pipeline with OpenCV ORB using identical
  `640x544` infrared inputs and geometry-aware RANSAC/rectified-stereo rules.
- Both methods passed the robust-pair threshold on every sampled pair. XFeat
  produced broader spatial coverage, while ORB produced more and cleaner
  geometry-consistent temporal and overall stereo inliers.
- Correlation with `/robot_pose_tracking_ok` found that XFeat exceeded ORB's
  strict stereo-inlier count during sampled failure times, but remained worse
  on temporal inliers. This supports an auxiliary stereo experiment rather
  than an immediate full frontend replacement.
- Evidence: `docs/xfeat-bag-evaluation.md` and
  `xfeat_rdk_x5/artifacts/bag_2026-06-05-09-59-04/`.

- On 2026-06-05, `F13` was intentionally reopened as Scheme B: `/robot_pose`
  now uses an external `continuous_map` frame, holds the last valid continuous
  pose during tracking loss after the first valid pose, and publishes
  `/robot_pose_tracking_ok` so consumers can distinguish fresh tracking from
  held output.
- On 2026-06-05, a follow-up change added `/robot_pose_map` plus
  `PoseCorrection.*` YAML keys and `/pose_correction/set_enabled` so operators
  can request a gradual map-semantic correction stream separate from the
  externally continuous `/robot_pose`.
- Later on 2026-06-05, the trigger semantics were tightened to match the
  intended behavior: routine `map_change` increments no longer restart
  correction, ordinary `map_id` switches only preserve `/robot_pose`
  continuity, and only `GetLastBigChangeIdx()` changes now start the gradual
  correction window for `/robot_pose_map` and `/robot_pose`.
- The same follow-up also fixed the `PoseCorrection.AutoEnableOnMapEvent`
  one-shot path so a merge/loop event can start correction even when the
  static enable flag is off.
- On 2026-06-06, the pose publication flow gained an explicit
  tracking-loss unstable window. Once `Fail to track local map!` occurs,
  `/robot_pose_map` keeps publishing its last continuous planning output but
  stops converging toward `/robot_pose_slam` until the next merge-equivalent
  `GetLastBigChangeIdx()` event closes that window.
- Later on 2026-06-06, `/robot_pose` convergence toward `/robot_pose_slam`
  was removed entirely because it materially damaged continuity. The control
  pose now only rebases across map switches and holds the last valid output
  during tracking loss, while `/robot_pose_map` remains the only gradually
  corrected stream.
- The Scheme B ROS wrapper detects active Atlas map changes and big map
  corrections through non-consuming `System` accessors, then recomputes an
  external output offset so raw internal map-origin resets are not exposed
  through `/robot_pose`.
- `/odometry` now derives from the same continuous pose and broadcasts
  `continuous_map -> odom -> left_camera_link -> left_camera`; runtime
  validation is still required before marking F11 or F13 passing.
- On 2026-05-29, `F11` was intentionally reopened as a semantics change: at
  that point the desired behavior was session-relative odometry that started
  near zero in an `odom` frame while leaving `/robot_pose` unchanged as the
  absolute `map` pose. The 2026-06-05 Scheme B update supersedes the
  `/robot_pose` frame contract with external `continuous_map`.
- The 2026-05-29 `ros_stereo.cc` change latched the first successfully tracked
  `Twc` as a session-stable odom origin and computed low-confidence
  differential `twist` only from consecutive valid tracked poses. The current
  implementation keeps that odometry idea but derives it from the external
  continuous pose and uses the body frame `left_camera_link`.
- The 2026-05-29 TF follow-up broadcast `map -> odom -> left_camera`. The
  current TF chain is `continuous_map -> odom -> left_camera_link ->
  left_camera`.
- Tracking loss now resets only the differential-velocity state. The latched
  odom origin is intentionally preserved across recovery within the same node
  session, so the local frame does not jump while twist resumes from a fresh
  valid-pose pair.
- `Tracking` and `System` now bridge relocalization status into the ROS layer,
  and the ROS package now builds the new `RelocalizationStatus.msg`.
- The historical 2026-05-25 runtime evidence for `/odometry` and
  `/robot_pose` reflects the old absolute-pose implementation and must not be
  reused as proof for the new local-origin F11 semantics.

### 2026-07-31 — F18 XFeat INT8 deployment on RDK X5

- Corrected the toolchain mapping: `hb_mapper 1.23.8` is provided by
  OpenExplore `v1.2.6`; the locally verified `v1.2.2` image contains 1.23.5.
- `hb_mapper checker` placed every reported XFeat core node in one Bayes-e BPU
  subgraph. Generated 100 normalized calibration tensors evenly across the
  644 left-infrared bag frames and completed default KL PTQ plus O3 compile.
- Generated `xfeat_backbone_480x640_bayes_e.bin` (920,712 bytes, SHA-256
  `0ff4a43f8b9085c56da2173969f9e42989c79cb6c296ce484d0eb7360a693dea`).
- Added a minimal board-side C++ DNN API runner because this board has neither
  `hobot_dnn` Python bindings nor `hbm_runtime`/`hrt_model_exec`; the legacy
  `tc_hbdk3` sample does not accept packaged DNN `.bin` files.
- Board DNN 1.23.10 / HBRT 3.15.54 loaded the builder-1.23.8 model. Fifty
  BPU0 runs after ten warmups averaged 13.473 ms. Returned output tensors were
  bit-identical to the quantized ONNX simulation and had a minimum 0.990869
  cosine against float ONNX.
- Evidence: `xfeat_rdk_x5/artifacts/ptq_board_validation_480x640.json` and
  `docs/xfeat-rdk-x5-research.md`. The next integration step is native
  `640x544` export plus CPU sparse postprocessing and end-to-end bag timing.

### 2026-07-31 — F19 complete XFeat frontend and board ROS benchmark

- Exported and calibrated a native `1x1x544x640` XFeat core. OpenExplore
  v1.2.6 / `hb_mapper 1.23.8` placed all 33 reported nodes in one Bayes-e BPU
  subgraph; the 986,248-byte model has SHA-256
  `cc90741a05e7b3df4d44d4f61d7472a2766369ead88b2cd401016373f2039fee`.
- Added board C++ CPU preprocessing, persistent BPU execution, sparse decode,
  NMS/top-k, bicubic descriptor sampling, mutual float matching, and a ROS
  benchmark node alongside an OpenCV ORB baseline. CPU optimization reduced
  the smoke-test postprocess from about 81 to 41 ms/image and float matching
  from about 152 to 31 ms/pair without changing sampled match counts.
- Configured the host at `192.168.1.3` as the ROS master and the board at
  `192.168.1.10` as a remote ROS node. Bidirectional XML-RPC/TCPROS contact and
  the three required bag-topic subscriptions were verified before replay.
- Replayed the complete 644-frame bag with a one-second connection delay. The
  CSV contains the expected original sample indices and 390 records: 130
  stereo and 65 temporal pairs for each method, with 100% robust pair rate.
- Alternated which method ran first at each anchor to balance CPU `schedutil`
  ramp/cache effects, and seeded pairwise RANSAC deterministically. XFeat mean
  and median image extraction were 64.85/60.00 ms versus ORB 68.67/66.34 ms.
  Mean stereo end-to-end latency was 185.63 versus 161.98 ms. XFeat provided 48–51% more
  inlier-grid coverage but 18.4% fewer stereo and 25.2% fewer temporal strict
  inliers.
- Added repeatable host LAN, board launch/deploy, and CSV summary scripts.
  Evidence: `docs/xfeat-rdk-x5-board-benchmark.md` and
  `xfeat_rdk_x5/artifacts/board_ros_benchmark_544x640/`.

### 2026-07-31 — F20 single-BPU stereo/CPU parallelization

- Probed `HB_BPU_CORE_1` with a real native-height model inference. DNN returned
  `-6000001` and reported valid numeric IDs `[0,1]` (ANY/BPU0); Linux exposes
  `/dev/bpu_core0` and `bpu0` sysfs only. The board has no schedulable BPU1.
- Added two independent XFeat model/tensor contexts and left/right task
  threads. A shared mutex covers only BPU0 inference, allowing CPU preprocess
  and postprocess to overlap with the other image's BPU work. Added equivalent
  parallel ORB instances and explicit extraction wall-time reporting.
- Parallelized dense normalization, keypoint softmax and bicubic descriptor
  sampling with OpenCV. CPU=1/4/8 smoke tests selected eight workers at
  83.00/69.70/57.32 ms XFeat stereo extraction wall time.
- Full replay produced 390 rows. XFeat stereo extraction averaged 57.78 ms
  versus parallel ORB 74.48 ms. XFeat stereo end-to-end improved from 185.63
  to 125.46 ms, though ORB remained faster at 99.38 ms because its matching
  and RANSAC were cheaper. XFeat temporal end-to-end was 153.22 ms versus ORB
  169.40 ms under the benchmark definition.
- All 390 quality records exactly matched the serial F19 output, covering
  timestamps, keypoint/match counts and every geometry metric. Evidence:
  `docs/xfeat-rdk-x5-parallel-benchmark.md` and
  `xfeat_rdk_x5/artifacts/board_ros_benchmark_parallel_544x640/`.

### 2026-08-03 — F21 XFeat* semi-dense plus fixed-size Fine Matcher retest

- Corrected the candidate from sparse-point refinement to the upstream XFeat*
  semi-dense contract. Host single- and dual-scale refinement parity is within
  `3.1e-5 px` of the official implementation.
- Added fixed M256/M384/M512 Fine ONNX export/PTQ artifacts and a board DNN
  wrapper. All three builder-1.23.8 models load on DNN 1.23.10 / HBRT 3.15.54.
  Standalone BPU means were 7.085, 10.264, and 14.670 ms respectively.
- Added a single-scale semi-dense board decode, scale-aware right-coordinate
  refinement, fixed candidate padding/count handling, and an NCC side metric.
  NCC is necessary because same-row dense-grid points make rectified RANSAC
  strict-inlier counts insensitive to wrong along-row disparity identities.
- Host all-anchor search rejected M256 on quality and selected M384 as the
  current balance. M512 increases host photometric inliers further but has the
  slowest fixed BPU inference. Official dual-scale M384 gives only a small
  quality gain while requiring four serialized backbone calls per stereo pair
  on this single-BPU board.
- Full board replay generated 520 expected rows across source frames 0–641.
  M384 Fine averaged 54.89 ms (P90 59.37), 376.50 strict inliers, 289.42
  NCC-qualified inliers, and 0.529 photometric coverage. Fine improved the
  coarse photometric count by 16.6%; minimum Fine strict/photometric counts
  were 330/176, so there were no fewer-than-30 fragile samples.
- The standalone OpenCV ORB control averaged 88.70 ms (P90 101.43), 286.28
  photometric inliers, and 0.383 photometric coverage. This establishes a
  useful component comparison but cannot satisfy F21's production gate.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_semidense_single_m{256,384,512}_all.json`,
  `xfeat_rdk_x5/artifacts/f21_semidense_m{256,384}_board_ncc_smoke*`, and
  `xfeat_rdk_x5/artifacts/f21_semidense_m384_board_full{,_summary.json}`.
- F21 remains `active`. One actual S316 run is now recorded below, but five
  alternating runs, XFeat TrackStereo integration, and EuRoC ATE/RPE are not
  complete; F22–F24 remain gated.

### 2026-08-03 — F21 ARM64 cross-build and first production ORB baseline

- Verified `/home/ywl/temp/ros1_arm64_ubuntu22.04.tar` as a complete OCI/Docker
  archive (4,304,815,616 bytes, SHA-256 `66a8b417...6354`) and loaded
  `ros1_arm64_ubuntu22.04:v3`. It is Ubuntu 22.04.4 arm64 with ROS Noetic,
  GCC 11.4, CMake 3.22, and working QEMU binfmt execution.
- Added a derived build image with `libepoxy-dev`, changed the reusable build
  script to override the archive's entrypoint, and fixed ROS environment
  loading before `set -u`. The existing Pangolin, DBoW3, and g2o libraries
  were confirmed as AArch64.
- A `-j4` build preserved 23 objects but hit the 9 GiB container limit while
  compiling `Optimizer.cc`. A `-j1` continuation reused them and completed
  `robot_interfaces`, `liborb_slam3_ros.so`, and `ros_stereo`. Output hashes
  are `cada3095...957e` and `71198355...057e`.
- Uploaded both outputs without replacing production files to
  `/userdata/orb_slam/f21_crossbuild_staging_20260803`. Board `ldd` found no
  missing libraries, hashes matched, and the staged binary completed S316
  configuration, vocabulary, Atlas, and ROS initialization.
- The first full replay wrote 644 callbacks. After 30 warmups and exclusion of
  one invalid default-Frame statistics row during a map reset, ORB mean
  extraction, stereo matching, frontend, and TrackStereo were 57.73, 13.24,
  70.96, and 88.15 ms; frontend P90 was 81.76 ms. It averaged 215.51 valid
  depths and 0.755 depth coverage, with one 30-frame/2.97 s loss segment.
- Preliminary M384 comparison: XFeat Fine mean/P90 55.05/59.37 ms, 77.6% of
  ORB mean frontend latency; mean strict matches 375.4; strict coverage 0.580,
  only 76.8% of ORB depth coverage. The speed gates pass but coverage fails,
  therefore `promotion_to_f22` remains false.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_orb_crossbuild_run1_manifest.json`,
  `f21_orb_production_run1.csv`, and
  `f21_preliminary_production_gate_run1.json`.
- Remaining TODO: snapshot stats safely across Atlas reset, make the final
  build explicitly target Cortex-A55 rather than QEMU's generic armv8-a,
  complete five alternating runs, improve XFeat coverage, and obtain XFeat
  SLAM tracking plus EuRoC trajectory evidence.

### 2026-08-03 — F21 same-frame feature-distribution visualization

- Added a host helper that runs the real ORB-SLAM3 stereo `Frame` path with the
  S316 600-feature/5-level parameters and a renderer for the official XFeat*
  semi-dense M384 coarse/Fine candidate.
- On frame 320, ORB produced 270 valid depths over 37/48 grid cells. XFeat*
  coarse produced 384 strict candidates and Fine retained 369, but both
  occupied only 29/48 cells. Fine changed median horizontal disparity from
  zero to 3.00 px without changing left-point coverage.
- Evidence is under `xfeat_rdk_x5/artifacts/feature_visualization/`.

### 2026-08-03 — F21 09-56-06 bag feature-distribution visualization

- Repeated the same renderer on representative frame 440 of
  `2026-06-05-09-56-06.bag` (892 synchronized stereo images, frame timestamp
  `2435541055661`). The real ORB-SLAM3 path detected 605 left points and
  returned 224 valid stereo depths over 43/48 cells. XFeat* returned 600 raw
  points; M384 coarse/Fine occupied 20/48 cells, and Fine retained 371 of 384
  candidates while changing median disparity from 0.0 to 2.45 px.
- Evidence is under
  `xfeat_rdk_x5/artifacts/feature_visualization_bag_2026-06-05-09-56-06/`.
  F21 remains `active`: the visualization confirms that Fine refines existing
  correspondences but does not repair the XFeat detector's spatial coverage.

### 2026-08-03 — F21 reliability return and balanced grid quotas

- Extended the official dense XFeat result with the selected reliability
  values and removed the host adapter's placeholder all-one scores. The same
  values were already available from the board BPU reliability output.
- Added deterministic two-stage 8x6 balancing: semi-dense extraction selects
  600 cells in per-cell rounds with an automatic maximum of 13, then guided
  coarse matching admits at most eight pairs per left-image cell into M384.
  Both stages retain reliability-weighted ranking and expose zero values to
  disable their quota.
- Frame 440 of `2026-06-05-09-56-06.bag` changed from 20/48 occupied cells
  before the change to 47/48 for both coarse and Fine. Coarse/Fine strict
  counts were 325/285 versus ORB's 224 valid depths over 43/48 cells. All 600
  extracted XFeat points occupied 48/48 cells with at most 13 per cell.
- Three regression tests passed. A five-anchor host smoke averaged 333.0
  coarse and 298.2 Fine strict correspondences; mean coverage was 0.992 and
  0.971, with Fine parity error `7.63e-6 px`. The final C++ source compiled and
  linked on RDK-X5. Runtime replay remains TODO because the short board start
  had no ROS master.
- Evidence:
  `xfeat_rdk_x5/artifacts/feature_visualization_bag_2026-06-05-09-56-06_grid_quota_v3/`
  and `xfeat_rdk_x5/artifacts/f21_grid_quota_host_smoke_09-56-06.json`.
  F21 remains `active` pending a full board replay and the formal gates.

### 2026-08-03 — F21 hard-grid full replay and common-PnP evaluation

- Replayed the complete `2026-06-05-09-59-04.bag` over the ROS1 LAN to the
  deployed RDK-X5 binary. The output contains all 520 expected rows across
  source frames 0–641 and aligns exactly with the pre-grid run by frame and
  timestamp. CSV SHA-256 is
  `eb59d7d63ed450a5742c9904b7e5727b0d28e7f8f655def3e93aec68c18850c2`.
- Hard-grid M384 remained faster than production ORB at `54.53/61.27 ms`
  mean/P90 versus `70.96/81.76 ms`, but its mean NCC-qualified stereo count
  was only `186.03`; the pre-grid candidate produced `289.42`. Its standalone
  temporal RANSAC count also fell from `357.52` to `270.80`.
- Added `xfeat_rdk_x5/evaluate_stereo_vo.py` to test 65 identical frame pairs
  through common stereo depth, temporal association, official XFeat Fine
  refinement, and PnP RANSAC. The hard-grid candidate averaged `99.38` PnP
  inliers versus the ORB control's `200.25`, failed one pair with only 24
  inliers, and achieved 64/65 success despite 1.52x ORB PnP grid coverage.
- The no-hard-grid XFeat+Fine control averaged `213.0` PnP inliers, 1.09x ORB
  coverage, 65/65 success, and no fragile pair. During the three sampled
  `orb_tracking_ok=false` pairs it averaged `159.33` PnP inliers versus ORB's
  `127.67`. This isolates the regression to strict cell equalization: nominal
  coverage increased by admitting low-reliability points and truncating
  repeatable high-quality clusters.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_grid_quota_m384_board_full_20260803.csv`, its
  summary and preliminary gate JSON, plus
  `xfeat_rdk_x5/artifacts/f21_stereo_vo_full/{per_pair.csv,summary.json}`.
  The common-PnP ORB method is an OpenCV control and does not replace the real
  S316 ORB-SLAM3 trajectory baseline. F21 remains active; F22 is not promoted.

### 2026-08-03 — F21 four-way sparse/Fine/grid/ORB comparison

- Extended the common PnP evaluator with a 600-feature official sparse XFeat
  method that disables both Fine Matcher and grid quotas. All four methods
  produced 65 finite, frame/timestamp-aligned rows.
- Sparse XFeat averaged 191.14 NCC-valid stereo depths, 223.80 temporal RANSAC
  inliers, and 83.0 PnP inliers. It reached only 41.4% of the ORB control's PnP
  count and failed the 30-inlier threshold at frames 511 and 561.
- No-hard-grid XFeat*+M384 Fine remained the winner among XFeat variants:
  294.95 NCC-valid stereo depths, 213.0 PnP inliers, 65/65 success, and no
  fragile pair versus ORB's 280.17, 200.25, and 65/65.
- Full interpretation and metric-scope caveats are recorded in
  `docs/xfeat-fourway-comparison.md`. F21 stays active because common PnP is a
  frontend proxy and does not replace SLAM-integrated trajectory evaluation.

### 2026-08-03 — deployed sparse XFeat changed to 600 features

- Changed the board runner and C++ defaults from 1024 to 600 sparse features.
  Because this profile is explicitly “sparse, no Fine”, its coarse-match limit
  is also 600, Fine is disabled, and extraction/match grid quotas are zero.
- Rebuilt natively on RDK-X5 and deployed the new binary/script. Board SHA-256
  values are `3fa4f964...adb98` for `xfeat_ros_benchmark` and
  `6b961c23...83fd` for `run_ros_benchmark.sh`.
- Startup log confirmed `top_k=600`, `semidense_single=0`, both quotas zero,
  and `fine=disabled`. A one-anchor bag smoke produced 600/600 XFeat points in
  both stereo samples and the temporal sample; temporal matching returned 429
  matches and 383 RANSAC inliers in 104.09 ms.
- Evidence: `xfeat_rdk_x5/artifacts/f21_sparse600_board_smoke.csv`, SHA-256
  `473263d93c84c6f36e504a1698dacef56b61de7390d1a68fd0c71efbaaf8546f`.
  This is configuration/runtime verification, not a replacement for the next
  complete alternating full-bag run.

### 2026-08-03 — F21 formal gate closed

- Froze the winning deployable candidate as single-scale XFeat*, 600 points,
  automatic 8x6 extraction balancing, loose match maximum 16 per cell, M384
  Fine Matcher, and confidence 0.20. Hard quota 8 and sparse/no-Fine remain
  rejected controls.
- Rebuilt the real S316 ORB baseline with `-mcpu=cortex-a55`, fixed the empty
  Frame snapshot after Atlas reset, staged the ARM64 binary/library without
  replacing production, and verified matching hashes and complete `ldd`.
- Completed five alternating full-bag runs: all ORB runs recorded 644/644
  callbacks and all XFeat runs recorded 520/520 rows. After identical 30-source
  frame warmup, ORB frontend mean/P90 was `70.505/81.827 ms`; XFeat was
  `54.633/59.402 ms` and therefore passed both latency gates.
- Board XFeat averaged 212.31 NCC-valid points versus 215.46 production ORB
  valid depths (98.54%). Under the common real-ORB-SLAM3/XFeat estimator,
  XFeat exceeded ORB in NCC count (`213.94/209.68`), NCC coverage
  (`0.844/0.742`), PnP inliers (`123.77/105.63`), and PnP coverage
  (`0.596/0.550`); both achieved 65/65 with no fragile pair.
- Two sequential 1000-iteration M384 board tests passed, produced identical
  output hashes, and had peak RSS 15,408/15,404 KiB. Threshold-0.20 PTQ/board
  decision agreement was 100% and offset error P95 was 0.0507 px.
- F21 is `passing`; F22 is permitted but remains `not_started`. Full SLAM
  integration and EuRoC trajectory gates remain F22–F24 work. Evidence:
  `docs/f21-gate-report.md` and `xfeat_rdk_x5/artifacts/f21_gate_final.json`.

### 2026-08-04 — post-F21 XFeat feature-count sensitivity

- Kept the passing F21 matcher policy fixed (single-scale XFeat*, loose 8x6
  match quota 16, cosine 0.82, ratio 0.90 and Fine confidence 0.20) and varied
  only `top_k` over the same 65 sampled bag pairs. The frozen 600-point result
  remains unchanged.
- M384 results for 600/576/560/512/480/384/320 points returned mean PnP
  inliers `123.77/118.48/118.00/109.72/102.00/81.71/69.28`; minimum PnP
  inliers were `37/35/31/28/25/19/15` respectively.
- 576 and 560 achieved 65/65 PnP success with no pair below 30 inliers. The
  560-point candidate has only one inlier of worst-frame margin and is not a
  production recommendation. 512 and 480 fell to 64/65; 384 and 320 fell to
  62/65.
- A 576-point M256 test fell to 64/65 with a 24-inlier minimum, so reducing the
  fixed Fine batch is not currently quality-safe. Because top-k reduction does
  not reduce the dense backbone or fixed M384 BPU workload, 576-point M384 is
  only a board-latency candidate, not yet a new default.
- Evidence is under `xfeat_rdk_x5/artifacts/feature_count_sweep/`. No source,
  board deployment, or frozen F21 configuration was changed; F22 remains
  `not_started`.

### 2026-08-04 — isolated analysis of the 512-point fragile pair

- The only top-k 512 M384 failure is current frame 561 at stamp
  `2626521639835`, using frame 560 stereo depth. `solvePnPRansac` did return a
  pose, but only 28 inliers, so the diagnostic rejected it against the
  intentionally strict 30-inlier acceptance threshold.
- A same-process 600/512 reconstruction showed that 512 is a strict subset of
  the 600 selected points. Reducing the budget removed 88 points, then reduced
  temporal matches `130 -> 107`, NCC-valid stereo depths `217 -> 196`, PnP
  candidates `70 -> 59`, and PnP inliers `37 -> 28`.
- Of the 600-point PnP set, 14/70 candidates and 7/37 inliers used at least one
  feature absent from the 512-point subset. Removing those points alone leaves
  only 30 inliers; changed matching/RANSAC consensus accounts for the final
  drop to 28. PnP inlier cells also fell from 11/48 to 10/48.
- The frame contains two moving people, repeated vertical wall texture, a
  bright doorway, and a largely planar floor. The image is not notably blurred
  (left Laplacian variance `1183 -> 1110`), but accepted static PnP inliers are
  concentrated around the central doorway/signage rather than the lower image.
- Raising RANSAC iterations from 200 to 500 changed nothing. Relaxing the
  reprojection threshold from 2.0 to 2.5 px raised inliers from 28 to 33 but
  changed the translation estimate from about 0.072 m to 0.115 m; it is not a
  justified fix without trajectory truth.
- Evidence and visualization:
  `xfeat_rdk_x5/artifacts/feature_count_sweep/frame561_analysis/analysis.json`
  and `topk600_vs_512_pnp.png` in the same directory.

### 2026-08-06 — F25 temporary stereo gate 25 board replay

- Built a separate RDK X5 diagnostic library that changes only the pure-stereo
  local-map acceptance gate from 30 to 25. It records pose-step and reprojection
  statistics only near the gate or on a large pose step; inertial gates and the
  production board deployment remain unchanged.
- Replayed both S316 bags at 1.0x with live-camera/SLAM CPU isolation and unique
  topic prefixes. `09-59-04` completed as one 135-KF/2822-MP map with no lost
  frame. Its critical frame 523 had exactly 25 map inliers, a 0.063 m/1.61 deg
  step, and 1.26/2.37 px reprojection mean/P90, so this run supplies positive
  evidence for the lower gate.
- `09-56-06` lost tracking for 30 callbacks and created a second map. The first
  failure was only 9 reference-keyframe BoW matches at frame 432, after frame
  431 had 50 local-map inliers. No 25-29 gate decision occurred before the
  failure, while the earlier threshold-30 run passed the same bag. This is a
  separate reference-KF/map-state nondeterminism and is not fixed by the gate.
- Sparse logging added only 0.56 ms mean callback time on `09-59-04` and 0.77
  ms on `09-56-06` relative to the CPU-isolated threshold-30 runs. F25 is
  `passing` as a diagnostic experiment, but 25 is not promoted as a production
  default without repeated alternating trials.
- Evidence is in `/tmp/orb_slam3_095606_*gate25_sparse*` and
  `/tmp/orb_slam3_095904_*gate25_sparse*`; Atlas saves remain on the board as
  `ORB_SLAM3_Map_1786001728.921163.osa` and
  `ORB_SLAM3_Map_1786001967.955328.osa`.

### 2026-08-10 — F22 Hybrid XFeat stereo-depth integration

- Added a generic `StereoDepthProvider` seam to the pinhole stereo `Frame`
  path. The F22 provider runs the frozen 600-point XFeat* backbone and M384
  Fine Matcher, associates refined disparities to existing ORB keypoints, and
  accepts them only after a 9x9 patch-NCC check. It can add missing ORB depth;
  replacement requires a 0.05 NCC margin, and it never deletes ORB depth.
- ORB remains authoritative for temporal tracking, MapPoint descriptors, BoW,
  relocalization and loop closing. `Feature.Type` explicitly selects `ORB` or
  `Hybrid`; unsupported values fail at startup. Per-frame Hybrid counters,
  latency and Tcw pose were added to the ROS CSV evidence.
- Restored the RDK pure-stereo local-map gate from the temporary F25 value 25
  to the production value 30. The Cortex-A55 cross-build passed at `-j1`
  after a four-job OOM and produced node/library SHA-256 values
  `5a507a46...1eaf9b` and `56987b52...7766f9`. They were staged independently
  at `/userdata/orb_slam/f22_hybrid_20260810`.
- Replayed the three 2026-08-06 lawn bags at 0.5x on isolated topic prefixes.
  The unindexed `09-50-14` original was preserved; only
  `/tmp/f22_2026-08-06-09-50-14.bag` was reindexed. ORB/Hybrid callbacks were
  `994/994`, `803/810`, and `704/713`; comparisons use the 992/802/702 common
  timestamps.
- Every common frame retained the identical ORB keypoint count and Hybrid had
  zero valid-depth regression. Mean added depth was `18.39/17.12/16.13`, and
  mean tracked-inlier delta was `+18.79/+13.88/+17.82`. ORB/Hybrid loss frames
  were `0/0`, `34/34`, and `34/0`; map creation counts were `1/1`, `2/2`, and
  `2/1`. The third ORB run had a 2.94 m/95.25 deg map-reset discontinuity while
  Hybrid stayed within 0.10 m/3.14 deg maximum consecutive pose steps.
- Conditional near-gate reprojection mean/P90 means were
  `1.24/2.30` versus `1.45/2.76` px on the first ORB/Hybrid pair,
  `1.32/2.45` versus `1.27/2.40` px on the second, and `1.35/2.53` versus
  `1.24/2.34` px on the third. No runtime error marker occurred.
- Hybrid costs about 53.3 ms per frame and raises mean callback time from
  `100.72-107.11 ms` to `155.92-162.41 ms`. F22 therefore passes its tracking
  and observation gates at controlled playback, but cannot meet the S316
  11 Hz period while also computing the full ORB stereo frontend. F23 must
  remove duplicate extraction rather than promote this Hybrid mode unchanged.
- Evidence: `xfeat_rdk_x5/artifacts/f22_hybrid_ab/summary.json` and the six
  paired CSV/log files in the same directory.
