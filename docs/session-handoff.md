# Session Handoff

## 2026-08-14 F26 board deployment and runtime timing

- Active feature and final state: F26, passing including board runtime CSV
  verification and three controlled bag replays.
- The first replay found the new fields accidentally dependent on disabled
  legacy `REGISTER_TIMES`. The corrected implementation always records only
  the new RGB-D/LocalMapping fields and does not enable the wider legacy
  statistics machinery.
- Corrected ARM64 build passed with `RDK_BUILD_JOBS=1` in 1 h 22 min. Node
  SHA-256 remains `8a5d230d...7c09c`; core SHA-256 is
  `6df2983c...efe0d9b`.
- Board deployment: `/userdata/orb_slam/rgbd_timing_20260814`; old production
  `/userdata/orb_slam/rgbd_20260812` was not modified. The incomplete debug
  core is backed up as `liborb_slam3_ros.r1_incomplete.so`.
- Valid runtime rows (frame/LocalMapping): `13-54-55 @ 1.0x` 477/93;
  `13-56-52 @ 1.0x` 597/123; `13-56-52 @ 0.5x` 624/134. All play/node exit
  codes were 0 and all shutdowns flushed CSVs cleanly.
- Main evidence: foreground callback mean 89.5--91.2 ms against a 90 ms
  period; ORB mean 49.5--52.5 ms; depth association 0.13 ms; Local BA abort
  ratio 60%--75% at 1.0x versus 33% at 0.5x.
- `13-54-55 @ 1.0x` lost tracking for 35 frames after a critical dropped pair
  and a keyframe/LBA-abort burst. `13-56-52 @ 1.0x` stayed OK despite 27 drops,
  proving drops are a risk multiplier rather than a sufficient cause.
- Report and raw evidence:
  `/home/ywl/Project/ORB_SLAM3_DBoW3/board_rgbd_timing_test_20260814/REPORT.md`.
- Next recommended step: test 8 Hz and 9 Hz five times each, then optimize ORB
  extraction before changing depth or ROS publishing.

## 2026-08-13 F26 RGB-D timing debug branch

- Active feature and final state: F26, passing for isolated source and ARM64
  cross-build. Runtime CSV population remains TODO.
- Core branch/worktree: `debug/rgbd-track-timing` at
  `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing/core`.
- ROS companion workspace: parent `rgbd_track_timing/`, also on a local
  `debug/rgbd-track-timing` branch because the original RDK workspace had an
  empty `.git` directory.
- Commands: `git diff --check`; initial four-job Docker cross-build (failed by
  OOM at `Optimizer.cc`); single-job incremental cross-build (passed).
- Evidence: aarch64 `ros_rgbd` SHA-256
  `8a5d230d0756acee8a18a5ddddd5b18597e936da751e628d8e66c76a1c87c09c`;
  core library SHA-256
  `31e050f8f53e3d37afa959930434f3545affaf52ea224845aab290f5da61836b`.
- Touched core files: `include/RgbdTiming.h`, `include/{Frame,Tracking,System,
  LocalMapping}.h`, `src/{Frame,Tracking,System,LocalMapping}.cc`, and F26
  docs. Touched ROS files: `src/ros_rgbd.cc`, `launch/s316_rgbd.launch`, the
  Docker build wrapper and `DEBUG_BRANCH.md`.
- Unverified assumptions/TODO: board deployment was not performed; neither
  bag was replayed with this build; confirm both CSV files are populated on a
  clean shutdown and correlate the known failing bag interval at 1.0x/0.5x.
- Next step: stage the two ARM64 artifacts separately on the RDK-X5, replay the
  accepted RGB-D bags, copy back `rgbd_frame_timing.csv` and
  `local_mapping_timing.csv`, then compare P50/P90/P99 and deadline misses.

## Summary

- Active feature: F25
- Final state: passing as a temporary diagnostic experiment; the gate is not
  promoted as a production default.
- Main result: the 25-inlier gate kept `09-59-04` in one map, including a
  geometrically plausible frame at exactly 25 local-map inliers. `09-56-06`
  still split because reference-keyframe matching failed before the local-map
  gate, demonstrating a separate asynchronous map-state repeatability issue.

## Commands Run

```text
git status --short
sed -n '1,260p' README.md
sed -n '1,260p' docs/repo-map.md
sed -n '1,260p' docs/features.md
sed -n '1,260p' docs/validation.md
sed -n '1,260p' docs/progress.md
sed -n '1,260p' docs/session-handoff.md
rg -n "robot_pose_map|pose_correction|PoseCorrection|map_anchor|GetCorrectedMapPose|BlendPose|AutoEnableOnMapEvent|CorrectionHorizonFrames|EnableCorrectedMapPose" Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml README.md docs/features.md docs/validation.md docs/progress.md docs/session-handoff.md
sed -n '1,320p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc
sed -n '320,620p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc
sed -n '620,860p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc
sed -n '1,240p' include/System.h
sed -n '330,430p' src/System.cc
sed -n '110,150p' Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml
sed -n '110,150p' Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml
rg -n "robot_pose_map|PoseCorrection|map event|loop closure|merge|continuous_map" README.md docs/features.md docs/validation.md docs/progress.md docs/session-handoff.md
nl -ba Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc | sed -n '300,700p'
nl -ba Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc | sed -n '700,860p'
sed -n '110,155p' Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml
sed -n '110,155p' Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml
./build_ros.sh
```

## Commands Not Run

- `./build.sh` was not rerun in this follow-up because the change surface was
  limited to the ROS wrapper and its governance/docs updates.
- `./run_stereo_inertial.sh mapping stereo --log` and
  `./run_stereo_inertial.sh localization stereo --log` were not run in this
  turn, so the new unstable-window behavior, TF visibility, and runtime service
  semantics remain unverified here.

## Touched Files

- `README.md`
- `Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml`
- `Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml`
- `Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`
- `docs/features.md`
- `docs/progress.md`
- `docs/validation.md`
- `docs/session-handoff.md`

## Evidence Paths

- No new runtime logs or bag captures were produced in this session.
- `./build_ros.sh` completed successfully in this follow-up, so the ROS wrapper
  change set is at least build-verified even though runtime semantics are still
  pending.
- Static evidence came from `Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`,
  `Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml`,
  `Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml`, `include/System.h`,
  and `src/System.cc`.

## Blockers and Assumptions

- TODO: Runtime validation still depends on a reachable ROS master plus live or
  replayed stereo image topics that can trigger both `Fail to track local
  map!` and a later merge-equivalent big Atlas event in the same run.
- TODO: Confirm in runtime that `/robot_pose` stays continuous in
  `continuous_map` across tracking loss, new-map recovery, and later big map
  events, with no residual convergence toward `/robot_pose_slam`.
- TODO: Confirm in runtime that `/robot_pose_map` stays continuous in
  `planning_map` throughout the tracking-loss window and only resumes
  convergence toward `/robot_pose_slam` after the subsequent merge-equivalent
  big map event closes that unstable window.
- TODO: Confirm in runtime that ordinary `map_change` increments still do not
  spam correction warnings, and that only big map events such as merge or loop
  closure resume gradual convergence for `/robot_pose_map`.
- TODO: Confirm in runtime that `PoseCorrection.EnableCorrectedMapPose = 0`
  plus `PoseCorrection.AutoEnableOnMapEvent = 1` still produces a one-shot
  correction window after the unstable window closes.

## Next Recommended Step

- Launch a stereo mapping or localization session that can reproduce tracking
  loss followed by a later merge-equivalent big map event, and record
  `/robot_pose_map`, `/robot_pose_slam`, `/robot_pose`,
  `/robot_pose_tracking_ok`, and `/tf`.
- In that session, explicitly verify the unstable-window contract and then test
  the service toggle plus the case `EnableCorrectedMapPose = 0` with
  `AutoEnableOnMapEvent = 1` to confirm the runtime still matches the intended
  one-shot auto-correction design after instability clears.

## Addendum 2026-06-06

- Follow-up scope: make `Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` accept
  configurable stereo image topics so the node can consume bags that publish
  `/camera/infra1/image_raw` and `/camera/infra2/image_raw` without rewriting
  the bag itself.
- Commands run:
  `sed -n '700,780p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`,
  `git diff -- Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`,
  two one-off `python3` assertions for topic-param coverage before/after the
  patch, and `./build_ros.sh`.
- Result: `./build_ros.sh` passed and relinked `Stereo` successfully after
  adding private ROS params `~left_image_topic` and `~right_image_topic`,
  keeping the existing `image_rect_raw` defaults while allowing runtime
  override to `image_raw`.
- Touched files:
  `Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`,
  `Examples_old/ROS/ORB_SLAM3/S316_stereo.yaml`,
  `docs/session-handoff.md`.
- TODO: runtime playback with the original bag is still unverified in this
  session; use `_left_image_topic:=/camera/infra1/image_raw` and
  `_right_image_topic:=/camera/infra2/image_raw` during the next replay test.

## Addendum 2026-07-22

- Follow-up scope: add a board-side debug collection helper for the RDK X5
  catkin deployment found at `/userdata/orb_slam`, so `orb_slam3_ros
  kitti_stereo.launch` sessions can be captured and brought back to the laptop
  for Codex analysis.
- Commands run:
  `sed -n '1,260p' docs/features.md`,
  `sed -n '1,240p' docs/validation.md`,
  `sed -n '1,220p' docs/repo-map.md`,
  `git status --short`,
  `chmod +x collect_slam_debug.sh`,
  `bash -n collect_slam_debug.sh`, and
  `./collect_slam_debug.sh --help`.
  Board-side follow-up commands copied the script to
  `/userdata/orb_slam/collect_slam_debug.sh`, ran `bash -n`, and ran a short
  `timeout 8s ... attach --no-bag --no-images --no-grid --no-future-debug`
  smoke test writing to `/tmp/codex_slam_debug_smoke`.
- Result: added `collect_slam_debug.sh` with `attach` and `launch-slam` modes,
  per-run output under `/userdata/orb_slam/debug_runs/<run_id>/`, optional
  image/grid/future-debug topic recording, ROS state snapshots, topic hz logs,
  selected message samples, resource logs, `dmesg` before/after, and ROS latest
  log copying. `bash -n` passed locally and on the board, `--help` printed
  expected usage, and the short board-side smoke run produced metadata,
  ROS/topic snapshots, and `dmesg.before.txt`/`dmesg.after.txt` before timeout
  stopped the collector.
- Touched files:
  `collect_slam_debug.sh`, `docs/validation.md`, and
  `docs/session-handoff.md`.
- TODO: full board-side runtime execution with active camera image topics and
  `orb_slam3_ros kitti_stereo.launch` is still unverified; the smoke test used
  `--no-bag --no-images` and only proved collection/setup/cleanup mechanics.

## Addendum 2026-07-30 — F15 XFeat / RDK X5

- Final state: `passing` for the requested local reproduction and BPU
  feasibility assessment. PTQ, `.bin` generation, and board runtime remain a
  clearly separated follow-up rather than being claimed from ONNX inspection.
- Commands run included the governance initialization reads and `git status`,
  official source clone, dependency inspection/install into
  `/tmp/xfeat-rdk-x5-pydeps`, official CPU smoke test, fixed-shape ONNX export,
  ONNX Runtime parity test, split-pipeline test, calibration helper smoke test,
  Python syntax compilation, graph/operator inspection, local Docker/toolchain
  checks, and read-only SSH probing of `root@192.168.1.10`.
- Passing evidence:
  `xfeat_rdk_x5/artifacts/local_smoke.json`,
  `xfeat_rdk_x5/artifacts/onnx_validation.json`, and
  `xfeat_rdk_x5/artifacts/onnx_pipeline_smoke.json`.
- Generated evidence: the gitignored 2.6 MiB ONNX model at
  `xfeat_rdk_x5/artifacts/xfeat_backbone_480x640_opset11.onnx`, SHA-256
  `479da17c7d8ab12cfb21125b5a85e617f9aa849ea606fa2bcd2bb085c7c1ca26`.
- Touched files/directories:
  `Thirdparty/accelerated_features/`, `xfeat_rdk_x5/`,
  `docs/xfeat-rdk-x5-research.md`, `docs/repo-map.md`, `docs/features.md`,
  `docs/validation.md`, `docs/progress.md`, and `docs/session-handoff.md`.
- Commands not run: `./build.sh`, `./build_ros.sh`, and ROS runtime commands
  were outside F15's isolated evaluation lane. `hb_mapper checker` and
  `hb_mapper makertbin` could not run because `hb_mapper` is absent and the
  current user cannot access the Docker daemon. Board inference could not run
  because SSH to `192.168.1.10` timed out during banner exchange.
- TODO: acquire about 100 representative camera images, confirm the candidate
  YAML against the exact OpenExplorer release paired with the board runtime,
  run checker/PTQ/model verifier, deploy `.bin`, and measure BPU-only plus
  end-to-end latency and INT8 feature/matching quality.
- TODO: do not connect XFeat descriptors directly to the current ORB-SLAM3
  front end. The existing tree assumes 32-byte binary ORB descriptors,
  Hamming matching, and an ORB-specific DBoW3 vocabulary; a separate design
  and validation feature is required for that integration.

## Addendum 2026-07-30 — F16 XFeat bag evaluation

- Final state: `passing` for the offline feature/matching comparison on
  `2026-06-05-09-59-04.bag`.
- Commands run included `rosbag info`, direct ROS bag metadata/image reads,
  Python syntax compilation, a three-anchor smoke test, two full evaluator
  runs, JSON/CSV inspection, and visual inspection of XFeat/ORB stereo and
  temporal inliers at representative frames including tracking-failure frame
  521.
- The final full run sampled 130 stereo pairs and 65 adjacent temporal pairs.
  Both methods achieved a 100% robust-pair rate. XFeat had substantially
  broader spatial coverage, but fewer strict stereo and temporal geometry
  inliers and 13.40x OpenCV ORB's host CPU extraction latency.
- Correlation with `/robot_pose_tracking_ok` found 7 sampled stereo times in
  the false state. In those samples XFeat averaged 364.3 strict stereo inliers
  versus ORB's 343.0, but XFeat remained weaker on temporal inliers (339.7
  versus 504.7).
- Touched files:
  `xfeat_rdk_x5/evaluate_rosbag.py`, `xfeat_rdk_x5/README.md`,
  `docs/xfeat-bag-evaluation.md`, `docs/features.md`, `docs/validation.md`,
  `docs/progress.md`, and `docs/session-handoff.md`.
- Evidence:
  `xfeat_rdk_x5/artifacts/bag_2026-06-05-09-59-04/summary.json`,
  `per_pair_metrics.csv`, and the `frame_*.png` visualizations.
- Commands not run: ORB-SLAM3 was not rebuilt or launched because F16 is an
  isolated evaluation-lane script and does not modify the SLAM implementation.
  `hb_mapper` and RDK X5 board inference remain unavailable as recorded in F15.
- TODO: export and calibrate a native `640x544` BPU model before using this
  bag for an end-to-end latency comparison; the existing F15 ONNX artifact is
  fixed at `640x480`.
- TODO: a trajectory-level comparison requires a deliberately designed hybrid
  integration. This bag has no external ground truth, so ATE/RPE cannot be
  computed from the current evidence alone.

## Addendum 2026-07-30 — F17 Git storage cleanup

- Final state: `passing` for safe cleanup. `.git` decreased from about 87 GiB
  to 20 GiB, and `/home` ended at 159 GiB used with 124 GiB available. The
  remaining 19.07 GiB is recent unreachable data protected by Git's default
  recovery grace period; confirmed temporary-pack garbage is zero.
- Large-object cause: normal heads/remotes contained 0.467 GiB of blobs, while
  six `refs/codex` worktree snapshots contained 30.405 GiB. The largest blobs
  were rosbag files between roughly 2 and 5.6 GB each.
- Those six stale snapshot refs were removed. Codex later created one new
  current-session checkpoint with about 483 MiB of blobs and no ignored bags;
  most of its objects are shared with normal branch history.
- Main commands run:
  `git count-objects -vH`,
  `git rev-list --objects --branches --remotes`,
  `git for-each-ref ... refs/codex`,
  `git fsck --connectivity-only --no-progress`,
  six exact `git update-ref -d refs/codex/...` commands,
  `git gc`,
  removal of the five exact `tmp_pack_*` paths still reported as garbage,
  `git fsck --full --no-progress`,
  representative `git check-ignore -v` checks,
  `du -sh .git bags`, `df -h /home`, and `git status --short --branch`.
- Validation: both fsck passes exited 0. The final full pass reported dangling
  objects only, not missing or corrupt objects. `master` and `origin/master`
  stayed at `7f22eb93ef034449ef11d96440eee1b8bdae58d8`; the other local and remote
  branch refs were also unchanged. The index contained no staged changes.
- Ignore coverage added in `.gitignore`: `*.bag`, `/bags/`, `*.log`, `/logs/`,
  `/.ros_runtime/`, root trajectory outputs, `/.cache/`, `/.codex`, generated
  reports, `compile_commands.json`, `*.bak`, ROS-generated Python/message
  bindings, temporary dependency checkouts, `Thirdparty/DBoW3.zip`, and
  `Vocabulary/*.dbow3`.

## Addendum 2026-07-31 — F19 complete board frontend benchmark

- Final state: `passing`. The native-height INT8 model, full XFeat CPU/BPU
  frontend, ROS LAN transport, full bag replay, and aggregate comparison all
  completed successfully.
- Commands run included governance initialization reads/status, native
  `640x544` ONNX export and parity validation, 100-frame calibration
  extraction, OpenExplore `hb_mapper checker`/`makertbin`, SHA checks, `scp`
  deployment, board CMake build, two three-second ROS replay smoke tests, ROS
  node/subscription inspection, the complete 65.95-second replay, CSV transfer,
  summary generation, shell syntax checks, and Python syntax compilation.
- Runtime coverage: host ROS Noetic master at `192.168.1.3`, board ROS Noetic
  node at `192.168.1.10`, full 644-frame replay with the expected original
  sample indices, 130 stereo plus 65 temporal pairs per method, and 390 rows.
- Main result: XFeat median extraction 60.00 ms/image versus ORB 66.34 ms;
  mean extraction 64.85 versus 68.67 ms; mean stereo end-to-end 185.63 versus
  161.98 ms. XFeat retained substantially wider spatial coverage but returned
  fewer strict geometry inliers. Both methods passed the robust-pair rule on
  every sample.
- Touched files: `xfeat_rdk_x5/export_onnx.py`, native-height mapper config and
  evidence, `xfeat_rdk_x5/rdk_x5/`, `host_ros_lan.sh`,
  `deploy_board_benchmark.sh`, `summarize_board_benchmark.py`,
  `xfeat_rdk_x5/README.md`, `docs/xfeat-rdk-x5-board-benchmark.md`,
  `docs/xfeat-rdk-x5-research.md`, `docs/features.md`, `docs/validation.md`,
  `docs/progress.md`, and this handoff.
- Evidence:
  `xfeat_rdk_x5/artifacts/board_ros_benchmark_544x640/raw.csv` (SHA-256
  `25603920f73b2c408127eb661129c8ae772e3ad2be64d403812ce9877a03c5f1`),
  its `summary.json`, native-height ONNX validation/calibration manifests, and
  `docs/xfeat-rdk-x5-board-benchmark.md`.
- Commands not run: `./build.sh` and `./build_ros.sh` were not needed because
  F19 is isolated under the evaluation lane and does not modify ORB-SLAM3.
  The board benchmark was built natively rather than cross-compiled.
- TODO: trajectory-level XFeat versus ORB is still unverified. The current
  SLAM frontend cannot consume 64D float descriptors or use them with the ORB
  DBoW3 vocabulary, and this bag has no independent ground truth for ATE/RPE.
- Next recommended step: retain ORB temporal tracking/place recognition and
  prototype XFeat only as an auxiliary stereo/recovery source; then measure
  tracking-loss duration and map continuity on this bag before attempting a
  complete frontend replacement.

## Addendum 2026-07-31 — F20 parallel XFeat stereo frontend

- Final state: `passing`. A real BPU1 inference failed with DNN error
  `-6000001`; device/sysfs inspection confirms this board exposes only BPU0.
- Implemented two independent XFeat instances and tensor buffers, two stereo
  workers, one BPU0-only mutex, OpenCV CPU parallel loops, two independent
  parallel ORB instances, BPU queue timing, and stereo extraction wall timing.
- Board CMake build passed. Two-anchor CPU=1/4/8 replays selected eight workers.
  The full replay generated 390 expected rows with frames 0–641 represented.
- XFeat stereo extraction wall time is 57.78 ms versus parallel ORB 74.48 ms.
  XFeat stereo end-to-end is 125.46 ms versus 99.38 ms; serial-to-parallel
  XFeat end-to-end improvement is 32.4%. Temporal end-to-end is 153.22 versus
  169.40 ms.
- A field-by-field comparison of all 390 records against F19 found zero quality
  mismatches. The final CSV SHA-256 is
  `caba91000f6b3fd8a06fe3cc8f86ad55ea9a2c4275ed9e7e4033b36c48346359`.
- Touched files: `xfeat_rdk_x5/rdk_x5/xfeat_frontend.{hpp,cpp}`,
  `ros_benchmark.cpp`, `run_ros_benchmark.sh`,
  `summarize_board_benchmark.py`, XFeat README, F20 evidence, feature,
  validation, progress, report and handoff documents.
- Commands not run: top-level `./build.sh` and `./build_ros.sh` remain outside
  this isolated evaluation feature. The board-specific target was compiled and
  exercised directly on RDK X5.
- TODO: global 1024x1024 float matching and RANSAC now dominate stereo latency.
  Add rectified row/disparity candidate gating before considering batch=2 or
  image concatenation; spatial concatenation remains unsuitable because it
  introduces a false convolution boundary and does not create another BPU.
- Recovery event: ignored `bags/` files unexpectedly disappeared during Git
  maintenance. `git restore --source=61fa312b... --worktree -- bags` restored
  them from the fully validated dangling tree. A `diff` between tree entries
  and filesystem filenames/byte sizes passed for all 15 files; `bags/` is
  again 28 GiB and is reported as ignored. The exact trigger remains unknown;
  ordinary `git gc` is not expected to remove untracked worktree files.
- Commands not run: `git gc --prune=now` was rejected because it would
  irreversibly delete every unreachable local object, not only known rosbag
  snapshots. `./build.sh`, `./build_ros.sh`, and ROS runtime commands were not
  run because F17 changed repository hygiene/docs only.
- Touched files: `.gitignore`, `docs/features.md`, `docs/progress.md`, and
  `docs/session-handoff.md`. Git-internal snapshot refs and garbage objects
  were removed. Existing source/config changes were preserved.
- TODO: after the default recovery grace period, run ordinary `git gc` again
  to reclaim the remaining unreachable objects. If immediate reclamation is
  required, obtain explicit approval for the irreversible scope before using
  `git gc --prune=now`.

## Addendum 2026-08-03 — F21 XFeat* semi-dense Fine retest

- Active feature/state: F21 remains `active`; F22–F24 were not started.
- Implemented official-contract single/dual-scale host evaluation, selectable
  `sparse|semi_dense_single|semi_dense_dual` modes, target-scale coordinate
  refinement, M256/M384/M512 fixed Fine export/PTQ, board single-scale
  semi-dense decoding, online BPU Fine execution, and 9x9 patch-NCC reporting.
- Host parity: official single-scale refinement maximum coordinate difference
  was `3.05e-5 px`; dual-scale was `1.53e-5 px`.
- Host full-bag M384: Fine raised mean NCC-qualified inliers from `251.98` to
  `295.42`, inlier ratio from `0.659` to `0.811`, and mean patch-NCC median
  from `0.664` to `0.813`. M256 failed the quality target; M512 was slower and
  was not promoted over the quality-sufficient M384 candidate.
- Board full-bag M384: exactly 520 CSV rows, 130 stereo frames per method,
  65 temporal pairs per control method, and source frame range 0–641. Fine
  latency was `54.887 ms` mean / `59.367 ms` P90. It produced `376.50` mean
  strict inliers, `289.42` mean NCC-qualified inliers, `0.529` photometric
  grid coverage, and no fragile frame (minimum strict/photometric `330/176`).
  Against coarse, Fine added `41.12` NCC-qualified inliers (`+16.6%`).
- Same-process OpenCV ORB control: `88.699/101.427 ms` mean/P90, `286.28`
  photometric inliers, and `0.383` photometric coverage. It is not the
  production ORB-SLAM3 baseline and must not be used to mark F21 passing.
- Main commands included Python syntax checks; official parity and all-anchor
  evaluations; OpenExplore conversion/board DNN checks; board CMake build
  with `-j1`; ROS LAN node launch; full `rosbag play --delay=1`; `wc`, method
  count, frame-range and SHA checks; CSV download; summary generation; and
  `git diff --check`.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_semidense_single_m256_all.json`,
  `f21_semidense_single_m384_all.json`,
  `f21_semidense_single_m512_all.json`,
  `f21_semidense_m256_board_ncc_smoke_summary.json`,
  `f21_semidense_m384_board_ncc_smoke_summary.json`,
  `f21_semidense_m384_board_full.csv` (SHA-256
  `8b3b6d419082f266ceb5f9951fcecc1c991c2581cc30938b99fcdd74c62eed70`),
  and `f21_semidense_m384_board_full_summary.json`.
- Touched files:
  `xfeat_rdk_x5/evaluate_fine_matcher.py`,
  `xfeat_rdk_x5/rdk_x5/xfeat_frontend.{hpp,cpp}`,
  `xfeat_rdk_x5/rdk_x5/ros_benchmark.cpp`,
  `xfeat_rdk_x5/deploy_board_benchmark.sh`,
  `xfeat_rdk_x5/summarize_semidense_board.py`,
  `xfeat_rdk_x5/README.md`, and F21 governance/evidence files.
- Superseded blocker: a complete ARM64 archive was supplied later on
  2026-08-03; the cross-build and first production S316 replay are recorded in
  the following addendum. Five alternating runs, XFeat TrackStereo evidence,
  EuRoC ATE/RPE, and SLAM integration are still not complete.
- Next recommended step: fix coverage and the map-reset statistics snapshot,
  then repeat ORB/XFeat M384 in alternating order. Only if the formal 90%
  speed and 95% quality gates pass should F22 begin.

## Addendum 2026-08-03 — complete ARM64 image and production ORB run 1

- Archive `/home/ywl/temp/ros1_arm64_ubuntu22.04.tar` is complete and loaded
  as `ros1_arm64_ubuntu22.04:v3`; archive SHA-256 is
  `66a8b417b8f74dcce80029b6eabff9b143176abef716d58707775126b61c6354`.
- Modified `/home/ywl/Project/RDK_X5_orb_slam/tools/build_rdk_arm64_docker.sh`
  for the new image/entrypoint and ROS `set -u` compatibility. Added
  `Dockerfile.rdk-arm64-orb-build`, which installs only `libepoxy-dev`, and
  built local image `rdk-orb-slam3-arm64-build:v3`.
- Build result: `-j4` was OOM-killed at `Optimizer.cc` under the 9 GiB limit;
  `-j1` reused completed objects and passed both catkin packages. New hashes:
  `ros_stereo` `71198355...057e`, core library `cada3095...957e`.
- Staged, without overwriting existing deployment, under
  `/userdata/orb_slam/f21_crossbuild_staging_20260803`. Board `ldd`, hashes,
  S316/vocabulary loading, ROS initialization, full replay, and clean Shutdown
  all passed.
- Run 1 processed all 644 callbacks. With 30 warmups and one invalid
  map-reset row excluded: ORB extract 57.73 ms, stereo 13.24 ms, frontend
  70.96 ms mean / 81.76 ms P90, TrackStereo 88.15 ms, valid depths 215.51,
  depth coverage 0.755, and one 2.97 s loss segment.
- M384 Fine run-1 comparison: 55.05/59.37 ms mean/P90, so speed passes;
  strict count 375.4 passes, but coverage 0.580 is only 76.8% of ORB and fails.
  F21 remains active and F22 promotion remains false.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_orb_crossbuild_run1_manifest.json`,
  `f21_orb_production_run1.csv` (SHA-256 `dd4f0a7e...61f1`), and
  `f21_preliminary_production_gate_run1.json`.
- Unverified/remaining: final Cortex-A55-specific rebuild, safe stats snapshot
  across Atlas map reset, four additional alternating runs, XFeat SLAM
  tracking-loss comparison, and EuRoC trajectory metrics.

## Addendum 2026-07-31 — F18 XFeat INT8/BPU deployment

- Final state: `passing` for the fixed `1x1x480x640` XFeat BPU core.
- Toolchain: Docker image
  `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.6`, `hb_mapper 1.23.8`,
  HBDK 3.49.14, and `horizon-nn 1.0.6.3`.
- Calibration: 100 evenly spaced mono8 left-infrared frames from
  `2026-06-05-09-59-04.bag`, converted to normalized float32 tensors. The
  manifest is stored outside the tensor directory because `hb_mapper` reads
  every directory entry as calibration data.
- Conversion: `hb_mapper checker` placed all 33 reported nodes on one BPU
  subgraph. `hb_mapper makertbin` completed and produced a 920,712-byte model
  with SHA-256
  `0ff4a43f8b9085c56da2173969f9e42989c79cb6c296ce484d0eb7360a693dea`.
- Board: files and the compiled runner are under `/userdata/xfeat_rdk_x5/` on
  `root@192.168.1.10`. DNN 1.23.10 / HBRT 3.15.54 loaded the model and reported
  builder version 1.23.8. Fifty BPU0 inferences after ten warmups averaged
  13.473 ms (13.429–13.584 ms); model load took 125.450 ms.
- Accuracy: all three board outputs exactly matched `hb_mapper infer` on the
  quantized ONNX. Board-vs-float cosine was 0.995747 for dense descriptors,
  0.990869 for keypoint logits, and 0.997176 for reliability.
- Main commands run: `hb_mapper checker`, `hb_mapper makertbin`,
  `hb_model_info`, float/quantized `hb_mapper infer`, board CMake build, the
  repository DNN runner, SHA-256 checks, and `compare_ptq_outputs.py`.
- Touched files: `xfeat_rdk_x5/prepare_calibration.py`,
  `xfeat_rdk_x5/compare_ptq_outputs.py`,
  `xfeat_rdk_x5/config/xfeat_backbone_480x640_bayes_e.yaml`,
  `xfeat_rdk_x5/rdk_x5/`, `xfeat_rdk_x5/.gitignore`,
  `xfeat_rdk_x5/README.md`, `docs/xfeat-rdk-x5-research.md`,
  `docs/features.md`, `docs/validation.md`, and `docs/progress.md`.
- Evidence: `xfeat_rdk_x5/artifacts/ptq_board_validation_480x640.json` and
  `xfeat_rdk_x5/artifacts/calibration_bag_2026-06-05-09-59-04_480x640_manifest.json`.
- Not run: complete sparse XFeat postprocessing/matching on the board and an
  XFeat-integrated ORB-SLAM trajectory replay. The tested core resizes native
  `640x544` bag images to `640x480`.
- TODO: export and calibrate a native `640x544` core, implement/tune the CPU
  sparse postprocessing path, then measure full frontend latency and quantized
  stereo/temporal inliers before ORB-SLAM integration.

## Addendum 2026-08-03 — same-frame ORB/XFeat visualization

- Added `xfeat_rdk_x5/orb_slam3_stereo_dump.cpp`, its build wrapper, and
  `visualize_feature_distribution.py`. ORB uses the real
  `Frame::ComputeStereoMatches`; XFeat uses official `semi_dense_single`,
  top-k 600, M384, cosine 0.82, ratio 0.9, and Fine confidence 0.25.
- Bag frame 320 matches the production ORB CSV at 603 raw left points and 270
  valid depths. ORB covers 37/48 cells; XFeat* coarse/Fine cover 29/48. Fine
  changes median disparity from 0.0 to 3.00 px and filters 384 coarse
  candidates to 369 strict points, but cannot fill empty left-image cells.
- Evidence is under `xfeat_rdk_x5/artifacts/feature_visualization/`.

## Addendum 2026-08-03 — 09-56-06 bag feature visualization

- Active feature/state: F21 remains `active`; no later feature was started.
- Rendered frame 440 of `2026-06-05-09-56-06.bag` with the real ORB-SLAM3
  stereo path and official host-FP32 XFeat* semi-dense M384 coarse/Fine path.
- ORB: 605 raw left points, 224 valid stereo depths, 43/48 occupied cells.
  XFeat*: 600 raw points, 384 coarse candidates, 371 Fine-accepted candidates,
  and 20/48 occupied cells for both coarse and Fine.
- Evidence directory:
  `xfeat_rdk_x5/artifacts/feature_visualization_bag_2026-06-05-09-56-06/`.
- Verification completed: JSON parse, PNG type (`2880x1600 RGBA`), and SHA-256
  `d4126d5389282c42b271d114dd8a41d61ee6d0b5029a34bd0f0ed88a5a9e01d4`.
- TODO: repeat with board INT8 coordinate dumps before treating the host-FP32
  XFeat positions as exact BPU deployment evidence.

## Addendum 2026-08-03 — reliability and grid-quota implementation

- Active feature/state: F21 remains `active`; F22–F24 remain gated.
- Changed the nested official XFeat checkout's dense API to return selected
  reliability values. Host and board adapters now use the same score semantics.
- Added balanced extraction and match quotas. Defaults are 8x6 cells,
  automatic extraction maximum `ceil(top_k/48)` (13 for top-k 600), and eight
  coarse matches per cell. Zero disables the corresponding quota.
- Verification commands completed: Python compile, three regression tests,
  frame-440 visualization/JSON validation, five-anchor Fine smoke, two RDK-X5
  CMake builds, and final deployment under
  `/userdata/xfeat_rdk_x5/benchmark_build/`.
- Final board executable is 439,368 bytes with SHA-256
  `9c16d6d5d1ef2332af842fd816a8b57e4d885276c78e574f72e1f273ef1bcd42`.
- Frame 440 result: XFeat raw/coarse/Fine coverage is 48/48, 47/48, and 47/48;
  coarse/Fine strict counts are 325/285. Before this change coarse/Fine covered
  only 20/48 cells on the same frame.
- Evidence:
  `xfeat_rdk_x5/artifacts/feature_visualization_bag_2026-06-05-09-56-06_grid_quota_v3/`
  and `xfeat_rdk_x5/artifacts/f21_grid_quota_host_smoke_09-56-06.json`.
- Not run: final board bag replay. A six-second node start could not reach a
  ROS master at `localhost:11311`, so it does not count as runtime validation.
- Next: start the configured LAN ROS master, replay the full bag against the
  deployed binary, and compare board latency/NCC/coverage with the frozen ORB
  production baseline before changing the F21 state.

## Addendum 2026-08-03 — hard-grid full replay and tracking diagnostic

- Active feature/state: F21 remains `active`; F22–F24 remain gated.
- Completed the final board replay of the current hard-grid M384 binary over
  the LAN ROS master. The board produced all 520 expected records (130 each
  for XFeat coarse, XFeat Fine, and ORB stereo; 65 each for XFeat and ORB
  temporal), covering source frames 0–641. The fetched CSV SHA-256 is
  `eb59d7d63ed450a5742c9904b7e5727b0d28e7f8f655def3e93aec68c18850c2`.
- Board result: hard-grid Fine stayed fast at `54.53/61.27 ms` mean/P90, but
  NCC-qualified stereo correspondences fell from the pre-grid `289.42` to
  `186.03`, and temporal RANSAC inliers fell from `357.52` to `270.80`.
- Added and ran `xfeat_rdk_x5/evaluate_stereo_vo.py` over all 65 sampled frame
  pairs. It computes previous-frame stereo depth, temporal XFeat Fine
  refinement, and a shared PnP RANSAC for hard-grid XFeat, no-hard-grid XFeat,
  and an OpenCV ORB control.
- PnP result: ORB control `200.25` mean inliers, 65/65 success; no-hard-grid
  XFeat+Fine `213.0`, 65/65 success; hard-grid XFeat+Fine `99.38`, 64/65
  success. Frame 561 was fragile with 45 candidates and 24 inliers, while the
  same frame had 59 XFeat-global and 58 ORB inliers.
- Interpretation: strict round-robin selection confuses occupancy with useful
  information. It admits weak points from sparse cells and caps repeatable
  high-quality cells, so coverage rises while stereo photometric validity and
  temporal/PnP constraints fall. Fine Matcher itself remains viable because
  the no-hard-grid control meets this diagnostic.
- Verification commands completed: evaluator full run, `python3 -m py_compile
  xfeat_rdk_x5/evaluate_stereo_vo.py`, `python3 xfeat_rdk_x5/test_grid_quota.py
  -v` (3/3 passed), row/frame/timestamp and finite-value checks, SHA-256
  checks, and `git diff --check`.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_grid_quota_m384_board_full_20260803.csv`,
  `f21_grid_quota_m384_board_full_20260803_summary.json`,
  `f21_grid_quota_preliminary_gate_run1.json`, and
  `xfeat_rdk_x5/artifacts/f21_stereo_vo_full/{per_pair.csv,summary.json}`.
- Commands not run: no second production ORB replay and no EuRoC trajectory
  run were performed. The common-PnP ORB row is an OpenCV control and must not
  replace the existing real S316 production baseline.
- Next recommended step: replace hard equalization with a soft selector that
  first guarantees a small minimum per occupied cell, then globally fills the
  remaining budget by reliability while enforcing only a loose maximum. Tune
  it offline against stereo NCC and PnP, then rebuild/replay on the board only
  after it removes the frame-561 failure and reaches at least 95% of ORB
  quality. Do not start F22 before the five alternating production runs and
  trajectory gates also pass.

## Addendum 2026-08-03 — four-way frontend comparison

- Active feature/state: F21 remains `active`; F22–F24 remain gated.
- Extended `xfeat_rdk_x5/evaluate_stereo_vo.py` with a 600-point official
  sparse XFeat method that bypasses Fine Matcher and quotas. The previous
  three methods retain their behavior.
- Full four-way run produced 260 aligned finite data rows. Sparse XFeat had
  83.0 mean PnP inliers, 63/65 success, and two fragile pairs (frame 511: 9;
  frame 561: 6). No-hard-grid XFeat+Fine retained 213.0 mean inliers and 65/65
  success, versus the ORB control's 200.25 and 65/65.
- Verification: Python compilation, a two-pair four-way smoke run, full 65-pair
  run, row/frame/timestamp/finite checks, SHA-256 checks, and `git diff
  --check`. Evidence is in
  `xfeat_rdk_x5/artifacts/f21_stereo_vo_fourway/`; detailed comparison is
  `docs/xfeat-fourway-comparison.md`.
- The existing deployed sparse result used 1024 points and global matching;
  it remains valid as a deployment reference but is not conflated with this
  600-point local-matching PnP result.
- Next: retain the no-hard-grid M384 candidate, implement a softened quota only
  as an optional challenger, and proceed to SLAM/EuRoC gates only after the
  frontend result is reproduced in alternating board runs.

## Addendum 2026-08-03 — deployed sparse-600 profile

- Active feature/state: F21 remains `active`; F22–F24 remain gated.
- Updated `rdk_x5/run_ros_benchmark.sh`, `ros_benchmark.cpp`, and
  `xfeat_frontend.hpp` so the deployed sparse/no-Fine control defaults to 600
  features. The run profile also uses a 600 coarse-match limit, no Fine model,
  and zero extraction/match grid quotas.
- `bash -n` and `git diff --check` passed. The board native CMake build reached
  100%, and the deployed program printed the exact expected runtime parameters.
- One-anchor replay wrote six data rows. Every XFeat row reported 600/600
  keypoints; Fine timings were zero. The temporal row recorded 429 matches,
  383 RANSAC inliers, and 104.09 ms end-to-end.
- Evidence: `xfeat_rdk_x5/artifacts/f21_sparse600_board_smoke.csv` (SHA-256
  `473263d93c84c6f36e504a1698dacef56b61de7390d1a68fd0c71efbaaf8546f`).
  Deployed binary/script hashes are `3fa4f964...adb98` and
  `6b961c23...83fd` respectively. No benchmark process was left running.
- Not run: a complete 65-pair/full-bag sparse-600 board benchmark. The smoke
  proves the deployed feature budget and profile, not its aggregate latency or
  quality distribution.

## Addendum 2026-08-03 — F21 formal gate passing

- Active feature final state: F21 is now `passing`. No F22 code was started;
  F22 remains `not_started` and is no longer gated by F21.
- Frozen candidate: `xfeat_rdk_x5/config/f21_xfeat_star_m384.yaml` with
  XFeat* single scale, 600 points, extraction quota `-1`, match quota `16`,
  M384 Fine, confidence `0.20`, cosine `0.82`, and eight CPU workers on BPU0.
- Five alternating production runs completed. ORB recorded 644 callbacks in
  every run; XFeat recorded all 520 expected rows in every run. Combined
  frontend mean/P90 was ORB `70.505/81.827 ms` and XFeat `54.633/59.402 ms`.
- Common real-S316-ORB/XFeat NCC+PnP verification passed every gate; both had
  65/65 PnP success and zero fragile pairs. The formal machine result is
  `xfeat_rdk_x5/artifacts/f21_gate_final.json` with SHA-256
  `f07e1a445443015fb8846b7ab406379295e03f860457194bbc092f6dade6acba`.
- Cortex-A55 cross-build passed. Board staging hashes are `c15c99cb...ebe4`
  for `ros_stereo` and `26b833f0...f836` for `liborb_slam3_ros.so`; no
  production file was overwritten. Source backups are in
  `/home/ywl/Project/RDK_X5_orb_slam/f21_backups/20260803_gate_close/`.
- Two sequential 1000-iteration Fine runs passed with identical output hash
  and stable peak RSS. No `ros_stereo` or `xfeat_ros_benchmark` process was
  left on the board.
- Commands verified: Python compilation, helper build, threshold-0.20 PTQ
  comparison, common 65-pair estimator, five-run gate summarization, shell
  syntax, `git diff --check`, board hashes/`ldd`, and process cleanup.
- Not run by design: F22 hybrid integration, F23 EuRoC ATE/RPE, and F24 XFeat
  vocabulary/relocalization/Atlas compatibility. Next recommended step is F22
  hybrid stereo-depth integration using the frozen F21 profile.

## Addendum 2026-08-04 — feature-count sensitivity after F21

- Active feature/state: no implementation feature was activated. F21 remains
  `passing` with its frozen 600-point M384 profile; F22 remains `not_started`.
- Ran the common 65-pair XFeat tracking diagnostic at M384 for top-k
  576/560/512/480/384/320, plus 576 with M256. Outputs are under
  `xfeat_rdk_x5/artifacts/feature_count_sweep/`.
- M384 top-k 576 and 560 both returned 65/65 PnP success and zero fragile
  pairs, with minimum inliers 35 and 31. Top-k 512 and below failed the
  existing no-fragile-pair requirement. M256 at top-k 576 also failed with
  64/65 success and a 24-inlier minimum.
- No build, board replay, source modification, or deployment was performed.
  Before considering 576 M384, run a complete board replay and compare
  tracking loss, P90 latency and tail coverage against frozen 600 M384. Do not
  adopt 560 based on the current one-inlier worst-case margin.

## Addendum 2026-08-04 — frame 561 top-k 512 diagnosis

- Active feature/state remains unchanged: F21 `passing`, F22 `not_started`.
- Exported bag frames 560/561 and reconstructed the same XFeat* M384 pipeline
  for top-k 600 and 512. The 512-point solver returned a plausible pose but
  only 28 inliers, below the 30-inlier diagnostic gate; this is a gate failure,
  not a numerical PnP failure or proof that integrated ORB-SLAM3 must lose.
- The top-k reduction removed 14 of the 600-profile's 70 PnP candidates and 7
  of its 37 PnP inliers directly. Subsequent consensus changes produced the
  final `70/37 -> 59/28` candidate/inlier reduction. Dynamic people, repeated
  texture and weakly distributed static geometry make this a tail-risk frame.
- 500 RANSAC iterations did not improve the result. A 2.5 px threshold reached
  33 inliers but materially changed the estimated motion, so thresholds were
  not modified. No source, build, deployment, or frozen configuration changed.
- Evidence: `xfeat_rdk_x5/artifacts/feature_count_sweep/frame561_analysis/`.

## Addendum 2026-08-06 — branch publication verification

- Active feature/state remains unchanged: F21 is `passing`; F22 is
  `not_started`. This session packaged the existing ROS pose-continuity and
  XFeat/RDK X5 work for publication and did not start F22 implementation.
- Added the missing direct `std_srvs` rosbuild dependency required by the
  `/pose_correction/set_enabled` service in `ros_stereo.cc`.
- Verification completed successfully with `./build.sh`, `./build_ros.sh`,
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages
  python3 xfeat_rdk_x5/test_grid_quota.py -v` (3/3 passed), and `bash -n` for
  the staged shell helpers. The ROS build linked `Stereo` successfully.
- Staged-scope checks covered 215 files (about 63.2 MB); the largest file was
  4.9 MB, and a credential-pattern scan found no secrets. Fifteen generated
  CSV evidence files retain their original line endings, so `git diff
  --check` reports trailing whitespace for those data files only.
- The generated `.ros_runtime_11312/` PID directory was intentionally left
  untracked and excluded from publication.
- Not run: no live camera, rosbag replay, RDK X5 board session, loop-closure,
  map-merge, or tracking-loss runtime validation was repeated. Existing
  runtime TODOs for `/robot_pose`, `/robot_pose_map`, TF continuity, and the
  correction service remain applicable.
- Publication target: branch
  `agent/ros-pose-continuity-xfeat-rdk-evaluation` based on `master`.

## Addendum 2026-08-06 — F25 stereo local-map gate diagnostic

- Active feature/final state: F25 `passing` as a temporary diagnostic. The
  tested threshold remains isolated and is not a production recommendation.
- Changed the RDK cross-build source at
  `/home/ywl/Project/RDK_X5_orb_slam/src/orb_slam3_ros-master/orb_slam3/src/Tracking.cc`:
  the pure-stereo local-map gate is 25, inertial gates are unchanged, and
  pose-step/reprojection diagnostics run only near the gate or on a large pose
  step. No source under this repository's `src/` or `include/` was changed.
- The user ran `RDK_BUILD_JOBS=4 ./tools/build_rdk_arm64_docker.sh`; both catkin
  packages passed. The ARM64 node/library hashes are `d6df0d5e...f71ccb` and
  `3f4de91f...e085c54f`. Deployment used the independent board directory
  `/userdata/orb_slam/f25_stereo_gate25_sparse_20260806` and did not replace
  the previous staging or `/userdata/orb_slam/devel` artifacts.
- Runtime controls: camera PID 39105 and all its threads were temporarily
  pinned to CPUs 0-1; ORB-SLAM3 was pinned to CPUs 2-7; Pangolin was disabled;
  bags were played at 1.0x from host `192.168.1.3`; left/right images were
  remapped beneath isolated `/baggate25_sparse_*` topics. Camera affinity was
  restored to CPUs 0-7 and no test `ros_stereo` process remained afterward.
- `09-59-04` passed in one map: 643 state-2 callbacks, 135 KFs and 2822 MPs.
  Frame 523 was accepted at exactly 25 local-map inliers with 0.062627 m,
  1.613700 deg, and 1.264135/2.374674 px reprojection mean/P90. Atlas:
  `ORB_SLAM3_Map_1786001967.955328.osa` on the board.
- `09-56-06` produced 853 state-2 plus 30 state-3 callbacks and split into
  74-KF and 61-KF maps. Frame 432 failed before the local-map gate because the
  reference KF produced only 9 BoW matches; frame 431 still had 50 local-map
  inliers. Atlas: `ORB_SLAM3_Map_1786001728.921163.osa` on the board.
- Evidence copied to the host:
  `/tmp/orb_slam3_095606_rate1_camera_cpuiso_gate25_sparse.csv`,
  `/tmp/orb_slam3_095606_gate25_sparse_diag.log`,
  `/tmp/orb_slam3_095904_rate1_camera_cpuiso_gate25_sparse.csv`, and
  `/tmp/orb_slam3_095904_gate25_sparse_diag.log`. Exact SHA-256 values are in
  `docs/validation.md`.
- Commands not run: no production deployment, install-space build, IMU mode,
  or viewer-enabled replay was performed. The local RDK diagnostic source is
  still at 25 for follow-up experiments; restore it to 30 before using that
  workspace for a production build.
- Remaining uncertainty: one replay per final sparse-logging configuration is
  insufficient to separate threshold effects from asynchronous LocalMapping
  scheduling. The next meaningful step is an alternating repeated A/B test of
  threshold 30 and 25 from separately staged, otherwise identical libraries.

## Addendum 2026-08-10 — F22 Hybrid XFeat stereo depth passing

- Active feature/final state: F22 `passing`. F23 remains `not_started` and is
  now unblocked. The RDK target used was `root@192.168.1.10`.
- Implemented a provider seam that refines only stereo depth on existing ORB
  keypoints. ORB binary descriptors and all temporal/MapPoint/BoW/loop paths
  remain unchanged. The provider uses the frozen F21 600-point backbone and
  M384 Fine model, patch NCC acceptance, add-only default behavior and guarded
  replacement.
- Verification passed: host `./build.sh`; standalone frontend/provider syntax
  compilation; shell/Python checks; ARM64 catkin cross-build at `-j1`; board
  `ldd`, hashes and model-init smoke; and six complete board runs over the
  three user-provided lawn bags at controlled 0.5x playback.
- The third bag was unindexed. Its original at the repository root was left
  byte-for-byte untouched; `rosbag reindex` was run only on
  `/tmp/f22_2026-08-06-09-50-14.bag`.
- On all 2,496 paired timestamps, ORB keypoint counts were identical and no
  frame lost valid stereo depth. Hybrid added 16.13-18.39 mean depths and
  13.88-18.79 mean tracked inliers. Loss frames were equal on the first two
  bags and improved from 34 to zero on the third.
- Cross-build artifacts are isolated at
  `/userdata/orb_slam/f22_hybrid_20260810`; node/library SHA-256 values are
  `5a507a46...1eaf9b` and `56987b52...7766f9`. Existing devel, F21 and F25
  staging artifacts were not replaced.
- Evidence is under `xfeat_rdk_x5/artifacts/f22_hybrid_ab/`; the authoritative
  aggregate is `summary.json`. No F22 `ros_stereo` process remained after the
  run. The existing host ROS master was reused and left running.
- Touched current-repository files: `include/StereoDepthProvider.h`,
  `include/{Frame,System,Tracking}.h`, `src/{Frame,System,Tracking}.cc`, the
  Hybrid provider, deploy/replay/summary helpers under `xfeat_rdk_x5/`, F22
  artifacts, and the four project progress/validation documents. The RDK
  build workspace additionally changed its package `CMakeLists.txt`,
  `src/ros_stereo.cc`, `config/Stereo/S316.yaml`, mirrored core headers/sources,
  and `tools/build_rdk_arm64_docker.sh`; a minimal board-derived BPU link
  sysroot is under `tools/rdk_bpu_sysroot/` there.
- Remaining limitation: Hybrid callback mean is 155.92-162.41 ms, about 53 ms
  slower than ORB, so it cannot sustain the 11 Hz S316 stream. This is expected
  for a bridge that runs both extractors. The next step is F23 descriptor-aware
  tracking, beginning with an explicit feature/descriptor abstraction and
  initialization path before touching relocalization or vocabulary handling.
- Not run: 1.0x/11 Hz Hybrid replay, EuRoC MH_01/MH_03/MH_05 ATE/RPE, XFeat
  temporal descriptor integration, XFeat vocabulary, relocalization, loop
  closing, or Atlas model-tag compatibility. Those belong to F23/F24 and must
  not be inferred from the passing F22 depth-only bridge.
