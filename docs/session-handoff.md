# Session Handoff

## RD47 completed — 2026-09-12

Wrapper now preserves ROS MessageEvent receipt time, callback wall/steady, input
lock acquisition and receive-body end in bounded input_transport.csv (20000 rows,
1.37MiB). Both direct/FIFO supported; default-off diagnostic_receive_only uses
same C++ reception without creating System. Core library reused byte-identically.
New node SHA256 c1d386b22024dd4609b4a254ba6366c1c93036e89b7bf87b111ade457cd12208.
Deployment /userdata/rgbd_transport_timing_20260912/deploy_runtime.
Full receive-only2910/2910; full SLAM received2910/processed2899 (11 FIFO drops),
non-OK0/map1; direct8s89/89. All diagnostics/drain/save/exit checks pass.
seq7582 callback skew502.264ms, ROS receipt skew502.186ms: almost all before
subscription/deserialization, but publisher/TCP/socket cause not yet separated.
Historical1.5s not reproduced. Maps archived/checksummed, no reload; all owned
processes stopped. See outer experiments/rgbd_transport_timing_20260912/report.md.
Code remains uncommitted for review. Older source_manifest refers to pre-RD47
wrapper; use new manifest.json for this build.

## RD44–RD46 completed — 2026-09-11

Prior pending RGB-D source/docs committed before edits:outer18ba684/core4d6df6a.
Task workspace is the outer rgbd_track_timing repository and this core worktree.
Do not build from the separate XFeat worktree or touch its user modifications.

Implemented opt-in exact pairing/complete-frame FIFO, bounded image payload and
scalar diagnostics, private receiver queue, serial main-thread SLAM and controls,
synchronous stop/drain service and safe logging after receiver join. Defaults
remain direct. Host state-machine sanitizers and random delivery tests passed;
Docker ARM64 cross build and board component parity passed. No binfmt registered.
Core changes only LocalMapping timing-row cap; algorithm and headers unchanged.

Nine full1x replays:direct sync10 A1/A2 processed2908/2905,maps1/2; direct sync16
B1/B2/B3 processed2889/2897/2892,all tracking OK,one map. FIFO8 C1 received every
image but capacity dropped6 complete pairs. FIFO16 three repeats received every
image,processed2901/2910/2910,non-OK35/0/0,maps2/1/1. The failing repeat hit1500ms
pair timeout for7238–7246 (actual stream skew1510–1616ms). FIFO16 maximum wait
1.25–1.42s. No stable production recommendation; no further parameter expansion.
All9 processed historical7112–7132,so that exact old failure was not reproduced.
A2 lost at7909 before its first missing7914; avoid claiming all loss follows gaps.

New independent board deployment:/userdata/rgbd_fifo_20260911/deploy_runtime.
Node f0f5c159b9e973a21e35a3575290172991fabbcdcac56bdeb64c19b8c86c2607;
core c1258675ee591f67948aa12cb76677a8742f7bc0cb5684f420035a4f2e3dcb00;
YAML bbd550b26de4225e8988dc10681c5d1d921405d8976eac0372df58405db6d44f.
Sources and build images in outer experiments/rgbd_fifo_20260911/source_manifest.json.
Read conclusion.md,report.md,README.md,design.md there. Raw timings,resource logs,
plots,trajectories and maps are in runs/*; map downloads were SHA-verified before
only their new board copies were removed. No map reload validation. Atlas backup
entry duplication and stale failure-frame inliers are documented,not modified.

All formal nodes exited0 with empty new-mode input and no new OOM. Frozen
production preserved. New implementation remains uncommitted for review; only
pre-change commits were authorized/requested. Additional lifecycle/final
verification evidence is in lifecycle_validation.json and final_verification.json.


## RD43 deployment package completed — 2026-09-09

User requested a portable700-feature queue8 version for another board.
Archive: outer packages/rdk_x5_rgbd700_q8_20260909.tar.gz with .sha256 sidecar.
Package inputs/evidence: packages/rgbd700_q8_20260909, including recipe/,
collect_runtime.py, release.json and package_validation.json. Includes original
node/core, vocabulary, private ARM64 loader and library closure, Chinese guide,
board check/run scripts and host replay/save/stop tools. No board ROS setup
is needed; host ROS1 Noetic and original bag are separate prerequisites.

Validated on the existing RDK in a new isolated directory using package scripts:
225 shared objects loaded only from package lib/, component parity passed,
full2128-pair replay hadzero non-OK frames andone map; Atlas/trajectories saved,
exit0. This is relocation/integration validation, not a physical new-board test.
Target Ubuntu22.04 ARM64/RDK X5. Production unchanged; no source algorithm
edits or Git commit. Package tests and owned11332 master have ended.


## RD42 parameter followup completed — 2026-09-09

The user authorized feature/queue tests on the existing level 4 optimized
build. No algorithm edits, rebuild or Git commits were made. Evidence is in
outer experiments/rgbd_tuning_20260909_0938: read conclusion.md, report.md,
run_manifest.json and final_verification.json.

Nine full 1x host-bag runs completed. Screening compared 700/600/500 features
with queue 1, and 700 features with queues 4/8. The 700-feature queue 1/8
configurations each received three interleaved runs. All provenance and exit
checks passed. Queue 8 delivered all 2128 pairs with zero tracking loss and
one map in all three runs; queue 1 missed 4/21/36 pairs, with loss episodes
0/1/1 and map counts 1/2/2. The 600/500 single runs missed 14/16 pairs and
both lost tracking, with lower online support and more saved keyframes.

Retain 700 features, 5 levels and 3 workers. The strongest complete-input
candidate on this bag is RGB/depth queues 8/8, sync 10, level 4, viewer off.
No production defaults were changed. Queue 8 median mean/P95/P99 was
70.04/89.45/103.76 ms. Its third run had P95 93.28 ms and maximum relative
schedule backlog 674.30 ms before draining. Input continuity passed 3/3 on
this bag, but strict low-latency and P99 <90 ms acceptance did not pass.
No external ATE/RPE, broader-load or longer-map validation is claimed.

All experiment runs and the owned 11332 master ended; unrelated 11311 was
preserved. Production/optimized binaries and the original bag are unchanged.
No bag was copied to the board. Further work, if requested, should target
callback/dispatch tails; deeper queues do not make computation faster.

## RGB-D exact optimizations completed — 2026-09-08

Latest user request authorized implementation, Docker build and board tests,
with a commit of existing changes before runtime edits. Pre-change commits:
outer b838bcb, core eb7aee3. New implementation/evidence remain uncommitted for
review; the original prohibition on unsolicited later commits is respected.

Implemented RD37–RD40: pure RGB-D opt-in `RGBD.ExactOptimizationLevel` 0–4
(default 0): grid scratch/references, exact ARM64 ORB32 Hamming, locked
caller-storage descriptor snapshots, headless display-cache gating.
ORBextractor.cc and Optimizer.cc are byte-identical to the pre-change commit.

Docker v3 ARM64 build passed; core SHA
c41d2d0ba1ca2ae0f61d70207fae0bfb11ff44d17eb3a9162154bf287c6945d1;
node SHA2797d3fd...6239a. Three jobs, 12 GiB final memory cap. Temporary arm64
binfmt was removed after compilation. Board component parity passed: 6,000
distance cases, 49,344 queries, 96 matcher and exactly equal pose cases.

RD41 completed 10 benchmark runs plus one separate perf run. L0/L4 each had
three interleaved repeats. Median per-run callback mean/P95/P99:
L0 76.59/95.90/110.81 ms; L4 70.20/89.35/103.20 ms. L4 missing input 9/24/15;
loss episodes 0/0/2; maps 1/1/2. Thus measurable improvement, but no run met
complete input + no loss + one map, and no L4 P99 was below90ms. Do not
claim production acceptance or universal precision non-regression. Default
remains0; keep frozen production. No external trajectory truth was available.

Read outer `experiments/rgbd_exact_20260908_1901/conclusion.md`, `report.md`,
`implementation.md`, `run_manifest.json`, `validation.json`, and source/binary
manifests. All raw runs, trajectories, saved maps and perf data remain local.
Independent deployment: /userdata/orb_slam/rgbd_exact_20260908_1901/deploy.
Production and bag hashes were verified unchanged; no experiment SLAM/master
or build container remains. No bag was copied to the board. XFeat worktree
user modifications remain untouched.

Further work, if requested, should measure local-map substages and callback-
external dispatch costs; self perf samples cannot attribute waiting or libc
costs to callers. Memory pressure exists late in the bag, but early no-swap
frames also exceed90ms. No threshold/feature/map-scope reductions were made.
The historical stop-testing direction below was superseded only for this
explicit implementation/build/test task, now completed.

## RGB-D callback code review completed — 2026-09-08

The user requested deeper acceleration analysis without compromising extraction,
matching or tracking quality. Read `docs/rgbd-callback-optimization-review.md`.
Analysis artifacts are in outer `experiments/rgbd_callback_review_20260908/`:
18 existing runs / 37,905 callbacks, source hashes, frozen ARM64 disassembly,
per-run tail statistics and explicitly hypothetical arithmetic sensitivity.
No new board replay, SLAM build, deployment or runtime source edit was performed.

Frozen binary confirms per-byte Hamming popcount, grid copies/allocations,
MapPoint descriptor clone and FrameDrawer updates. First candidates: preserve
candidate order with grid references and reusable scratch; exact 32-byte ORB
SIMD distance; locked descriptor snapshot into caller storage; headless-only
visualization cache gating. No threshold, feature-count, optimization-iteration
or map-selection reductions. Local-map substage costs are still unmeasured;
no speedup or P99 target is claimed achieved. Future implementation must verify
fixed-snapshot output equivalence and asynchronous mapping quality separately.
RD37 implementation remains not_started. Existing RD36 stop-testing direction
and frozen deployment protections remain in effect.

## RD36 completed diagnostic study — 2026-09-08

Completed 18 full recovered-bag replays in the owning outer workspace. Evidence: `../experiments/rgbd_queue_20260908_1518/`; read `conclusion.md`, `report.md`, `matrix.json` and `run_manifest.json`. The user requested convergence on existing results: no more parameter tests were started after the already-running final 8/8/20 replay. Prepared but unexecuted runs are excluded.

Increasing subscription capacity alone is not a reliable, sufficient repair. Production 700 and 500 each lost tracking in 2/3 runs. Diagnostic 500/4/4/10 missed 1/0/31 pairs across three runs and lost tracking in two. B700/8/8/10 lost 28 pairs after both streams were dequeued: ExactTime trace replay reproduces capacity eviction. Separately, C500/4/4/10 r1 lost tracking at seq761 before its only missing input at1199. Thus input completeness alone cannot ensure stable tracking. No production configuration passed repeated complete-input/single-map validation. Final 700/8/8/20 missed one pair without tracking loss, but is one observation with substantial relative latency variation, not a validated recommendation.

The diagnostic wrapper was built in local ARM64 Docker and ABI-checked against frozen core (same 12 ORB symbol references, System size1888, timing size1424 and offsets). Node SHA2797d3fd...6239a; library51c3e4d7...a2860. All 18 SLAM/player exits were zero; the first controller's collection error was recovered with rsync and preserved in evidence. Validation passed measurement/provenance invariants, not runtime correctness. Frozen production and bag hashes are unchanged. Experiment processes and master11332 ended; unrelated master11311 remains. User-approved temporary arm64 binfmt was removed after compilation. No Git commit, core algorithm change, or original XFeat user-edit modification.

No further replay is pending in this task. Future work, if requested, should locate pre-subscription reception/eviction and two-stream dispatch skew, and explain tracking failures with complete input. Source patches and final board/host checks are in `provenance/`. Historical sections below describe earlier work and are not the current task status.

## Summary

- Active feature: BR01; final state: `passing` for branch/source/archive separation.
- Scope: organize existing changes into the requested branches and archive
  generated evidence. See `docs/branch-layout.md` for exact ownership and bases.
- The original branch tips and frozen board production binaries are preserved.
- Next recommended step: Continue RGB-D runtime diagnostics here with the same-named outer ROS/build branch; explicitly choose a validated deployment before a board test.

## Validation and evidence

- Pre-change snapshots: `backups/` and `backup_manifest.json` in the local
  `20260907_git_reorganization` archive.
- Archive move mapping and fingerprints: `artifact_moves.json` and
  `artifact_manifest.json`; all 7,626 records verified after relocation.
- Descriptor, syntax, source-preservation, ancestry, ignore and status checks:
  `validation.json` in the same archive. Descriptor checks passed; all 9 shell
  scripts and 10 Python modules passed syntax checks. Build-path defaults and
  explicit overrides resolved to the expected source/workspace.
- No host/ARM full build, board replay, new ATE/RPE gate or deployment was run
  during this organization. C++ logic is inherited or preserved byte-for-byte;
  build/data-root routing was adjusted and checked separately.

## Files and remaining limitations

- Branch routing, feature list, progress, validation and handoff documentation;
  ignore rules; branch-owned tests or runtime scripts/core fixes.
- The outer ROS/build repository has its own same-named runtime branch.
- TODO: F23 full-lawn tracking and the required EuRoC gate remain blocked.
- TODO: archived board evidence needs to be supplied separately in a fresh clone.
- The local runtime core includes changes absent from the frozen production
  library. A clean Git branch does not mean those changes are deployed.

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

## Archived RD35 handoff before branch separation


### Summary

- Active feature: RD35
- Final state: `passing`.
- Main result: the 3163-pair ultra-long host bag completed a cold-start 1.0x
  LAN replay. The board processed 3136 callbacks and 741 KFs across five maps,
  exited cleanly, and showed no kernel OOM or allocation failure. Peak RSS was
  923.6 MiB; no loop/GBA occurred, so the reported 1000+ single-map failure
  remains outside this run's coverage.

### Commands Run

```text
F33_RUN_PREFIX=f35_long154955 F33_CONFIG_ROOT=<RD33 configs> \
  F33_ARTIFACTS=<RD35 root> F33_REMOTE_ROOT=<RD35 board root> \
  F33_BAG=2026-08-25-15-49-55.bag \
  ./evaluation/run_f33_rgbd_orb_one.sh f700_l5 1
python3 evaluation/analyze_f35_rgbd_oom.py \
  evaluation/artifacts/f35_rgbd_oom_20260825_154955 \
  2026-08-25-15-49-55.bag
roscore -p 11312 with ROS_IP=192.168.1.3
rosbag play --rate=1.0 <host bag> <unique RGB/depth LAN remaps>
ssh/scp to launch the production board binary and retrieve timing only
sha256sum 2026-08-25-15-49-55.bag
bash -n evaluation/run_f33_rgbd_orb_one.sh
git diff --check
```

### Commands Not Run

- `./build.sh` and `./build_ros.sh` were not run because no SLAM source or
  binary changed; the previously verified production ARM64 hashes were checked
  in every run.
- ATE/RPE was not run because the bag has no external trajectory ground truth.
- A second replay or a forced 1000+ single-map workload was not run; the
  requested bag produced only 741 KFs and fragmented into five maps.

### Touched Files

- `docs/features.md`, `docs/progress.md`, `docs/validation.md`, and this file
- `evaluation/run_f33_rgbd_orb_one.sh` (OOM telemetry and system snapshots)
- `evaluation/analyze_f35_rgbd_oom.py`
- `evaluation/artifacts/f35_rgbd_oom_20260825_154955/`
- No SLAM source, build artifact, vocabulary, or deployed production binary was
  modified.

### Evidence Paths

- `evaluation/artifacts/f35_rgbd_oom_20260825_154955/report.md`
- `evaluation/artifacts/f35_rgbd_oom_20260825_154955/summary.json`
- `memory_pressure.png` and raw timing/log/pose/tracking/telemetry plus
  before/after kernel snapshots beneath the same root. Board results are under
  `/userdata/orb_slam/rgbd_long_154955_20260829/`; no bag exists there.

### Blockers and Assumptions

- The bag has no external ground truth, so absolute map accuracy is unverified.
- This run had zero loop/merge events and no GBA. It therefore cannot validate
  the suspected `vpKFs.size() * vpMP.size()` reserve behavior at a 1000+ KF
  single-map loop closure.
- RSS growth of about 0.590 MiB per processed KF is an observed workload slope,
  not proof of a leak. Atlas intentionally retains all five maps, and memory
  returned to baseline after node exit.
- Twenty-seven paired inputs were lost in two overload clusters; the run is a
  valid memory stress observation but not a complete-delivery mapping result.

### Next Recommended Step

- Reproduce the failure with a workload that keeps one map beyond 1000 KFs and
  actually closes a loop, while retaining the RD35 commit/RSS/swap telemetry and
  capturing `std::terminate`/exception output. That is the discriminating test
  for the suspected Global BA allocation failure; merely replaying this
  fragmented no-loop bag longer is not equivalent.

## 2026-09-09 legacy-format RGB-D700/q8 package

Per user request, rebuilt as `packages/rdk_x5_rgbd700_q8_orb_slam_20260909.tar.xz`.
Archive member paths exactly match the 20260819 `orb_slam/` package, including
three scripts and config/S316.yaml. Uses board ROS/OpenCV runtime dependencies,
original camera helper and default camera topics. Node/core/YAML are unchanged
from validated 700/q8/L4 artifacts; queue flags are explicit in legacy launcher.
Extracted final archive in an isolated existing-board directory and completed
1.0x full bag:2121/2128 (7 missing),zero non-OK,one saved map,exit0; bundled
core/DBoW3/g2o loading verified. Deployment works but zero-missing acceptance
FAILED on this replay. Do not infer packaging/system libraries caused the gaps.
No live-camera/new-physical-board test. Read deployment requirements in
packages/rgbd700_q8_legacy_20260909/recipe/README_DEPLOY.md; evidence in its
package_validation.json and layout_validation.json. Previous tar.gz retained.
Production was not modified; no Git commit.

## 2026-09-11 new bag board test

New 2026-09-10-11-35-57.bag, legacy700/q8/L4 package:2876/2910,34 missing,
21 consecutive sync-stage drops at7112–7132 despite both streams dequeued;
7133 lost,7168 initialized new map,2 live maps. P95/P99=98.19/118.08ms.
RSS peak sampled1048.97MiB,Swap284.54MiB incl saving; no new OOM,SLAM exit0.
Repeated PreSave appends backup entries (2 live maps,4 backup entries at
second save); archive reload untested. Production hashes verified unchanged.
Evidence: experiments/bag_20260910_113557_board_20260911/report.md.
No algorithm changes, no Git commit, test processes ended.

### Failure-image followup
101 frames7090–7190 extracted. Runtime-source host ORB: adjacent7111→7112
187 matches/154H vs jump7111→7133 41/32, no0.2–4m matched support on jump.
Bag stream skew~11ms vs board post-dequeue~850–977ms. ExactTime trace replay
capacity10 matches actual missing set;16 recovers21 critical pairs on fixed
trace only,13 upstream missing remain. Failure-frame inliers140 is stale
mnMatchesInliers when TrackLocalMap skipped. No board retest/source edits.
See experiments/bag_20260910_113557_board_20260911/failure_images/report.md.


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
