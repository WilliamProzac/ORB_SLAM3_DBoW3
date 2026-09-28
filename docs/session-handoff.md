# GF03 current handoff — 2026-09-28 (active)

User authorized evaluation of all existing GF modules individually and in combination. Exactly one active feature GF03. No unsolicited subagents.

- Independent root: `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_modules_20260928`. Source branch `repro/module-factorial-20260928`, current source `1ad2246`, parent GF02 `33e5064`; official base `2a60498`. Main checkpoint `39e5ebaf` (initially clean `e4902eaa`).
- Eight modes baseline/f/h/g/fh/fg/hg/fhg. F existing GFM including skip-local and KF insertion behavior, 400 scalar constraints and 800 extracted ORB features. H legacy bucket MIH+OTS with inherited common synchronization, not paper bounded/all-descriptor policy. G existing fixed30/candidates60 Good Graph; no dynamic anticipation/budget or unfinished stereo SLAM++ variant. 2-D graph selection information also on RGB-D; downstream g2o BA keeps depth constraints. No loop, GPU, external prior, viewer or map saves.
- SLAM++ base dependencies built; hard-coded Eigen/Armadillo discovery corrected in source. Harness links system BLAS/LAPACK instead of legacy mixed-Eigen wrappers. Failed build logs preserved. Actual F active/skip counters added; G selection already logs triggers/completion.
- Main eight-target build running in `logs/build.log`; baseline/f/h/g/fh/fg complete at this handoff; hg/fhg pending. All native dependency targets passed. Main exec session 44640. Do not benchmark formal timing concurrently with builds.
- Functional short runs completed: baseline/f/h EuRoC 400 each, G EuRoC800 (22 actual selections), baseline/f/h/g TUM400 each; all complete, zero non-OK after init. `smoke_summary_initial.json`. No formal performance claim.
- Functional runner freeze guard stopped `smoke_single_tum` after baseline/f when replay.cc was temporarily changed. Old source restored and archived by SHA in provenance; h/g completed as smoke_single_tum_hg. `logs/functional_freeze_note.txt`. New driver with schedule lateness held at `harness/replay_next.cc`; apply after all functional runners/build finish, rebuild every executable, then freeze.
- Existing GF01/GF02 16 binary hashes unchanged. 3 accuracy helper tests pass. Input manifests rechecked 2912/5057. TUM GT only about24% coverage; do not claim full trajectory accuracy.
- Remaining: finish builds; apply replay_next.cc; final rebuild; smoke all combinations; commit source/harness, run harness/freeze.py once; full sequential 8 modes x3 repeats on both datasets (48 rounds), 1x, tags euroc_modules/tum_modules; analyze.py, factorial_report.py, audit_freeze.py; archive evidence/report/plots with Git commits and update feature/handoff. Scripts live in independent harness; some archived files at initial checkpoint are earlier versions, refresh before final commit. `freeze.py` refuses overwrite; formal inputs must never change during tests.
- No production ORB3/board changes. User asked actual tests, continue until results delivered, not only a plan.

---

# GF02 current handoff — 2026-09-28

GF02 evaluation delivery is complete (`passing` for the research workflow). The repaired hashing candidate is **not accepted for acceleration or production**. LM01 remains blocked/default off.

- Isolated root: `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_paper_20260928`; source worktree `upstream/`, branch `repro/paper-buckets-20260928`, commit `33e506481bf4daf8bf37b1f4377950c28a8c54a7`. Git object storage is shared with GF01; preserve that original checkout. Full official-base and GF01-delta patches are archived.
- IDE started clean at `4a99b5e7`; implementation/harness checkpoint `ee1c6bec`; final evidence commit is the history of the GF02 REPORT. GF01 source/build hashes are unchanged.
- Four modes: baseline, legacy mih_ots with common synchronization fixes, paper_ots bounded/recent/migrating buckets, paper_all also querying every descriptor. No feature/matcher/tracking threshold reduction.
- Release and focused ASan/UBSan/LSan/TSan index fixtures pass; accuracy helper 3 tests pass. Four 400-frame smoke runs pass (no hash queries in that short interval).
- Full `euroc_paper` and `tum_paper`, 12 rounds each, completed 95,628 frames. `harness/audit_freeze.py` passes all record and freeze checks. 23/24 loss-free; TUM first bucket-only run lost at frame 28 (24 inliers, early-window threshold 25), before any hash query. Root cause beyond this trigger is unresolved; later two bucket-only rounds succeeded.
- EuRoC baseline / old hash / bounded / bounded+all-descriptor API mean: 26.603 / 25.737 / 30.724 / 31.871 ms; P95: 35.609 / 38.627 / 63.127 / 64.263 ms. ATE about 3.3 cm throughout.
- TUM corresponding mean: 19.824 / 20.790 / 23.392 / 26.067 ms; P95: 27.550 / 32.291 / 52.368 / 59.477 ms. Bucket-only numbers include a failed run, so cannot establish gains. GT covers 24.3%; successful-only accuracy is explicitly annotated.
- Internal pose-ready timing improves for old hashing, but full API tails do not. Current public ROS wrappers publish after Track returns. Core pose-ready timing is not ROS publication latency.
- Evidence: `artifacts/gf_orb_slam2_paper_20260928/REPORT.md`, raw indices, machine summaries, PNG/PDF figures, sources and build manifests, fixture logs, startup failure diagnostic. No maps generated or board files changed.
- Remaining research: independently profile/update index synchronization and OTS actual bucket membership/information semantics; do not interpret this partial semantic repair as exact reconstruction of the paper. No full-System sanitizer, board/ROS, loop/save/reload or long-NewCollege acceptance.
- Do not rerun or alter existing frozen tags; use a new experiment tag for any later candidate.

---

## Historical handoffs retained for provenance

The current instructions are above. The following earlier state is preserved as historical evidence.

# GF01 current handoff — 2026-09-28

GF01 host reproduction/evidence delivery is `passing`; stable acceleration and
full-paper reproduction are **not** demonstrated. Production LM01 stays blocked
and default off after its completed C6 matrix; do not continue the stale C1
instructions from earlier handoffs.

Independent directory:
`/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_repro_20260928`.
Official catkin base: `2a60498cb889965bd5b4ab05c918ab28b433e882`;
local compatibility/isolated-hashing source commit: `39e794a`.
IDE initial checkpoint: `9fa86c71`; final evidence commit is the history of
`artifacts/gf_orb_slam2_repro_20260928/REPORT.md`.

- Four CPU modes built using `cmake -S harness -B build`, then
  `cmake --build build --target replay_baseline replay_mih32 replay_mih_ots replay_mih_ots_default -j3`.
- Completed sequential interleaved matrices: `euroc_formal` (9),
  `euroc_default` (6), `tum_bgr_formal` (9). All 24 inputs complete,
  89,193 frames total. One EuRoC narrow-OTS run lost its last 68 frames;
  all other formal runs remained OK after initialization.
- EuRoC first batch baseline/OTS mean 27.52/26.34 ms, P95 36.10/38.20 ms;
  narrow OTS fails the three-run tracking gate. Wider parameters are slower.
  TUM baseline/8-table/24-table means 18.70/20.16/20.71 ms,
  P95 26.42/31.13/32.46 ms. Native API timing is not board ROS latency.
- TUM input adaptation: OpenCV PNG is BGR, so `Camera.RGB=0`; depth factor
  5000 for uint16 PNG. Initial `tum_formal` with RGB interpretation is
  explicitly invalidated: 3 completed + 1 interrupted runs retained.
- Ran `python3 -m unittest discover -s harness -p 'test_*.py' -v` (3 pass),
  `python3 harness/prepare_inputs.py --check-existing`,
  `python3 harness/analyze.py runs/<matrix>`, export/plot/archive tools,
  and `bash harness/run_bucket_probe.sh` after timing finished.
  Probe confirms the current upstream deque stores 25 entries despite
  nominal MAX_BUCKET_SIZE=10; query limit 10 still returns 10.
- Frozen binary/config/driver hashes audited against current or preserved
  historical files. Source checkout is clean. Raw frames, poses and logs
  remain local with per-file SHA256 indices in the IDE archive.
- EuRoC uses global Sim(3); RGB-D uses SE(3). TUM GT covers only 1227/5057
  poses with a 129.865 s gap; metre-scale ATE is not full-sequence accuracy
  acceptance. Read the report before using any aggregate.

Touched: only GF reproduction source checkout, independent harness/configs,
`artifacts/gf_orb_slam2_repro_20260928/`, and these evaluation docs.
No production ORB-SLAM3 edit, deployment, board process or board-file cleanup.
No map files generated.

Not run: 11-sequence/10-repeat EuRoC or NewCollege 43-minute loop SLAM;
combined Good Feature Matching/Good Graph; ARM deployment; GF sanitizer /
concurrency certification. Current published source is not a confirmed 2019
paper revision. Hardware, libraries, timing boundary and bucket semantics
limit claims of exact paper reproduction.

TODO for a separate follow-up, if authorized: identify the paper revision and
resolve bounded-bucket/refresh semantics before attributing a performance gap
to the algorithm; profile query/maintenance/postprocessing costs separately.
For production LM01, snapshot/commit cost and nonbinding MP limits remain the
measured issues. Do not port GF solely on the paper's average-latency figure.

Historical handoffs follow.

# Session Handoff

## Summary

- Active feature: BR01; final state: `passing` for branch/source/archive separation.
- Scope: organize existing changes into the requested branches and archive
  generated evidence. See `docs/branch-layout.md` for exact ownership and bases.
- The original branch tips and frozen board production binaries are preserved.
- Next recommended step: Continue F23 tracking-quality work on `feature/xfeat-f23`; the EuRoC/full-lawn gate remains blocked.

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

## Addendum 2026-08-10 — F23 full XFeat tracking blocked

- Active feature/final state: F23 is `blocked`. F24 was not started because it
  remains gated on F23 passing.
- Implemented descriptor-aware ORB/XFeat distance and medoid utilities; a full
  stereo feature-provider seam; an RDK-X5 XFeat provider; float-descriptor
  Frame/MapPoint/reference-keyframe tracking; single-scale-aware projection;
  and last-frame/current-frame mutual-nearest-neighbor plus M384 Fine temporal
  matching. Float BoW is intentionally bypassed and loop closing is explicitly
  disabled until F24 supplies a float vocabulary and descriptor-tagged Atlas.
- Host commands passed: the `-Wall -Wextra -Werror` descriptor test,
  `cmake --build build --target ORB_SLAM3 -j1`, `bash -n
  xfeat_rdk_x5/run_f23_xfeat_smoke.sh`, and `git diff --check`.
- RDK cross-build passed with `RDK_BUILD_JOBS=1
  ./tools/build_rdk_arm64_docker.sh` in
  `/home/ywl/Project/RDK_X5_orb_slam`. Final ARM64 node/library SHA-256 values
  were `1d1f2ae6f1c8f218ebcae036431e92a2bb5082beab2383a6c4c6a2ffc42ebd8b`
  and `7cad809094747f0ec4d64e7e9ed9bd24574939405c907c464ef6f2f18e595029`.
- Board stage: `/userdata/orb_slam/f23_xfeat_20260810`. The test stage contains
  the final core library. The final ROS node was not copied after a combined
  upload approval-channel failure, but no ROS-facing ABI was changed in that
  iteration; the staged node loaded and exercised the final core library.
- Best short evidence:
  `xfeat_rdk_x5/artifacts/f23_xfeat_smoke_temporal_fine/smoke.csv` and
  `smoke.log`; 87/87 state-2 and pose-valid callbacks, zero loss segment,
  158.25 mean inliers, 72.23 ms frontend mean. SHA-256 values are
  `9ed85cb222016c9f8a1e5bc0cf376fedae4028e1acd8cc1e9e79570f8e5d5e82`
  and `74dd7bbbf2f82c3b8f647e5d3a63f7c03cf959bc49cc9e12dd314101bfc964af`.
- Complete first-lawn evidence:
  `xfeat_rdk_x5/artifacts/f23_lawn_full_094639_temporal_fine/smoke.csv` and
  `smoke.log`; 981 callbacks, 682 state-2, 299 state-3, 29 loss segments,
  2.97 s maximum loss and eight maps. SHA-256 values are
  `49cfbba2c226190117eb1d19a04cffef43eeb277f0945d1feb362878bab1079c`
  and `096cb1f8051817a1acbf7e403dcf9cc725618e3a48f27765bf8731d85cdb7946`.
- Commands not run: the formal MH_01/MH_03/MH_05 five-run ORB/XFeat gate,
  because no matching dataset directories were found under the local project
  roots; the other two lawn bags were not promoted as substitutes because
  the first complete bag already fails long-run continuity and none supplies
  external ground truth for ATE/RPE. `./build.sh` was not rerun end-to-end;
  the directly affected `ORB_SLAM3` target and ARM64 ROS package both passed.
- Touched current-repository areas: `CMakeLists.txt`,
  `include/{FeatureDescriptor,StereoFeatureProvider,Frame,LoopClosing,ORBmatcher,System,Tracking}.h`,
  `src/{FeatureDescriptor,Frame,KeyFrame,MapPoint,ORBmatcher,System,Tracking}.cc`,
  F23 provider/test/replay files and evidence under `xfeat_rdk_x5/`, and the
  four governance documents. The RDK workspace additionally changed its
  package CMake/source/config/launch and cross-build wrapper.
- Preserved user/runtime content: `.ros_runtime_11312/` remains untracked and
  untouched; all three original lawn bags remain unchanged. The existing host
  ROS master at `192.168.1.3:11311` was reused and left running.
- Next recommended step: inspect why long-run LocalMapping culls the map to
  zero optimization vertices despite strong short-run temporal tracking.
  Add per-keyframe MapPoint creation/culling and covisibility evidence, then
  validate a targeted fix on the complete first lawn bag before acquiring and
  running the formal EuRoC five-run gate. Do not lower the 30-inlier tracking
  gate or mark F23 passing from the 8-second smoke alone.

2026-09-07: RGB-D keyframe policy research (analysis only)

- Scope: literature/source review and historical-log analysis, not a new
  implementation feature. Existing feature states were not changed.
- Audited runtime core `fc1c50c8cf1c096725126d88510ed4c626bdddd3` in
  `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing/core`;
  compared the relevant source lineage against F27 commit `84a964b`.
- Recomputed insertion intervals and same-map relative motion from
  `handheld_oom_runs/handheld_20260904_154857/`: 966/1396 insertion gaps were
  one input frame; 816/1390 valid consecutive KF pairs were below 5 cm and
  3 degrees. These are online-pose statistics, not ground-truth accuracy.
- Corrected the prior interpretation of LBA logging: 281/287 stop flags at
  the end of LM cycles do not establish only six effective BA updates.
  The flag is sampled after optimization/culling; an interrupted optimizer
  can still write estimates. Actual iterations/writeback need instrumentation.
- Evidence and proposed policy: local generated report
  `evaluation/artifacts/keyframe_policy_study_20260907/report.md`, including
  primary literature/official-code links, exact local source references,
  initial parameter ranges and replay/accuracy verification design.
- Recommended policy: lightweight visual novelty and risk signals, ordinary
  insertion budget with explicit turn/near-point exceptions, and nonblocking
  backend feedback before `InterruptBA`. Actual enqueue success commits the
  budget. Map/reset events invalidate geometry history. Map-object reclamation
  is a separate requirement for bounded long-term memory.
- Touched: this handoff and the generated report only; no algorithm, board,
  configuration, dataset or process state changed. Runtime worktree clean.
- Validation: read-only source/log checks, CSV calculations and documentation
  whitespace/link checks. Builds and ROS replay not run because no runtime
  implementation was requested or produced.
- TODO: exact production compile commit/dirty diff is unavailable; no raw
  images or ground truth were recorded for this handheld run. Shadow-policy
  logs, real BA progress telemetry, same-bag 1.0x repeats and outdoor turn
  accuracy tests remain necessary before claiming a best operating point.

2026-09-08: bag integrity audit (analysis only)

- Scope: inspect `2026-09-08-09-56-57.bag`; no implementation feature was
  selected or changed. Existing user YAML and earlier handoff content preserved.
- Result: original is truncated, 5,188,853,760 bytes with header index pointer
  6,370,758,821; 4257 complete chunks, one partial, 969 absent relative to
  header declaration. Original left in place; independent reindexed copy
  contains 4257 readable image/depth messages spanning 191.524 seconds.
- Verification: original `rosbag info --yaml` fails unindexed; physical record
  scan and 4257 local index checks pass up to truncated tail; output reindex
  and info pass; full message deserialization, payload checks and timestamp
  analysis completed. Retained raw chunk/index byte region hashes identical.
- Evidence and touched generated outputs:
  `evaluation/artifacts/bag_integrity_20260908_095657/` (report, JSON, NPZ,
  logs, scripts and recovered bag). This handoff was appended.
- Not run: builds, ROS publishing replay or SLAM; this task only audits data.
- TODO: compare original recording-device file size/hash and recopy full bag;
  source checksum and original end time unavailable. Depth units, outlier
  conventions, camera calibration and geometric alignment not verified.

2026-09-08: recovered bag playback follow-up

- User playback error: connection refused / failed to contact master at
  localhost:11311. Advised starting roscore before replay.
- Actual full-file `rosbag play --immediate --quiet` passed with exit code 0
  and Done, using isolated localhost master port 11329; test master stopped.
- Evidence: bag integrity artifact directory `play_check.log`,
  `play_check.json`, `play_check_roscore.log`, `verify_play.py`; report updated.
- No bag, SLAM source, camera YAML, or feature-state changes. Fast publishing
  verified, but subscriber delivery rate and 1x real-time playback untested.

2026-09-08: recovered bag RDK-X5 1.0x mapping baseline

- Scope: one RGB-D baseline replay under the existing F27 production behavior;
  no algorithm implementation or feature-state changes. Preserved user YAML
  and all prior handoff content. Bag stayed on the laptop.
- Ran isolated host ROS master 192.168.1.3:11332 and board production_w3,
  frozen 700 features / 5 levels / 3 workers, with namespaced RGB/depth.
  Generated run_once.sh adapts F33 with owned PID handling and persistent
  logs. Corrected its preflight because rosnode info returns zero for an
  unknown node; that rejected preflight never started SLAM or playback.
- Replay and SLAM exits 0; 2105/2128 paired inputs processed, 23 missing
  callbacks; 2070 OK, 34 RECENTLY_LOST and one new-map state0. One interruption
  at source 126.720 s, restored by new map after 3.150 s. No loop or merge.
- Atlas has 2 actual maps, 1288 KFs and 49770 MPs. No OOM/crash/reboot evidence.
  Full-process sampled RSS peak 1009.45 MiB, Swap peak 257.39 MiB,
  same-sample sum peak 1232.51 MiB; saving contributes additional memory.
- Save service and board collector exited 0. SCP directory '/.' collection
  failed (controller exit 1); rsync recovery copied all 41 regular files,
  175620425 bytes. Preserved failure evidence; matching host/board SHA256
  verified for Atlas and both timing CSVs. Test processes/master stopped.
- Found PreSave appends to mvpBackupMaps without clearing; second save reports
  4 backup entries while live-map set remains 2. Do not double-count map/KF/MP
  totals. No core fix was made. TUM KF export covers only active map; frame
  TUM mixes map references and includes RECENTLY_LOST, so not ground truth.
- Evidence/touched generated outputs:
  evaluation/artifacts/board_recovered_20260908_095657/ (run/analysis scripts,
  logs, summary.json, report.md, three plots, recovered board results).
  This handoff was appended; no SLAM source, binary or camera settings edited.
- Validation: bash -n, actual 1.0x board replay, exact source seq/stamp matching,
  CSV analysis, plot inspection, map/timing transfer hash checks, clean stop.
  Builds not run: frozen binary baseline, no source changes. Atlas reload,
  ground-truth accuracy, failure-image review and longer/full-bag OOM testing
  remain TODO. Original bag tail is missing; source only lasts 191.524 s.

2026-09-08: failure-image follow-up for recovered-bag baseline

- Analysis-only continuation of the F27 baseline; no feature-state or SLAM
  changes. Extracted 151 exact RGB/depth pairs (seq1350-1500, 121.500-135.000s)
  from host recovered bag, including the five locally missing callbacks.
- Added RGB PNGs, lossless float32 depth NPZ, previews, HTML gallery, phase
  montages, timing detail plot and evidence report under the run's
  failure_images/ directory. Compiled a standalone host ORBextractor helper;
  146/146 delivered-frame keypoint counts agree with board CSV (not a
  byte-identical descriptor claim across OpenCV versions).
- Evidence supports right turn plus missing intermediate observations:
  source1403->1408 spans450ms, host-runtime ORB mutual/H-inliers45/30,
  compared with81-156 H-inliers in each omitted adjacent pair. Surviving
  jump H-inliers all lie in upper half; near floor features are numerous
  but poorly persistent at that span. No blackout/depth-wide dropout seen.
- First failure executes motion-model and reference-KF paths, not LocalMap;
  shared failure text is misleading. KF856 is last OK seq1403, so this is
  not sparse KF insertion. Pure RGB-D relocalizes unsuccessfully for3s,
  then seq1443 initializes new map/KF857. Stale inliers99 hidden in plots.
- OOM/Swap not observed at loss. Input backlog/queue loss specific layer is
  still TODO; no full-delivery counterfactual replay, internal MapPoint
  correspondence replay or metric ground truth. Preliminary OpenCV-ORB
  PnP fields are unvalidated/degenerate and explicitly excluded from claims.
- Validation: source pair count, timing/seq alignment, host extractor build
  and run, offline matching, PNG/NPZ generation and visual inspection.
  Preserved existing user YAML changes. No board process actions this turn.

2026-09-08: missing-input queue forensic follow-up

- Read-only board ELF disassembly verified frozen production_w3 subscribers
  use queue_size1 each and TimeSynchronizer10, with single-thread ros::spin.
  SHA256 matches baseline a41fd835...; current source20/20/20 is NOT the
  frozen deployment. Outer runtime a28ac28 has1/1/10; a71c1b1 changed20/20/20.
- Source interval90ms versus observed callback interval101-104ms accumulates
  ~350ms relative age;1403->1408 skips450ms source time in101.407ms wall time.
  Combined with depth/RGB independent size1 queues, ROS oldest-message eviction
  plus exact-sync incomplete-pair cleanup is now the leading explanation.
- Do not overclaim: raw arrival/drop events absent, so exact per-message
  eviction sequence and which stream dropped remain TODO. INFO-only logs do
  not exclude queue overflow (roscpp reports it atDEBUG).
- Evidence: run/input_drop_forensics/report.md and production_main_disassembly.txt;
  failure_images/report.md linked update. No source/config/deployment changes,
  no new replay. Future diagnostic should preserve core and instrument queues.

## 2026-09-08: Docker source provenance follow-up (analysis only)

- Verified archived August 17 production_w3/ros_rgbd SHA-256 is exactly
  a41fd8359313a0c78bb690743c08b964a18491b94c21d797019986666a99cc7b,
  matching the board replay. Current runtime devel node is 4962aece...1f684,
  a different artifact; do not infer its queue values from source alone.
- Docker bind-mounts WORKSPACE to /ros1_project/orb_slam and CORE_SOURCE to
  /ros1_project/orb_slam3_core. Generated build.make explicitly compiles
  WORKSPACE/src/orb_slam3_ros-master/src/ros_rgbd.cc; queue constants belong
  to this executable, not the core shared library or Docker image.
- Outer a28ac28 versus a71c1b1 ros_rgbd.cc diff is exclusively 1/1/10 to
  20/20/20. September 7 commit preserves pre-existing changes; it does not
  establish their original edit date. Original top-level RDK workspace still
  has 1/1/10. Runtime script now defaults to its own workspace/core; historical
  DEBUG_BRANCH.md documents explicit overrides for that same isolated tree.
- Checked historical/current build scripts, CMakeCache/build.make, deployment
  README, archived hashes and both Git histories. Historical full build env
  and immutable source manifest remain unverified; source baseline similarity
  must not be represented as byte-exact provenance of every compiled file.
- No compilation, Docker execution, replay, deployment or source changes.
  Only this handoff was appended; unrelated user YAML changes preserved.

2026-09-12: RDK X5 RGB-D map-memory research (analysis only)

- User confirmed 2 GB RAM, 2–4 GB swap, RGB-D, and substantial swap use
  around 1400 keyframes. Historical Linux-visible RAM was about 1307.55 MiB;
  current board memory/other workloads were not remeasured.
- Reviewed primary papers, author repositories, a firsthand Quanser blog,
  upstream ORB-SLAM memory issues, current/runtime source, and prior X5 reports.
- Report: docs/map-memory-research-20260912.md. Proposes measured attribution,
  observation-based GBA capacity, idempotent save preparation, safe retirement,
  compact feature grids, quality-aware sparsification and disk-backed submaps.
- Verified runtime source still has nine KF×MP vector reserves, pointer-only
  Map erasure, disabled bad-map deletion, and PreSave backup-list appending.
  Capacity estimates are not measured RSS or a diagnosis of prior OOM.
- No implementation feature selected or feature states changed. Only the
  research document and this appended handoff were written; existing user
  YAML and prior handoff changes preserved.
- Validation: read-only source/report checks, static arithmetic, and document
  link/whitespace checks. No builds, replay, SSH, deployment or runtime tests:
  this request was for research and a plan, not an implementation.
- TODO: current memory budget, per-object lifetime telemetry, target ABI,
  loop/GBA long-run data, independent accuracy/area evidence and storage I/O.
- Next step: P0 attribution in the runtime core with frozen binary/config
  provenance, then implement and validate one optimization at a time.

2026-09-12: Map-object reclamation clarification (analysis only)

- Clarified that delete and shared_ptr are both usable with a complete
  lifetime protocol; immediate deletion and all-strong pointer replacement
  are the problematic shortcuts.
- Read stella_vslam map/keyframe/landmark/graph/frame-statistics source:
  explicit shared ownership, weak reverse/graph links, retirement cleanup,
  and historical frame-reference migration provide a concrete reference.
- Read Buffalo thesis sections 4.3/4.4 on custom counting and deferred
  collection; no verified drop-in author patch was found in this review.
- Extended docs/map-memory-research-20260912.md with evidence, proposed
  ORB-SLAM3 ownership roles, trajectory rebinding and staged migration.
- Validation: source reading and document whitespace checks only. No build,
  runtime test, source implementation or deployment; feature states unchanged.
- TODO: complete lifetime audit, map-merge/serialization contracts and ARM
  runtime overhead before enabling physical reclamation.

2026-09-14: MS-SLAM omission correction (analysis only)

- User identified missing MS-SLAM (JFR, online 2024-09-04, 2025 issue).
  Acknowledged incomplete candidate coverage, missing author-followup search
  and failure to inspect the already available local MS-SLAM checkout.
- Read publication method/evaluation sections, official README and local
  MS-SLAM 51142b3 sparsification, KF/MP retirement, descriptor compaction and
  historical reference migration paths.
- Corrected docs/map-memory-research-20260912.md at the top and in the source
  table; added concrete local references and revised prioritization.
- MS-SLAM uses shared_ptr with explicit relation teardown; weak_ptr is an
  alternative design, not a mandatory condition for reclaiming cyclic graphs.
- Prioritize MS-SLAM implementation review and separate lifetime, payload and
  selection ablations; retain stella_vslam as an ownership-design comparison.
- No source/build/deployment changes. Existing MS-SLAM build-file changes,
  compare_results/, current-repo YAML changes and artifacts/ were preserved.
- Validation: online and local source checks, document whitespace checks.
  No compilation/replay: scope is correcting research and candidate ranking.
- TODO: full concurrency/reset/merge audit, RGB-D, ARM solver feasibility,
  DBoW3/serialization integration, same-data memory and accuracy experiments.

2026-09-14: camera_field_03 transfer and save-stall analysis

- Analysis only; feature states unchanged. Copied the user-specified board run
  `/userdata/rd53_camera_delivery_20260914/camera_field_03` to local
  `artifacts/save_stall_20260914/camera_field_03`; board data retained.
- All 31 source regular files (239278752 bytes) match fresh remote SHA-256.
  Final status complete, camera/slam/control/log exit codes zero. Earlier
  first-save snapshot is also preserved separately; no map reload performed.
- First save service 351.715 s; shutdown to final map mtime about 903.986 s.
  Logs prove repeated SaveAtlas and duplicate PreSave map processing (1 -> 2
  backup entries). Swap peak 1842.1 MiB; available minimum 13.8 MiB. Runtime
  source corroborates uncleared backup list and shutdown save filename reuse.
- Existing diagnostic analyzer and local summarize.py generated report.md,
  evidence.json and memory.png under artifacts/save_stall_20260914/analysis.
  Chart visually checked; 1706 memory samples, 2 parsed diagnostic events.
  Recorder dropped 92; event_1 prewindow only 8.446 s; event_2 triggered after
  normal camera shutdown, so it is not an independent camera-failure event.
- Benchmark has 4559 paired callbacks, max 23.393 s, 265 unobserved seq IDs
  within 0..4823. Loop, lost tracking/new map and merge are logged; their
  precise causal relationship to memory pressure remains unverified.
- No code build, deployment, board cleanup or process interruption. Existing
  YAML changes preserved. TODO: save idempotence and atomic replacement,
  thread-consistent snapshot, reload/accuracy validation and memory attribution.

2026-09-14: camera_field_03 map visualization

- Generated local standalone read-only Atlas exporter and PNG/HTML/PLY views
  under artifacts/save_stall_20260914/visualization; original map unchanged.
- Native archive v18 header/ABI validated explicitly; host Boost 1.71 uses
  headerless decoding with input library version18. Full237410143-byte consume,
  2 backup entries/1 unique map,73953 points/1801 KFs, finite coordinates.
  KF positions match saved TUM to max6.07e-6m; timestamps within5.01e-6s.
- Main view displays72994 points,959 outside crop retained in full view and
  HTML/PLY.18 points beyond100m from origin; farthest3185m. Do not infer real
  site extent or metric accuracy from outliers. Display -Y is not gravity up.
- Exporter compiled/ran; Python plots generated; local headless Chrome page
  rendering and static/browser screenshots inspected. Initial compiler flags
  and old Matplotlib3D import issues resolved; commands recorded in README.
- No main runtime rebuild/deployment or board changes. Atlas PostLoad and
  localization replay not performed; this validates geometric extraction,
  not all map references or tracking accuracy. Existing user changes retained.

2026-09-14: 500m² mower memory sizing (analysis only)

- User confirmed0.5m lane spacing and0.3m/s speed for estimates; with assumed
  10% turning overhead gives1100m and61.1min. Actual KF trajectory161.757m XZ,
  1801KFs/422.008s; single-run density11.134KF/m,4.27KF/s.
- Fit318 pre-late-transient samples: RSS+swap MiB=523.631+0.459593×KF,
  R²0.9953. This includes correlated map/history costs, not per-object size.
  Distance/time extrapolations yield12.2–15.6kKFs and roughly6–8GiB steady
  process footprint. A separately labeled linear peak stress scenario gives
  16–20GiB; neither is a verified physical-RAM requirement or upper bound.
- Report: artifacts/save_stall_20260914/analysis/lawn_500m2_memory_estimate.md.
  Current2GB insufficiently supported;8GB optimization target,32GB baseline
  test host suggestion conditional on current growth. No purchase or deployment.
- Primary ORB-SLAM paper checked for redundant map selection principle;
  exact lawn memory depends on overlap/culling/reclamation and global peaks.
  Feature states unchanged; no new runtime tests or algorithm changes.


2026-09-14: MS-SLAM reuse audit for RDK X5 RGB-D

- Analysis/docs only; no coding feature selected and no feature state changed.
- Reviewed MS-SLAM51142b3, timing outerc6acc9e/core705c710, then latest field
  outer18ba684/core4d6df6a including existing worktree changes and delivery
  provenance. Preserve all existing user/source/artifact changes.
- Report: /home/ywl/Project/RDK_X5_orb_slam/reviews/ms_slam_reuse_20260914/README.md.
  source-audit.json records52 source/evidence fingerprints, ownership/API/build
  inventory and timing/field map-core equality. review-validation.json:52
  hashes unchanged,49 local links valid. Updated research document with link.
- Findings: pure RGB-D sparsification enters via LocalMapping; Tracking still
  retains an unused shared KF candidate list in that mode. Explicit graph
  teardown incomplete on clear/reset; Replace clears replacement reference
  while Tracking still reads it. KF compaction requires index/DB consistency.
  Gurobi has no resource/status controls; shutdown batches all remaining KFs
  of CURRENT map (Atlas getter is current-map only), before other workers stop.
  Save/load and ROS API integration missing. ARM64 Gurobi is officially
  supported; locally inspected MS/Gurobi libraries are x86-64 only.
- Latest field delivery is direct8/8/10, not archived FIFO16. Its incremental
  cross build replaces Tracking/LocalMapping objects and assumes unchanged
  class layout; ownership/payload port requires full core/node rebuild.
  Existing camera_field_03 evidence confirms long-run/save pressure and
  repeated PreSave entries; it is not a new replay from this session.
- Verification: rg/sed and lexical inventories, file architecture inspection,
  local hash/link validation, primary paper/repo/Gurobi documentation checks.
  Initial one-off inventory emitter had a parenthesis syntax error, fixed
  before artifact generation; final inventory/validation completed.
- Not run: new build, Gurobi/license initialization, sanitizers, RGB-D replay,
  board experiment/deployment or map reload. This audit does not certify
  runtime correctness. Next: full field-source build baseline and solver/model
  budget probe, then ownership, payload/persistence and bounded sparsification
  as separate validated implementation stages.


2026-09-14: Gurobi commercial licensing correction

- User identified a missing deployment consideration. Earlier audit mentioned
  license initialization but incorrectly retained Gurobi as the default P0.
- Verified official commercial/academic licensing descriptions, Academic WLS
  restrictions, Gurobi standard EULA deployment scope, and HiGHS MIT license.
- Rewrote report summary, dependency assessment and P0: default evaluation is
  HiGHS MILP model port plus bounded greedy; Gurobi is optional only with an
  applicable license. No license pricing or device entitlement inferred.
- Updated docs/map-memory-research-20260912.md and the RDK audit report and
  review-validation.json. Existing runtime sources and source fingerprints
  unchanged by this docs-only correction. All local report links valid.
- No license credentials read, software installed, build, solver benchmark,
  board connection or deployment. Replacement performance remains unverified.


2026-09-14: MS-SLAM native RGB-D bag memory validation

- User requested actual MS-SLAM validation with the previous bag. Evaluation
  lane lives outside this repo at /home/ywl/Project/MS-SLAM/evaluation/bag_memory_20260914;
  no ORB_SLAM3_DBoW3 feature state or runtime source changed. Preserve existing edits.
- Losslessly extracted all 2910 RGB8/32FC1 exact-stamp pairs from
  2026-09-10-11-35-57.bag. Same 700-feature/5-level YAML, native 1x timestamps,
  no ROS/viewer, DBoW2, isolated full rebuild of MS51142b3 and ORB4452a3c.
  Actual loaded libraries and input/source/binary hashes captured.
- Gurobi initialized in host environment; sandbox host identity failed.
  Existing installation/license unchanged; no commercial entitlement inferred.
- Completed orb_1, ms_1, ms_2, orb_2: each2910 frames, all state2, single map,
  no logged loop/merge, swap0. Mean pre-shutdown PSS1226.90 ->985.98MiB (-19.64%);
  baseline-subtracted growth773.29 ->529.92MiB (-31.47%). Full-lifetime peak
 1227.06 ->1256.29MiB (+2.38%); shutdown batches569/602KFs, solve1.96/2.14s.
- Online MS destructed327137/324420 MapPoints; liveMP36897/39026 versus
  ORB370137/372063. Descriptor rows roughly halved. Candidate KF strong refs
  still1407/1410; no KF destruction observed. Not a bounded-memory guarantee.
- Optional keep_1 ablation skips only solver-result SetBadFlag, still solves
  and compacts KFs. Stalled after last complete statistics at frame2136;
  three futex-wait threads. Ptrace denied and sudo -n required password;
  no system setting changed. Terminated only this owned process after176s
  without progress; exit-15 and partial buffers preserved. Excluded from
  complete comparisons; contribution of MIP-only deletion remains unresolved.
- Commands: extraction/prepare scripts; cmake configure and full -j2 build;
  run_suite.py --smoke, run_suite.py, --diagnose, --stop-stalled-keep,
  --finish-repeats; analyze.py, compare_trajectories.py, verify_results.py,
  probe_sizes.py. Initial hook compilation issue fixed in isolated sources;
  full build passed. Postprocessing handled truncated failed-run output;
  size probe linked required Pangolin support. Four main evidence checks pass;
  ablation explicitly unusable. Counter adds8 bytes/MP,0 bytes/KF in both cores.
- Report, plot, CSV, manifests, patches, logs, verification and size probes
  retained in the evaluation directory. RDK reuse report links new evidence.
- Not run: X5/ARM/DBoW3 integration, alternative solver, sanitizers, Atlas
  save/reload/reset or larger new area. No truth in bag: pairwise SE3 pose
  disagreement is not ATE. No deployed runtime files modified.
- Next: independent reclamation/payload controls, diagnose ablation stall,
  fix lifecycle and shutdown budgets, then open-source solver and X5 validation.

2026-09-18: REC01 standalone S316 RGB-D recording — passing

- Added laptop entrypoint `record_rgbd_loop.sh`, worker/checker
  `evaluation/rgbd_recorder.py`, offline regression
  `evaluation/test_rgbd_recorder.py`, and `docs/rgbd-loop-recording.md`.
  Updated README, repo map, validation, feature list and the narrow docs
  ignore exception. No SLAM/ROS C++ node or driver source was modified.
- Board inspection confirmed `root@192.168.1.10`, Noetic,
  `/ros1_project/camera_ws/install/setup.bash`, and `/home/metoak/mo_init.sh`.
  `/home/mo` does not exist. Source initialization in the camera's shell,
  retaining SDK paths and skipping only already-loaded kernel modules.
- Flow: isolated board-local ROS master, live RGB-D preflight, local board
  LZ4 recording, explicit recording notification, Enter/Ctrl+C stop, recorder
  flush before camera shutdown, resumable rsync, SHA-256 and full bag reads.
  Board originals and logs are retained. `--fetch` retries without opening
  the camera. Disk reserve and control heartbeat trigger cleanup.
- Passed `bash -n record_rgbd_loop.sh`, Python compilation, `--help`, and
  `/usr/bin/python3 evaluation/test_rgbd_recorder.py` after sourcing Noetic.
  Offline cases cover healthy, absent depth, all-zero depth, non-monotonic
  timestamps, malformed payload, unsynchronized streams, truncated and active bags.
- Hardware command: `./record_rgbd_loop.sh --duration 6 --output
  artifacts/rgbd_recorder_validation`; run `rgbd_loop_20260918_145800_5dc9d3`
  returned 134217703 bytes, 1311 messages, 65 RGB / 66 depth frames, 65 pairs.
- Manual Ctrl+C command: `./record_rgbd_loop.sh --output
  artifacts/rgbd_recorder_validation`; run `rgbd_loop_20260918_150021_125c59`
  returned 172202854 bytes, 1688 messages, 84/84 exact-stamp pairs at 11.11 Hz.
  Both board results and local bag checks passed; second initialization
  confirmed all four previously loaded modules were handled successfully.
- `--fetch rgbd_loop_20260918_145800_5dc9d3 --output
  artifacts/rgbd_recorder_validation` passed. Evidence under that directory:
  per-run logs, snapshots, board_result.json, stream_check.json, bag_check.json,
  fetch_test.log, and empty final_board_processes.txt (no camera/ROS leftovers).
- Existing user's S316_stereo.yaml trailing blank line and prior handoff
  additions were preserved. Whole-repo `git diff --check` reports that
  pre-existing YAML whitespace; the task's changed files pass scoped checks.
- Not run: C++ builds (no compiled source change), real connection loss,
  disk exhaustion, power loss or physical closed-loop SLAM evaluation.
  TODO: user records the actual route; basic integrity/pairing checks cannot
  establish geometric calibration or successful SLAM loop detection.

2026-09-18: REC01 follow-up — delete board bag after verified transfer (passing)

- User requested automatic board-space reclamation. `verify_download` now
  gates deletion on matching SHA-256, full bag/RGB-D validation and successful
  recording/SSH status. Fsync the local bag/directory, then recheck the remote
  hash and delete only that session's `rgbd.bag`. Preserve logs, configuration,
  other sessions and `.active` files; failures keep the source for retry.
- Updated `evaluation/rgbd_recorder.py`, `evaluation/test_rgbd_recorder.py`,
  README, recording guide, feature list, validation notes and this handoff.
  Existing user changes remain untouched.
- Passed shell syntax, Python compilation and all 6 offline test groups.
  New cases cover successful cleanup/retry, failed validation/recording/SSH,
  changed remote contents, and unfinished recording protection.
- Real command: `./record_rgbd_loop.sh --fetch
  rgbd_loop_20260918_145800_5dc9d3 --output artifacts/rgbd_recorder_validation`.
  Revalidated 1311 messages and deleted only its 134217703-byte board bag;
  the matching local bag, board logs, and second session bag remain.
  Evidence: `fetch_cleanup_test.log` and `remote_cleanup_check.txt` under
  `artifacts/rgbd_recorder_validation/`.
- No camera restart or C++ build needed for this post-transfer change.
  TODO: actual network loss during deletion and storage hardware failure were
  not induced; cleanup errors report uncertainty and allow --fetch retry.

2026-09-18: F17 follow-up — ignore root artifacts (passing)

- Added `/artifacts/` to `.gitignore` after user approval. The 45,683
  untracked delivery/evaluation files no longer appear in Git status;
  local artifacts and existing user edits are preserved.
- Verified `git check-ignore -v` on delivery and recorder evidence paths,
  `git status --short --untracked-files=all` (10 remaining changes), and
  representative local file existence. Scoped `git diff --check` passed.
- Touched `.gitignore` and this handoff only. No builds or hardware tests
  were needed for this ignore-rule change; no commit, push or Git cleanup ran.
- Next: review the remaining 10 changes before committing. TODO: remote
  status remains unrefreshed; this check used local Git metadata only.

2026-09-24: REC02 D435i synchronized 10 FPS bag recorder — passing

- Added laptop entrypoint `record_d435i_10fps_bag.sh` and
  `docs/d435i-10fps-recording.md`. It connects to `sunrise@192.168.1.11`, starts
  `/userdata/ros1_realsense_ws/start_rgbd10.sh`, validates the exact-pair
  `/camera_10fps` RGB/aligned-depth streams, records those images plus their
  CameraInfo and TF, then reuses REC01 finalization, transfer, full-read,
  SHA-256 and verified remote cleanup logic.
- Generalized `evaluation/rgbd_recorder.py` with an optional camera start
  script, busy-process guards, configurable CameraInfo topics, snapshots,
  run prefix and retry wrapper while retaining S316 defaults. Added a
  configurable preflight warmup; D435i discards two startup seconds before its
  independent five-second stability sample.
- Passed shell syntax, Python compilation, scoped diff checks, `--help`, and
  all 6 existing offline regression groups. No ORB-SLAM C++ build was needed.
- Real command: `./record_d435i_10fps_bag.sh --duration 8 --output
  artifacts/d435i_10fps_recorder_validation`. Passing run
  `d435i10_20260924_111100_130a26` preflighted at 9.99 Hz/100% pairing and
  produced a 48,284,291-byte indexed LZ4 bag with 317 fully read messages:
  79 each of RGB, aligned depth and both CameraInfo streams, plus one TF static.
  RGB/depth were both 9.9929 Hz and paired 79/79 within 0.1 ms. SHA-256:
  `e34f0df7f7583f7da17c51f254a59cbeb6cf14867c57e4205a1a79f41343f1c4`.
- The verified board bag was removed; board logs remain and no RealSense,
  adapter, rosbag, roscore or roslaunch process remained. Evidence is under
  `artifacts/d435i_10fps_recorder_validation/d435i10_20260924_111100_130a26/`.
- Initial directory creation required one-time ownership setup for
  `/userdata/d435i_10fps_bags`. An earlier attempt correctly rejected the
  startup-transient 9.477 Hz window before recording; its logs are retained.
  TODO: the user still needs to record the physical closed-loop route. Network
  loss, disk exhaustion and power loss were not induced.

2026-09-24: RGBD01 D435i + MEM18/perf20 adaptation — implemented, tracking blocked by scene

- User requested adaptation and real joint validation, choosing static scene first.
  Added `run_d435i_slam.sh`, `evaluation/d435i_slam.py`,
  `evaluation/test_d435i_slam.py`, `evaluation/summarize_d435i_slam.py`, and
  `docs/d435i-slam.md`; updated README, docs routing/validation/features, and
  added only the new guide to the existing .gitignore whitelist.
- Live CameraInfo generates 640x480/10 FPS PinHole settings with measured K/D,
  rgb8 ordering and 16UC1 millimetre scale 1000. Input subscribes directly to
  `/camera_10fps/color/image_raw` and aligned depth. No C++ change or rebuild.
  Pinned perf20 node/core hashes, file/ldd and loaded core path were verified;
  existing deployments, YAMLs, Swap and governors were unchanged.
- Ran `./run_d435i_slam.sh 120 off` and `./run_d435i_slam.sh 60 on`.
  Final off run `20260924_114152_off_3817` processed all 1200 measurement-window
  pairs; on run `20260924_113803_on_730` processed all 600. About 9.99 Hz,
  exact pairing, no input-window loss, graceful stop. Off peak PSS 487.03 MiB,
  VmSwap=0, pswpin/out deltas=0; all transferred off files passed SHA256 checks.
- Tracking did NOT pass: all callbacks non-OK, no keyframe/map point/valid pose.
  On run reports state=1 throughout, keypoints 268–320 (median 292), against
  existing `Tracking::StereoInitialization` >500 gate. Image points mainly at
  ceiling/light/plain walls. Asked user to aim at textured objects and hold
  still; no completed adjustment confirmation yet. Do not lower the gate.
  TODO: re-run off/on after scene adjustment; motion/loop/accuracy/navigation
  TF remain unvalidated. Low callback times are uninitialized-path cost only.
- Fixed validation harness issues: use actual ROS subscriptions rather than
  buffered stdout for readiness; separate resource stderr and allow natural
  sampler exit; generate remote SHA manifest after process/rospy atexit ends.
  Initial failed runs and their stale manifests remain documented; separate
  `analysis_review.json` reanalyses preserve original results.
- Checks: 6 adaptation tests and all 6 prior recorder regression groups pass;
  bash/Python syntax and scoped diff checks pass. Global diff-check still
  flags pre-existing S316_stereo.yaml line 87 whitespace, left untouched.
  No git commit/reset/cleanup/push/merge. Final board check shows no camera,
  adapter, SLAM, roslaunch or ROS master remaining; new board outputs ~2.9 MiB.
- Evidence/report/JSON/CSV/log index and final archive checksum manifest:
  `artifacts/d435i_slam_20260924/`. Active feature RGBD01 is `blocked` only for
  actual initialized tracking validation, not for input/config adaptation.

2026-09-24: RGBD02 laptop live launcher — passing (launcher, not tracking gate)

- Added executable `start_rdk_rgbd_slam.sh` and `evaluation/rdk_rgbd_live.py`.
  Laptop needs only Python3 and ssh/scp; starts a unique board-local ROS master,
  D435i exact-pair 10 FPS input and pinned MEM18/perf20 with live CameraInfo YAML.
  Reuses existing calibration/process helpers without modifying finite validation.
- SLAM stdout/stderr uses stdbuf line buffering and is forwarded immediately to
  the terminal with [SLAM] prefix; original node log stays on board and combined
  terminal output is saved locally. Latest tracking status is shown every 5 s.
  Default runs until Ctrl+C; optional `--duration N` stops after N ready seconds.
  No unbounded Python per-frame statistics, timing, benchmark, bag or atlas save.
- Host 2-second heartbeats plus board EOF/stop/15-second timeout control stop only
  owned process groups; busy camera/SLAM and lock guards reject double starts.
- Real run `live_20260924_131622_eb6ff1`: actual settings/vocabulary/subscription
  output appeared while process was live; sent host terminal Ctrl+C after 232
  tracking-status messages. camera/SLAM exited 0, all cleanup checks true, final
  board process scan empty. Evidence `logs/rdk_rgbd/live_20260924_131622_eb6ff1/`.
  Tracking was false in the same low-texture scene: RGBD01 remains blocked.
- 4 new offline tests passed (live line forwarding before EOF, control EOF,
  timeout, explicit stop); previous 6 calibration tests passed. Shell/Python
  syntax, help and scoped diff checks passed. No C++ rebuild required.
- Updated README, docs/d435i-slam.md, local repo-map, features and validation.
  No existing deployment or system setting modified. TODO: real network-fault
  injection/power loss not tested; successful initialized tracking still needs
  a textured scene, independently of the launcher feature.


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


## LM01 C5b — completed MP/statistics and tier screen

All six immutable board rounds failed input completeness and realtime:
legacy80 2436/2910, nonOK35, P95=133.027ms; attempted80 2358,35,134.134ms;
MP80 2443,0,133.633ms; adaptive96 2158,0,155.713ms; adaptive80 2446,70,136.946ms;
adaptive64 2434,0,137.711ms. Fixed-tail P95 ranges137.412–161.387ms.
The5000/6500/8000 post-filter MP limits never bind (eligible maximum2025).
All measured second/third unique-KF observations and2/3/5-KF survival deltas
are within1 percentage point of legacy80; the conditional10% young-reservation
long replay is not triggered. Its allocation/feedback mechanics passed the
purpose-built64-MP stress fixture. Different-KF/adaptive comparisons are
screening diagnostics, not isolation of a single causal change.

Cohort waiting time now distinguishes actual event wall time from dataset
progress; observing an older KF during fusion cannot turn a later observation
into a zero wait. Six analyzer tests pass; all native/board cohorts were
recomputed without changing count/survival gates. Core algorithm and tested
binaries are unchanged. Source core05e83e55, outer base9feff3c; implementation
C5a core906d870f/outer9188413. C6 matrix remains pending. DefaultOFF.
Evidence: IDE artifacts/local_map_budget_20260924/c5/completed/{board_matrix,native_analysis_v2}.json.
Conditional query-before-gather MIH and independently budgeted BA work is
specified in memory/LM01_FOLLOWUP.md; neither is implemented or accepted.


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

Final LM01 runtime commits: core `be52247f`, outer `9c43a6c`; IDE final evidence is the commit containing artifacts/local_map_budget_20260924/DELIVERY.json.
