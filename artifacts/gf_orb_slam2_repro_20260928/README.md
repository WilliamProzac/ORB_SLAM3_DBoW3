# GF01: independent GF-ORB-SLAM2 reproduction

Upstream: https://github.com/ivalab/gf_orb_slam2, `catkin`,
`2a60498cb889965bd5b4ab05c918ab28b433e882` (2024-10-11).
Official configurations: https://github.com/ivalab/ORB_Data,
`f66cc0e176c50961542f312c8624ffffbd3b9acc`.
This is the current published implementation, **not an identified 2019 paper commit**.
No production ORB-SLAM3 source, deployment or board map is changed.

## Scope

Test the local-map-hashing claim of
[Zhao et al., ICRA 2019](https://arxiv.org/abs/1905.07797), independently of
Good Feature Matching and Good Graph BA. The latter two are disabled in all
variants. The baseline is the same GF fork with hashing disabled; it
is not a separate canonical ORB-SLAM2 revision.

All builds use CPU, OpenCV 4.2, Eigen 3.3.7, Armadillo 9.800, system Pangolin,
GCC, `-O3 -march=native`, one OpenCV thread and single-thread BLAS/OpenMP environment settings. The installed
Armadillo links ATLAS (not the paper's custom single-thread OpenBLAS); the
environment variables do not prove ATLAS thread count. Mapping remains a
background thread. Upstream OTS information-matrix construction still uses
`USE_MULTI_THREAD=true` and `hardware_concurrency()` (16 on this host);
setting OpenCV/BLAS environment threads does not make the whole process
single-threaded. No viewer, GPU, external odometry, ground-truth prior,
planner anticipation, feature-count reduction, stereo tricks or map saving.
Loop closure is disabled for the EuRoC VO comparison, matching the paper's
EuRoC experiment. The supplemental TUM RGB-D comparison uses the same VO
build; it does not validate RGB-D loop closure or the NewCollege SLAM result.

| Mode | Local map | Hashing settings |
|---|---|---|
| baseline | co-visibility | disabled |
| mih32 | upstream Combined: co-visibility intersect appearance | 32 tables, retrieval cap 10 |
| mih_ots | same Combined, online table selection | initially 8 tables; at most 8, logDet target 80 |
| mih_ots_default | same algorithm, wider current-upstream parameters | initially 8, at most 24 tables, nominal bucket/retrieval 20 |

Common hash trigger: at least 2000 co-visible MPs (source has an additional
initialization-age gate). No HashOnly substitution. The Combined path still
collects the full co-visible point set before intersection. Bucket size 10,
maximum selected tables 8 and target 80 follow paper Section III/IV; Section
V also contains an inconsistent `x = 10` description. Current upstream
defaults (nominal bucket 20, max selected 24) differ and are not silently called the
paper configuration. We record the actual settings used.

## Build adaptations

`harness/CMakeLists.txt` uses installed CPU libraries and only required
upstream sources; avoids upstream scripts that install into `/opt`, build
CUDA, or require a closed-loop planner. The original build files remain
available. DBoW2/g2o only have dependency-discovery changes. One legacy
OpenCV header is replaced, and `compat.h` supplies moved legacy constants.
Algorithm switches in headers are controlled explicitly by targets.

Isolated-hashing integration corrections in `Tracking.cc`:

* Initialize the query vector for fixed-table hashing too (relocalization
  reads/writes it), and initialize/reset the per-frame hash flags.
* Clear the OTS scratch list every frame when Good Feature Matching is off;
  upstream only did this inside the feature-selection branch.
* Log completed/failed OTS updates and actual selected-table counts.

Matching thresholds, detector parameters, pose optimization and ordinary BA
remain the upstream implementation. These limited changes are recorded in
`upstream.patch`; the checkout is not represented as byte-identical upstream.
This experiment does not certify upstream raw-pointer/concurrency safety.

```sh
python3 harness/prepare_inputs.py --check-existing  # Or omit flag on a fresh input directory
cmake -S harness -B build
cmake --build build --target replay_baseline replay_mih32 replay_mih_ots replay_mih_ots_default -j3
python3 harness/run_matrix.py --dataset V1_01_easy --repeats 3 --tag euroc_formal
python3 harness/analyze.py runs/euroc_formal
python3 harness/run_matrix.py --dataset V1_01_easy --modes baseline mih_ots_default --repeats 3 --tag euroc_default
python3 harness/analyze.py runs/euroc_default
python3 harness/run_matrix.py --dataset tum_large --modes baseline mih_ots mih_ots_default --repeats 3 --tag tum_bgr_formal
python3 harness/analyze.py runs/tum_bgr_formal
```

## Recreate outside the existing checkout

Copy the archived `harness/`, `configs/`, and `upstream.patch` into a fresh
reproduction directory, then pin both official repositories:

```sh
git clone --branch catkin https://github.com/ivalab/gf_orb_slam2.git upstream
git -C upstream checkout 2a60498cb889965bd5b4ab05c918ab28b433e882
git -C upstream apply ../upstream.patch
git clone https://github.com/ivalab/ORB_Data.git ORB_Data
git -C ORB_Data checkout f66cc0e176c50961542f312c8624ffffbd3b9acc
python3 harness/prepare_inputs.py
```

`prepare_inputs.py` accepts `--euroc-root` and `--tum-root`. The runner's
vocabulary path and analyzer's TUM truth path are currently explicit host
paths: adjust them together before freezing a new experiment when moving
machines. This harness needs the installed development libraries listed
above, Boost, and `/opt/ros/noetic/include` headers; no ROS master is needed.
The source checkout is complete; neither this evidence archive nor Git
contains the datasets, binary vocabulary, build tree or full raw logs.

## Measurements

800 ORB features, 8 levels, factor 1.2, FAST 20/7; original 2000-feature
monocular initializer retained. Official raw EuRoC cam0 calibration; TUM
PNG depth factor corrected from the official config's ROS float-depth 1
to 5000; Camera.RGB is 0 for OpenCV BGR input. The initial TUM matrix
with the ROS RGB setting was invalidated (runs/tum_formal/INVALIDATED.json);
three completed and one interrupted runs are retained only as input diagnostics.
All variants use identical inputs/calibration. Camera ground truth
is evaluated only after running, never supplied to the tracker.

Native replay uses dataset timestamps at 1x, consumes every listed frame,
and runs late instead of discarding input. Decode and explicit wait are
outside `track_ms`; the complete public Track call, including keyframe
work and OTS/delayed matching, is inside. This is not ROS callback latency.
`pose_stage_sum_ms` is only a diagnostic sum of upstream stage timers,
not a measured publication latency. `hash_field_ms` is upstream's field,
which is overwritten with the whole local-point update duration; it is
**not pure hash-query latency**. Avoid adding overlapping stage percentiles.

Report all frames, initialization onward, and the fixed final 25% by input
index. Record counts, tracking failures, local MPs, inliers and memory.
Failed attempts are retained and never treated as valid fast runs.
Three repeats per variant are an initial reproduction, not the paper's
10-repeat / 11-EuRoC / 43-minute-NewCollege matrix.

EuRoC online camera trajectory uses one global Sim(3) alignment; RGB-D uses
SE(3) with unit scale. `trajectory_se3.py` is reused from the prior runtime
evaluation tool, with a separate tested Sim(3) adapter in `analyze.py`.
EuRoC official camera GT was checked against `T_WB * T_BC`: maximum position
difference 8.55e-8 m. TUM large-with-loop truth has a long central gap, so
report associated-pose count and truth gap, not full-sequence accuracy.

Initial 400-frame baseline smoke overlapped compilation: functionality only,
excluded from all formal timing conclusions. Runs freeze runner/config/input/
binary hashes before execution and record exit status, including timeouts.

## Preserved experiment history

The first EuRoC matrix predates the fourth build target; the first three
binaries and g2o/DBoW2 retained their hashes after the fourth target build.
Content-addressed CMake/runner/config snapshots are in `provenance/`.
`source_manifest.json` describes the final files; `source_manifest_initial.json`
preserves the initial snapshot. Frozen hashes inside each run remain authoritative.

After all matrices finish (avoid analysis/build load during timing):

```sh
python3 -m unittest discover -s harness -p "test_*.py" -v
python3 harness/export_summary.py runs/euroc_formal runs/euroc_default runs/tum_bgr_formal
python3 harness/plot_results.py runs/tum_bgr_formal
```

## Paper/implementation difference found during audit

In current `Hashing.cc`, `Bucket::add` only prepends: capacity eviction is
commented out. `max_feature_num_per_bucket` is stored/printed but does not
bound that deque. `NUM_CANDIDATE_FEATURES` actually limits query output.
`insert` skips already-hashed points via `mvbHashed` rather than refreshing
their recency on every revisit. Thus the nominal bucket-size setting is not
a verified ring-buffer capacity as described in the 2019 paper.
`harness/test_bucket.cc` checks the actual container using null entries
(which add/back never dereference). Build/run it with `bash harness/run_bucket_probe.sh` only after timed matrices.
This is an audit finding, not an algorithm fix, and does not establish which
source difference caused a particular latency or tracking outcome.
