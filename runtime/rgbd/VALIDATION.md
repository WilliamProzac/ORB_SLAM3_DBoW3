# Validation scope and results

This is the source publication of the RGBD25 atomic_02 research candidate,
not a new board deployment or a declaration that the original performance
contract passes. All times below are milliseconds. Each row uses every
completed callback from a full 2910-pair, 1x replay starting at the first frame.
Missing callbacks are retained separately.

## Preserved long-bag comparison

The fixed eight-round order was B/G/A/GA/GA/A/G/B. B is the existing container,
KeyFrame-atomic and ORB-P combination; G adds merged geometry; A adds MapPoint
atomic queries; GA adds both. Only A is selected for this source publication.

| Run | Candidate | Completed | Missing | Callback mean | P95 | Tracking |
|---|---|---:|---:|---:|---:|---|
| b_01 | B | 2900 | 10 | 78.352 | 96.309 | recorded frames state2/fresh/valid/map0 |
| g_01 | G | 2899 | 11 | 78.867 | 98.082 | same |
| a_01 | A | 2907 | 3 | 74.883 | 94.153 | same |
| ga_01 | GA | 2910 | 0 | 74.549 | 89.395 | same |
| ga_02 | GA | 2910 | 0 | 73.713 | 87.401 | 35 invalid fresh poses, then new map |
| a_02 | A | 2910 | 0 | 75.353 | 90.976 | recorded frames state2/fresh/valid/map0 |
| g_02 | G | 2910 | 0 | 76.628 | 95.012 | same |
| b_02 | B | 2909 | 1 | 76.652 | 97.159 | same |

Across the two B and A rounds, equal-run medians of the per-run statistics:
callback mean 77.502 ->75.118 (-3.08%), P95 96.734 ->92.564 (-4.31%).
A3 gather CPU mean 8.642 ->7.076; Search CPU mean 13.287 ->12.483.
Total missing positions 11 ->3. Both A pairings improved, without recorded
tracking failure in completed callbacks.

These are observational full-system improvements. Control means differ by
1.700 ms, above the 1.550 ms budget. Local KF/MP/scan workload differs too.
They do not establish an exact fixed-workload callback speedup. A still fails
the full-input and <=90 ms goals. G shows no net long-bag benefit; GA2's lower
latency is invalid as success evidence because it lost tracking and changed
map. No candidate passed every predeclared engineering gate.

The original setup was 640x544, 700 features, five levels, three ORB executors,
queues 8/8/10, LocalMapping and LoopClosing enabled, viewer disabled and
LocalMapMode=off. Both sides retained the same base Scale/detail statistics;
no new fine lock recorder or CPU sampler was added to these eight runs.

## Native and implementation validation already completed

- Three independent candidate cold ARM builds, each 41 core and two node units;
  zero reused core objects. Baseline binaries were reused exactly as recorded.
- 4411 static geometry cases per variant matched all fields and bytes.
- 168 native process cases passed across the four versions: retirement,
  replacement, lock behavior, strong-owner lifetime, text/binary archive
  compatibility and exceptions, including 32 post-bool truncation cases.
- Fixed hot-cache, uncontended live isBad CPU cost was about 94.68 ->7.34 ns.
  This is a local operation measurement, not a callback speedup estimate.
- Actual ARM isBad disassembly has LDARB and no mutex call on its normal path.
- Independent raw CSV reduction matched 3308 comparisons. The final local
  archive verifier passed 664 frozen files and 1169 old evidence inputs.

The tested ARM compiler was GNU 11.3 with GNU C++17/cortex-a55; OpenCV was 4.5.4.
The offline toolchain image ID was
sha256:c073994816af4aa6c776a98adf5bb1020b92134b4f59f5dec357c5dcb3fdd349.
Exact tested binary and compilation-input hashes are in PROVENANCE.json.

## State and lifetime contract

isBad is an acquire load of an atomic flag. Writers release-publish true only
after their local retirement/replacement transition. Features/position locks,
replacement-chain synchronization, identity barriers and strong keepAlive
ownership remain. A false query may return during a pre-publication retirement
window; true does not promise external map/keyframe cleanup has finished.
Reading the flag does not permit unsynchronized access to other ordinary fields.
Serialization uses a temporary bool in the original archive position.

No OOM injection, exhaustive thread interleaving, full Atlas online-save,
inertial/merge/reset qualification or ground-truth ATE/RPE acceptance is claimed.
Historical full-System leak/lifetime limitations remain open.

## Publication checks

The source integrity verifier checks 130 published source/support/fixture/config
paths and all 116 original ARM compilation inputs. An independent full-import
review passed 272 checks: source identity, complete support inputs, the exact
41-TU core CMake list, GNU17 flags and generated service dependencies.
The final reduced publication set additionally passed 398 checks, including
all 130 working/staged SHA bindings and all 116 exact ARM input mappings.

ROS CMake/publication adaptations are explicitly recorded in PROVENANCE.json.
The compiled C++ source and headers remain byte-identical to atomic_02.
Seven uncompiled camera/AR/before-reference files from the input archive are
explicitly omitted; none is one of the 116 actual compilation inputs.
Publication host validation completed on 2026-10-23:
- The fresh catkin target build compiled 41 core units, two RGB-D node units
  and the two fixture units successfully with GNU 9.4/GNU17/OpenCV 4.2.
- Both custom service headers were generated in this workspace; the 43
  core/node compile commands contain no previous catkin devel include path.
- The actual newly built host DSO passed all 11 MapPoint contract cases and
  the geometry probe's 4411 static cases plus concurrent snapshot checks.
- Existing host DBoW3/g2o shared libraries were reused through ignored local
  symlinks for this integration build; these binary files are not committed.
  This is not a claim of a new full dependency bootstrap or board run.
- The source-only verifier passed all 130 publication files, 116 original
  ARM compilation inputs and the exact 41-unit core declaration.
No board replay or production update is part of publication.
