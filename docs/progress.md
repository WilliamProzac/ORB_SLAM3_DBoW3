# Progress Log

## Current Active Feature

- Feature ID: none
- Started: 2026-05-17
- Current state: no active feature; most recently completed feature is `F12` (`passing`)
- Owner/session: stable pose interface for grid-map consumers on `grid_map`
- Blockers:
  - TODO: `F13` remains blocked on loop-closure rebuild runtime evidence.
  - TODO: `F14` and `F15` remain not started.
  - TODO: `F18` remains blocked on gravity-aligned activation and tilt-test evidence.

## Command Log

| Date | Command | Result | Evidence |
|---|---|---|---|
| 2026-05-17 | `git status --short` | passed | confirmed dirty worktree before opening `F12` |
| 2026-05-17 | `sed -n '1,120p' docs/features.md` | passed | `F12` and `F15` boundaries re-read before edits |
| 2026-05-17 | `sed -n '1,180p' docs/validation.md` | passed | `F12` verification path re-read before edits |
| 2026-05-17 | `sed -n '1,220p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | current `Stereo` node data flow reviewed before adding the pose publisher |
| 2026-05-17 | `sed -n '1,80p' Examples_old/ROS/ORB_SLAM3/manifest.xml` | passed | ROS package dependencies reviewed before adding `geometry_msgs` |
| 2026-05-17 | `git diff -- docs/features.md docs/validation.md Examples_old/ROS/ORB_SLAM3/manifest.xml Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | verified the `F12` doc and code changes before build |
| 2026-05-17 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh` | passed | ROS package rebuilt successfully after adding the `/grid_map/pose` `PoseStamped` publisher |
| 2026-05-17 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 20s ./run_stereo_inertial.sh mapping stereo --log` | blocked | sandboxed ROS runtime could not open sockets or write `~/.ros`, so runtime validation had to be retried outside the sandbox |
| 2026-05-17 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 20s ./run_stereo_inertial.sh mapping stereo --log` | partial | outside the sandbox, the `Stereo` node launched, initialized `/grid_map`, and stayed up long enough for live topic inspection, but no tracked pose message was captured within the bounded window |
| 2026-05-17 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic list \| grep '^/grid_map/pose$'` | passed | confirmed `/grid_map/pose` is advertised during runtime |
| 2026-05-17 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic type /grid_map/pose` | passed | confirmed `/grid_map/pose` type is `geometry_msgs/PoseStamped` |
| 2026-05-17 | `rg -n "grid_map|pose|Tracking|track|GrabStereo|map" logs/mapping_stereo_20260517_135331.log` | passed | bounded runtime log showed startup and grid-map initialization but no tracked-frame publish evidence |
| 2026-05-17 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 30s ./run_stereo_inertial.sh mapping stereo --log` | partial | second bounded runtime was started outside the sandbox for longer live-publish observation, then manual validation was handed off to the user |
| 2026-05-17 | User manual runtime validation of `/grid_map/pose` extraction | passed | user confirmed `PoseStamped` output can be extracted successfully from `/grid_map/pose` and matches the intended `map`-frame downstream contract |
| 2026-05-14 | `git checkout master` | passed | branch switched to `master` before branch creation |
| 2026-05-14 | `git checkout -b grid_map` | passed | current branch is `grid_map` |
| 2026-05-14 | `git show feature/map-sparsification:AGENTS.md` | passed | previous branch template reviewed |
| 2026-05-14 | `git show feature/map-sparsification:docs/build-and-run.md` | passed | previous branch template reviewed |
| 2026-05-14 | `git show feature/map-sparsification:docs/repo-map.md` | passed | previous branch template reviewed |
| 2026-05-14 | `git show feature/map-sparsification:docs/validation.md` | passed | previous branch template reviewed |
| 2026-05-14 | `sed -n '1,220p' build.sh` | passed | current branch build entrypoint checked |
| 2026-05-14 | `sed -n '1,220p' run_stereo_inertial.sh` | passed | current branch runtime wrapper checked |
| 2026-05-14 | `sed -n '1,160p' build_ros.sh` | passed | current branch ROS build wrapper checked |
| 2026-05-14 | `rg -n "find_package\\(|GUROBI|realsense2|add_executable\\(" CMakeLists.txt Examples_old/ROS/ORB_SLAM3/CMakeLists.txt` | passed | build facts captured for docs |
| 2026-05-15 | `sed -n '1,220p' docs/features.md` | passed | narrowed the next feature boundary for grid-map work |
| 2026-05-15 | `git status --short` | passed | confirmed dirty worktree before F10 docs-only edits |
| 2026-05-15 | `sed -n '1,220p' AGENTS.md` | passed | governance rules re-read before F10 work |
| 2026-05-15 | `sed -n '1,220p' docs/repo-map.md` | passed | repo lane and target package reconfirmed |
| 2026-05-15 | `sed -n '1,220p' docs/validation.md` | passed | F10 manual verification path reconfirmed |
| 2026-05-15 | `sed -n '1,260p' /home/ywl/Project/ORB-SLAM3-GRID-MAP/README.md` | passed | source repo runtime entrypoints reviewed |
| 2026-05-15 | `sed -n '1,260p' /home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_ROS/src/ros_mono_pub.cc` | passed | source producer node reviewed |
| 2026-05-15 | `sed -n '200,520p' /home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_ROS/src/ros_mono_sub.cc` | passed | source rasterizer and occupancy logic reviewed |
| 2026-05-15 | `sed -n '1,260p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo_inertial.cc` | passed | current repo primary ROS target node reviewed |
| 2026-05-15 | `sed -n '1,260p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | fallback stereo target reviewed |
| 2026-05-15 | `sed -n '1,260p' Examples_old/ROS/ORB_SLAM3/src/ros_rgbd.cc` | passed | fallback RGBD target reviewed |
| 2026-05-15 | `sed -n '1,260p' include/System.h` | passed | current repo export surface checked against source assumptions |
| 2026-05-15 | `sed -n '1,260p' /home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_LIB/include/System.h` | passed | source repo export surface checked for publisher dependencies |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh` | passed | ROS package build completed after correcting Python path precedence for ROS tooling |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages timeout 12s ./run_stereo_inertial.sh mapping stereo` | passed | stereo runtime wrapper launched and the `Stereo` node entered active tracking |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages rostopic list \| grep '^/grid_map$'` | passed | `/grid_map` topic advertised during runtime validation |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages rostopic type /grid_map` | passed | `/grid_map` verified as `nav_msgs/OccupancyGrid` |
| 2026-05-15 | `sed -n '1,240p' README.md` | passed | F13 session initialization re-read project workflow context |
| 2026-05-15 | `sed -n '1,240p' docs/repo-map.md` | passed | F13 work lane reconfirmed as `ros` with minimal core surface changes only if necessary |
| 2026-05-15 | `sed -n '1,240p' docs/validation.md` | passed | F13 verification path defined before code edits |
| 2026-05-15 | `sed -n '1,260p' docs/features.md` | passed | F13 activated as the only in-progress feature for this session |
| 2026-05-15 | `sed -n '1,260p' docs/progress.md` | passed | prior session notes and active-feature state reviewed before F13 edits |
| 2026-05-15 | `sed -n '1,260p' docs/session-handoff.md` | passed | prior handoff reviewed before F13 edits |
| 2026-05-15 | `sed -n '1,260p' docs/grid-map-port-plan.md` | passed | source-port constraints and source reset strategy re-read before F13 edits |
| 2026-05-15 | `sed -n '1,240p' AGENTS.md` | passed | governance rules re-read before F13 edits |
| 2026-05-15 | `rg -n "loop|Loop|MapChanged|BigChange|GetAllMapPoints|GetTrackedMapPoints|KeyFramesInMap|Atlas|Reset" include src Examples_old/ROS/ORB_SLAM3/src` | passed | existing map-change and export interfaces located for the F13 design |
| 2026-05-15 | `sed -n '1,320p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | current stereo grid-map accumulation path inspected before edit |
| 2026-05-15 | `sed -n '1,260p' /home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_ROS/src/ros_mono_sub.cc` | passed | source `updateGridMap(...)` and `resetGridMap(...)` reference logic reviewed for F13 |
| 2026-05-15 | `sed -n '1,260p' include/System.h` | passed | minimal public export surface reviewed for keyframe access |
| 2026-05-15 | `sed -n '520,560p' src/System.cc` | passed | `System::MapChanged()` implementation confirmed for F13 trigger wiring |
| 2026-05-15 | `sed -n '1,260p' include/Atlas.h` | passed | active-map keyframe export path confirmed through `Atlas` |
| 2026-05-15 | `sed -n '1,260p' src/Atlas.cc` | passed | `Atlas::GetAllKeyFrames()` behavior confirmed |
| 2026-05-15 | `sed -n '300,380p' src/KeyFrame.cc` | passed | `KeyFrame::GetMapPoints()` behavior confirmed for rebuild iteration |
| 2026-05-15 | `rg -n "InformNewBigChange\\(" src include` | passed | loop-closure big-change signal sources identified |
| 2026-05-15 | `./build.sh` | passed | rebuilt `libORB_SLAM3.so` so the new `System::GetAllKeyFrames()` interface linked successfully |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh` | passed | rerun after native rebuild; ROS `Stereo` target linked successfully with the F13 interface change |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 20s ./run_stereo_inertial.sh mapping stereo --log` | passed | bounded runtime produced live stereo tracking and keyframe creation, but not a confirmed loop-closure refresh event |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic list` | passed | `/grid_map` remained advertised during the F13 runtime session |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic type /grid_map` | passed | `/grid_map` remained `nav_msgs/OccupancyGrid` after the F13 changes |
| 2026-05-15 | `rg -n "Grid map rebuild triggered|Grid map full rebuild completed|Loop|loop" logs/mapping_stereo_20260515_172506.log` | passed | no loop-closure rebuild evidence was found in the bounded runtime log |
| 2026-05-15 | `git status --short` | passed | confirmed dirty worktree before opening the F16 dynamic-grid feature |
| 2026-05-15 | `sed -n '1,220p' README.md` | passed | F16 session initialization re-read project workflow context |
| 2026-05-15 | `sed -n '1,240p' docs/repo-map.md` | passed | F16 work lane reconfirmed as `ros` |
| 2026-05-15 | `sed -n '1,260p' docs/validation.md` | passed | F16 verification path defined before code edits |
| 2026-05-15 | `sed -n '1,260p' docs/features.md` | passed | F16 activated as the only in-progress feature for this session |
| 2026-05-15 | `sed -n '1,260p' docs/progress.md` | passed | prior progress and active-feature state reviewed before F16 edits |
| 2026-05-15 | `sed -n '1,260p' docs/session-handoff.md` | passed | prior handoff reviewed before F16 edits |
| 2026-05-15 | `rg -n "grid_scale_factor|grid_min_x|grid_max_x|grid_min_z|grid_max_z|resolution_|width_|height_|OccupancyGrid" Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | fixed stereo grid bounds and resolution logic located for F16 |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh` | passed | ROS `Stereo` target rebuilt successfully with dynamic grid growth and 5 cm resolution |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 20s ./run_stereo_inertial.sh mapping stereo --log` | passed | bounded runtime verified stereo startup, keyframe creation, and new 5 cm grid initialization |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic type /grid_map` | passed | `/grid_map` remained `nav_msgs/OccupancyGrid` after the F16 changes |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic echo -n 1 /grid_map/info` | passed | published grid resolution verified as `0.05` with initial size `400 x 420` and origin `(-10, -5)` |
| 2026-05-15 | `rg -n "Grid map initialized|Grid map expanded" logs/mapping_stereo_20260515_204606.log` | passed | runtime log confirmed 5 cm initialization but showed no dynamic expansion event within the bounded window |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh` | passed | ROS `Stereo` target rebuilt successfully after changing the default grid resolution to `0.50 m` and adding grid-storage logging |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 20s ./run_stereo_inertial.sh mapping stereo --log` | passed | bounded runtime verified stereo startup with the rebuilt binary and emitted expansion logs carrying estimated grid storage |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic echo -n 1 /grid_map/info` | passed | published grid resolution verified as `0.5` with early runtime size `46 x 54` and origin `(-10, -5)` |
| 2026-05-15 | `rg -n "Grid map initialized|Grid map expanded" logs/mapping_stereo_20260515_205644.log` | passed | runtime log confirmed `0.5 m` initialization and repeated `approx storage ... MiB` expansion logs, including large outlier-driven growth to `5788 x 26258` cells |
| 2026-05-15 | `tail -n 30 logs/resource_usage_mapping_stereo_20260515_205644.txt` | passed | latest bounded runtime showed `Stereo` RSS around `2.21 GB` during the outlier-driven grid expansion run |
| 2026-05-15 | `sed -n '1,220p' README.md` | passed | F17 session initialization re-read project workflow context |
| 2026-05-15 | `sed -n '1,220p' docs/repo-map.md` | passed | F17 work lane reconfirmed as `ros` |
| 2026-05-15 | `sed -n '1,220p' docs/validation.md` | passed | F17 verification path defined before code edits |
| 2026-05-15 | `sed -n '1,220p' docs/features.md` | passed | F17 activated as the only in-progress feature for this session |
| 2026-05-15 | `rg -n "GetWorldPos|GetMaxDistanceInvariance|GetMinDistanceInvariance|GetNormal|PredictScale|isBad" include/MapPoint.h src/MapPoint.cc` | passed | MapPoint distance-related interfaces reviewed before selecting the distance-filter design |
| 2026-05-15 | `rg -n "ThDepth|Depth Threshold|thDepth|mThDepth|fps|Camera" Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml Examples/Stereo-Inertial/EuRoC.yaml include src` | passed | existing stereo depth-threshold behavior reviewed to compare with the proposed grid-map distance filter |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh` | passed | ROS `Stereo` target rebuilt successfully after adding the grid-map planar-distance filter |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 20s ./run_stereo_inertial.sh mapping stereo --log` | passed | bounded runtime verified the new filter fired repeatedly while `/grid_map` remained healthy |
| 2026-05-15 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic type /grid_map` | passed | `/grid_map` remained `nav_msgs/OccupancyGrid` after the F17 change |
| 2026-05-15 | `rg -n "Grid map initialized|Grid map expanded|Grid map ignored" logs/mapping_stereo_20260515_210715.log` | passed | runtime log confirmed `15 m` planar filtering, repeated ignored-point logs, and restrained grid growth to roughly `56 x 58` cells |
| 2026-05-15 | `tail -n 40 logs/resource_usage_mapping_stereo_20260515_210715.txt` | passed | latest bounded runtime showed `Stereo` RSS around `686-705 MB`, far below the earlier outlier-driven `2.21 GB` run |
| 2026-05-16 | `git status --short` | passed | confirmed dirty worktree before documenting the gravity-aligned projection follow-up |
| 2026-05-16 | `sed -n '1,220p' README.md` | passed | F18 session initialization re-read project workflow context |
| 2026-05-16 | `sed -n '1,220p' docs/repo-map.md` | passed | F18 work lane reconfirmed as `ros` |
| 2026-05-16 | `sed -n '1,220p' docs/validation.md` | passed | F18 verification path and validation gaps checked before editing docs |
| 2026-05-16 | `sed -n '1,220p' docs/features.md` | passed | current feature inventory checked before adding the gravity-aligned projection feature |
| 2026-05-16 | `rg -n "gravity|Gravity|GetGravity|g_dir|IMU|imu|GetImu|GetVelocity|GetPoseInverse|Stereo_Inertial|bImu" include src Examples_old/ROS/ORB_SLAM3/src` | passed | gravity-related interfaces and inertial ROS paths reviewed for the F18 design analysis |
| 2026-05-16 | `nl -ba Examples_old/ROS/ORB_SLAM3/src/ros_stereo_inertial.cc \| sed -n '1,260p'` | passed | current stereo-inertial ROS node reviewed as the closest existing gravity-capable runtime path |
| 2026-05-16 | `rg -n "GetImuBias|GetVelocity|GetPose|GetPoseInverse|GetMapChangeIndex|MapChanged|IMU is not or recently initialized|gravity direction|gravity vector|mRwg|Rwg|GetIniertialBA|GetGravityDirection" include/System.h include/Atlas.h include/Map.h include/KeyFrame.h src/System.cc src/Atlas.cc src/Map.cc src/KeyFrame.cc` | passed | confirmed there is no public gravity-direction getter even though inertial gravity alignment exists internally |
| 2026-05-16 | `rg -n "class LocalMapping|mRwg|isImuInitialized|GetImuPose|GetImuRotation|GetImuPosition" include/LocalMapping.h include/KeyFrame.h src/LocalMapping.cc src/KeyFrame.cc` | passed | identified `LocalMapping::mRwg` as the existing gravity-alignment state and `KeyFrame` IMU pose accessors as relevant implementation hooks |
| 2026-05-16 | `nl -ba src/System.cc \| sed -n '1348,1365p'` | passed | confirmed the current code already exports gravity alignment only through debug-data saving, not through a reusable public API |
| 2026-05-16 | `nl -ba Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc \| sed -n '1,760p'` | passed | current stereo grid-map implementation and callback structure re-read before F18 edits |
| 2026-05-16 | `git diff -- Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | existing uncommitted stereo grid-map work reviewed before applying the gravity-aligned projection change |
| 2026-05-16 | `git diff -- include/System.h src/System.cc` | passed | confirmed the only pre-existing core diff relevant to grid-map was the F13 keyframe export surface |
| 2026-05-16 | `sed -n '1188,1260p' src/LocalMapping.cc` | passed | inertial gravity initialization path reviewed to avoid accidentally changing the visual-only SLAM core |
| 2026-05-16 | `sed -n '1,140p' Examples_old/ROS/ORB_SLAM3/MyD435i.yaml` | passed | IMU calibration layout compared with stereo settings before reusing `IMU.T_b_c1` in the ROS-side estimator |
| 2026-05-16 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh` | passed | ROS `Stereo` target rebuilt successfully after adding IMU-backed gravity projection and rotated occupancy-grid metadata in `ros_stereo.cc` |
| 2026-05-16 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 25s ./run_stereo_inertial.sh mapping stereo --log` | blocked | sandboxed ROS runtime failed to create sockets and `~/.ros` cache files, so runtime validation was retried outside the sandbox |
| 2026-05-16 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 25s ./run_stereo_inertial.sh mapping stereo --log` | passed | outside the sandbox, the stereo node launched and `/grid_map` remained advertised while the new gravity-aware binary initialized successfully |
| 2026-05-16 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic type /grid_map` | passed | `/grid_map` remained `nav_msgs/OccupancyGrid` during F18 runtime validation |
| 2026-05-16 | `rg -n "Grid map|gravity|fallback|projection" logs/mapping_stereo_20260516_213326.log` | passed | first bounded runtime showed only grid initialization because no tracked grid updates were emitted before timeout |
| 2026-05-16 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 12s ./run_stereo_inertial.sh mapping stereo --log` | passed | bounded runtime after restoring the `0.50 m` default produced tracked grid-map updates and explicit fallback logs |
| 2026-05-16 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic list \| grep '^/camera/imu$'` | passed | `/camera/imu` topic name existed during the bounded stereo runtime |
| 2026-05-16 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH timeout 10s ./run_stereo_inertial.sh mapping stereo --log` | passed | bounded runtime after switching the stereo node to `AsyncSpinner` still stayed in fallback mode while producing live grid-map update logs |
| 2026-05-16 | `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH rostopic echo -n 1 /camera/imu` | blocked | no live IMU message was captured within the bounded runtime window, so gravity-aligned activation remained unverified |

## Notes

- `test_euroc.sh` exists on the earlier working branch template but was not found on current `master` or `grid_map`.
- The harness/governance docs were regenerated to match current branch facts instead of copying stale validation commands forward.
- `F10` is now intentionally scoped as a research-and-port-plan feature, not an implementation feature.
- `F10` completed by static source reading only. No ROS build or runtime validation was attempted in this session.
- The main porting conclusion is to reuse the source rasterization logic but avoid a literal `Monopub`/`Monosub` clone in this repo because the source publisher depends on internal SLAM interfaces that are not exposed here.
- `F13` was implemented by keeping the `Stereo` node single-process, adding a minimal `System::GetAllKeyFrames()` export, and rebuilding the occupancy counters from active-map keyframes whenever `System::MapChanged()` reports a loop-closure or global-BA-scale map change.
- The F13 implementation explicitly follows the source repo idea in `ros_mono_sub.cc` of separating incremental accumulation from full-map reset/rebuild, but adapts it to this repo's existing `Stereo` node instead of cloning the source `Monopub`/`Monosub` split.
- `F16` replaces the previous fixed-size stereo occupancy grid with an initial window that can expand outward when the camera or map points exceed the current bounds. The follow-up change in the same feature raises the default cell size to `0.50 m` and adds per-expansion storage estimates to the ROS log.
- The repo-local wrapper and ROS Python tools required corrected `PYTHONPATH` ordering in this shell because the default conda Python path masked part of the ROS Python stack.
- The bounded 20-second runtime session produced tracking and keyframe creation, and `/grid_map` remained available as `nav_msgs/OccupancyGrid`, but the session did not contain a confirmed loop closure, so the new rebuild path was not observed firing in runtime logs.
- The later F16 runtime session verified `0.50 m` resolution through `/grid_map/info` and emitted repeated `Grid map expanded ... approx storage ... MiB` logs, which makes the grid-side memory growth observable during runtime.
- That same runtime also revealed a remaining risk: even at `0.50 m`, outlier-driven bounds can still inflate the grid to `5788 x 26258` cells and roughly `1304 MiB` of estimated grid storage, so the dynamic-growth path is functional but still vulnerable to extreme map extents.
- `F17` addresses that outlier-driven growth at the grid-map layer by ignoring map-point obstacle contributions beyond a configurable planar distance from the current accumulation pose. In the validating runtime, the filter rejected map-point references with reported furthest rejected distances in the hundreds to thousands of meters while the occupancy grid itself stayed near `56 x 58` cells and about `0.03 MiB`.
- `F18` is now implemented in `ros_stereo.cc` without changing the visual-only `System::STEREO` SLAM path. The stereo node now subscribes to `/camera/imu`, reads `IMU.T_b_c1` from the stereo YAML, estimates gravity on the ROS side, projects camera/map-point/ray geometry onto a gravity-aligned plane when samples are available, rotates the `OccupancyGrid` origin pose accordingly, and falls back explicitly to legacy map-frame `x-z` projection when gravity samples are unavailable.
- The implementation also preserves the prior grid-map behavior surface: `F13` rebuilds still use the same builder, `F16` dynamic growth still works on the projected plane, and `F17` planar-distance filtering now measures distance in the projected 2D plane instead of raw `x-z`.
- Bounded runtime validation confirmed the new fallback logs and kept `/grid_map` typed as `nav_msgs/OccupancyGrid`, but it did not capture a gravity-aligned activation log. Even after switching the stereo node to an `AsyncSpinner`, the bounded runtime still reported insufficient IMU samples while `/camera/imu` existed as a topic name, and `rostopic echo -n 1 /camera/imu` did not capture a live message before timeout.
- `F12` is now implemented in `ros_stereo.cc` as a minimal downstream pose interface, separate from `F15`. The `Stereo` node publishes `/grid_map/pose` as `geometry_msgs/PoseStamped` from `Tcw.inverse()` only when tracking is `OK` or `OK_KLT`, using the same `map` world frame expected by `/grid_map`.
- The `F12` doc boundary is now explicit: this feature is only the single `/grid_map/pose` consumer-facing pose topic, while `F15` remains reserved for any later generalized ego-motion or odometry-style contract.
- `F12` is now `passing`: after the earlier build and bounded runtime checks confirmed advertise and type, the user completed the runtime validation and confirmed the pose stream can be extracted successfully from `/grid_map/pose`.
