# Grid-Map Port Plan

Status: complete for `F10` manual planning review on `2026-05-15`

This document records the source read and migration plan required before any
grid-map implementation begins in `ORB_SLAM3_DBoW3`.

## Source Read Summary

- The source project `/home/ywl/Project/ORB-SLAM3-GRID-MAP` implements
  grid-map export as a two-node ROS pipeline, not as a direct extension of the
  normal `Mono` node.
- `ORB_SLAM3_ROS/src/ros_mono_pub.cc` runs `ORB_SLAM3::System` in
  `MONOCULAR` mode and publishes pose-plus-map-point data after each tracked
  frame or keyframe event.
- `ORB_SLAM3_ROS/src/ros_mono_sub.cc` subscribes to that exported data,
  rasterizes it onto a fixed 2D grid in the `x/z` plane, publishes
  `nav_msgs/OccupancyGrid`, and writes debug image outputs under `results/`.
- The source data flow is:
  `Monopub -> pts_and_pose / all_kf_and_pts -> Monosub -> grid_map`.
- The node that produces pose and map points is `Monopub`.
- The node that performs grid rasterization is `Monosub`.
- The published ROS topics in the checked-in source path are:
  `pts_and_pose`, `all_kf_and_pts`, and `grid_map`.
- The source README launches the producer with:
  `rosrun ORB_SLAM3_ROS Monopub ...`
  and the rasterizer with:
  `rosrun ORB_SLAM3_ROS Monosub ...`.
- The saved map outputs actually implemented in the grid-map path are image
  files written by `saveMap()` in `ros_mono_sub.cc`:
  `results/grid_map.jpg`,
  `results/grid_map_thresh.jpg`,
  `results/grid_map_thresh_resized.jpg`,
  plus suffixed variants when a frame/key id is passed.
- `ros_mono_pub.cc` also contains commented-out save calls for
  `results/map_pts_out.obj`,
  `results/map_pts_and_keyframes.txt`,
  and `results/key_frame_trajectory.txt`,
  but those are not active outputs in the checked-in runtime path.
- The algorithm core is not inherently monocular. The rasterizer only consumes
  camera/keyframe pose and 3D map points, projects them onto the ground-plane
  `x/z` grid, and then accumulates free/occupied counts.
- The monocular-specific parts are:
  `TrackMonocular(...)`,
  the image/topic/camera input handling in `ros_mono_pub.cc`,
  and the source-only publisher logic that depends on internal SLAM state such
  as `getMap()`, `getTracker()`, `getLoopClosing()`,
  `mCurrentFrame.is_keyframe`, and `loop_detected`.
- The reusable grid-map logic is concentrated in
  `processMapPt()`, `processMapPts()`, `updateGridMap()`,
  `resetGridMap()`, and `getGridMap()` inside `ros_mono_sub.cc`.
- The core rasterization method is a 2D ray-tracing occupancy accumulator:
  each map point marks an occupied cell, and the ray from the camera/keyframe
  to that point increments visit counts along traversed cells.
- The source implementation uses a fixed grid extent and fixed thresholds.
  Important defaults from `ros_mono_sub.cc` are:
  `scale_factor=3`,
  `cloud_max_x=10`,
  `cloud_min_x=-10`,
  `cloud_max_z=16`,
  `cloud_min_z=-5`,
  `free_thresh=0.55`,
  `occupied_thresh=0.50`,
  `visit_thresh=0`.
- The checked-in README command for `Monosub` passes more positional arguments
  than `parseParams()` actually consumes. The code parses only the first ten
  optional values, so the README and implementation have drift.

## Source Files

- Direct grid-map runtime and packaging files in the source project:
  `/home/ywl/Project/ORB-SLAM3-GRID-MAP/README.md`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_ROS/src/ros_mono_pub.cc`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_ROS/src/ros_mono_sub.cc`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_ROS/CMakeLists.txt`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_ROS/package.xml`
- Supporting source files that the producer path depends on for full-map export
  behavior and loop/keyframe detection:
  `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_LIB/include/System.h`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_LIB/include/Tracking.h`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_LIB/include/LoopClosing.h`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_LIB/include/Frame.h`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_LIB/src/System.cc`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_LIB/src/Tracking.cc`
- `/home/ywl/Project/ORB-SLAM3-GRID-MAP/ORB_SLAM3_LIB/src/LoopClosing.cc`

## Target Integration Path In `ORB_SLAM3_DBoW3`

- The target ROS package in the current repo is
  `Examples_old/ROS/ORB_SLAM3/`.
- The most suitable first integration point is
  `Examples_old/ROS/ORB_SLAM3/src/ros_stereo_inertial.cc`.
- `Stereo_Inertial` is the recommended first mode because:
  this repo's wrapper `run_stereo_inertial.sh` defaults to it,
  the branch already contains custom shutdown/runtime handling there,
  and the D435i workflow in this repo is centered on the stereo-inertial path.
- `Stereo` is the next-best fallback target if IMU-thread integration becomes
  the main blocker, because it shares the same left/right image topology and
  already returns pose from `TrackStereo(...)`.
- `RGBD` should be treated as a later extension, not the first port target,
  because it is not the branch's primary wrapper path and uses a different
  sensor pairing.
- The intended target files for the first implementation pass are:
  `Examples_old/ROS/ORB_SLAM3/src/ros_stereo_inertial.cc`
- `Examples_old/ROS/ORB_SLAM3/CMakeLists.txt`
- `Examples_old/ROS/ORB_SLAM3/manifest.xml`
- A new helper file under `Examples_old/ROS/ORB_SLAM3/src/` is optional, but
  it is not required for the smallest first implementation. If a separate file
  is created, prefer a sensor-agnostic name such as
  `ros_grid_map.cc` instead of a direct `mono_*` port.
- The first-version minimum implementation boundary should stay inside the ROS
  layer. It should not require edits under `src/` or `include/` if that can be
  avoided.
- The current repo already exposes `System::GetTrackedMapPoints()` in
  `include/System.h`, so the first port can reuse tracked map points without
  importing the source repo's monocular publisher node.
- The current repo does not expose the source repo's helper getters and flags
  `getMap()`, `getTracker()`, `getLoopClosing()`, `loop_detected`, or
  `mCurrentFrame.is_keyframe`, so a literal port of `ros_mono_pub.cc` is not a
  drop-in fit.
- Because of that interface mismatch, the cleanest first port is to reuse the
  rasterization logic from `ros_mono_sub.cc` but feed it directly from the
  active `Stereo_Inertial` node after `TrackStereo(...)`, using the returned
  pose and `GetTrackedMapPoints()`.
- For the first port, avoid copying the source repo's `Monopub`/`Monosub`
  split, avoid copying its long positional `Monosub` CLI, and avoid porting
  the source `package.xml` because this repo uses `rosbuild` with
  `manifest.xml`, not the source catkin package layout.
- Keep pose output beyond the occupancy grid out of the first implementation
  boundary because that belongs to `F12`, not `F10` or the first `F11` pass.

## Proposed Migration Steps

- Keep `F10` scoped to documentation only and do not edit implementation files
  during this session.
- When implementation begins in a later session, activate exactly one feature
  first, most likely `F11`.
- Reuse the grid logic from source `ros_mono_sub.cc`, especially the
  occupancy/visit counters, ray traversal in `processMapPt()`, and occupancy
  conversion in `getGridMap()`.
- Adapt that logic to the current `Stereo_Inertial` runtime path instead of
  cloning the source monocular publisher/subscriber topology.
- Derive the current camera pose from the return value of
  `TrackStereo(imLeft, imRight, tImLeft, vImuMeas)` in
  `ros_stereo_inertial.cc`.
- Collect map points from `System::GetTrackedMapPoints()` after each accepted
  frame and feed them into the rasterizer.
- Publish `nav_msgs::OccupancyGrid` from the current repo ROS package under a
  stable topic such as `/grid_map`.
- Keep source-style debug image saving optional and behind an explicit flag or
  later follow-up; it is not necessary for the smallest first behavior goal.
- Treat loop-closure-triggered full-map rebuild as out of scope for the first
  pass unless the implementation proves too unstable without it.
- If loop closure drift must be corrected later, make that a deliberate follow-up
  that either:
  exposes a safe full-map export interface from the core library,
  or adds a controlled ROS-side reset path after a map-change signal.
- Do not open `F12` work while implementing `F11`. Pose interfaces for
  downstream consumers must stay separately scoped.

## Verification Plan

- `F10` verification for this session is manual only:
  confirm that this document cites the source implementation files under
  `/home/ywl/Project/ORB-SLAM3-GRID-MAP`
  and names the intended target files under
  `Examples_old/ROS/ORB_SLAM3/`.
- No build or runtime claim is made for `F10`.
- Before any implementation session begins, the next active feature must carry
  a concrete verification command in `docs/features.md`.
- Expected future validation path for the first implementation feature is:
  `./build_ros.sh`
  followed by a `run_stereo_inertial.sh` runtime session in
  `mapping stereo_inertial` mode,
  plus a ROS topic check that confirms `grid_map` is actually published.
- That future validation path is still a proposal here. It has not been run in
  this session and it has not yet replaced the `TODO` verification entry in
  `F11`.

## Open Risks And Assumptions

- The source implementation is partially coupled to internal monocular SLAM
  state through `getMap()`, `getTracker()`, `getLoopClosing()`,
  `loop_detected`, and `mCurrentFrame.is_keyframe`.
- The current repo does not expose those exact interfaces, so a line-for-line
  port of `ros_mono_pub.cc` would either fail or force extra core-library
  surface changes.
- The source rasterizer assumes a ground-plane-style 2D projection using `x`
  and `z`. It is not a general 3D occupancy map and does not classify
  traversability beyond that projection.
- The grid extent in the source is fixed, not dynamically resized, so the port
  must decide whether to keep fixed bounds or to introduce a new parameter
  source.
- The source README and source parser have drift for `Monosub` arguments, so
  the current repo should not inherit that exact CLI contract.
- The source package metadata is also imperfect for the checked-in code path.
  For example, `ros_mono_sub.cc` uses `nav_msgs/OccupancyGrid`, but the source
  `package.xml` is not the correct build contract to reuse in this repo.
- This session did not run `./build_ros.sh` or any ROS runtime command, so
  there is no runtime correctness claim.
- This plan assumes the first implementation can tolerate incremental map
  accumulation without immediate full loop-closure rebuild support. If that
  assumption proves false, the implementation feature must be rescoped before
  more code is written.
