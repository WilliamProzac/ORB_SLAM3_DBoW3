# RGB-D Track Timing Debug Build

Feature F26 instruments the RDK-X5 RGB-D path with steady-clock measurements
for controlled 1.0x versus 0.5x rosbag comparisons.

`rgbd_frame_timing.csv` correlates input cadence and dropped sequence numbers
with image clone/resize, grayscale/depth conversion, Frame construction, ORB
extraction, RGB-D depth association, map-lock wait, initialization,
reference-keyframe/motion-model/relocalization paths, local-map tracking and
keyframe decisions. It also records tracking state, keypoints, valid depths,
inliers and LocalMapping queue depth.

ORB extraction is additionally split into six mutually scoped measurements:
pyramid construction, FAST/OctTree keypoint selection, orientation, pyramid
clone/Gaussian blur, descriptor generation, and final keypoint/descriptor
assembly. The original `orb_extract_ms` remains available as the enclosing
total so instrumentation closure can be checked per frame.

`local_mapping_timing.csv` contains one row per keyframe with queue depth/wait,
BoW, keyframe processing, map-point culling/creation, local BA, keyframe
culling, total time, BA problem sizes and abort status.

The node reserves in-memory storage before spinning. No per-frame disk I/O is
performed. Both CSV files are written after `System::Shutdown()`. The new
RGB-D ORB/depth and LocalMapping records are deliberately independent of the
legacy `REGISTER_TIMES` switch; enabling the broad legacy statistics machinery
is not required.

The ARM64 cross-build passed on 2026-08-13 with:

```bash
RDK_WORKSPACE=/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing \
RDK_CORE_SOURCE=/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing/core \
RDK_BUILD_JOBS=1 \
  ./tools/build_rdk_arm64_docker.sh
```

Board runtime validation passed on 2026-08-14 after a corrected single-job
rebuild. Corrected core SHA-256 is
`6df2983c49023752c301b714da2a8f46f9b1c1d76d20b25244fb62541efe0d9b`.
Three valid replays populated both CSVs and shut down cleanly. Full analysis is
in `/home/ywl/Project/ORB_SLAM3_DBoW3/board_rgbd_timing_test_20260814/REPORT.md`.

The six-stage extension was cross-built and replayed at 11 Hz on 2026-08-14.
The new `ros_rgbd` SHA-256 is `35332603...c4494`; the core SHA-256 is
`3a45dc2f...40f81a`. The 470-frame smoke result is under
`/home/ywl/Project/ORB_SLAM3_DBoW3/board_rgbd_orb_stage_timing_20260814/`.

F27 extends the optional debug CSV with initial/fallback FAST time, OctTree
time, standalone copy and Gaussian time, per-level candidates/fallback cells/
split count, complete ORB output hashes and the per-level timing arrays. These
counters are controlled by `ENABLE_ORB_FINE_TIMING` and
`ENABLE_ORB_OUTPUT_HASH`; both default OFF in production. F27 production uses
three persistent extractor workers and passed five 11 Hz runs per bag. See
`/home/ywl/Project/ORB_SLAM3_DBoW3/board_rgbd_orb_optimization_20260817/REPORT.md`.
