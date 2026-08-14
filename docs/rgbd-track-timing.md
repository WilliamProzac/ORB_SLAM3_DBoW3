# RGB-D Track Timing Debug Build

Feature F26 instruments the RDK-X5 RGB-D path with steady-clock measurements
for controlled 1.0x versus 0.5x rosbag comparisons.

`rgbd_frame_timing.csv` correlates input cadence and dropped sequence numbers
with image clone/resize, grayscale/depth conversion, Frame construction, ORB
extraction, RGB-D depth association, map-lock wait, initialization,
reference-keyframe/motion-model/relocalization paths, local-map tracking and
keyframe decisions. It also records tracking state, keypoints, valid depths,
inliers and LocalMapping queue depth.

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
