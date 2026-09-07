# Branch and local archive layout

The worktrees were separated on 2026-09-07 without changing the existing branch
tips or deploying a new board binary.

| Worktree | Branch | Starting commit | Responsibility |
|---|---|---|---|
| `/home/ywl/Project/ORB_SLAM3_DBoW3` | `feature/xfeat-f23` | `419878d` | F23 float descriptors and XFeat stereo/temporal tracking, descriptor tests and F23 evidence |
| `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing/core` | `debug/rgbd-runtime-diagnostics` | `84a964b` | Optimized ORB core, map statistics, IMU startup and localization fixes, runtime evaluation and OOM tooling |
| `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing` | `debug/rgbd-runtime-diagnostics` | `a28ac28` in a separate local repository | ROS RGB-D/IMU nodes, build configuration and board runners |

The last two entries are separate Git repositories even though one directory
contains the other. Commit core and ROS/build changes in their owning repository.
The existing `agent/ros-pose-continuity-xfeat-rdk-evaluation`,
`debug/rgbd-track-timing`, `experiment/lawn-roi-ablation`, `master`, `grid_map`
and `feature/map-sparsification` branches retain their previous tips.

## Source inheritance

`419878d` already committed the full F23 source snapshot, `SaveMap`, and the
integer global-BA generation counter. These files are inherited on the XFeat
branch rather than copied into another implementation commit. F23 remains
`blocked`: its short replay passed, but the full lawn replay lost tracking and
the required EuRoC gate has not been run. F24 remains `not_started`.

The runtime branch inherits the RGB-D timing and ORB acceleration commits through
`84a964b`. Its pending core changes are map population accessors,
`mpCurrentKeyFrame` initialization, `IsImuInitialized`, and velocity-model
recovery after relocalization. The 18 lines previously present in both worktrees
are included once here. Later Frame/Tracking/ORBextractor optimizations were not
overwritten with the older XFeat snapshot.

## Feature IDs and historical evidence

F26 and F27 retain their meaning in the RGB-D base: detailed timing and ORB
extraction optimization. Imported experiments use RD26–RD35 to avoid collisions.
Their scripts, environment variables and archived artifact paths retain the
historical f26–f35 spelling. ROI experiments and their F32–F35 IDs remain on
`experiment/lawn-roi-ablation`. BR01 records this branch/archive separation only;
it does not re-certify SLAM runtime quality.

## Local archive

Archive root:
`/home/ywl/Project/RDK_X5_orb_slam/local_archive/20260907_git_reorganization`.

- `backups/`: original patches, source files, statuses, refs and worktree paths
  for all three repositories/worktrees involved.
- `backup_manifest.json`: SHA-256 and modes for 54 saved source/config files.
- `artifacts/`: runtime CSVs/logs, images, board backups, deployment packages,
  F23 replay outputs and the local ARM debugger bundle.
- `artifact_manifest.json`: 7,626 file/link records, including file SHA-256.
- `artifact_moves.json`: original-to-archive path mapping.
- `migration_summary.json` and `validation.json`: migration and validation results.

Original artifact locations are ignored local symlinks in both core worktrees.
They preserve script/report paths while sharing one archived copy. Symlinks and
archive contents are not part of the commits. A fresh clone needs its datasets
and archived evidence supplied separately. Existing tracked historical evidence
is unchanged; the ignore rules do not remove files from Git history.

To recover a source file, locate its original worktree and relative path in
`backups/*/files/` and copy only that file to the intended checkout. To recover a
complete pending patch, apply `backups/*/unstaged.patch` to the recorded base in
a separate checkout. `artifact_moves.json` identifies every archived data path.

## Runtime build and data roots

Use the outer runtime workspace's `tools/build_rdk_arm64_docker.sh`. Its default
workspace is the directory containing that script's `tools/` directory, and its
default core is that workspace's `core/`. `RDK_WORKSPACE` and `RDK_CORE_SOURCE`
remain explicit overrides. The independent XFeat support/model source root can
be set with `RDK_XFEAT_REPO`.

The runtime evaluation runners accept `SLAM_DATA_ROOT` for host bag files:

```bash
cd /home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing/core
export SLAM_DATA_ROOT=/home/ywl/Project/ORB_SLAM3_DBoW3
```

Run `evaluation/run_f27_stereo_vs_imu_rate1.sh`,
`evaluation/run_f28_stereo_parallel_one.sh`,
`evaluation/run_f33_rgbd_orb_one.sh`, or the scripts under `tools/oom_debug/`
from their owning runtime core worktree. Historical replay runners still need
their documented board stage, ROS setup, bags and reindexed bag where specified.
No replay or board deployment was performed during this reorganization.

## Frozen production deployment

The board's last observed hand-held run on 2026-09-04 used
`/userdata/orb_slam/rgbd_orb_optimization_20260817/production_w3/` with
`S316_f700_l5.yaml` from the long-run configuration directory.

- `ros_rgbd`: SHA-256 `a41fd8359313a0c78bb690743c08b964a18491b94c21d797019986666a99cc7b`.
- `liborb_slam3_ros.so`: SHA-256 `51c3e4d720e5e8c22e63405b44c2b39a782e55464347016ff2bf1db9987a2860`.

The local runtime workspace's newer library is `852a8b19dc5d6a11e798296b8b673777e6cc91a9d0f8495c9110f03dc7b19c1e`.
It exposes map statistics and IMU initialization accessors that the frozen
production library does not. Branch organization does not imply these newer
changes have been deployed. OOM scripts intentionally retain the frozen binary
hash checks so their evidence remains tied to the observed production version.
