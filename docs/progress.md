# Progress Log

## Current Active Feature

- Feature ID: F11
- Current state: blocked
- Owner/session: F11 odometry semantics implementation on 2026-05-29
- Blockers:
  - TODO: Fresh ROS runtime validation for the new `odom -> left_camera`
    semantics is blocked because `rosnode list` and `rostopic list` could not
    communicate with a ROS master during this session.
  - TODO: Until a live stereo session is rerun, the new implementation still
    needs runtime confirmation that the first successful `/odometry` pose is
    near the local origin, that `twist` becomes non-zero under motion, and
    that RViz can resolve the new `map -> odom -> left_camera` TF chain.

## Command Log

| Date | Command | Result | Evidence |
|---|---|---|---|
| 2026-05-29 | `./build_ros.sh` after adding TF broadcasting in `ros_stereo.cc` | passed | rosbuild rebuilt the ROS package and relinked `Stereo` successfully after adding `map -> odom` and `odom -> left_camera` TF publication |
| 2026-05-29 | `sed -n '1,320p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `sed -n '320,460p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, and `rg -n "odometry|robot_pose|twist|TrackStereo|tracking|odom" Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` | passed | isolated the F11 change surface to the stereo ROS wrapper and confirmed the existing twist-guard rails before editing |
| 2026-05-29 | `./build_ros.sh` | passed | rosbuild rebuilt the ROS package and relinked `Stereo` successfully after the local-origin odometry change |
| 2026-05-29 | `which pidstat`, `which rosbag`, `rosnode list`, and `rostopic list` | partial | confirmed `pidstat` and `rosbag` exist locally, but both ROS graph checks failed with `ERROR: Unable to communicate with master!`, so runtime validation stayed blocked |
| 2026-05-29 | `git status --short`, `sed -n '1,260p' README.md`, `docs/repo-map.md`, `docs/features.md`, `docs/validation.md`, `docs/progress.md`, and `docs/session-handoff.md` | passed | completed the governance-mandated initialization read for the docs-only F11 follow-up |
| 2026-05-29 | `sed -n '1,260p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `sed -n '240,380p' src/System.cc`, `sed -n '1350,1395p' src/Tracking.cc`, and `sed -n '3600,3725p' src/Tracking.cc` | passed | confirmed the current localization startup flow and why `/odometry` begins in the loaded `map` frame instead of a local zeroed frame |
| 2026-05-25 | `git status --short` | passed | confirmed the worktree already contained unrelated modified and untracked runtime artifacts before implementation |
| 2026-05-25 | `sed -n '1,260p' README.md`, `docs/repo-map.md`, `docs/features.md`, `docs/validation.md`, `docs/progress.md`, and `docs/session-handoff.md` | passed | completed the governance-mandated initialization read before code edits |
| 2026-05-25 | `sed -n '1,320p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `Examples_old/ROS/ORB_SLAM3/CMakeLists.txt`, `Examples_old/ROS/ORB_SLAM3/manifest.xml`, `include/Tracking.h`, `src/Tracking.cc`, `include/System.h`, and `src/System.cc` | passed | captured the `master` branch baseline for the ROS stereo wrapper and relocalization bridge |
| 2026-05-25 | `git show grid_map:Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`, `.../CMakeLists.txt`, `.../manifest.xml`, `.../msg/RelocalizationStatus.msg`, `include/Tracking.h`, `src/Tracking.cc`, `include/System.h`, and `src/System.cc` | passed | extracted the minimum reusable migration surface from `grid_map` without bringing back occupancy-grid code |
| 2026-05-25 | `./build_ros.sh` | passed | rosbuild regenerated `ORB_SLAM3/RelocalizationStatus` and linked the updated `Stereo` node successfully |
| 2026-05-25 | `which pidstat` and `which rosbag` | passed | confirmed wrapper prerequisites were present on the host |
| 2026-05-25 | `rosnode list`, `rostopic list`, and `rostopic list | rg '^/camera/(infra1/image_rect_raw|infra2/image_rect_raw)$'` | passed | confirmed a host ROS master and live stereo image topics were available for runtime validation |
| 2026-05-25 | `./run_stereo_inertial.sh localization stereo --log` | passed | launched `/Stereo` against live stereo inputs, printed `MyD435i_stereo_load.yaml`, and reached relocalization success in the node log |
| 2026-05-25 | `rostopic info /robot_pose`, `rostopic type /robot_pose`, and `rostopic echo -n 1 /robot_pose` | passed | verified `geometry_msgs/PoseStamped` on `/robot_pose` with `header.frame_id: "map"` |
| 2026-05-25 | `rostopic info /odometry`, `rostopic type /odometry`, and `rostopic echo -n 3 /odometry` | passed | verified `nav_msgs/Odometry` on `/odometry`, `map -> left_camera` framing, and non-zero differential `twist` with conservative covariance |
| 2026-05-25 | `rostopic info /relocalization_status`, `rostopic type /relocalization_status`, `rostopic hz /relocalization_status`, and `rostopic echo -n 1 /relocalization_status` | passed | verified package-local status message on `/relocalization_status`, observed `RelocalizationSucceed`, and measured about `10.0 Hz` publish rate |
| 2026-05-25 | `timeout 10s rostopic echo /relocalization_status` during a second localization run | partial | captured the steady-state status stream repeatedly publishing `RelocalizationSucceed`, but did not individually catch `RelocalizationRunning` or `RelocalizationFailed` before convergence |

## Notes

- On 2026-05-29, `F11` was intentionally reopened as a semantics change: the
  current implementation is valid absolute-map odometry, but the new desired
  behavior is session-relative odometry that starts near zero in an `odom`
  frame while leaving `/robot_pose` unchanged as the absolute `map` pose.
- `ros_stereo.cc` now latches the first successfully tracked `Twc` as a
  session-stable odom origin, publishes `/odometry` in the `odom` frame, keeps
  `/robot_pose` unchanged in the `map` frame, and still computes low-confidence
  differential `twist` only from consecutive valid tracked poses.
- `ros_stereo.cc` now also broadcasts `map -> odom -> left_camera`, which is
  required for RViz fixed-frame resolution because the odometry message alone
  does not register those frames in TF.
- Tracking loss now resets only the differential-velocity state. The latched
  odom origin is intentionally preserved across recovery within the same node
  session, so the local frame does not jump while twist resumes from a fresh
  valid-pose pair.
- `Tracking` and `System` now bridge relocalization status into the ROS layer,
  and the ROS package now builds the new `RelocalizationStatus.msg`.
- The historical 2026-05-25 runtime evidence for `/odometry` and
  `/robot_pose` reflects the old absolute-pose implementation and must not be
  reused as proof for the new local-origin F11 semantics.
