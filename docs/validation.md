# Validation

Source basis: `README.md`, `build.sh`, `build_ros.sh`,
`run_stereo_inertial.sh`, and repository tree scan on `master`.

## First-Party Validation Entry Points Found

- `./build.sh`
- `./build_ros.sh`
- `./run_stereo_inertial.sh <mode> <sensor> [--log]`

## Build Validation

- `F01`: run `./build.sh` from repo root.
- Success means the script completes its third-party builds, extracts
  `Vocabulary/ORBvoc.txt.tar.gz`, and finishes the top-level CMake build.
- `F02`: export `ROS_PACKAGE_PATH` as documented in `README.md`, then run
  `./build_ros.sh`.
- Success means the rosbuild package under `Examples_old/ROS/ORB_SLAM3/`
  finishes building.

## Runtime Validation With `run_stereo_inertial.sh`

- Valid modes on this branch are `mapping`, `mapping_save`, and
  `localization`.
- Valid sensors on this branch are `stereo_inertial`, `rgbd`, and `stereo`.
- The wrapper writes terminal logs to `logs/` only when `--log` is used.
- The wrapper always starts `pidstat` monitoring and `rosbag record`.
- No checked-in `--no-bag` switch exists on this branch.
- Before treating a runtime attempt as meaningful, confirm the required ROS
  inputs are already live for the selected sensor, because the wrapper records
  bags but does not replay them.
- If ROS, camera topics, IMU topics, `pidstat`, or bag-recording prerequisites
  are missing, record the validation as `blocked` or `skipped` instead of
  inventing an unsupported workaround.

## Feature-Specific Runtime Notes

- `F03`: run `./run_stereo_inertial.sh mapping stereo_inertial --log`.
- During `F03`, confirm the script prints the selected mapping YAML, creates a
  log file under `logs/`, creates a resource log under `logs/`, starts bag
  recording under `bags/`, and keeps the ROS node alive until the session is
  intentionally stopped or a real runtime error occurs.
- `F05`: run `./run_stereo_inertial.sh localization <sensor> [--log]`.
- During `F05`, confirm the script prints the expected `*_load.yaml` path for
  the chosen sensor and launches the matching ROS executable without extra
  undocumented flags.
- `F11`: first run `./build_ros.sh`, then run either
  `./run_stereo_inertial.sh mapping stereo --log` or
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F11`, confirm `/odometry` is advertised as `nav_msgs/Odometry`,
  confirm `header.frame_id` is `odom`, confirm `child_frame_id` is
  `left_camera`, confirm the first published pose after successful tracking is
  near a fresh local origin instead of inheriting the loaded map origin, and
  confirm `twist` is derived from consecutive tracked poses instead of staying
  permanently zero once the robot is moving.
- For `F11`, low-confidence twist is acceptable, but the implementation should
  make that uncertainty explicit through covariance or another documented
  contract instead of pretending the differential estimate is high confidence.
- For `F11`, also confirm `/robot_pose` still reports the absolute `map` pose
  after any local-origin odometry change.
- For `F11`, also confirm the node broadcasts a TF chain that makes `map`,
  `odom`, and `left_camera` visible to RViz. The 2026-05-29 follow-up added
  `map -> odom` plus `odom -> left_camera` broadcasting specifically because
  publishing `/odometry` alone does not create RViz fixed frames.
- The 2026-05-29 implementation session reran `./build_ros.sh` successfully
  after the local-origin odometry change, but host-side runtime validation was
  blocked because `rosnode list` and `rostopic list` could not communicate
  with a ROS master before `run_stereo_inertial.sh` could be exercised.
- `F12`: first run `./build_ros.sh`, then run
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F12`, confirm `/relocalization_status` is advertised at
  approximately `10 Hz`, confirm the built message carries only
  `timestamp_ns` and `status`, and confirm the runtime output can emit exactly
  `RelocalizationRunning`, `RelocalizationSucceed`, and
  `RelocalizationFailed`.
- For `F12`, `grid_map` already provides a migration template, but validation
  must reject a straight port if it still emits `RelocalizationSucceeded`
  instead of the requested `RelocalizationSucceed`.
- The 2026-05-25 validation session measured `/relocalization_status` at about
  `10.0 Hz` and directly sampled `RelocalizationSucceed`; because the
  localization session converged quickly, the short topic captures did not
  separately observe `RelocalizationRunning` or `RelocalizationFailed` before
  the steady-state publish loop resumed.
- `F13`: first run `./build_ros.sh`, then run either
  `./run_stereo_inertial.sh mapping stereo --log` or
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F13`, confirm `/robot_pose` is advertised as
  `geometry_msgs/PoseStamped`, confirm `header.frame_id` is `map`, and
  confirm the published pose follows the live tracked position.
- If F11 is updated to local-origin odometry, confirm F13 still exposes the
  absolute tracked pose in `map` rather than the rebased `odom` pose.
- `F04`: validate by reading the documented entrypoints and confirming this
  branch's governance docs do not claim unsupported wrapper options or
  branch-only runtime topics.
- `F10`: validate by confirming `docs/grid-map-port-plan.md` is clearly marked
  as historical reference material rather than current `master` behavior.

## Validation Done Criteria

- Record the exact command that was run.
- Record whether the validation covered the native build, ROS build, or ROS
  runtime path.
- If `run_stereo_inertial.sh` was used, record the selected mode and sensor.
- Record whether live ROS inputs were present.
- If validation was not run, say so explicitly and explain why.

## Known Gaps

- No top-level unit-test entrypoint was found.
- No repository CI workflow file was found.
- No checked-in automated replay wrapper was found for the modified ROS flow.
- `run_stereo_inertial.sh` couples runtime launch with `pidstat` and bag
  recording, so runtime validation currently depends on those external tools
  being available.
