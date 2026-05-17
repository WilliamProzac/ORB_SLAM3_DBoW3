# Validation

Source basis: `run_stereo_inertial.sh`, `README.md`, repository tree scan, and build entrypoints.

## First-Party Validation Entry Points Found

- `./run_stereo_inertial.sh <mode> <sensor> [--log]`

## ROS Validation With `run_stereo_inertial.sh`

- Use `mapping`, `mapping_save`, or `localization` according to the target workflow.
- The sensor mode must be one of `stereo_inertial`, `rgbd`, or `stereo`.
- `run_stereo_inertial.sh` records bags with `rosbag record`; it does not replay existing bags.
- The runtime wrapper writes terminal logs to `logs/` when `--log` is used.
- It also writes resource logs in `logs/resource_usage_*`.
- It also records output bags in `bags/`.

## Feature-Specific ROS Validation Notes

- For `F11`, use the stereo path first:
  `./run_stereo_inertial.sh mapping stereo --log`
- During that runtime session, confirm the grid-map topic exists and publishes
  `nav_msgs/OccupancyGrid`, for example with:
  `rostopic echo -n 1 /grid_map/info`
- Equivalent topic inspection is acceptable when message capture is timing-sensitive,
  for example:
  `rostopic list | grep '^/grid_map$'`
  plus
  `rostopic type /grid_map`
- For `F12`, first run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh`
  then run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh mapping stereo --log`
- During the `F12` runtime session, confirm `/grid_map/pose` is advertised,
  confirm `rostopic type /grid_map/pose` reports
  `geometry_msgs/PoseStamped`, confirm `rostopic echo -n 1 /grid_map/pose`
  returns a live message, and confirm the echoed `header.frame_id` is `map`
  so the pose topic stays aligned with the published grid-map world frame.
- For `F12` to be `passing`, the ROS build must succeed and the runtime session
  must produce live `/grid_map/pose` data. If runtime conditions prevent live
  message capture, record `F12` as `blocked` instead of claiming `passing`.
- For `F13`, first run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh`
  then run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh mapping stereo --log`
- During the `F13` runtime session, provoke a loop closure and confirm the
  stereo grid-map path logs a loop-closure-triggered full rebuild or refresh
  after `System::MapChanged()`, and confirm `/grid_map` is still published
  after that rebuild.
- If loop closure cannot be induced reliably with the available ROS/hardware
  setup, record `F13` runtime validation as blocked or partial instead of
  claiming that stale pre-closure accumulation was removed.
- For `F16`, first run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh`
  then run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh mapping stereo --log`
- During the `F16` runtime session, confirm `/grid_map` still publishes
  `nav_msgs/OccupancyGrid`, confirm the published resolution is `0.50 m`, and
  if the scene extends beyond the old fixed coverage window, confirm the node
  logs a grid expansion with estimated storage in MiB and `/grid_map/info`
  width or height increases instead of clipping the map to the old hard
  bounds.
- If the bounded runtime session does not naturally reach the previous map
  boundary, record the dynamic-expansion verification as partial instead of
  claiming the growth path fired.
- For `F17`, first run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh`
  then run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh mapping stereo --log`
- During the `F17` runtime session, confirm `/grid_map` still publishes
  `nav_msgs/OccupancyGrid`, and confirm the stereo grid-map path logs that it
  ignored distant map-point obstacle contributions beyond the configured
  planar-distance threshold while continuing to publish initialization or
  expansion metadata normally.
- If the bounded runtime session never encounters map points beyond the chosen
  threshold, record `F17` as partial or blocked instead of claiming the filter
  was exercised.
- For `F18`, first run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh`
  then run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh mapping stereo --log`
- During the `F18` runtime session, confirm `/grid_map` still publishes
  `nav_msgs/OccupancyGrid`, confirm the stereo grid-map path logs whether it is
  using the gravity-aligned plane or falling back to the legacy map-frame
  `x-z` projection, and confirm `/grid_map/info` still publishes valid
  metadata in the `map` frame after the projection change.
- In the bounded 2026-05-16 `mapping stereo --log` validation session, the
  stereo node repeatedly logged fallback to map-frame `x-z` due to
  insufficient IMU samples for gravity alignment. The `/camera/imu` topic name
  existed, but no live IMU message was captured within the bounded runtime
  window, so gravity-aligned activation remained unverified.
- For `F18` to be `passing`, perform a live tilt test and confirm the
  published occupancy grid remains aligned with gravity rather than simply
  rotating with map-frame pitch or roll.
- If the build passes and the topic still publishes but no live tilt
  correctness evidence is captured, record `F18` as `blocked` instead of
  claiming `passing`.
- If ROS, live stereo topics, IMU data, or hardware are unavailable, record
  the runtime validation as skipped or blocked instead of guessing.
- In this workstation session, ROS Python tooling required
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH`
  to avoid a conda/system path conflict while running `build_ros.sh`,
  `rosbag`, and `rostopic`.

## Native Build Validation

- A successful `./build.sh` confirms the top-level native build path.
- A successful `./build_ros.sh` confirms the ROS package build path under `Examples_old/ROS/ORB_SLAM3/`.

## Validation Done Criteria

- Report which command was run.
- Report whether the validation covered the native build path, the ROS build path, or a runtime ROS session.
- If `run_stereo_inertial.sh` was used, report the selected mode and sensor.
- If dataset paths, ROS environment, or hardware prerequisites are missing, report validation as skipped instead of guessing.

## Known Gaps

- No top-level unit-test framework entrypoint was found.
- No repository CI workflow file was found.
- No dedicated repo-local validation wrapper such as `test_euroc.sh` was found on this branch.
- No checked-in deterministic loop-closure validation flow is currently documented for grid-map refresh behavior after loop closure.
- No checked-in fixture or replay-based validation flow is currently documented for height-threshold occupancy classification in the grid-map path.
- No checked-in runtime validation flow is currently documented for a
  generalized odometry-class or richer ego-motion topic beyond the minimal
  `/grid_map/pose` contract now scoped under `F12`.
- No checked-in deterministic fixture or bag-replay validation flow is
  currently documented for proving gravity-aligned horizontal-grid correctness;
  `F18` still depends on live runtime evidence under an intentionally tilted
  camera/device session and on live IMU traffic actually reaching the stereo
  ROS node.
