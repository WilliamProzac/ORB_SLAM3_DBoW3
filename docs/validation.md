# Validation

Source basis: `run_stereo_inertial.sh`, `README.md`, repository tree scan, and build entrypoints.

## First-Party Validation Entry Points Found

- `./run_stereo_inertial.sh <mode> <sensor> [--log]`

## ROS Validation With `run_stereo_inertial.sh`

- Use `mapping`, `mapping_save`, or `localization` according to the target workflow.
- The sensor mode must be one of `stereo_inertial`, `rgbd`, or `stereo`.
- `run_stereo_inertial.sh` records bags with `rosbag record`; it does not replay existing bags.
- Before treating a runtime validation attempt as meaningful, verify whether the
  required upstream ROS input topics are already live for the selected mode,
  especially stereo image topics and IMU topics when the feature depends on
  them.
- If the required upstream topics are absent, start the wrapper with
  `--no-bag` and replay a checked-in bag manually rather than letting the
  wrapper record a nested bag while no input exists.
- The runtime wrapper writes terminal logs to `logs/` when `--log` is used.
- It also writes resource logs in `logs/resource_usage_*`.
- It also records output bags in `bags/`.
- A known replay path for this workstation is:
  start the wrapper with
  `./run_stereo_inertial.sh <mode> <sensor> --log --no-bag`
  and then from `/home/ywl/Project/ORB_SLAM3_DBoW3/bags` run
  `rosbag play mapping_stereo_20260517_133550.bag`.

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
- For `F15`, first run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh`
- Before launching the `F15` runtime, inspect whether the required stereo
  image source topics are already live with:
  `ROS_MASTER_URI=http://localhost:11311 timeout 6s rostopic hz /camera/infra1/image_rect_raw`
  and
  `ROS_MASTER_URI=http://localhost:11311 timeout 6s rostopic hz /camera/infra2/image_rect_raw`
- If the stereo image topics are already live, run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh mapping stereo --log`
- If the stereo image topics are not live, run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh mapping stereo --log --no-bag`
  and then from `/home/ywl/Project/ORB_SLAM3_DBoW3/bags` run:
  `rosbag play mapping_stereo_20260517_133550.bag`
- During the `F15` runtime session, confirm `/odometry` is advertised,
  confirm `rostopic type /odometry` reports `nav_msgs/Odometry`, confirm
  `rostopic echo -n 1 /odometry` returns a live message, confirm the echoed
  `header.frame_id` is `map`, confirm the echoed `child_frame_id` is
  `left_camera`, confirm the pose values change plausibly with live or replayed
  motion, confirm `twist.twist` remains zero-filled, confirm pose and twist
  covariance diagonals are set to the documented large sentinel values rather
  than a real estimated covariance, and confirm `/grid_map/pose` still reports
  `geometry_msgs/PoseStamped`.
- For `F15`, the pose source is still `Tcw.inverse()` from the valid tracked
  stereo frame. The richer contract adds clearer odometry framing metadata, but
  it does not add a new velocity estimator, TF broadcaster, or covariance
  estimator.
- For `F15`, validation must reject a runtime procedure that launches
  `run_stereo_inertial.sh` with no live stereo input and no replay source. If
  the image topics are dormant, use `--no-bag` and replay the documented bag
  so the node actually receives frames before judging the odometry topic.
- For `F15` to be `passing`, the ROS build must succeed and the runtime
  session must produce live `/odometry` messages that match the documented
  `map -> left_camera` contract. If code is implemented but runtime evidence is
  missing, leave `F15` as `active` or mark it `blocked` instead of claiming
  `passing`.
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
- For `F19`, first run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./build_ros.sh`
- Before launching the `F19` runtime, inspect whether the required source
  topics for localization are already live, for example the stereo image
  topics and any IMU topic expected by the running setup.
- If those source topics are already live, run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh localization stereo --log`
- If those source topics are not live, run:
  `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages:$PYTHONPATH ./run_stereo_inertial.sh localization stereo --log --no-bag`
  and then from `/home/ywl/Project/ORB_SLAM3_DBoW3/bags` run:
  `rosbag play mapping_stereo_20260517_133550.bag`
- During the `F19` runtime session, confirm the relocalization-status topic is
  advertised as `/grid_map/relocalization_status`, confirm
  `rostopic type /grid_map/relocalization_status` reports
  `ORB_SLAM3/RelocalizationStatus`, confirm the payload includes
  `timestamp_ns` and `status`, confirm `timestamp_ns` is populated from
  `ros::Time::now().toNSec()` at publish time rather than from a stored event
  timestamp, confirm no relocalization-status message is published before the
  first relocalization attempt begins, confirm the latest status is then
  published every `0.1 s`, confirm `rostopic hz /grid_map/relocalization_status`
  or equivalent inspection reports an approximately `10 Hz` publish rate while
  the node is running, confirm pure localization startup emits
  `RelocalizationRunning` when the tracker is forced into `LOST`, and confirm
  the stream later emits either `RelocalizationSucceeded` on successful
  relocalization or `RelocalizationFailed` when `Tracking::Relocalization()`
  returns false.
- For `F19`, the implementation boundary intentionally excludes
  `RelocalizationNone` and `RelocalizationCanceled` because the current code
  does not expose clean, first-class state transitions for those cases. If a
  later implementation needs either status, document the new source-of-truth
  transition before expanding the contract.
- For `F19`, the `10 Hz` requirement means validation must reject an
  implementation that only publishes on state-transition edges. The runtime
  contract is to publish the latest relocalization status continuously at
  approximately `10 Hz` during pure-localization operation, while updating the
  `status` field when the underlying relocalization outcome changes.
- For `F19`, validation must also reject a runtime procedure that launches
  `run_stereo_inertial.sh` with no live input topics and no replay source.
  If upstream topics are absent, use `--no-bag` and replay the documented bag
  so the localization node actually receives data.
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
- No checked-in deterministic fixture or bag-replay validation flow is
  currently documented for proving gravity-aligned horizontal-grid correctness;
  `F18` still depends on live runtime evidence under an intentionally tilted
  camera/device session and on live IMU traffic actually reaching the stereo
  ROS node.
