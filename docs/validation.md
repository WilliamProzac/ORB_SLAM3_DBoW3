# Validation

Source basis: `test_euroc.sh`, `run_stereo_inertial.sh`, `README.md`, repository tree scan, and `bags/`.

## First-Party Validation Entry Points

- `./test_euroc.sh`
- `./test_euroc.sh <SEQUENCE>`
- `./run_stereo_inertial.sh <mode> <sensor> [--log]`

## `test_euroc.sh`

- Runs `./Examples/Stereo-Inertial/stereo_inertial_euroc`
- Uses `./Vocabulary/orbvoc.dbow3`
- Uses `./Examples/Stereo-Inertial/EuRoC.yaml`
- Expects dataset path `/home/ywl/dataset/vicon_room1/<SEQUENCE>`
- Expects timestamps under `./Examples/Stereo-Inertial/EuRoC_TimeStamps/`
- Default sequence is `V1_01_easy`

## ROS Validation With `run_stereo_inertial.sh`

- If validation is done through `./run_stereo_inertial.sh`, also replay a test bag from `bags/` during the session.
- Use `rosbag play` for playback.
- Confirm `roscore` is already running before starting the ROS node. `README.md` documents `roscore` as part of the ROS example flow.
- `run_stereo_inertial.sh` itself records bags with `rosbag record`; it does not replay existing bags.
- The sensor type is encoded in the bag filename, for example `mapping_stereo_20260418_125545.bag`.
- When multiple bags match the target sensor mode, prefer the largest bag first.
- In the current repository scan, the largest bag under `bags/` is `bags/mapping_stereo_20260418_125545.bag`.
- TODO: choose playback remaps and arguments according to the selected bag's topic layout.

## Runtime Evidence

- `run_stereo_inertial.sh --log` can produce terminal logs in `logs/`
- It also writes resource logs in `logs/resource_usage_*`
- It also records output bags in `bags/`

## Validation Done Criteria

- Report which command was run.
- Report whether `roscore` was confirmed running for ROS validation.
- Report which bag under `bags/` was replayed when using `run_stereo_inertial.sh`.
- If dataset paths, ROS environment, or hardware prerequisites are missing, report validation as skipped instead of guessing.

## Known Gaps

- No top-level unit-test framework entrypoint was found.
- No CI workflow file was found.
- No top-level `tests/` directory was found.
