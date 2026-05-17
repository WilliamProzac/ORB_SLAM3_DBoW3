# Build And Run

Source basis: `README.md`, `CMakeLists.txt`, `build.sh`, `build_ros.sh`, and `run_stereo_inertial.sh`.

## Build Prerequisites Found In Repo

- Required by `CMakeLists.txt`: `OpenCV 4.2`, `Eigen3`, `Pangolin`.
- Optional in `CMakeLists.txt`: `realsense2`.
- Mentioned in `README.md`: ROS is optional and needed for ROS examples.
- TODO: confirm local `realsense2` installation before assuming RealSense example builds will be enabled.

## Core Build Path

Commands documented in `README.md` and implemented by `build.sh`:

```bash
chmod +x build.sh
./build.sh
```

What `build.sh` does:

- Builds `Thirdparty/DBoW3`
- Builds `Thirdparty/g2o`
- Builds `Thirdparty/Sophus`
- Extracts `Vocabulary/ORBvoc.txt.tar.gz`
- Configures and builds the top-level project in `build/`

## ROS Build Path

Commands documented in `README.md` and implemented by `build_ros.sh`:

```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/ywl/Project/ORB_SLAM3_DBoW3/Examples_old/ROS
chmod +x build_ros.sh
./build_ros.sh
```

Notes from repo facts:

- `build_ros.sh` builds `Examples_old/ROS/ORB_SLAM3`.
- The ROS package uses `rosbuild`, not `catkin` or `colcon`.
- ROS executables declared in `Examples_old/ROS/ORB_SLAM3/CMakeLists.txt`: `Mono`, `Stereo`, `RGBD`, `Mono_Inertial`, `Stereo_Inertial`.

## Runtime Wrapper

Documented in `README.md` and implemented by `run_stereo_inertial.sh`:

```bash
./run_stereo_inertial.sh <mode> <sensor> [--log]
```

Modes found in script:

- `mapping`
- `mapping_save`
- `localization`

Sensors found in script:

- `stereo_inertial`
- `rgbd`
- `stereo`

Behavior found in script:

- Requires repo root and checks `Vocabulary/ORBvoc.bin`
- Uses `Vocabulary/orbvoc.dbow3` at runtime
- Selects YAMLs from `Examples_old/ROS/ORB_SLAM3/`
- `--log` writes to `logs/`
- Starts `pidstat` monitoring
- Starts `rosbag record` into `bags/`

TODO:

- Confirm `pidstat` is installed on the target machine.
- Confirm ROS topics in the script match the active camera setup.
- Confirm the intended map save/load YAML settings before using `mapping_save` or `localization`.

## Native Example Build Outputs

Facts from `CMakeLists.txt`:

- The top-level build declares native example executables under `Examples/`.
- RealSense-specific native examples are only added when `realsense2` is found.

TODO:

- No single repo-local script was found that wraps the native example binaries into one validation flow.
