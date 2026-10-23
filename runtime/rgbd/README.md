# RGB-D runtime research candidate

This branch publishes the latest recommended RGBD25 candidate from 2026-10-20:
covisibility container optimization, atomic KeyFrame status queries, ORB row
parallelism with FAST workspace reuse, and atomic MapPoint status queries.
The core is the repository root; this directory contains the matching ROS1
workspace, support headers, tests and the exact board configuration.

This is a research checkpoint. Complete-input, callback P95 <=90 ms, workload
comparability, full lifetime and trajectory-accuracy qualification are not all
passing. Publishing the code does not update a board deployment.

## Source identity

The 106 core files match the tested atomic snapshot byte for byte. All 116
actual ARM compilation inputs, including the timing CSV support header, have
SHA256 bindings in [PROVENANCE.json](PROVENANCE.json). The ROS CMake and package
declarations have explicit publication adaptations: the core is resolved from
this repository, MapScaleObservation.cc is included, service-generation order
is declared, and GNU C++17 matches the validated compiler mode. No compiled
C++ implementation or header was edited during publication.

The optional merged-geometry candidate is excluded: it showed no standalone
long-bag net benefit, and its combination with atomic MapPoint queries lost
tracking in one repeat. This branch retains the ordinary RGB-D projection
path and LocalMapMode=off.

## Build the matching ROS wrapper

Use ROS Noetic, CMake >=3.10, a C++17 compiler, OpenCV 4, Eigen, Pangolin, PCL,
Boost serialization/system and OpenSSL development libraries. Install the
ROS dependencies declared in src/orb_slam3_ros-master/package.xml. The custom
robot_interfaces service package is included; no previous devel cache is
required to supply its source.

From the repository root, build DBoW3 and g2o for the SAME architecture and ABI:

```bash
cmake -S Thirdparty/DBoW3 -B Thirdparty/DBoW3/build -DCMAKE_BUILD_TYPE=Release
cmake --build Thirdparty/DBoW3/build -j2
cmake -S Thirdparty/g2o -B Thirdparty/g2o/build -DCMAKE_BUILD_TYPE=Release
cmake --build Thirdparty/g2o/build -j2
source /opt/ros/noetic/setup.bash
catkin_make -C runtime/rgbd -j2 --cmake-args   -DBUILD_ORB_EXTRACTOR_VALIDATION=OFF -DBUILD_RGBD_QUERY_PROBES=ON   --make-args ros_rgbd rgbd_mappoint_probe rgbd_geometry_probe
```

For a native ARM build, also pass -DRDK_TARGET_CPU=cortex-a55, and provide
ARM versions of every dependency. The original full ARM build used GNU 11.3,
OpenCV 4.5.4 and the pinned toolchain image recorded in VALIDATION.md. Its
external SDK/sysroot, compiled dependencies, vocabulary and dataset remain
separate inputs. The legacy root build_ros.sh targets a different rosbuild
wrapper; it is not the matching entry point for this research workspace.

## Method-level checks

After building, the probe binaries are in runtime/rgbd/devel/lib/orb_slam3_ros:

```bash
runtime/rgbd/devel/lib/orb_slam3_ros/rgbd_mappoint_probe contract
mkdir -p /tmp/rgbd-geometry
runtime/rgbd/devel/lib/orb_slam3_ros/rgbd_geometry_probe /tmp/rgbd-geometry 0
```

The MapPoint probe also supports individual cases, uncontended microbenchmarks
and text/binary write/read/reject modes. These tests exercise actual methods
from the matching complete shared library. They do not certify every System,
Atlas, inertial, merge/reset or online-serialization interleaving.

## Results and review

[VALIDATION.md](VALIDATION.md) records the full long-bag matrix, numerical
results, negative candidates, remaining gates and publication checks.
[verify_source.py](verify_source.py) checks the published source hashes and
build declarations without running ROS, accessing a board or replaying data.
Use python3 -B runtime/rgbd/verify_source.py from the repository root.

The original per-run raw artifacts and the 40-run interactive chart remain
local. Their absence from this source-only checkpoint is intentional.
