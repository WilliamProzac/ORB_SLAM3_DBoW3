# Repo Map

Source basis: repository tree scan plus `CMakeLists.txt` and ROS package files.

## Core Areas

- `src/`: main C++ implementation of system, tracking, local mapping, loop closing, optimization, map sparsification, and camera models.
- `include/`: public headers matching the core implementation.
- `CMakeLists.txt`: top-level build graph for `libORB_SLAM3.so` and native example binaries.

## Runtime And Example Areas

- `Examples/`: native example source files, YAML configs, calibration tools, and compiled example binaries already present in this workspace.
- `Examples_old/`: older example tree plus the active ROS package path used by `build_ros.sh`.
- `Examples_old/ROS/ORB_SLAM3/`: ROS package files, rosbuild config, YAMLs, and ROS node sources.
- `Vocabulary/`: vocabulary assets including `ORBvoc.txt`, `ORBvoc.bin`, and `orbvoc.dbow3`.

## Analysis And Utility Areas

- `evaluation/`: trajectory evaluation scripts.
- `tools/`: helper utilities such as `plot_resource_usage.py`.
- Top-level reports such as `DIAGNOSIS_AND_FIXES.md` and memory reports are useful references, but they are not the canonical routing layer.

## Dependency Boundary

- `Thirdparty/`: vendored dependencies including `DBoW3`, `g2o`, and `Sophus`.
- Default stance: do not edit vendored code unless the task is explicitly about dependency behavior or integration.

## Generated Or Environment-Specific Areas

- `build/`, `lib/`: build outputs.
- `logs/`, `bags/`, `compare_results/`, `memory_compare/`: runtime logs, recordings, and analysis outputs.
- Large `.bag` files and generated logs should not be treated as source-of-truth design docs.

## Gaps Found During Scan

- No top-level `docs/` directory existed before these routing docs were added.
- No top-level `tests/` directory was found.
- No repository CI config directory/file was found.
