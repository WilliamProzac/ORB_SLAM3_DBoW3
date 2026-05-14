# AGENTS.md

Root entrypoint for contributors and coding agents working in `ORB_SLAM3_DBoW3`.
Keep this file short. Put detailed procedures in `docs/`.

## Project Overview

- This repo is a modified `ORB-SLAM3` with `DBoW3` support, localization startup changes, ROS wrapper changes, and helper scripts documented in `README.md`.
- The main build system is top-level `CMakeLists.txt` plus shell wrappers `build.sh`, `build_ros.sh`, and `run_stereo_inertial.sh`.
- Primary code lives in `src/` and `include/`. Native example binaries and YAMLs live in `Examples/`. The ROS package directory is `Examples_old/ROS/ORB_SLAM3/`, and `build_ros.sh` exports `Examples_old/ROS` into `ROS_PACKAGE_PATH`.
- No `package.json`, no `pyproject.toml`, no top-level `Makefile`, and no repository CI config (`.github/` or `.gitlab-ci.yml`) were found during scan.
- No top-level `tests/` or `test/` directory was found during scan.

## Start Here

1. Read `README.md` for project intent and the customized workflow notes at the top.
2. Read `docs/repo-map.md` to choose the correct work area before editing.
3. Pick the narrowest task lane: core library, ROS package, evaluation/tooling, or docs.
4. Check `git status --short` before edits because this repo may already contain user changes and generated artifacts.

## Commands

- Core build:
  `./build.sh`
- ROS build:
  `./build_ros.sh`
- ROS runtime wrapper:
  `./run_stereo_inertial.sh mapping stereo_inertial --log`
  `./run_stereo_inertial.sh localization rgbd`
- EuRoC validation entrypoint:
  `./test_euroc.sh`
  `./test_euroc.sh <SEQUENCE>`
- TODO: No repo-local CI command was found.
- TODO: No top-level automated unit-test command was found. The only first-party validation entrypoint discovered at repo root is `test_euroc.sh`.

## Hard Rules

- Do not invent Node, Python packaging, or Make-based workflows for this repo. The scanned entrypoints are CMake, shell scripts, and ROS.
- Prefer existing scripts over ad hoc command sequences when the script already encodes the workflow.
- Run `build.sh`, `build_ros.sh`, `run_stereo_inertial.sh`, and `test_euroc.sh` from the repo root if you use their checked-in relative paths.
- `run_stereo_inertial.sh` checks for `Vocabulary/ORBvoc.bin`, but its `VOCAB_PATH` is set to `Vocabulary/orbvoc.dbow3`. It does not pass `Vocabulary/ORBvoc.txt` to `rosrun`.
- Treat `Thirdparty/` as vendored code. Do not edit it unless the task explicitly targets a dependency.
- Treat `build/`, `lib/`, `logs/`, `bags/`, and similar runtime/output directories as generated artifacts unless the task says otherwise.
- If a command depends on local hardware, ROS environment, dataset paths, or external system dependencies, mark the uncertainty with `TODO` instead of guessing.

## Task Scope

- `src/`, `include/`: core SLAM library, tracking, mapping, loop closing, map sparsification, serialization, camera models.
- `Examples/`: native examples, compiled example binaries, dataset YAMLs, calibration helpers.
- `Examples_old/ROS/ORB_SLAM3/`: ROS package, ROS nodes, ROS YAML configs, `manifest.xml`, and rosbuild-based build path.
- `evaluation/`, `tools/`: offline analysis and plotting helpers.
- `Vocabulary/`: required vocabulary assets.
- Out of default scope unless requested: `Thirdparty/`, large bag files, generated logs, and build outputs.

## Definition of Done

- Every command written in docs or comments must exist in the repo or be directly supported by scanned config/scripts.
- Relevant build path is either run successfully or explicitly reported as not run.
- For ROS-facing changes, confirm the touched node/config lives under `Examples_old/ROS/ORB_SLAM3/`.
- For validation work, use `./test_euroc.sh` only when the expected dataset path `/home/ywl/dataset/vicon_room1/<SEQUENCE>` exists; otherwise report the skip clearly.
- If a workflow or directory contract changes, update the matching doc in `docs/`.

## Documentation Routing

- Build, prerequisites, and run modes: `docs/build-and-run.md`
- Codebase map and ownership boundaries: `docs/repo-map.md`
- Validation entrypoints and evidence expectations: `docs/validation.md`
- `README.md` remains the combined upstream/custom background document; do not duplicate it here.

## Session End Checklist

- Record what commands were actually run.
- Record what was not run and why.
- List touched files.
- Note any unverified environment assumptions such as ROS setup, `realsense2`/RealSense availability, GUROBI installation, `pidstat`, `rosbag`, or dataset paths.
- Leave explicit `TODO` markers for unresolved facts instead of silent assumptions.
