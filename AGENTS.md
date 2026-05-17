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

## Initialization Phase

Before editing code:

1. Read `README.md`, `docs/repo-map.md`, `docs/validation.md`, and `docs/features.md`.
2. Check `git status --short`.
3. Select exactly one active feature from `docs/features.md`.
4. Identify the verification command or documented manual verification procedure for that feature before editing.
5. If no valid verification path exists, record the gap in `docs/validation.md` and mark the feature as `blocked` instead of guessing.

## Commands

- Core build:
  `./build.sh`
- ROS build:
  `./build_ros.sh`
- ROS runtime wrapper:
  `./run_stereo_inertial.sh mapping stereo_inertial --log`
  `./run_stereo_inertial.sh localization rgbd`
- TODO: No repo-local CI command was found. Track validation gaps in `docs/validation.md`.
- TODO: No top-level automated unit-test or dedicated validation script was found during scan. Prefer adding small repo-local validation scripts before claiming feature completion.

## Feature List Rules

- The authoritative feature list is `docs/features.md`.
- Each coding session must select exactly one active feature unless the user explicitly says otherwise.
- A feature must include: `id`, `behavior`, `verification`, `state`, and `evidence`.
- Valid states are: `not_started`, `active`, `blocked`, and `passing`.
- Do not mark a feature as `passing` unless its verification step has completed successfully and the evidence is recorded.
- If verification cannot be run because of hardware, ROS setup, dataset paths, or external dependencies, mark the feature as `blocked` or leave a `TODO` with the missing condition.

## Hard Rules

- Do not invent Node, Python packaging, or Make-based workflows for this repo. The scanned entrypoints are CMake, shell scripts, and ROS.
- Prefer existing scripts over ad hoc command sequences when the script already encodes the workflow.
- Run `build.sh`, `build_ros.sh`, and `run_stereo_inertial.sh` from the repo root if you use their checked-in relative paths.
- Work in progress limit is one feature. Do not start a second feature until the current feature is verified, blocked, or explicitly handed off.
- Do not perform opportunistic refactors, dependency upgrades, formatting sweeps, or unrelated cleanup while working on a feature.
- Do not mark a feature as `passing` unless its verification step completed successfully and evidence is recorded.
- Do not claim runtime correctness from static inspection alone.
- `run_stereo_inertial.sh` checks for `Vocabulary/ORBvoc.bin`, but its `VOCAB_PATH` is set to `Vocabulary/orbvoc.dbow3`. It does not pass `Vocabulary/ORBvoc.txt` to `rosrun`.
- Treat `Thirdparty/` as vendored code. Do not edit it unless the task explicitly targets a dependency.
- Treat `build/`, `lib/`, `logs/`, `bags/`, and similar runtime/output directories as generated artifacts unless the task says otherwise.
- If a command depends on local hardware, ROS environment, dataset paths, or external system dependencies, mark the uncertainty with `TODO` instead of guessing.

## Task Lanes

- `core`: `src/`, `include/`
- `native-examples`: `Examples/`
- `ros`: `Examples_old/ROS/ORB_SLAM3/`
- `evaluation`: `evaluation/`
- `vocabulary`: `Vocabulary/`
- `docs`: `README.md`, `docs/`, `AGENTS.md`
- `vendored`: `Thirdparty/`, out of default scope unless explicitly requested
- `generated`: `build/`, `lib/`, `logs/`, `bags/`, and similar outputs, out of default scope unless explicitly requested

## Definition of Done

- The active feature in `docs/features.md` has a concrete behavior and verification step.
- Every command written in docs or comments exists in the repo or is directly supported by scanned config/scripts.
- The relevant build path is either run successfully or explicitly reported as not run.
- The feature-specific verification step passes, or the blocker is recorded.
- A feature is not `passing` just because code was edited, compiled partially, or appears correct by inspection.
- For ROS-facing changes, confirm the touched node/config lives under `Examples_old/ROS/ORB_SLAM3/`.
- If a workflow or directory contract changes, update the matching doc in `docs/`.
- Record command evidence, touched files, unverified assumptions, and next steps in the session handoff.

## Documentation Routing

- Build, prerequisites, and run modes: `docs/build-and-run.md`
- Codebase map and ownership boundaries: `docs/repo-map.md`
- Validation entrypoints and evidence expectations: `docs/validation.md`
- Feature list and feature states: `docs/features.md`
- Cross-session progress log: `docs/progress.md`
- Session handoff template: `docs/session-handoff.md`
- `README.md` remains the combined upstream/custom background document; do not duplicate it here.

## Session End Checklist

Update `docs/session-handoff.md` with:

- Active feature ID and final state.
- Commands run, results, and evidence paths.
- Commands not run and exact reasons.
- Touched files.
- Unverified assumptions, including ROS setup, RealSense availability, `pidstat`, `rosbag`, vocabulary files, or dataset paths.
- Next recommended step.
- Explicit `TODO` markers for unresolved facts.
