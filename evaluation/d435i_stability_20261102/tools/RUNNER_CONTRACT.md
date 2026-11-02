# RGBD36 runner / identity / reduction contract

These tools do not build, deploy or modify a package. Default invocation is local dry-run. Only `run_trials.py --execute --stage ...` opens SSH/ROS and replays data. Old RGBD31/32 archives and the source bag are read-only. No bag is copied to the board.

## Create and freeze the16-round plan

Run with the host ROS environment available (`source /opt/ros/noetic/setup.bash` before actual execution). Host ROS_IP is192.168.1.3, board is sunrise@192.168.1.11, isolated master is11431, namespace is/x5_8g_test. New packages contain matched node/core; their vocabulary, tools and shared libraries may point to the existing read-only runtime. The runner hashes shared files and pins their identity and CPU governor across rounds.

Example after the build driver has completed and the main operator has deployed node/core:

```bash
python3 tools/make_plan.py --root /absolute/archive \
  --build-result /absolute/archive/builds/arm/RESULT.json \
  --source-commit ACTUAL_RUNTIME_COMMIT \
  --package /userdata/rgbd_stability_trials_20261102/packages/runtime \
  --shared-lib-dir /userdata/rgbd_tracking_optimizations_20261020/atomic_02/lib
python3 tools/run_trials.py --root /absolute/archive --prepare
python3 tools/run_trials.py --root /absolute/archive --stage observation
python3 tools/run_trials.py --root /absolute/archive --execute --stage observation --package runtime
```

Here `tools/...` means this directory. Default `--root` is the checkout's parent archive; default `--configs-dir` is the checkout's `evaluation/d435i_stability_20261102/configs`. `make_plan --package` is a remote directory; `run_trials --package` only asserts a frozen package ID or its exact directory, never overrides hashes.

`make_plan` consumes the original PLAN's exact bag/index/record identity and a completed build RESULT with `build_complete=true`, `binaries={ros_rgbd:sha256,liborb_slam3_ros.so:sha256}`; optional sibling manifest.json and source_ready_sha256 are preserved. It writes PLAN.json once. Configs P0–P3 may differ only in nFeatures/nLevels. `--prepare` verifies the full bag SHA, all5878 original identities, all YAML SHAs and creates identical per-round source-index copies. Interrupted identical input preparation can resume; mismatching files are never overwritten. A pre-existing operator INPUT_IDENTITY.json is preserved; this runner writes RUNNER_INPUT_IDENTITY.json.

PLAN schema is `rgbd_stability_trials_v1`:

- `input`: bag, bag_sha256, bag_bytes, bag_mtime_ns, expected_pairs=5878, index/index_sha256, records/records_sha256, original_record_start_ns/end_ns, rgb_topic, depth_topic.
- `packages[id]`: remote_path, binaries (exact matched node/core SHA strings), source_commit, shared_lib_dir, vocabulary, helpers_dir; optional local build_result/build_manifest/source_ready_sha256.
- `configs[P0..P3]`: path and sha256 (absolute or relative to archive).
- `runs`:16 unique ordered IDs with stage, config_id, package_id, observation, bow_ready. Null package/switch values require a later stage binding.
- Optional network, remote_root, min_board_free_bytes, max_start_temperature_c, diagnostic_windows, protocol_file. The protocol file is included in the freeze.

Frozen order is observation OFF→ON→ON→OFF, all P0/BoWoff; BoW OFF→ON→ON→OFF, all P0 and a common selected observation mode; matrix P0 P1 P2 P3 P3 P2 P1 P0, common package/observation/BoW mode. All use1x, original stamps/seq/payload, queues8/8/10, scale_enabled=true and scale_detail=true. Added observations are independently toggled by ORB_RGBD_STABILITY_DIR; eager BoW uses ORB_RGBD_EAGER_BOW=0/1. OFF explicitly unsets inherited stability DIR/WINDOWS.

## One-time stage decisions

After the preceding stage is fully sealed or explicitly reviewed as a candidate failure, bind the next stage once. The operator applies the pre-frozen protocol; tools never select the best round or silently change baseline. Repeating a bind fails without overwriting it.

Bow decision JSON example:

```json
{"package_id":"runtime","observation":false,"decision_evidence":{"protocol":"preflight/EXPERIMENT_PROTOCOL.json","summary_sha256":"ACTUAL_SUMMARY_SHA","reason":"record the frozen observation-perturbation gate result"}}
```

Matrix decision additionally supplies `"bow_ready":true` or false. Binding bow_ready for the BoW control is forbidden because its OFF/ON order is already frozen.

```bash
python3 tools/run_trials.py --root /absolute/archive --bind-stage bow --decision /absolute/bow-decision.json
python3 tools/run_trials.py --root /absolute/archive --execute --stage bow
python3 tools/run_trials.py --root /absolute/archive --bind-stage matrix --decision /absolute/matrix-decision.json
python3 tools/run_trials.py --root /absolute/archive --execute --stage matrix
```

## Execution and safe continuation

Before each round: verify frozen tools/config/index, bag size/mtime (full SHA is repeated once at each explicit execution-stage entry), package node/core SHA, read-only shared files, available space, temperature if a limit was frozen, CPU governor, absence of other SLAM/compiler/perf work and availability of master11431. Port probing uses SO_REUSEADDR, matching ROS behavior: TIME_WAIT is permitted; a live listener remains rejected.

Each round gets a new host/board directory and ROS process; remote node/sampler ownership is recorded before exec using PID/start_ticks/run cwd/role. Only those owned records may be signalled, with INT/TERM/KILL escalation. Host children have separate new process groups. Actual `/proc/exe`, sole loaded core library, their hashes, YAML and relevant startup environment are checked before playback. Input and both pose observers must connect before a2-second delayed native rosbag play. No queue/rate changes or replay retries occur.

After playback: call scale_seal, await node exit, stop observers, await sampler, owned cleanup, collect all board files and independently verify their returned SHA. Read-only postflight checks owned PIDs gone, no remaining SLAM, unchanged package and no new Atlas files; host children must all exit and master port must be reusable. Any cleanup exception or remaining child forces failure. Completion never implies background threads joined: legacy joined_verified/null and diagnostic workers_joined=false are preserved.

A valid round writes RESULT.json, SUMMARY.json and write-once RUN_COMPLETE.json hashing all artifacts. Resume verifies completed hashes and resolved package/config/switch identity, then skips that round. Existing unsealed/failed rounds stop the runner. No completed or bad round is overwritten, renamed or automatically retried.

The operator may explicitly retain a computationally overloaded candidate as a failed scheduled result and continue later rounds. This cannot waive binary/settings identity, transfer SHA, host/board cleanup or residual-process failures. Before review, every observed host/board input and optional diagnostic frame must match the frozen source ordinal/stamp/sequence identity; host combined identity/duplicate/overflow and callback counters and board identity/callback/CPU-clock counters must be zero. Missing inputs and board recorder overflow remain failed evidence; missing identity evidence or any actual source identity mismatch blocks continuation:

```json
{"classification":"candidate_overload","continue_predeclared_schedule":true,"reason":"specific observed overload and source-missing/timeout conclusion","evidence":["SUMMARY.json","node.log","board/timing/completion.json"]}
```

```bash
python3 tools/run_trials.py --root /absolute/archive --review-failure C_02_P1 --decision /absolute/reviewed-failure.json
python3 tools/run_trials.py --root /absolute/archive --execute --stage matrix
```

This writes immutable REVIEWED_FAILURE.json, leaves RESULT and raw evidence unchanged and keeps trial_valid=false. Resume may skip it and proceed only to the next predeclared round. Such a round is `reviewed_candidate_failure`, never `valid`. Infrastructure/identity/cleanup errors remain blocking.

## Offline reduction

```bash
python3 tools/summarize_trials.py --root /absolute/archive --stage observation
python3 tools/summarize_trials.py --root /absolute/archive
```

Each reduction verifies exact all5878 denominators/identities, host and board received/paired/returned sets, recorder overflow/error counters, actual package content identity, settings/runtime modes, source omissions, per-stage executed values and two pose consumers' fresh coverage. It reports fresh/nonfresh cost, fixed source quartiles, resource samples/deltas and all failed/unrun rounds. Diagnostic frames must cover all5878 original identities with frame_drops=0 and complete file flush; point/lifetime drops and unjoined backend limitations remain explicit rather than being called complete tracking evidence.

Map-ID changes, creation log events and completed active-reset log events are separate. Same-ID reset hints are separately recovered from raw LOST→empty-map reinitialization; unidentified merge/return transitions are not called new-map creation. Nonfresh duration weights each input by its next original header delta, with unknown/unprocessed duration and final right-censoring distinct. Trajectory steps require finite fresh poses, same coordinate epoch and consecutive source ordinals; unlocalized merge events suppress step statistics. No cross-map/reset/merge world trajectory, GT accuracy, backend join or automatic parameter winner is claimed.

The retained `legacy_reduce.py` is an unchanged copy of RGBD31's tested offline reducer; only its per-run BINARIES expectation is injected from the frozen package manifest, then restored. It does not run ROS or write old archives. The old report/main entrypoint is not invoked.

## Validation

`python3 -m py_compile tools/*.py` and `python3 tools/test_trials.py` check source identity replacement with unchanged metadata, config confounding, interrupted preparation, immutable stage decisions, completed-round tampering, explicit failed-round continuation and its safety block, runtime flags, weighted durations/merge boundary, and actual loopback LISTEN/TIME_WAIT behavior. No SSH, ROS or SLAM is started by these tests. Loopback socket tests need a local socket-capable execution context.
