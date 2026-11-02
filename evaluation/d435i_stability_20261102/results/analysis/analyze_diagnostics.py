#!/usr/bin/env python3
"""Read sealed RGBD36 evidence. Writes only this analysis directory; stdlib only."""
import argparse
import bisect
import collections
import csv
import json
import math
import re
import statistics
from pathlib import Path

TARGETS = ["C_01_P0", "C_08_P0", "C_04_P3", "C_05_P3"]
LIFE_NAMES = ["unused", "birth_all_constructors", "retire_other", "retire_ratio",
              "retire_low_observations", "retire_observation_removal", "retire_reset",
              "replace", "kf_add", "kf_erase", "observation_add", "observation_erase",
              "recent_list_graduate_not_retirement"]


def number(value):
    if value is None or value in ("NA", "", "nan", "-nan"):
        return None
    # Preserve original uint64 nanoseconds and IDs; never round through float.
    if "." not in value and "e" not in value.lower():
        return int(value)
    result = float(value)
    return result if math.isfinite(result) else None


def read_csv(path):
    with path.open() as stream:
        for row in csv.DictReader(stream):
            yield {key: number(value) for key, value in row.items()}


def distribution(values):
    values = sorted(x for x in values if x is not None and math.isfinite(x))
    if not values:
        return {"n": 0, "mean": None, "p10": None, "p50": None, "p90": None}
    def quantile(q):
        at = (len(values) - 1) * q
        low, high = math.floor(at), math.ceil(at)
        return values[low] * (high - at) + values[high] * (at - low) if low != high else values[low]
    return {"n": len(values), "mean": statistics.mean(values),
            "p10": quantile(.1), "p50": quantile(.5), "p90": quantile(.9)}


def ratio(row, numerator, denominator):
    a, b = row.get(numerator), row.get(denominator)
    return a / b if a is not None and b is not None and b > 0 else None


def metrics(rows):
    fields = ["input_keypoints", "motion_first", "motion_final", "motion_map",
              "reference_coarse", "reference_map", "local_points", "search_projected",
              "search_visible", "search_matches", "local_inliers", "pose_last_before",
              "pose_last_outliers"]
    result = {field: distribution(row[field] for row in rows) for field in fields}
    definitions = {
        "visible_per_projected": ("search_visible", "search_projected"),
        "new_matches_per_visible": ("search_matches", "search_visible"),
        "inliers_per_input_keypoint": ("local_inliers", "input_keypoints"),
        "local_inliers_per_pose_input": ("local_inliers", "pose_last_before"),
        "outliers_per_pose_input": ("pose_last_outliers", "pose_last_before"),
        "motion_map_per_final_coarse": ("motion_map", "motion_final"),
        "reference_map_per_coarse": ("reference_map", "reference_coarse"),
    }
    for name, (a, b) in definitions.items():
        eligible = [row for row in rows if row.get(a) is not None and row.get(b) is not None and row[b] > 0]
        result[name] = distribution(ratio(row, a, b) for row in eligible)
        result[name]["ratio_of_sums"] = sum(row[a] for row in eligible) / sum(row[b] for row in eligible) if eligible else None
    return result


def classify_onset(row):
    if row["local_inliers"] is not None:
        if row["local_inliers"] < 30:
            return "local_inliers_below_30"
        if row["local_threshold"] == 50 and row["local_inliers"] < 50:
            return "local_30_to_49_protection_gate"
        return "local_other_requires_review"
    if row["reference_coarse"] is not None and row["reference_coarse"] < 10:
        return "reference_coarse_below_10"
    if row["reference_map"] is not None and row["reference_map"] < 10:
        return "reference_optimized_map_below_10"
    return "unknown"


def episodes(rows, scales, epoch_starts):
    result = []
    for index, row in enumerate(rows):
        if not index or rows[index - 1]["state_after"] not in (2, 5) or row["state_after"] in (2, 5):
            continue
        end = index + 1
        while end < len(rows) and rows[end]["state_after"] not in (2, 5):
            end += 1
        changes = [x for x in epoch_starts if row["source_ordinal"] < x <= end + 1]
        if end == len(rows):
            outcome = "eof_right_censored"
        elif not changes:
            outcome = "same_epoch_recovery"
        elif scales[index]["map_id"] != scales[end]["map_id"]:
            outcome = "new_map_before_fresh"
        else:
            outcome = "same_id_reset_before_fresh"
        stamp_end = scales[min(end, len(rows) - 1)]["source_stamp_ns"]
        result.append({"start": int(row["source_ordinal"]), "end": end,
                       "recovery": end + 1 if end < len(rows) else None,
                       "frames": end - index, "observed_s": (stamp_end - scales[index]["source_stamp_ns"]) / 1e9,
                       "outcome": outcome, "classification": classify_onset(row), "onset": row})
    return result


def decay_by_epoch(rows, epoch_definitions):
    starts = [x["start_ordinal"] for x in epoch_definitions]
    groups = collections.defaultdict(list)
    for row in rows:
        if row["local_inliers"] is not None and row["local_points"] is not None and row["current_frame_valid"] == 1:
            groups[bisect.bisect_right(starts, row["source_ordinal"]) - 1].append(row)
    def sample(items):
        return {"start": int(items[0]["source_ordinal"]), "end": int(items[-1]["source_ordinal"]),
                "inliers_median": statistics.median(x["local_inliers"] for x in items),
                "local_points_median": statistics.median(x["local_points"] for x in items)}
    result = []
    for epoch, items in sorted(groups.items()):
        if len(items) < 30:
            continue
        first, last = sample(items[:10]), sample(items[-10:])
        row = {"epoch": epoch, "map_id": epoch_definitions[epoch]["map_id"], "local_calls": len(items),
               "first10": first, "last10": last,
               "more_points_fewer_inliers": last["local_points_median"] > first["local_points_median"] and
                   last["inliers_median"] < first["inliers_median"]}
        if len(items) >= 40:
            settled = sample(items[10:20])
            row["calls11_20"] = settled
            row["settled_more_points_fewer_inliers"] = last["local_points_median"] > settled["local_points_median"] and last["inliers_median"] < settled["inliers_median"]
        result.append(row)
    return {"method": "Within separate coordinate epoch; only actual local calls with current_frame_valid. Nonoverlapping first10/last10 for >=30 calls; sensitivity calls11-20/last10 for >=40. Exploratory sign comparison, not causal proof or a decay rate.",
            "eligible_epochs": len(result), "more_points_fewer_inliers": sum(x["more_points_fewer_inliers"] for x in result),
            "settled_eligible_epochs": sum("calls11_20" in x for x in result),
            "settled_more_points_fewer_inliers": sum(x.get("settled_more_points_fewer_inliers", False) for x in result),
            "epochs": result}


def point_windows(path, rows, summary):
    windows = summary["windows"]
    starts = [x["start"] for x in windows]
    groups = collections.defaultdict(lambda: {"n": 0, "outliers": 0, "z": [], "depth": [], "pixel_distance": []})
    bounds = {}
    for point in read_csv(path):
        ordinal = int(point["source_ordinal"])
        which = bisect.bisect_right(starts, ordinal) - 1
        if which < 0 or ordinal > windows[which]["end"]:
            raise ValueError("point outside declared windows")
        bound = bounds.setdefault(which, [ordinal, ordinal]);bound[1] = ordinal
        # No conflation of motion/reference with local; kind3 is post-call only.
        if point["stage"] != 3 or point["kind"] != 3:
            continue
        state = "onset" if ordinal > 1 and rows[ordinal - 2]["state_after"] in (2, 5) and rows[ordinal - 1]["state_after"] not in (2, 5) else "other"
        group = groups[(which, state)]
        group["n"] += 1;group["outliers"] += point["outlier"] == 1
        if point["camera_z"] is not None:group["z"].append(point["camera_z"])
        if point["input_depth"] is not None and point["input_depth"] > 0:group["depth"].append(point["input_depth"])
        if None not in (point["projected_u"], point["projected_v"], point["observed_u"], point["observed_v"]):
            group["pixel_distance"].append(math.hypot(point["projected_u"] - point["observed_u"], point["projected_v"] - point["observed_v"]))
    return {"scope": "Stored prefix only; stage3/kind3 post-call snapshots. Live-world projection distance is NOT the optimizer edge chi2; no GT depth accuracy. Selection/drop differences preclude global cross-config pixel/depth conclusions.",
            "coverage": [{**window, "stored_source_bounds": bounds.get(i)} for i, window in enumerate(windows)],
            "local_post_snapshots": [{"window": which, "state": state, "n": group["n"],
                "outliers": group["outliers"], "outlier_fraction": group["outliers"] / group["n"],
                "camera_z_m": distribution(group["z"]), "positive_input_depth_m": distribution(group["depth"]),
                "live_snapshot_pixel_distance": distribution(group["pixel_distance"])}
                for (which, state), group in sorted(groups.items())]}


def life_prefix(path, summary):
    counts = collections.Counter()
    births = []
    generations = set()
    for row in read_csv(path):
        kind = int(row["kind"]);counts[LIFE_NAMES[kind]] += 1;generations.add(int(row["map_generation"]))
        if kind == 1:
            births.append(row)
    referenced = [row for row in births if row["keyframe_id"] is not None]
    return {"scope": "100000-row truncated serial prefix; no full lifetime survival ratio. KF-referenced construction is not proof of retained persistent point. Aggregate Birth also includes temporary constructors; do not equate it with mapping supply.",
            "prefix_counts": dict(counts), "prefix_generations": sorted(generations),
            "prefix_births_kf_referenced": len(referenced),
            "prefix_births_without_kf_reference_temporary": len(births) - len(referenced),
            "prefix_birth_frame_id_range": [min(row["value"] for row in births), max(row["value"] for row in births)] if births else None,
            "aggregate_all_admitted_events": dict(zip(LIFE_NAMES, summary["life_totals"]))}


def bow_logs(run, rows):
    records = []
    pattern = re.compile(r"got (\d+), RefKF=(\d+), RefKF_MPs=(\d+), RefKF_BowEmpty=(\d), CurFrame_BowEmpty=(\d)")
    with (run / "node.log").open(errors="replace") as stream:
        for line, text in enumerate(stream, 1):
            match = pattern.search(text)
            if match:
                values = [int(x) for x in match.groups()]
                records.append(dict(zip(["matches", "reference_kf", "reference_mp_slots", "reference_bow_empty", "current_bow_empty"], values), log_line=line))
    coarse = [row for row in rows if row["reference_coarse"] is not None and row["reference_coarse"] < 10] if rows else []
    complete_match = bool(rows) and len(coarse) == len(records) and all(row["reference_coarse"] == rec["matches"] for row, rec in zip(coarse, records))
    if complete_match:
        for row, rec in zip(coarse, records):
            rec["source_ordinal"] = int(row["source_ordinal"])
            rec["state_before"] = row["state_before"];rec["state_after"] = row["state_after"]
            rec["motion_final"] = row["motion_final"];rec["motion_map"] = row["motion_map"]
    return {"coarse_failure_log_count": len(records), "ordered_join_valid": complete_match,
            "empty_reference_count": sum(rec["reference_bow_empty"] == 1 for rec in records),
            "empty_reference_events": [rec for rec in records if rec["reference_bow_empty"] == 1],
            "scope": "Empty flag printed only on reference coarse<10 failure; zero logs is not a full audit of every KF read. Source mapping only if full ordered coarse-log/counter counts and values agree."}


def keyframes(rows):
    created = [row for row in rows if row["kf_id"] is not None]
    seeds = [row for row in created if row["kf_depth_slots"] is None]
    later = [row for row in created if row["kf_depth_slots"] is not None]
    evaluated = [row for row in rows if row["kf_need_close"] is not None]
    return {"created": len(created), "seed_creation_frames": len(seeds), "later_creation_frames": len(later),
            "seed_new_points": distribution(row["kf_new_points"] for row in seeds),
            "later_new_points": distribution(row["kf_new_points"] for row in later),
            "later_depth_slots": distribution(row["kf_depth_slots"] for row in later),
            "tracking_created_new_points_sum": sum(row["kf_new_points"] for row in created),
            "need_evaluated": len(evaluated), "close_trigger_true": sum(row["kf_need_close"] == 1 for row in evaluated),
            "close_trigger_true_at_creation": sum(row["kf_need_close"] == 1 for row in later),
            "close_trigger_false_at_creation": sum(row["kf_need_close"] == 0 for row in later),
            "tracked_close": distribution(row["kf_tracked_close"] for row in evaluated),
            "untracked_close": distribution(row["kf_untracked_close"] for row in evaluated),
            "frame_close_depths": distribution(row["close_depths"] for row in rows),
            "frame_close_zero": sum(row["close_depths"] == 0 for row in rows),
            "flags_true_among_later_creation": {field: sum(row[field] == 1 for row in later) for field in ["kf_c1a", "kf_c1b", "kf_c1c", "kf_c2", "kf_mapper_idle"]},
            "queue_observed": distribution(row["kf_queue"] for row in rows),
            "queue_ge3_observed": sum(row["kf_queue"] is not None and row["kf_queue"] >= 3 for row in rows)}


def analyze_run(run):
    raw_summary = json.loads((run / "SUMMARY.json").read_text())
    rows = list(read_csv(run / "board/stability/stability_frames.csv"))
    scales = list(read_csv(run / "board/timing/scale_frames.csv"))
    assert len(rows) == len(scales) == 5878
    for ordinal, (row, scale) in enumerate(zip(rows, scales), 1):
        assert row["source_ordinal"] == scale["source_ordinal"] == ordinal
        for key in ["source_stamp_ns", "rgb_seq", "depth_seq", "frame_id", "state_after"]:
            assert row[key] == scale[key], (run.name, ordinal, key)
        assert row["input_keypoints"] == scale["frame_keypoints"]
        assert row["frame_id"] == ordinal - 1
    summary = raw_summary["diagnostic"]
    epochs = raw_summary["dynamics"]["coordinate_epochs"]
    failures = episodes(rows, scales, [x["start_ordinal"] for x in epochs])
    local = [row for row in rows if row["local_inliers"] is not None]
    onset_local = [episode["onset"] for episode in failures if episode["onset"]["local_inliers"] is not None]
    outcomes = collections.Counter(x["outcome"] for x in failures)
    assert outcomes["new_map_before_fresh"] == raw_summary["dynamics"]["new_map_creation_logs"]["after_initial_atlas_creation"]
    assert outcomes["same_id_reset_before_fresh"] == raw_summary["dynamics"]["same_id_active_resets"]["completed_log_count"]
    assert all(row["local_points"] == row["search_seen"] + row["search_bad"] + row["search_projected"] for row in local)
    assert all(row["search_visible"] <= row["search_projected"] for row in local)
    result = {"id": run.name, "config": raw_summary["config_id"],
              "fresh_frames": raw_summary["whole"]["fresh_sources"], "coordinate_epochs": len(epochs),
              "diagnostic_summary": summary, "input_identity_join_5878_passed": True,
              "unlocalized_merge_log_events": raw_summary["dynamics"]["merge_events_unlocalized"],
              "map_history_scope": "Epochs split known map-ID changes and same-ID resets. P3 has unlocalized merges, so these are not fully isolated unchanged-map histories; no merge correctness qualification.",
              "local_partition_checks_passed": True,
              "local_post_nonoutlier_equals_local_inliers_calls": sum(row["pose_last_after"] is not None and row["pose_last_outliers"] is not None and row["pose_last_after"] - row["pose_last_outliers"] == row["local_inliers"] for row in local),
              "input_boundary_invalid_current_frames": [int(row["source_ordinal"]) for row in rows if row["current_frame_valid"] == 0],
              "onset_count": len(failures), "onset_classification": dict(collections.Counter(x["classification"] for x in failures)),
              "episode_outcomes": dict(outcomes),
              "nonfresh_episode_frames": distribution(x["frames"] for x in failures),
              "nonfresh_episode_observed_s": distribution(x["observed_s"] for x in failures),
              "onsets": failures, "all_local_calls": metrics(local), "onset_local_calls": metrics(onset_local),
              "keyframes": keyframes(rows), "epoch_decay": decay_by_epoch(rows, epochs),
              "point_windows": point_windows(run / "board/stability/stability_points.csv", rows, summary),
              "lifetime": life_prefix(run / "board/stability/stability_lifetime.csv", summary),
              "bow": bow_logs(run, rows)}
    return result, rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    base = args.artifact.resolve();output = base / "analysis"
    result = {"schema": "rgbd36_diagnostic_analysis_v1", "targets": TARGETS,
              "onset_definition": "previous callback state OK/OK_KLT (2/5), current state not 2/5; initialization is not an onset",
              "runs": {}, "bow_all_runs": {}}
    cache = {}
    for name in TARGETS:
        result["runs"][name], cache[name] = analyze_run(base / "runs" / name)
    for run in sorted((base / "runs").iterdir()):
        if not (run / "RESOLVED_RUN.json").exists():continue
        config = json.loads((run / "RESOLVED_RUN.json").read_text())
        rows = cache.get(run.name)
        if rows is None and (run / "board/stability/stability_frames.csv").exists():
            rows = list(read_csv(run / "board/stability/stability_frames.csv"))
        result["bow_all_runs"][run.name] = {"bow_ready": config["bow_ready"], "observation": config["observation"], **bow_logs(run, rows)}
    shared = [ordinal for ordinal in range(5878) if all(cache[name][ordinal]["local_inliers"] is not None for name in TARGETS)]
    shared_ok = [ordinal for ordinal in shared if all(cache[name][ordinal]["state_after"] in (2, 5) for name in TARGETS)]
    result["same_source_comparison"] = {
        "scope": "Identical original input ordinals, but maps/states/histories differ; conditional descriptive comparison, not overall benefit evidence or isolated per-match causal identification. The final_OK subset excludes difficult failures by selection.",
        "all_four_local_calls_source_count": len(shared), "all_four_local_calls_and_final_OK_count": len(shared_ok),
        "all_local_observed": {name: metrics([cache[name][i] for i in shared]) for name in TARGETS},
        "all_final_OK": {name: metrics([cache[name][i] for i in shared_ok]) for name in TARGETS}}
    result["limits"] = ["All four frame streams lossless; coordinate and lifetime detail truncated.",
        "NA is not zero; seed frame has no actual local inlier measurement.",
        "Pose counters are last method call; window pixels are live-world boundary projections, not frozen optimizer-edge chi2.",
        "Point Birth aggregate includes all constructors; seed/temporary/KF-referenced construction is not retained mapping population or lifetime success.",
        "Life details only serial prefix, aggregate counts continue but have no full cohort duration/exit attribution.",
        "P3 has 2/3 unlocalized merge groups; epoch labels do not fully isolate map-history changes. KF set additions/removals include transfers/reinsertions and are not unique creation counts.",
        "workers_joined=false: safe sealed recorder output is not proof every backend job completed.",
        "No ground truth pose or depth accuracy; 65535 is not assumed invalid."]
    (output / "DIAGNOSTIC_ANALYSIS.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    for name, run in result["runs"].items():
        print(name, json.dumps({"fresh": run["fresh_frames"], "onsets": run["onset_classification"],
            "outcomes": run["episode_outcomes"], "kf_created": run["keyframes"]["created"],
            "decay": {k: v for k, v in run["epoch_decay"].items() if k != "epochs"}}, ensure_ascii=False))
    print("same_source_local", len(shared), "same_source_OK", len(shared_ok))


if __name__ == "__main__":
    main()
