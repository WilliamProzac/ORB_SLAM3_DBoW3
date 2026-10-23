#pragma once
#include "RgbdTiming.h"
#include <iomanip>
#include <ostream>

// Shared ROS/native serialization of copied L0 records. No map access/locks,
// extra decisions, point scans, or wall-clock reads are performed here.
#define RGBD_V2_LIGHT_FIELDS(F) \
  F(l0_collected, int(r.l0_collected)) \
  F(source_stamp_ns, r.observation.source_stamp_ns) \
  F(source_ordinal, r.observation.source_ordinal) \
  F(run_id, r.observation.run_id) \
  F(context_provided, int(r.observation.provided)) \
  F(detail_window, int(r.observation.detail_window)) \
  F(tracking_thread_id, r.tracking_thread_id) \
  F(frame_id, r.frame_id) \
  F(state_before, r.state_before) \
  F(state_after, r.state_after) \
  F(keypoints, r.keypoints) \
  F(valid_depths, r.valid_depths) \
  F(tracked_inliers, r.tracked_inliers) \
  F(lm_queue_before, r.local_mapping_queue_before) \
  F(lm_queue_after, r.local_mapping_queue_after) \
  F(inserted_keyframe, r.inserted_keyframe) \
  F(kf_evaluated_mask, r.keyframe_decision.evaluated_mask) \
  F(kf_value_mask, r.keyframe_decision.value_mask) \
  F(kf_reason, static_cast<int>(r.keyframe_decision.reason)) \
  F(kf_create_result, static_cast<int>(r.keyframe_decision.create_result)) \
  F(kf_need_result, r.keyframe_decision.need_result) \
  F(kf_reference_matches, r.keyframe_decision.reference_matches) \
  F(kf_matches_inliers, r.keyframe_decision.matches_inliers) \
  F(kf_tracked_close, r.keyframe_decision.tracked_close) \
  F(kf_untracked_close, r.keyframe_decision.untracked_close) \
  F(kf_map_keyframes, r.keyframe_decision.map_keyframes) \
  F(kf_queue_depth, r.keyframe_decision.queue_depth) \
  F(kf_reference_ratio, r.keyframe_decision.reference_ratio) \
  F(kf_frames_since_keyframe, r.keyframe_decision.frames_since_keyframe) \
  F(kf_min_frames, r.keyframe_decision.min_frames) \
  F(kf_max_frames, r.keyframe_decision.max_frames) \
  F(kf_interrupt_called, int(r.keyframe_decision.interrupt_called)) \
  F(kf_interrupt_request_id, r.keyframe_decision.interrupt_request_id) \
  F(kf_interrupt_attempt_id, r.keyframe_decision.interrupt_attempt_id) \
  F(kf_interrupt_request_ns, r.keyframe_decision.interrupt_request_ns) \
  F(pose_calls, r.pose.calls) \
  F(pose_edges, r.pose.edges) \
  F(pose_mono_edges, r.pose.mono_edges) \
  F(pose_stereo_edges, r.pose.stereo_edges) \
  F(pose_body_edges, r.pose.body_edges) \
  F(pose_solves, r.pose.solves) \
  F(pose_returned_iterations, r.pose.returned_iterations) \
  F(pose_commits, r.pose.commits) \
  F(pose_phases_visited, r.pose.phases_visited) \
  F(pose_cpu_available, int(r.pose.cpu_available)) \
  F(projection_points, r.projection_match.points) \
  F(projection_queries, r.projection_match.queries) \
  F(projection_candidates, r.projection_match.candidates) \
  F(projection_occupied_skips, r.projection_match.occupied_skips) \
  F(projection_stereo_gate_skips, r.projection_match.stereo_gate_skips) \
  F(projection_distances, r.projection_match.distances) \
  F(pose_setup_wall_ms, r.pose.wall_ms[0]) \
  F(pose_setup_cpu_ms, r.pose.cpu_ms[0]) \
  F(pose_map_lock_wait_wall_ms, r.pose.wall_ms[1]) \
  F(pose_map_lock_wait_cpu_ms, r.pose.cpu_ms[1]) \
  F(pose_build_wall_ms, r.pose.wall_ms[2]) \
  F(pose_build_cpu_ms, r.pose.cpu_ms[2]) \
  F(pose_solve_wall_ms, r.pose.wall_ms[3]) \
  F(pose_solve_cpu_ms, r.pose.cpu_ms[3]) \
  F(pose_classify_wall_ms, r.pose.wall_ms[4]) \
  F(pose_classify_cpu_ms, r.pose.cpu_ms[4]) \
  F(pose_commit_wall_ms, r.pose.wall_ms[5]) \
  F(pose_commit_cpu_ms, r.pose.cpu_ms[5]) \
  F(pose_destroy_wall_ms, r.pose.wall_ms[6]) \
  F(pose_destroy_cpu_ms, r.pose.cpu_ms[6]) \
  F(voting_slots, r.voting_slots) \
  F(voting_associated, r.voting_associated) \
  F(voting_observation_entries, r.voting_observation_entries) \
  F(voting_unique_keyframes, r.voting_unique_keyframes) \
  F(gather_slots, r.gather.slots) \
  F(gather_visited_slots, r.gather.visited_slots) \
  F(gather_skipped_empty_slots, r.gather.skipped_empty_slots) \
  F(gather_bitmap_words, r.gather.bitmap_words) \
  F(gather_bitmap_fallback_keyframes, r.gather.bitmap_fallback_keyframes) \
  F(gather_index_retained_bytes, r.gather.index_retained_bytes) \
  F(gather_index_updates_cumulative, r.gather.index_updates_cumulative) \
  F(gather_index_rebuilds_cumulative, r.gather.index_rebuilds_cumulative) \
  F(gather_index_releases_cumulative, r.gather.index_releases_cumulative) \
  F(gather_index_alloc_failures_cumulative, r.gather.index_alloc_failures_cumulative) \
  F(gather_null_identity, r.gather.null_identity) \
  F(gather_hint_hits, r.gather.hint_hits) \
  F(gather_hint_collisions, r.gather.hint_collisions) \
  F(gather_weak_locks, r.gather.weak_locks) \
  F(gather_expired, r.gather.expired) \
  F(gather_frame_marked, r.gather.frame_marked) \
  F(gather_bad, r.gather.bad) \
  F(gather_accepted, r.gather.accepted) \
  F(search_associated_slots, r.search_associated_slots) \
  F(search_local_slots, r.search_local_slots) \
  F(search_seen_skips, r.search_seen_skips) \
  F(search_bad_skips, r.search_bad_skips) \
  F(search_projected, r.search_projected) \
  F(search_rejected, r.search_rejected) \
  F(search_visible, r.search_visible) \
  F(search_matches, r.search_matches) \
  F(local_keyframes_cpu_ms, r.local_keyframes_cpu_ms) \
  F(local_points_cpu_ms, r.local_points_cpu_ms) \
  F(local_search_cpu_ms, r.local_search_cpu_ms) \
  F(system_clone_ms, r.system_clone_ms) \
  F(system_resize_ms, r.system_resize_ms) \
  F(mode_mutex_wait_ms, r.mode_mutex_wait_ms) \
  F(mode_change_ms, r.mode_change_ms) \
  F(reset_mutex_wait_ms, r.reset_mutex_wait_ms) \
  F(reset_ms, r.reset_ms) \
  F(grab_rgbd_ms, r.grab_rgbd_ms) \
  F(state_copy_ms, r.state_copy_ms) \
  F(gray_convert_ms, r.gray_convert_ms) \
  F(depth_convert_ms, r.depth_convert_ms) \
  F(rgbd_depth_assoc_ms, r.rgbd_depth_assoc_ms) \
  F(system_total_ms, r.system_total_ms) \
  F(frame_construct_ms, r.frame_construct_ms) \
  F(frame_construct_cpu_ms, r.frame_construct_cpu_ms) \
  F(orb_l0_collected, int(r.orb.collected)) \
  F(orb_l1_collected, int(r.orb.detail_collected)) \
  F(orb_cpu_valid, int(r.orb.cpu_valid)) \
  F(orb_extract_caller_cpu_ms, r.orb.extract_caller_cpu_ms) \
  F(orb_helper_cpu_ms, r.orb.helper_cpu_ms) \
  F(orb_total_cpu_ms, r.orb.total_cpu_ms) \
  F(orb_worker_count, r.orb.worker_count) \
  F(orb_level_count, r.orb_level_count) \
  F(orb_pyramid_caller_cpu_ms, r.orb.stage_caller_cpu_ms[0]) \
  F(orb_pyramid_helper_cpu_ms, r.orb.stage_helper_cpu_ms[0]) \
  F(orb_keypoints_caller_cpu_ms, r.orb.stage_caller_cpu_ms[1]) \
  F(orb_keypoints_helper_cpu_ms, r.orb.stage_helper_cpu_ms[1]) \
  F(orb_orientation_caller_cpu_ms, r.orb.stage_caller_cpu_ms[2]) \
  F(orb_orientation_helper_cpu_ms, r.orb.stage_helper_cpu_ms[2]) \
  F(orb_blur_caller_cpu_ms, r.orb.stage_caller_cpu_ms[3]) \
  F(orb_blur_helper_cpu_ms, r.orb.stage_helper_cpu_ms[3]) \
  F(orb_descriptor_caller_cpu_ms, r.orb.stage_caller_cpu_ms[4]) \
  F(orb_descriptor_helper_cpu_ms, r.orb.stage_helper_cpu_ms[4]) \
  F(orb_assembly_caller_cpu_ms, r.orb.stage_caller_cpu_ms[5]) \
  F(orb_assembly_helper_cpu_ms, r.orb.stage_helper_cpu_ms[5]) \
  F(orb_worker_0_cpu_ms, r.orb.worker_cpu_ms[0]) \
  F(orb_worker_0_tasks, r.orb.worker_tasks[0]) \
  F(orb_worker_1_cpu_ms, r.orb.worker_cpu_ms[1]) \
  F(orb_worker_1_tasks, r.orb.worker_tasks[1]) \
  F(orb_worker_2_cpu_ms, r.orb.worker_cpu_ms[2]) \
  F(orb_worker_2_tasks, r.orb.worker_tasks[2]) \
  F(orb_worker_3_cpu_ms, r.orb.worker_cpu_ms[3]) \
  F(orb_worker_3_tasks, r.orb.worker_tasks[3]) \
  F(orb_level_0_candidates, r.orb_level_candidates[0]) \
  F(orb_level_0_outputs, r.orb.level_outputs[0]) \
  F(orb_level_1_candidates, r.orb_level_candidates[1]) \
  F(orb_level_1_outputs, r.orb.level_outputs[1]) \
  F(orb_level_2_candidates, r.orb_level_candidates[2]) \
  F(orb_level_2_outputs, r.orb.level_outputs[2]) \
  F(orb_level_3_candidates, r.orb_level_candidates[3]) \
  F(orb_level_3_outputs, r.orb.level_outputs[3]) \
  F(orb_level_4_candidates, r.orb_level_candidates[4]) \
  F(orb_level_4_outputs, r.orb.level_outputs[4]) \
  F(orb_level_5_candidates, r.orb_level_candidates[5]) \
  F(orb_level_5_outputs, r.orb.level_outputs[5]) \
  F(orb_level_6_candidates, r.orb_level_candidates[6]) \
  F(orb_level_6_outputs, r.orb.level_outputs[6]) \
  F(orb_level_7_candidates, r.orb_level_candidates[7]) \
  F(orb_level_7_outputs, r.orb.level_outputs[7]) \
  F(orb_level_8_candidates, r.orb_level_candidates[8]) \
  F(orb_level_8_outputs, r.orb.level_outputs[8]) \
  F(orb_level_9_candidates, r.orb_level_candidates[9]) \
  F(orb_level_9_outputs, r.orb.level_outputs[9]) \
  F(orb_level_10_candidates, r.orb_level_candidates[10]) \
  F(orb_level_10_outputs, r.orb.level_outputs[10]) \
  F(orb_level_11_candidates, r.orb_level_candidates[11]) \
  F(orb_level_11_outputs, r.orb.level_outputs[11]) \
  F(orb_level_12_candidates, r.orb_level_candidates[12]) \
  F(orb_level_12_outputs, r.orb.level_outputs[12]) \
  F(orb_level_13_candidates, r.orb_level_candidates[13]) \
  F(orb_level_13_outputs, r.orb.level_outputs[13]) \
  F(orb_level_14_candidates, r.orb_level_candidates[14]) \
  F(orb_level_14_outputs, r.orb.level_outputs[14]) \
  F(orb_level_15_candidates, r.orb_level_candidates[15]) \
  F(orb_level_15_outputs, r.orb.level_outputs[15]) \
  F(orb_extract_ms, r.orb_extract_ms) \
  F(orb_pyramid_ms, r.orb_pyramid_ms) \
  F(orb_keypoints_octree_ms, r.orb_keypoints_octree_ms) \
  F(orb_orientation_ms, r.orb_orientation_ms) \
  F(orb_copy_ms, r.orb_copy_ms) \
  F(orb_gaussian_ms, r.orb_gaussian_ms) \
  F(orb_descriptor_ms, r.orb_descriptor_ms) \
  F(orb_blur_ms, r.orb_blur_ms) \
  F(orb_result_assembly_ms, r.orb_result_assembly_ms) \
  F(track_internal_ms, r.track_internal_ms) \
  F(map_mutex_wait_ms, r.map_mutex_wait_ms) \
  F(initialization_ms, r.initialization_ms) \
  F(reference_kf_ms, r.reference_kf_ms) \
  F(motion_model_ms, r.motion_model_ms) \
  F(relocalization_ms, r.relocalization_ms) \
  F(local_map_ms, r.local_map_ms) \
  F(local_update_ms, r.local_update_ms) \
  F(local_keyframes_ms, r.local_keyframes_ms) \
  F(local_points_ms, r.local_points_ms) \
  F(local_search_ms, r.local_search_ms) \
  F(local_pose_ms, r.local_pose_ms) \
  F(local_keyframes, r.local_keyframes) \
  F(local_points, r.local_points) \
  F(need_new_kf_ms, r.need_new_kf_ms) \
  F(create_new_kf_ms, r.create_new_kf_ms)

namespace rgbd_v2 {
inline void WriteLightHeader(std::ostream& output) {
  const char* separator = "";
#define RGBD_V2_HEADER(name, expression) output << separator << #name; separator = ",";
  RGBD_V2_LIGHT_FIELDS(RGBD_V2_HEADER)
#undef RGBD_V2_HEADER
  output << '\n';
}
inline void WriteLightRow(std::ostream& output, const ORB_SLAM3::RgbdTimingRecord& r) {
  const char* separator = "";
  output << std::setprecision(17);
#define RGBD_V2_ROW(name, expression) output << separator << (expression); separator = ",";
  RGBD_V2_LIGHT_FIELDS(RGBD_V2_ROW)
#undef RGBD_V2_ROW
  output << '\n';
}

// L1 uses copied observations from the fixed source-ordinal windows only.
// Original three buckets stay first-four; D isBad buckets rotate by source/call.
inline void WriteLocalMapDetailHeader(std::ostream& output) {
  output << ",local_map_l1_collected,local_map_native_thread_id,a3_old_owner_count";
  for (const char* stage : {"a3_hint_clear", "a3_owner_clear"})
    output << ',' << stage << "_collected," << stage << "_clock_failures,"
           << stage << "_wall_ms," << stage << "_cpu_ms";
  for (const char* stage : {"a1_total", "a1_voting", "a1_filter"})
    for (const char* field : {"collected", "clock_failures", "wall_ms", "cpu_ms", "begin_ns", "end_ns"})
      output << ',' << stage << '_' << field;
  for (const char* group : {"a3_kf_features", "a1_mp_features", "a1_kf_connections",
                           "a1_mp_is_bad", "a1_kf_is_bad"}) {
    output << ',' << group << "_sample_count," << group << "_skipped," << group << "_calls";
    for (std::size_t i = 0; i < ORB_SLAM3::kLocalMapLockSamples; ++i)
      for (const char* field : {"site", "clock_failures", "object_id", "object_frame_id",
                               "object_timestamp", "request_ns", "acquired_ns", "before_unlock_ns", "call_ordinal"})
        output << ',' << group << '_' << i << '_' << field;
  }
}
inline void WriteLocalMapDetailRow(std::ostream& output,
    const ORB_SLAM3::LocalMapDetailObservation& detail) {
  output << ',' << int(detail.collected) << ',' << detail.native_thread_id
         << ',' << detail.old_owner_count;
  for (const auto* stage : {&detail.hint_clear, &detail.owner_clear})
    output << ',' << int(stage->collected) << ',' << int(stage->clock_failures)
           << ',' << stage->wall_ms << ',' << stage->cpu_ms;
  for (const auto* stage : {&detail.a1_total, &detail.a1_voting, &detail.a1_filter})
    output << ',' << int(stage->collected) << ',' << int(stage->clock_failures)
           << ',' << stage->wall_ms << ',' << stage->cpu_ms
           << ',' << stage->begin_ns << ',' << stage->end_ns;
  for (const auto* group : {&detail.gather_features, &detail.voting_features,
                           &detail.keyframe_connections, &detail.voting_is_bad, &detail.filter_is_bad}) {
    output << ',' << group->count << ',' << group->skipped << ',' << group->calls;
    for (const auto& sample : group->samples)
      output << ',' << int(sample.site) << ',' << int(sample.clock_failures)
             << ',' << sample.object_id << ',' << sample.object_frame_id
             << ',' << sample.object_timestamp << ',' << sample.request_ns
             << ',' << sample.acquired_ns << ',' << sample.before_unlock_ns << ',' << sample.call_ordinal;
  }
}
inline const char* const* OrbLevelPhaseNames() {
  static const char* names[] = {"fast_initial", "fast_fallback", "octree",
      "orientation", "copy", "gaussian", "descriptor"};
  static_assert(ORB_SLAM3::kOrbLevelObservationStages == 7, "update L1 phase columns");
  return names;
}
inline void WriteDetailHeader(std::ostream& output) {
  const char* separator = "";
#define RGBD_V2_HEADER(name, expression) output << separator << #name; separator = ",";
  RGBD_V2_LIGHT_FIELDS(RGBD_V2_HEADER)
#undef RGBD_V2_HEADER
  WriteLocalMapDetailHeader(output);
  output << ",voting_observation_copy_ms,voting_observation_copy_cpu_ms"
         << ",orb_lock_timing,orb_allocation_timing,orb_worker_idle_timing";
  for (int batch = 0; batch < 3; ++batch)
    output << ",orb_batch_" << batch << "_dispatch_ms,orb_batch_" << batch << "_barrier_ms";
  for (std::size_t level = 0; level < ORB_SLAM3::kMaxRgbdOrbLevels; ++level) {
    output << ",orb_level_" << level << "_rows,orb_level_" << level << "_cols"
           << ",orb_level_" << level << "_stride";
    for (std::size_t phase = 0; phase < ORB_SLAM3::kOrbLevelObservationStages; ++phase)
      output << ",orb_level_" << level << '_' << OrbLevelPhaseNames()[phase]
             << "_wall_ms,orb_level_" << level << '_' << OrbLevelPhaseNames()[phase] << "_cpu_ms";
    output << ",orb_level_" << level << "_fallback_cells,orb_level_" << level << "_octree_splits";
  }
  output << '\n';
}
inline void WriteDetailRow(std::ostream& output, const ORB_SLAM3::RgbdTimingRecord& r) {
  const char* separator = "";
  output << std::setprecision(17);
#define RGBD_V2_ROW(name, expression) output << separator << (expression); separator = ",";
  RGBD_V2_LIGHT_FIELDS(RGBD_V2_ROW)
#undef RGBD_V2_ROW
  WriteLocalMapDetailRow(output, r.local_map_detail);
  output << ',' << r.voting_observation_copy_ms << ',' << r.voting_observation_copy_cpu_ms
         << ",not_measured,not_measured,not_measured";
  for (int batch = 0; batch < 3; ++batch)
    output << ',' << r.orb.dispatch_ms[batch] << ',' << r.orb.barrier_ms[batch];
  const std::array<double, ORB_SLAM3::kMaxRgbdOrbLevels>* walls[] = {
      &r.orb_level_fast_initial_ms, &r.orb_level_fast_fallback_ms, &r.orb_level_octree_ms,
      &r.orb_level_orientation_ms, &r.orb_level_copy_ms, &r.orb_level_gaussian_ms,
      &r.orb_level_descriptor_ms};
  for (std::size_t level = 0; level < ORB_SLAM3::kMaxRgbdOrbLevels; ++level) {
    output << ',' << r.orb.level_rows[level] << ',' << r.orb.level_cols[level]
           << ',' << r.orb.level_stride[level];
    for (std::size_t phase = 0; phase < ORB_SLAM3::kOrbLevelObservationStages; ++phase)
      output << ',' << (*walls[phase])[level] << ',' << r.orb.level_cpu_ms[level][phase];
    output << ',' << r.orb_level_fallback_cells[level] << ',' << r.orb_level_octree_splits[level];
  }
  output << '\n';
}
} // namespace rgbd_v2
#undef RGBD_V2_LIGHT_FIELDS

#define RGBD_V2_MAPPING_FIELDS(F) \
  F(l0_collected, int(r.l0_collected)) \
  F(mapping_thread_id, r.mapping_thread_id) \
  F(source_stamp_ns, r.observation.source_stamp_ns) \
  F(source_ordinal, r.observation.source_ordinal) \
  F(run_id, r.observation.run_id) \
  F(context_provided, int(r.observation.provided)) \
  F(detail_window, int(r.observation.detail_window)) \
  F(keyframe_id, r.keyframe_id) \
  F(image_timestamp, r.image_timestamp) \
  F(refinement_attempt_id, r.refinement_attempt_id) \
  F(attempt_first_request_ns, r.attempt_first_request_ns) \
  F(attempt_requests, r.attempt_requests) \
  F(queue_depth_before_pop, r.queue_depth_before_pop) \
  F(queue_wait_ms, r.queue_wait_ms) \
  F(bow_ms, r.bow_ms) \
  F(process_keyframe_ms, r.process_keyframe_ms) \
  F(map_point_culling_ms, r.map_point_culling_ms) \
  F(map_point_creation_ms, r.map_point_creation_ms) \
  F(local_ba_ms, r.local_ba_ms) \
  F(local_ba_cpu_ms, r.local_ba_cpu_ms) \
  F(keyframe_culling_ms, r.keyframe_culling_ms) \
  F(total_ms, r.total_ms) \
  F(lba_executed, r.lba_executed) \
  F(legacy_lba_aborted, r.lba_aborted) \
  F(ingest_slots, r.ingest.slots) \
  F(ingest_null_points, r.ingest.null_points) \
  F(ingest_bad_points, r.ingest.bad_points) \
  F(ingest_existing_observations, r.ingest.existing_observations) \
  F(ingest_observations_added, r.ingest.observations_added) \
  F(ba_outcome, static_cast<int>(r.ba.outcome)) \
  F(ba_entered_ns, r.ba.entered_ns) \
  F(ba_first_observed_ns, r.ba.first_observed_ns) \
  F(ba_returned_ns, r.ba.returned_ns) \
  F(ba_iterations_requested, r.ba.iterations_requested) \
  F(ba_iterations_completed, r.ba.iterations_completed) \
  F(ba_free_keyframes, r.ba.free_keyframes) \
  F(ba_fixed_keyframes, r.ba.fixed_keyframes) \
  F(ba_map_points, r.ba.map_points) \
  F(ba_mono_edges, r.ba.mono_edges) \
  F(ba_body_edges, r.ba.body_edges) \
  F(ba_stereo_edges, r.ba.stereo_edges) \
  F(ba_solver_invoked, int(r.ba.solver_invoked)) \
  F(ba_cancel_observed, int(r.ba.cancel_observed)) \
  F(ba_committed, int(r.ba.committed)) \
  F(ba_phases_visited, r.ba.phases_visited)

#define RGBD_V2_MEDOID_FIELDS(F) \
  F(detail_window, int(m.detail_window)) \
  F(map_point_calls, m.map_point_calls) \
  F(bad_map_points, m.bad_map_points) \
  F(observation_entries, m.observation_entries) \
  F(bad_observers, m.bad_observers) \
  F(empty_observations, m.empty_observations) \
  F(empty_descriptors, m.empty_descriptors) \
  F(descriptor_capacity_sum, m.descriptor_capacity_sum) \
  F(descriptor_capacity_max, m.descriptor_capacity_max) \
  F(calls, m.calls) \
  F(orb32_calls, m.orb32_calls) \
  F(other_calls, m.other_calls) \
  F(n_sum, m.n_sum) \
  F(n_squared_sum, m.n_squared_sum) \
  F(n_max, m.n_max) \
  F(validated_rows, m.validated_rows) \
  F(pair_distances, m.pair_distances) \
  F(median_rows, m.median_rows) \
  F(cloned_rows, m.cloned_rows) \
  F(clone_bytes, m.clone_bytes) \
  F(published_rows, m.published_rows) \
  F(matrix_outer_capacity_sum, m.matrix_outer_capacity_sum) \
  F(matrix_row_capacity_sum, m.matrix_row_capacity_sum) \
  F(matrix_capacity_bytes_max, m.matrix_capacity_bytes_max) \
  F(median_row_capacity_sum, m.median_row_capacity_sum)

namespace rgbd_v2 {
inline const char* const* BaPhaseNames() {
  static const char* const names[] = {"collect", "build", "solve", "classify",
                                     "map_lock_wait", "commit", "destroy"};
  static_assert(sizeof(names) / sizeof(names[0]) ==
                static_cast<unsigned>(ORB_SLAM3::LocalBaPhase::Count), "BA phase schema changed");
  return names;
}
inline const char* const* IngestPhaseNames() {
  static const char* const names[] = {"total", "bow", "match_snapshot", "observation_add",
      "normal_depth", "connections", "map_publish"};
  static_assert(sizeof(names)/sizeof(names[0]) == static_cast<std::size_t>(
      ORB_SLAM3::KeyFrameIngestPhase::Count), "ingest phase schema changed");
  return names;
}
inline const char* const* MedoidPhaseNames() {
  static const char* const names[] = {"map_point_total", "observation_snapshot", "descriptor_gather",
      "validation", "matrix_allocation", "distances", "median", "clone", "publish",
      "snapshot_lock_wait", "snapshot_lock_held", "publish_lock_wait", "publish_lock_held"};
  static_assert(sizeof(names)/sizeof(names[0]) == static_cast<std::size_t>(
      ORB_SLAM3::MedoidPhase::Count), "medoid phase schema changed");
  return names;
}
inline const char* const* MedoidSiteNames() {
  static const char* const names[] = {"keyframe_ingest", "point_creation", "fusion"};
  static_assert(sizeof(names)/sizeof(names[0]) == static_cast<std::size_t>(
      ORB_SLAM3::MedoidSite::Count), "medoid site schema changed");
  return names;
}
inline void WriteLocalMappingHeader(std::ostream& output) {
  const char* separator = "";
#define RGBD_V2_HEADER(name, expression) output << separator << #name; separator = ",";
  RGBD_V2_MAPPING_FIELDS(RGBD_V2_HEADER)
#undef RGBD_V2_HEADER
  for (unsigned i = 0; i < static_cast<unsigned>(ORB_SLAM3::LocalBaPhase::Count); ++i)
    output << ",ba_" << BaPhaseNames()[i] << "_wall_ms,ba_" << BaPhaseNames()[i] << "_cpu_ms";
  for (std::size_t i=0; i<static_cast<std::size_t>(ORB_SLAM3::KeyFrameIngestPhase::Count); ++i)
    output << ",ingest_" << IngestPhaseNames()[i] << "_samples,ingest_" << IngestPhaseNames()[i]
           << "_wall_ms,ingest_" << IngestPhaseNames()[i] << "_cpu_ms";
  for (std::size_t site=0; site<static_cast<std::size_t>(ORB_SLAM3::MedoidSite::Count); ++site) {
#define RGBD_V2_MEDOID_HEADER(name, expression) output << ",medoid_" << MedoidSiteNames()[site] << '_' << #name;
    RGBD_V2_MEDOID_FIELDS(RGBD_V2_MEDOID_HEADER)
#undef RGBD_V2_MEDOID_HEADER
    for (std::size_t i=0; i<static_cast<std::size_t>(ORB_SLAM3::MedoidPhase::Count); ++i)
      output << ",medoid_" << MedoidSiteNames()[site] << '_' << MedoidPhaseNames()[i] << "_samples"
             << ",medoid_" << MedoidSiteNames()[site] << '_' << MedoidPhaseNames()[i] << "_wall_ms"
             << ",medoid_" << MedoidSiteNames()[site] << '_' << MedoidPhaseNames()[i] << "_cpu_ms";
  }
  output << '\n';
}
inline void WriteLocalMappingRow(std::ostream& output, const ORB_SLAM3::LocalMappingTimingRecord& r) {
  const char* separator = "";
  output << std::setprecision(17);
#define RGBD_V2_ROW(name, expression) output << separator << (expression); separator = ",";
  RGBD_V2_MAPPING_FIELDS(RGBD_V2_ROW)
#undef RGBD_V2_ROW
  for (unsigned i = 0; i < static_cast<unsigned>(ORB_SLAM3::LocalBaPhase::Count); ++i)
    output << ',' << r.ba.wall_ms[i] << ',' << r.ba.cpu_ms[i];
  for (std::size_t i=0; i<static_cast<std::size_t>(ORB_SLAM3::KeyFrameIngestPhase::Count); ++i)
    output << ',' << r.ingest.samples[i] << ',' << r.ingest.wall_ms[i] << ',' << r.ingest.cpu_ms[i];
  for (std::size_t site=0; site<static_cast<std::size_t>(ORB_SLAM3::MedoidSite::Count); ++site) {
    const auto& m=r.medoid[site];
#define RGBD_V2_MEDOID_ROW(name, expression) output << ',' << (expression);
    RGBD_V2_MEDOID_FIELDS(RGBD_V2_MEDOID_ROW)
#undef RGBD_V2_MEDOID_ROW
    for (std::size_t i=0; i<static_cast<std::size_t>(ORB_SLAM3::MedoidPhase::Count); ++i)
      output << ',' << m.samples[i] << ',' << m.wall_ms[i] << ',' << m.cpu_ms[i];
  }
  output << '\n';
}
} // namespace rgbd_v2
#undef RGBD_V2_MAPPING_FIELDS

#undef RGBD_V2_MEDOID_FIELDS
