#ifndef ORB_SLAM3_RGBD_TIMING_H
#define ORB_SLAM3_RGBD_TIMING_H

#include "MedoidTiming.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace ORB_SLAM3 {

constexpr std::size_t kMaxRgbdOrbLevels = 16;

// source_ordinal comes from the frozen input manifest, never a processed-frame
// counter. Wrappers set/clear this context on the thread calling TrackRGBD.
struct RgbdObservationContext {
  std::int64_t source_stamp_ns = 0;
  std::uint64_t source_ordinal = 0;
  std::uint64_t run_id = 0;
  bool provided = false;
  bool detail_window = false;
};
struct RgbdResetCounters {
  std::uint64_t reset = 0;
  std::uint64_t active_reset = 0;
};
void SetRgbdLightDiagnostics(bool enabled);
bool RgbdLightDiagnostics();
constexpr std::size_t kMaxRgbdObservationInputs = 8192;
// Configure the complete frozen input count before enabling L0/creating System.
// Invalid counts throw; LocalMapping snapshots this limit at construction.
void SetRgbdObservationCapacity(std::size_t expected_inputs);
std::size_t GetRgbdObservationCapacity();
void SetRgbdObservationContext(std::int64_t source_stamp_ns,
    std::uint64_t source_ordinal, bool detail_window, std::uint64_t run_id = 0);
void ClearRgbdObservationContext();
RgbdObservationContext GetRgbdObservationContext();
RgbdResetCounters GetRgbdResetCounters();
double RgbdThreadCpuMilliseconds();
std::uint64_t RgbdSteadyNanoseconds();
std::uint64_t RgbdLinuxThreadId(); // Linux kernel TID, 0 if unavailable.

// ORB CPU observations use fixed per-worker slots. Slot 0 is the caller;
// helper slots include task execution/claiming, not their sleeping lifetime.
enum class OrbObservationStage : unsigned {
  Pyramid, Keypoints, Orientation, Blur, Descriptor, Assembly, Count
};
constexpr std::size_t kOrbObservationStages =
    static_cast<std::size_t>(OrbObservationStage::Count);
constexpr std::size_t kMaxRgbdOrbWorkers = 4;
enum class OrbLevelObservationStage : unsigned {
  FastInitial, FastFallback, Octree, Orientation, Copy, Gaussian, Descriptor, Count
};
constexpr std::size_t kOrbLevelObservationStages =
    static_cast<std::size_t>(OrbLevelObservationStage::Count);
struct OrbTimingRecord {
  bool collected = false, detail_collected = false, cpu_valid = false;
  double extract_caller_cpu_ms = -1.0;
  double helper_cpu_ms = -1.0, total_cpu_ms = -1.0;
  std::array<double, kOrbObservationStages> stage_caller_cpu_ms{};
  std::array<double, kOrbObservationStages> stage_helper_cpu_ms{};
  std::array<double, kMaxRgbdOrbWorkers> worker_cpu_ms{};
  std::array<std::uint32_t, kMaxRgbdOrbWorkers> worker_tasks{};
  int worker_count = 0;
  std::array<std::uint32_t, kMaxRgbdOrbLevels> level_outputs{};
  std::array<std::uint32_t, kMaxRgbdOrbLevels> level_rows{}, level_cols{};
  std::array<std::uint64_t, kMaxRgbdOrbLevels> level_stride{};
  std::array<std::array<double, kOrbLevelObservationStages>, kMaxRgbdOrbLevels>
      level_cpu_ms{};
  // Exactly the existing keypoints/orientation/descriptor dispatch batches.
  // Zero outside detail_collected means unmeasured, not no waiting.
  std::array<double, 3> dispatch_ms{}, barrier_ms{};
};

enum class KeyFrameDecisionReason : int {
  NotEvaluated = 0, ImuStartupInsert, ImuStartupWait, OnlyTracking,
  MappingStopped, MappingStopRequested, RelocalizationGuard,
  ConditionsFalse, MappingAccepts, MappingInitializing,
  BusyQueueAccepts, BusyQueueFull, BusyMonocular
};
enum class KeyFrameCreateResult : int {
  NotCalled = 0, RejectedByTrackingState, Initializing,
  StopGuardRejected, Enqueued
};
enum KeyFrameDecisionBit : unsigned {
  KfImuStartup = 0, KfOnlyTracking, KfMappingStopped, KfStopRequested,
  KfRelocalizationGuard, KfMappingIdle, KfNeedClose, KfC1a, KfC1b,
  KfC1c, KfC2, KfC3, KfC4, KfCombinedConditions,
  KfMappingInitializing, KfQueueBelowThree
};
struct KeyFrameDecisionRecord {
  std::uint32_t evaluated_mask = 0;
  std::uint32_t value_mask = 0;
  KeyFrameDecisionReason reason = KeyFrameDecisionReason::NotEvaluated;
  KeyFrameCreateResult create_result = KeyFrameCreateResult::NotCalled;
  int need_result = -1;
  int reference_matches = -1;
  int matches_inliers = -1;
  int tracked_close = -1;
  int untracked_close = -1;
  int map_keyframes = -1;
  int queue_depth = -1; // Not queried in every original branch.
  float reference_ratio = 0;
  std::uint64_t frames_since_keyframe = 0;
  int min_frames = 0;
  int max_frames = 0;
  bool interrupt_called = false;
  std::uint64_t interrupt_request_id = 0, interrupt_attempt_id = 0;
  std::uint64_t interrupt_request_ns = 0;
};
struct LocalPointGatherCounts {
  std::uint64_t slots = 0, null_identity = 0, hint_hits = 0;
  std::uint64_t hint_collisions = 0;
  std::uint64_t weak_locks = 0, expired = 0, frame_marked = 0, bad = 0;
  std::uint64_t accepted = 0;
};
// N01: fixed first-four samples per site group, only in detail_window.
// Source identity is the containing record's observation; native_thread_id is
// the Linux TID copied from the containing record, not a std::thread::id hash.
struct LocalMapClearObservation {
  bool collected = false;
  // Failed clock reads: monotonic begin/end bits 0/1; thread CPU begin/end 2/3.
  std::uint8_t clock_failures = 0;
  double wall_ms = -1, cpu_ms = -1;
  std::uint64_t begin_ns = 0, end_ns = 0;
};
enum class LocalMapLockSite : std::uint8_t {
  NotCollected = 0, GatherKeyFrameFeatures, VoteMapPointFeatures,
  CovisibleConnections, ChildrenConnections, ParentConnections,
  VoteMapPointIsBadFeatures, FilterKeyFrameIsBadConnections
};
struct LocalMapLockSample {
  LocalMapLockSite site = LocalMapLockSite::NotCollected;
  // Failed monotonic reads: request/acquired/before-unlock bits 0/1/2.
  std::uint8_t clock_failures = 0;
  std::uint64_t object_id = 0, object_frame_id = 0;
  double object_timestamp = 0; // KF image timestamp; MP samples leave this 0.
  // Reads immediately before lock(), after lock() returns, and before unlock.
  // The last endpoint is NOT an observed unlock completion or writer trace.
  std::uint64_t request_ns = 0, acquired_ns = 0, before_unlock_ns = 0;
  std::uint64_t call_ordinal = 0;
};
constexpr std::size_t kLocalMapLockSamples = 4;
struct LocalMapLockSamples {
  std::array<LocalMapLockSample, kLocalMapLockSamples> samples{};
  std::uint32_t count = 0;
  std::uint64_t skipped = 0;
  std::uint64_t calls = 0;
  LocalMapLockSample* Next(LocalMapLockSite site) {
    ++calls;
    if (count == samples.size()) { ++skipped; return nullptr; }
    auto* sample = &samples[count++];
    sample->site = site;
    sample->call_ordinal = calls;
    return sample;
  }
  // D: one residue per 128 calls, rotated before capture by source identity.
  // Four samples maximum; calls beyond the fourth selected call are omitted.
  LocalMapLockSample* NextRotating(LocalMapLockSite site, std::uint64_t source) {
    ++calls;
    const auto offset = (source * 17 + static_cast<unsigned>(site) * 13) % 128;
    if ((calls - 1) % 128 != offset || count == samples.size()) {
      ++skipped; return nullptr;
    }
    auto* sample = &samples[count++];
    sample->site = site;
    sample->call_ordinal = calls;
    return sample;
  }
};
struct LocalMapDetailObservation {
  bool collected = false;
  std::uint64_t native_thread_id = 0, old_owner_count = 0;
  LocalMapClearObservation hint_clear, owner_clear;
  LocalMapLockSamples gather_features, voting_features, keyframe_connections;
  LocalMapClearObservation a1_total, a1_voting, a1_filter;
  LocalMapLockSamples voting_is_bad, filter_is_bad;
};
enum class LocalBaOutcome : int {
  NotInvoked = 0, Running, NoFixedKeyFrame, CancelledBeforeSolve,
  Committed, CommittedAfterCancelObserved
};
enum class LocalBaPhase : unsigned {
  Collect = 0, Build, Solve, Classify, MapLockWait, Commit, Destroy, Count
};
struct LocalBaObservation {
  LocalBaOutcome outcome = LocalBaOutcome::NotInvoked;
  std::uint64_t first_observed_ns = 0;
  std::uint64_t entered_ns = 0, returned_ns = 0;
  int iterations_requested = 0, iterations_completed = 0;
  int free_keyframes = 0, fixed_keyframes = 0, map_points = 0;
  int mono_edges = 0, body_edges = 0, stereo_edges = 0;
  bool solver_invoked = false, cancel_observed = false, committed = false;
  std::uint32_t phases_visited = 0;
  std::array<double, static_cast<unsigned>(LocalBaPhase::Count)> wall_ms{};
  std::array<double, static_cast<unsigned>(LocalBaPhase::Count)> cpu_ms{};
};

enum class PoseObservationPhase : unsigned {
  Setup = 0, MapLockWait, Build, Solve, Classify, Commit, Destroy, Count
};
struct PoseOptimizationObservation {
  std::uint64_t calls = 0, edges = 0, mono_edges = 0, stereo_edges = 0, body_edges = 0;
  std::uint64_t solves = 0, returned_iterations = 0, commits = 0;
  std::uint32_t phases_visited = 0;
  bool cpu_available = true;
  std::array<double, static_cast<unsigned>(PoseObservationPhase::Count)> wall_ms{};
  std::array<double, static_cast<unsigned>(PoseObservationPhase::Count)> cpu_ms{};
};
struct ProjectionMatchObservation {
  std::uint64_t points = 0, queries = 0, candidates = 0;
  std::uint64_t occupied_skips = 0, stereo_gate_skips = 0, distances = 0;
};

struct RgbdTimingRecord {
  PoseOptimizationObservation pose;
  ProjectionMatchObservation projection_match;
  bool l0_collected = false;
  RgbdObservationContext observation;
  std::uint64_t tracking_thread_id = 0;
  KeyFrameDecisionRecord keyframe_decision;
  std::uint64_t voting_slots = 0, voting_associated = 0;
  std::uint64_t voting_observation_entries = 0, voting_unique_keyframes = 0;
  double voting_observation_copy_ms = 0; // Only detail_window; zero otherwise.
  double voting_observation_copy_cpu_ms = 0;
  LocalPointGatherCounts gather;
  LocalMapDetailObservation local_map_detail;
  std::uint64_t search_associated_slots = 0, search_local_slots = 0;
  std::uint64_t search_seen_skips = 0, search_bad_skips = 0;
  std::uint64_t search_projected = 0, search_rejected = 0, search_visible = 0;
  int search_matches = 0;
  double local_keyframes_cpu_ms = 0, local_points_cpu_ms = 0;
  double local_search_cpu_ms = 0;
  double image_timestamp = 0.0;
  std::uint64_t frame_id = 0;
  int state_before = -1;
  int state_after = -1;
  int keypoints = 0;
  int valid_depths = 0;
  int tracked_inliers = 0;
  int local_mapping_queue_before = 0;
  int local_mapping_queue_after = 0;
  int inserted_keyframe = 0;

  double system_clone_ms = 0.0;
  double system_resize_ms = 0.0;
  double mode_mutex_wait_ms = 0.0;
  double mode_change_ms = 0.0;
  double reset_mutex_wait_ms = 0.0;
  double reset_ms = 0.0;
  double grab_rgbd_ms = 0.0;
  double state_copy_ms = 0.0;
  double system_total_ms = 0.0;

  double gray_convert_ms = 0.0;
  double depth_convert_ms = 0.0;
  double frame_construct_ms = 0.0;
  double frame_construct_cpu_ms = -1.0;
  OrbTimingRecord orb;
  double orb_extract_ms = 0.0;
  double orb_pyramid_ms = 0.0;
  double orb_keypoints_octree_ms = 0.0;
  double orb_orientation_ms = 0.0;
  double orb_blur_ms = 0.0;
  double orb_descriptor_ms = 0.0;
  double orb_result_assembly_ms = 0.0;
  double orb_fast_initial_ms = 0.0;
  double orb_fast_fallback_ms = 0.0;
  double orb_octree_ms = 0.0;
  double orb_copy_ms = 0.0;
  double orb_gaussian_ms = 0.0;
  std::uint64_t orb_output_hash = 0;
  int orb_mono_index = 0;
  int orb_level_count = 0;
  std::array<double, kMaxRgbdOrbLevels> orb_level_fast_initial_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_fast_fallback_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_octree_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_orientation_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_copy_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_gaussian_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_descriptor_ms{};
  std::array<std::uint32_t, kMaxRgbdOrbLevels> orb_level_candidates{};
  std::array<std::uint32_t, kMaxRgbdOrbLevels> orb_level_fallback_cells{};
  std::array<std::uint32_t, kMaxRgbdOrbLevels> orb_level_octree_splits{};
  double rgbd_depth_assoc_ms = 0.0;
  double track_internal_ms = 0.0;
  double map_mutex_wait_ms = 0.0;
  double initialization_ms = 0.0;
  double reference_kf_ms = 0.0;
  double motion_model_ms = 0.0;
  double relocalization_ms = 0.0;
  double local_map_ms = 0.0;
  double local_update_ms = 0.0;
  double local_keyframes_ms = 0.0;
  double local_points_ms = 0.0;
  double local_search_ms = 0.0;
  double local_pose_ms = 0.0;
  std::uint64_t local_keyframes = 0;
  std::uint64_t local_points = 0;
  double need_new_kf_ms = 0.0;
  double create_new_kf_ms = 0.0;
};

struct LocalMappingTimingRecord {
  bool l0_collected = false;
  std::uint64_t mapping_thread_id = 0;
  RgbdObservationContext observation;
  std::uint64_t refinement_attempt_id = 0;
  std::uint64_t attempt_first_request_ns = 0;
  std::uint64_t attempt_requests = 0;
  LocalBaObservation ba;
  KeyFrameIngestTiming ingest;
  std::array<MedoidTiming, static_cast<std::size_t>(MedoidSite::Count)> medoid;
  std::uint64_t keyframe_id = 0;
  double image_timestamp = 0.0;
  int queue_depth_before_pop = 0;
  double queue_wait_ms = 0.0;
  double bow_ms = 0.0;
  double process_keyframe_ms = 0.0;
  double map_point_culling_ms = 0.0;
  double map_point_creation_ms = 0.0;
  double local_ba_ms = 0.0;
  double local_ba_cpu_ms = -1.0; // calling-thread CPU; -1 when unavailable/not run
  double keyframe_culling_ms = 0.0;
  double total_ms = 0.0;
  int lba_executed = 0;
  int lba_aborted = 0;
  int lba_edges = 0;
  int lba_optimized_keyframes = 0;
  int lba_fixed_keyframes = 0;
  int lba_map_points = 0;
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_RGBD_TIMING_H
