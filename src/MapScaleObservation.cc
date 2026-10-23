#include "MapScaleObservation.h"
#include <ostream>
#include <time.h>

namespace ORB_SLAM3 {
namespace {
thread_local MapScaleObservationRecord record;
thread_local bool enabled = false;

std::int64_t ReadClock(clockid_t id) {
  timespec value;
  if (clock_gettime(id, &value) != 0) {
    if (enabled) ++record.clock_failures;
    return -1;
  }
  return static_cast<std::int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
}
constexpr const char* stage_names[] = {
  "orb", "track_local_map", "reference_copy", "a1", "a1_vote",
  "a1_filter", "a3", "a3_clear", "a3_gather", "search_local_points",
  "local_pose_optimization", "map_update_wait", "scale_snapshot"
};
static_assert(sizeof(stage_names) / sizeof(stage_names[0]) ==
              static_cast<unsigned>(MapScaleStage::Count), "stage schema");
void AddDuration(double& total, std::int64_t begin, std::int64_t end,
                 bool first) {
  if (begin < 0 || end < begin) { total = -1.0; return; }
  const double elapsed = static_cast<double>(end - begin) / 1000000.0;
  if (first) total = elapsed;
  else if (total >= 0.0) total += elapsed;
}
}
void ResetMapScaleObservation(bool collect) {
  record = MapScaleObservationRecord{};
  enabled = collect;
}
void DisableMapScaleObservation() { enabled = false; }
MapScaleObservationRecord* ActiveMapScaleObservation() {
  return enabled ? &record : nullptr;
}
const MapScaleObservationRecord& GetMapScaleObservation() { return record; }
std::int64_t MapScaleMonotonicNanoseconds() {
  return enabled ? ReadClock(CLOCK_MONOTONIC) : -1;
}
MapScaleSpan::MapScaleSpan(MapScaleStage stage)
    : record_(ActiveMapScaleObservation()), stage_(stage) {
  if (record_) {
    wall_start_ = ReadClock(CLOCK_MONOTONIC);
    cpu_start_ = ReadClock(CLOCK_THREAD_CPUTIME_ID);
  }
}
void MapScaleSpan::Finish() {
  if (!record_) return;
  const auto cpu_end = ReadClock(CLOCK_THREAD_CPUTIME_ID);
  const auto wall_end = ReadClock(CLOCK_MONOTONIC);
  auto& stage = record_->stages[static_cast<unsigned>(stage_)];
  const bool first = stage.calls++ == 0;
  AddDuration(stage.wall_ms, wall_start_, wall_end, first);
  AddDuration(stage.cpu_ms, cpu_start_, cpu_end, first);
  record_ = nullptr;
}
void WriteMapScaleHeader(std::ostream& out) {
  for (const auto* name : stage_names)
    out << ',' << name << "_wall_ms," << name << "_cpu_ms,"
        << name << "_calls";
  out << ",global_kf,global_mp,map_id,local_kf,local_mp,voting_kf"
      << ",gather_slots,visited_slots,accepted_points,frame_keypoints"
      << ",tracking_inliers,tracking_state,frame_id,scale_snapshot_mono_ns"
      << ",clock_failures";
}
void WriteMapScaleRow(std::ostream& out, const MapScaleObservationRecord& value) {
  for (const auto& stage : value.stages)
    out << ',' << stage.wall_ms << ',' << stage.cpu_ms << ',' << stage.calls;
  out << ',' << value.global_kf << ',' << value.global_mp << ',' << value.map_id
      << ',' << value.local_kf << ',' << value.local_mp << ',' << value.voting_kf
      << ',' << value.gather_slots << ',' << value.visited_slots
      << ',' << value.accepted_points << ',' << value.frame_keypoints
      << ',' << value.tracking_inliers << ',' << value.tracking_state
      << ',' << value.frame_id << ',' << value.scale_snapshot_mono_ns
      << ',' << value.clock_failures;
}
}

