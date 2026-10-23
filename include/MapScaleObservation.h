#ifndef ORB_SLAM3_MAP_SCALE_OBSERVATION_H
#define ORB_SLAM3_MAP_SCALE_OBSERVATION_H

#include <array>
#include <cstdint>
#include <iosfwd>

namespace ORB_SLAM3 {

// Matched experiment-only observation, independent of legacy L0/L1.
// ORB CPU is the calling thread only; helper worker CPU is not included.
enum class MapScaleStage : unsigned {
  Orb, TrackLocalMap, ReferenceCopy, A1, A1Vote, A1Filter, A3,
  A3Clear, A3Gather, SearchLocalPoints, LocalPoseOptimization,
  MapUpdateWait, ScaleSnapshot, Count
};
struct MapScaleStageTime {
  double wall_ms = -1.0;
  double cpu_ms = -1.0;
  std::uint64_t calls = 0;
};
struct MapScaleObservationRecord {
  std::array<MapScaleStageTime, static_cast<unsigned>(MapScaleStage::Count)> stages{};
  // Active current-map container sizes sampled together under Map::mMutexMap,
  // after Tracking has acquired its existing map-update mutex, before tracking.
  // These are neither lifetime creation counters nor full Atlas totals.
  std::int64_t global_kf = -1, global_mp = -1, map_id = -1;
  std::int64_t local_kf = -1, local_mp = -1, voting_kf = -1;
  std::int64_t gather_slots = -1, visited_slots = -1, accepted_points = -1;
  std::int64_t frame_keypoints = -1, tracking_inliers = -1;
  std::int64_t tracking_state = -1, frame_id = -1;
  std::int64_t scale_snapshot_mono_ns = -1;
  std::uint64_t clock_failures = 0;
};

// The wrapper resets on its TrackRGBD calling thread, then copies and disables.
// No record pointer is shared with worker/background threads.
void ResetMapScaleObservation(bool enabled);
void DisableMapScaleObservation();
MapScaleObservationRecord* ActiveMapScaleObservation();
const MapScaleObservationRecord& GetMapScaleObservation();
std::int64_t MapScaleMonotonicNanoseconds();

// Both CSV functions emit leading commas, and never run on a hot path.
void WriteMapScaleHeader(std::ostream& out);
void WriteMapScaleRow(std::ostream& out, const MapScaleObservationRecord& record);

class MapScaleSpan {
 public:
  explicit MapScaleSpan(MapScaleStage stage);
  ~MapScaleSpan() { Finish(); }
  void Finish();
  MapScaleSpan(const MapScaleSpan&) = delete;
  MapScaleSpan& operator=(const MapScaleSpan&) = delete;
 private:
  MapScaleObservationRecord* record_;
  MapScaleStage stage_;
  std::int64_t wall_start_ = -1, cpu_start_ = -1;
};

} // namespace ORB_SLAM3
#endif

