#ifndef ORB_SLAM3_MEDOID_TIMING_H
#define ORB_SLAM3_MEDOID_TIMING_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <time.h>

namespace ORB_SLAM3 {

enum class MedoidPhase : std::size_t {
  MapPointTotal, ObservationSnapshot, DescriptorGather, Validation, MatrixAllocation,
  Distances, Median, Clone, Publish, SnapshotLockWait, SnapshotLockHeld,
  PublishLockWait, PublishLockHeld, Count
};
enum class MedoidSite : std::size_t { KeyFrameIngest, PointCreation, Fusion, Count };
struct MedoidTiming {
  bool detail_window = false;
  std::uint64_t map_point_calls = 0, bad_map_points = 0;
  std::uint64_t observation_entries = 0, bad_observers = 0;
  std::uint64_t empty_observations = 0, empty_descriptors = 0;
  std::uint64_t descriptor_capacity_sum = 0, descriptor_capacity_max = 0;
  std::uint64_t calls = 0, orb32_calls = 0, other_calls = 0;
  std::uint64_t n_sum = 0, n_squared_sum = 0, n_max = 0;
  std::uint64_t validated_rows = 0, pair_distances = 0, median_rows = 0;
  std::uint64_t cloned_rows = 0, clone_bytes = 0, published_rows = 0;
  // Actual vector capacities, not allocator usable bytes or process RSS.
  std::uint64_t matrix_outer_capacity_sum = 0, matrix_row_capacity_sum = 0;
  std::uint64_t matrix_capacity_bytes_max = 0, median_row_capacity_sum = 0;
  std::array<std::uint64_t, static_cast<std::size_t>(MedoidPhase::Count)> samples{};
  std::array<double, static_cast<std::size_t>(MedoidPhase::Count)> wall_ms{};
  std::array<double, static_cast<std::size_t>(MedoidPhase::Count)> cpu_ms{};
};
enum class KeyFrameIngestPhase : std::size_t {
  Total, Bow, MatchSnapshot, ObservationAdd, NormalDepth, Connections, MapPublish, Count
};
struct KeyFrameIngestTiming {
  std::uint64_t slots = 0, null_points = 0, bad_points = 0;
  std::uint64_t existing_observations = 0, observations_added = 0;
  std::array<std::uint64_t, static_cast<std::size_t>(KeyFrameIngestPhase::Count)> samples{};
  std::array<double, static_cast<std::size_t>(KeyFrameIngestPhase::Count)> wall_ms{};
  std::array<double, static_cast<std::size_t>(KeyFrameIngestPhase::Count)> cpu_ms{};
};

// A null target performs no clock reads. CPU failure is represented as -1.
class RgbdMeasurementSpan {
 public:
  RgbdMeasurementSpan(double* wall, double* cpu) : wall_(wall), cpu_(cpu) {
    if (wall_) { wall_start_ = std::chrono::steady_clock::now(); cpu_start_ = Cpu(); }
  }
  RgbdMeasurementSpan(MedoidTiming* timing, MedoidPhase phase)
      : RgbdMeasurementSpan(timing ? &timing->wall_ms[static_cast<std::size_t>(phase)] : nullptr,
                            timing ? &timing->cpu_ms[static_cast<std::size_t>(phase)] : nullptr) {
    if (timing) ++timing->samples[static_cast<std::size_t>(phase)];
  }
  RgbdMeasurementSpan(KeyFrameIngestTiming* timing, KeyFrameIngestPhase phase)
      : RgbdMeasurementSpan(timing ? &timing->wall_ms[static_cast<std::size_t>(phase)] : nullptr,
                            timing ? &timing->cpu_ms[static_cast<std::size_t>(phase)] : nullptr) {
    if (timing) ++timing->samples[static_cast<std::size_t>(phase)];
  }
  ~RgbdMeasurementSpan() { Stop(); }
  void Stop() {
    if (!wall_) return;
    const double cpu_end = Cpu();
    *wall_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start_).count();
    if (*cpu_ < 0 || cpu_start_ < 0 || cpu_end < 0) *cpu_ = -1;
    else *cpu_ += cpu_end - cpu_start_;
    wall_ = nullptr;
  }
 private:
  static double Cpu() {
    timespec t{};
    return clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t) == 0
        ? t.tv_sec * 1000.0 + t.tv_nsec / 1000000.0 : -1.0;
  }
  double* wall_;
  double* cpu_;
  double cpu_start_ = 0;
  std::chrono::steady_clock::time_point wall_start_;
};

MedoidTiming* CurrentMedoidTiming();
// Scopes stay on the LocalMapping thread and cover nested Replace calls too.
class MedoidTimingScope {
 public:
  explicit MedoidTimingScope(MedoidTiming* timing);
  ~MedoidTimingScope();
  MedoidTimingScope(const MedoidTimingScope&) = delete;
  MedoidTimingScope& operator=(const MedoidTimingScope&) = delete;
 private:
  MedoidTiming* previous_;
};

} // namespace ORB_SLAM3
#endif
