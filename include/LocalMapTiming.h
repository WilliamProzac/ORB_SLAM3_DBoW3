#pragma once
#include "RgbdTiming.h"
#include <time.h>

namespace ORB_SLAM3 {
inline bool LocalMapClockNanoseconds(clockid_t clock, std::uint64_t& result) {
  timespec now{};
  if (clock_gettime(clock, &now) != 0) return false;
  result = std::uint64_t(now.tv_sec) * 1000000000ULL + now.tv_nsec;
  return true;
}
inline void InitializeLocalMapDetail(LocalMapDetailObservation* detail, std::uint64_t tid) {
  if (!detail || detail->collected) return;
  detail->collected = true;
  detail->native_thread_id = tid;
}
// Null observation means no clock reads, including when L1 is disabled.
class LocalMapClearSpan {
 public:
  explicit LocalMapClearSpan(LocalMapClearObservation* record) : record_(record) {
    if (!record_) return;
    record_->collected = true;
    if (!LocalMapClockNanoseconds(CLOCK_MONOTONIC, wall_begin_))
      record_->clock_failures |= 1;
    if (!LocalMapClockNanoseconds(CLOCK_THREAD_CPUTIME_ID, cpu_begin_))
      record_->clock_failures |= 4;
  }
  ~LocalMapClearSpan() {
    if (!record_) return;
    std::uint64_t wall_end = 0, cpu_end = 0;
    if (!LocalMapClockNanoseconds(CLOCK_THREAD_CPUTIME_ID, cpu_end))
      record_->clock_failures |= 8;
    if (!LocalMapClockNanoseconds(CLOCK_MONOTONIC, wall_end))
      record_->clock_failures |= 2;
    if (!(record_->clock_failures & 3) && wall_end >= wall_begin_)
      record_->wall_ms = (wall_end - wall_begin_) * 1e-6;
    if (!(record_->clock_failures & 12) && cpu_end >= cpu_begin_)
      record_->cpu_ms = (cpu_end - cpu_begin_) * 1e-6;
  }
 private:
  LocalMapClearObservation* record_;
  std::uint64_t wall_begin_ = 0, cpu_begin_ = 0;
};
// Declare after a deferred unique_lock so the final clock read precedes that
// lock's destructor. This observes a bounded acquisition/hold interval only.
class LocalMapLockSpan {
 public:
  explicit LocalMapLockSpan(LocalMapLockSample* record) : record_(record) {
    if (record_ && !LocalMapClockNanoseconds(CLOCK_MONOTONIC, record_->request_ns))
      record_->clock_failures |= 1;
  }
  void Acquired() {
    if (record_ && !LocalMapClockNanoseconds(CLOCK_MONOTONIC, record_->acquired_ns))
      record_->clock_failures |= 2;
  }
  ~LocalMapLockSpan() {
    if (record_ && !LocalMapClockNanoseconds(CLOCK_MONOTONIC, record_->before_unlock_ns))
      record_->clock_failures |= 4;
  }
 private:
  LocalMapLockSample* record_;
};
} // namespace ORB_SLAM3
