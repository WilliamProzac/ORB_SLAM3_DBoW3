#ifndef ORB_SLAM3_LOOP_LIFECYCLE_H
#define ORB_SLAM3_LOOP_LIFECYCLE_H
#include <cstdint>
namespace ORB_SLAM3 {
struct LoopLifecycleCounts {
  std::uint64_t loop_detected, gba_started, gba_completed;
  std::uint64_t gba_returned, gba_joined_count, gba_errors;
  std::uint64_t gba_cancelled, gba_drain_cancel_requests;
};
}
#endif
