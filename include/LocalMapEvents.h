#pragma once
#include <cstdint>
#include <mutex>
namespace ORB_SLAM3 {
enum class LocalMapEventType : unsigned {
  None=0, LoopCorrection=1, GlobalBA=2, Merge=3, Switch=4, Reset=5
};
struct LocalMapEventState {
  uint64_t generation=0, version=0;
  LocalMapEventType type=LocalMapEventType::None;
};
uint64_t NextLocalMapGeneration();
// Deliberately absent from Map::serialize: loaded objects start fresh history.
// Ordinary Local BA change indexes do not call Publish.
class LocalMapEvents {
 public:
  LocalMapEventState Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_); return state_;
  }
  void Publish(LocalMapEventType type) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++state_.version; state_.type=type;
  }
  void ResetGeneration() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.generation=NextLocalMapGeneration();
    ++state_.version; state_.type=LocalMapEventType::Reset;
  }
 private:
  mutable std::mutex mutex_;
  LocalMapEventState state_{NextLocalMapGeneration(),0,LocalMapEventType::None};
};
} // namespace ORB_SLAM3
