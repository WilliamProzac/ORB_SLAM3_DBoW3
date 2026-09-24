#include "LocalMapEvents.h"
#include <atomic>
namespace ORB_SLAM3 {
uint64_t NextLocalMapGeneration() {
  static std::atomic<uint64_t> next{1};
  return next.fetch_add(1,std::memory_order_relaxed);
}
} // namespace ORB_SLAM3
