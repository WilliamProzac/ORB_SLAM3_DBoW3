#pragma once
#include <mutex>

namespace ORB_SLAM3 {
// Lock hierarchy: existing map-update lock (when held), identity barrier,
// replacement-edge lock, object locks. Retirement may recurse from a KF to its
// MPs. Never acquire this barrier while holding an object feature/position/
// connection lock. Ordinary observation/descriptor updates do not take it.
// One process-wide barrier also makes two-map identity transfers unambiguous.
std::recursive_mutex& LocalTrackingIdentityMutex();
using LocalTrackingIdentityGuard = std::lock_guard<std::recursive_mutex>;
} // namespace ORB_SLAM3
