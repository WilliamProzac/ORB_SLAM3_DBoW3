#pragma once
#include <map>
#include <memory>
#include "WeakIdentitySlot.h"

namespace ORB_SLAM3 {
class MapPoint;
// Copy under the source container's data lock. A handle protects lifetime,
// independently of MapPoint/KeyFrame data locking.
using MapPointPtr = std::shared_ptr<MapPoint>;
using MapPointWeak = WeakIdentitySlot<MapPoint>;
// Archive-only identity table: adopt every raw loaded allocation exactly once.
using LoadedMapPointOwners = std::map<MapPoint*, MapPointPtr>;
}
