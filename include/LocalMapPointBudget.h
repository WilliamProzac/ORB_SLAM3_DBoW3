#pragma once
#include "LocalMapProjection.h"
#include <vector>
namespace ORB_SLAM3 {
class LocalTrackingAttempt;
struct LocalPointSelection {
  size_t eligible=0,selected=0,protected_points=0,young_reserved=0;
  bool truncated=false,valid=true;
  double milliseconds=0,observation_ms=0;
};
// Input is already deduplicated, visible, not protected/upstream-skipped and
// far-filtered. Output is a stable subsequence of that exact matcher order.
LocalPointSelection SelectLocalMapPoints(LocalTrackingAttempt& attempt,
    std::vector<LocalProjectedPoint>& candidates,size_t limit,bool reserve_young);
}
