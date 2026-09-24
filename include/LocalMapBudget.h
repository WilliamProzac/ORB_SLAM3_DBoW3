#pragma once
#include "LocalTrackingAttempt.h"
#include <string>
#include <vector>

namespace ORB_SLAM3 {
struct LocalKeyFrameSelection {
  size_t original = 0, candidates = 0, mandatory = 0, selected = 0;
  bool bounded = false;
  std::string reason;
  double milliseconds = 0;
};

// Only changes the attempt's KF vector on success. Selected KFs retain their
// original gather order; missing mandatory KFs are appended in safety order.
LocalKeyFrameSelection SelectLocalKeyFrames(LocalTrackingAttempt& attempt,
    size_t limit, const std::vector<KeyFrame*>& temporal);
} // namespace ORB_SLAM3
