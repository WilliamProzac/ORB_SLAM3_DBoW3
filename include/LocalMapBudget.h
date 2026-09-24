#pragma once
#include "LocalTrackingAttempt.h"
#include <string>
#include <vector>

namespace ORB_SLAM3 {
enum class LocalKeyFrameStrategy { Votes, Coverage };
struct LocalKeyFrameSelection {
  size_t original = 0, candidates = 0, mandatory = 0, selected = 0;
  size_t support_points=0,support_cells=0,mandatory_support=0,mandatory_cells=0;
  int saturation_selected=-1;
  size_t saturation_slots=0,saturation_votes=0,saturation_diversity=0,saturation_ties=0;
  bool coverage_used=false;
  bool bounded = false;
  std::string reason;
  double milliseconds = 0;
};

// Only changes the attempt's KF vector on success. Selected KFs retain their
// original gather order; missing mandatory KFs are appended in safety order.
LocalKeyFrameSelection SelectLocalKeyFrames(LocalTrackingAttempt& attempt,
    size_t limit, const std::vector<KeyFrame*>& temporal,
    LocalKeyFrameStrategy strategy=LocalKeyFrameStrategy::Votes);
} // namespace ORB_SLAM3
