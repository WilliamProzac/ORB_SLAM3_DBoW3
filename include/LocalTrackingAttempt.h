#pragma once
#include "Frame.h"
#include "LocalMapProjection.h"
#include <map>
#include <memory>
#include <set>

namespace ORB_SLAM3 {
class KeyFrame;
class Map;

enum class LocalStatistics { Legacy, Attempted };
struct LocalTrackingInput {
  const Frame& frame;
  Map* map = nullptr;
  KeyFrame* reference = nullptr;
  KeyFrame* last_keyframe = nullptr;
  KeyFrame* relocalization_keyframe = nullptr;
  bool only_tracking = false, recently_lost = false, record_projections = true;
  bool far_points = false;
  float far_distance = 50, search_threshold = 3;
  unsigned long last_relocation = 0;
  int max_frames = 30;
};

struct LocalPointDelta {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalPointSnapshot snapshot;
  LocalProjection projection;
  int visible = 0, found = 0;
  bool write_projection = false, mark_seen = false, mark_reference = false;
};

// One disposable attempt. No shared marker/cache/counter writes before Commit.
// Caller keeps the existing active-map update lock from input capture through
// commit. This is explicitly a visual RGB-D API, not an IMU Frame transaction.
class LocalTrackingAttempt {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit LocalTrackingAttempt(const LocalTrackingInput& input,
                                LocalStatistics statistics = LocalStatistics::Legacy);
  void BuildLocalMap();
  void SearchAndOptimize();
  bool Commit(Frame& destination);
  LocalPointDelta& Point(const MapPointPtr& point);
  bool TrackingSucceeded() const;

  LocalTrackingInput input;
  LocalStatistics statistics;
  Frame frame;
  KeyFrame* reference;
  KeyFrame* vote_winner = nullptr;
  std::map<KeyFrame*, int> votes;
  std::vector<KeyFrame*> keyframes;
  std::vector<MapPointPtr> points;
  std::map<MapPoint*, LocalPointDelta> deltas;
  int inliers = 0, new_matches = 0, visible_candidates = 0;
  int pre_associations = 0, pre_outliers = 0, pose_inliers = 0;
  int associated_slots = 0;
  bool committed = false;
  std::string rejection;
  double build_ms = 0, search_ms = 0, optimize_ms = 0, commit_ms = 0;
  double keyframes_ms = 0, gather_ms = 0, snapshot_ms = 0;
};
} // namespace ORB_SLAM3
