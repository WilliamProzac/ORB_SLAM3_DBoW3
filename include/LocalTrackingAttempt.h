#pragma once
#include "Frame.h"
#include "LocalMapProjection.h"
#include "LocalMapPointBudget.h"
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <iosfwd>

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
  bool far_points = false, record_selection = false;
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
  bool captured = false, gathered = false, discarded = false;
  bool mark_associated = false, selected_search = false, queried_search = false;
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
  void SearchAndOptimize(size_t point_limit=0,bool reserve_young=false);
  bool Commit(Frame& destination);
  LocalPointDelta& Point(const MapPointPtr& point);
  const LocalPointDelta& ExistingPoint(MapPoint* point) const;
  bool TrackingSucceeded() const;
  void WriteSelectionDiagnostics(std::ostream& output,uint64_t generation,const char* role) const;

  LocalTrackingInput input;
  LocalStatistics statistics;
  Frame frame;
  KeyFrame* reference;
  KeyFrame* vote_winner = nullptr;
  std::map<KeyFrame*, int> votes;
  std::vector<KeyFrame*> keyframes;
  std::vector<MapPointPtr> points;
  // Contiguous write set. Projection pointers are created only after gathering
  // seals it; adding a new snapshot after that boundary is a contract error.
  std::vector<LocalPointDelta> deltas;
  int inliers = 0, new_matches = 0, visible_candidates = 0;
  int pre_associations = 0, pre_outliers = 0, pose_inliers = 0;
  int associated_slots = 0, associated_points = 0;
  int entry_skipped=0,frustum_rejected=0,far_excluded=0;
  LocalPointSelection point_selection;
  std::vector<MapPointPtr> search_points;
  bool committed = false;
  std::string rejection;
  double build_ms = 0, search_ms = 0, optimize_ms = 0, commit_ms = 0;
  double keyframes_ms = 0, gather_ms = 0, snapshot_ms = 0, commit_lock_wait_ms = 0;
 private:
  LocalPointDelta& RememberPoint(const MapPointPtr& point);
  std::unordered_map<MapPoint*, size_t> point_index_;
  bool snapshots_sealed_ = false;
  struct SelectionSample {size_t delta;int grid,depth,age;};
  std::vector<SelectionSample> selection_samples_;
};
} // namespace ORB_SLAM3
