#include "LocalTrackingAttempt.h"
#include "LocalTrackingIdentity.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace ORB_SLAM3 {
std::recursive_mutex& LocalTrackingIdentityMutex() {
  static std::recursive_mutex mutex;
  return mutex;
}
namespace {
using Clock = std::chrono::steady_clock;
double Milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
}

LocalTrackingAttempt::LocalTrackingAttempt(const LocalTrackingInput& entry,
                                         LocalStatistics policy)
    : input(entry), statistics(policy), frame(entry.frame), reference(entry.reference) {
  if (frame.Nleft != -1 || frame.mpCamera2 || frame.mDescriptors.type() != CV_8UC1)
    throw std::invalid_argument("local tracking transaction requires RGB-D ORB");
  // Frame's visual writable fields (pose/derived matrices, vectors, display
  // maps, reference) are independent. IMU pointers are copied but never used.
  // Commit explicitly transfers only the local-tracking write set.
  deltas.reserve(frame.N);
  point_index_.reserve(frame.N);
}

const LocalPointDelta& LocalTrackingAttempt::ExistingPoint(MapPoint* point) const {
  return deltas.at(point_index_.at(point));
}

LocalPointDelta& LocalTrackingAttempt::RememberPoint(const MapPointPtr& point) {
  auto it = point_index_.find(point.get());
  if (it == point_index_.end()) {
    if (snapshots_sealed_) throw std::logic_error("new point after projection snapshot sealing");
    LocalPointDelta value;
    value.snapshot.owner = point;
    it = point_index_.emplace(point.get(), deltas.size()).first;
    deltas.emplace_back(std::move(value));
  }
  return deltas[it->second];
}

LocalPointDelta& LocalTrackingAttempt::Point(const MapPointPtr& point) {
  auto& state=RememberPoint(point);
  if (!state.captured) {
    const auto start=Clock::now();
    state.snapshot=point->SnapshotLocalTracking();
    state.captured=true;
    snapshot_ms+=Milliseconds(start);
  }
  return state;
}

void LocalTrackingAttempt::BuildLocalMap() {
  const auto start = Clock::now();
  // std::map pointer ordering and strict > vote winner reproduce legacy ties.
  for (auto& point : frame.mvpMapPoints) {
    if (!point) continue;
    auto& state = Point(point);
    if (!state.snapshot.valid || state.snapshot.map != input.map) { point.reset(); continue; }
    const auto observations = point->GetObservations();
    for (const auto& observation : observations) ++votes[observation.first];
  }
  int maximum = 0;
  std::set<KeyFrame*> visited;
  for (const auto& vote : votes) {
    auto* key = vote.first;
    if (key->isBad()) continue;
    if (vote.second > maximum) { maximum = vote.second; vote_winner = key; }
    keyframes.push_back(key);
    visited.insert(key);
  }
  const size_t initial_count = keyframes.size();
  auto unseen = [&](KeyFrame* key) {
    return key && !visited.count(key) && key->mnTrackReferenceForFrame != frame.mnId;
  };
  // The legacy loop captures the initial end; never iterate appended entries.
  for (size_t i = 0; i < initial_count; ++i) {
    if (keyframes.size() > 80) break;
    auto* key = keyframes[i];
    for (auto* neighbor : key->GetBestCovisibilityKeyFrames(10)) {
      if (!neighbor->isBad() && unseen(neighbor)) {
        visited.insert(neighbor); keyframes.push_back(neighbor); break;
      }
    }
    for (auto* child : key->GetChilds()) {
      if (!child->isBad() && unseen(child)) {
        visited.insert(child); keyframes.push_back(child); break;
      }
    }
    auto* parent = key->GetParent();
    if (unseen(parent)) { visited.insert(parent); keyframes.push_back(parent); break; }
  }
  if (vote_winner) { reference = vote_winner; frame.mpReferenceKF = reference; }
  keyframes_ms = Milliseconds(start);
  build_ms += keyframes_ms;
}

void LocalTrackingAttempt::SearchAndOptimize() {
  auto start = Clock::now();
  const size_t estimate=std::max(deltas.size(),std::min<size_t>(16384,keyframes.size()*size_t(frame.N)));
  deltas.reserve(estimate); point_index_.reserve(estimate); points.reserve(estimate);
  // Keep MEM18's cheap identity-before-weak-lock collection. All ownership
  // hints and exact collision resolution are private to this attempt. MP
  // metadata is read only after the KF slot locks have been released.
  RetainedPointCache hints;
  std::unordered_set<std::uintptr_t> identities;
  identities.reserve(estimate);
  for (auto it = keyframes.rbegin(); it != keyframes.rend(); ++it) {
    (*it)->AppendLocalMapCandidates(points,hints,identities);
  }
  // Capture the gathered batch once. Avoid two clock reads and a separately
  // allocated tree node for every scanned point. No projection pointers exist
  // until this vector has reached its final size and is sealed below.
  const auto capture_start=Clock::now();
  for (const auto& point : points) {
    auto& state=RememberPoint(point);
    if (!state.captured) {
      state.snapshot=point->SnapshotLocalTracking();
      state.captured=true;
    }
  }
  snapshot_ms+=Milliseconds(capture_start);
  points.erase(std::remove_if(points.begin(),points.end(),[&](const MapPointPtr& point) {
    auto& state=deltas[point_index_.at(point.get())];
    if (!state.snapshot.valid || state.snapshot.map!=input.map ||
        state.snapshot.entry_reference==frame.mnId) return true;
    state.mark_reference=true;
    return false;
  }),points.end());
  gather_ms = Milliseconds(start);
  build_ms += gather_ms;
  snapshots_sealed_=true;
  start = Clock::now();
  for (auto& point : frame.mvpMapPoints) {
    if (!point) continue;
    auto& state = Point(point);
    if (!state.snapshot.valid || state.snapshot.map != input.map) { point.reset(); continue; }
    if (statistics == LocalStatistics::Legacy) ++state.visible;
    else state.visible = 1;
    state.mark_seen = true;
    ++associated_slots;
  }
  std::vector<LocalProjectedPoint> candidates;
  candidates.reserve(points.size());
  for (const auto& point : points) {
    auto& state = Point(point);
    // A point rejected by upstream tracking remains forbidden even if it is
    // absent from Frame; private gather visitation is a different concept.
    if (state.mark_seen || state.snapshot.entry_seen == frame.mnId) continue;
    state.projection = frame.ProjectLocalPoint(state.snapshot.geometry, 0.5f);
    state.write_projection = true;
    if (!state.projection.visible) continue;
    ++visible_candidates;
    if (statistics == LocalStatistics::Legacy) ++state.visible;
    if (input.record_projections)
      frame.mmProjectPoints[point->mnId] = cv::Point2f(state.projection.x, state.projection.y);
    if (input.far_points && state.projection.distance > input.far_distance) continue;
    candidates.push_back({&state.snapshot, state.projection, false});
  }
  ORBmatcher matcher(0.8f);
  new_matches = matcher.SearchLocalProjection(frame, candidates, input.search_threshold);
  if (statistics == LocalStatistics::Attempted)
    for (const auto& candidate : candidates)
      if (candidate.queried) Point(candidate.snapshot->owner).visible = 1;
  search_ms += Milliseconds(start);
  start = Clock::now();
  std::map<MapPoint*, Eigen::Vector3f> positions;
  for (size_t i=0;i<frame.mvpMapPoints.size();++i) {
    const auto& point=frame.mvpMapPoints[i];
    if (point) {
      positions.emplace(point.get(), Point(point).snapshot.geometry.position);
      ++pre_associations;
      if (frame.mvbOutlier[i]) ++pre_outliers;
    }
  }
  pose_inliers = Optimizer::PoseOptimization(&frame, &positions);
  optimize_ms += Milliseconds(start);
  for (int i = 0; i < frame.N; ++i) {
    const auto& point = frame.mvpMapPoints[i];
    if (!point || frame.mvbOutlier[i]) continue;
    auto& state = Point(point);
    if (statistics == LocalStatistics::Legacy) ++state.found;
    else state.found = 1;
    if (input.only_tracking || state.snapshot.observations > 0) ++inliers;
  }
}

bool LocalTrackingAttempt::TrackingSucceeded() const {
  if (frame.mnId < input.last_relocation + input.max_frames && inliers < 50) return false;
  if (input.recently_lost && inliers > 10) return true;
  return inliers >= 30;
}

bool LocalTrackingAttempt::Commit(Frame& destination) {
  const auto start = Clock::now();
  LocalTrackingIdentityGuard identity(LocalTrackingIdentityMutex());
  auto fail = [&](const char* reason) {
    rejection = reason; commit_ms += Milliseconds(start); return false;
  };
  if (committed) return fail("duplicate_commit");
  if (input.map->IsBad()) return fail("map_invalidated");
  std::vector<KeyFrame*> keys = keyframes;
  if (reference) keys.push_back(reference);
  std::sort(keys.begin(), keys.end(), std::less<KeyFrame*>());
  keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
  // The identity barrier prevents retirement/transfer through commit. Validate
  // one KF at a time; collecting connection locks would invert the ordinary
  // UpdateConnections child->parent lock order. Brief metadata readers must
  // not manufacture failed frames merely because try_lock observed them.
  for (auto* key : keys) {
    if (key->isBad() || key->GetMap() != input.map) return fail("keyframe_invalidated");
  }
  std::vector<LocalPointDelta*> ordered;
  ordered.reserve(deltas.size());
  bool any_discarded=false;
  for (auto& state : deltas) {
    state.discarded=false;
    if (state.mark_reference || state.mark_seen || state.write_projection || state.found)
      ordered.push_back(&state);
  }
  std::sort(ordered.begin(), ordered.end(), [](const LocalPointDelta* a, const LocalPointDelta* b) {
    return a->snapshot.id < b->snapshot.id;
  });
  std::vector<std::unique_lock<std::mutex>> point_locks;
  point_locks.reserve(2 * ordered.size());
  for (auto* state : ordered) {
    auto& point = *state->snapshot.owner;
    point_locks.emplace_back(point.mMutexFeatures);
    point_locks.emplace_back(point.mMutexPos);
    // UpdateMap is serialized by the identity barrier already held here.
    // Snapshot callers still take the map mutex because they need not hold
    // that barrier. Commit need not acquire a redundant third lock per MP.
    const auto& snapshot = state->snapshot;
    if (point.mbBad || point.mpReplaced || point.mpMap != input.map) {
      if (state->found>0) return fail("point_invalidated");
      // Mapping can retire an unused candidate while matching. Its scratch
      // result has no pose constraint to invalidate: discard that point's
      // entire write set instead of manufacturing a tracking failure.
      state->discarded=any_discarded=true;
      continue;
    }
    // ProcessNewKeyFrame legitimately changes observation count, descriptor,
    // normal and distance bounds outside the map update lock. This attempt
    // used immutable copies, as legacy matching may use a descriptor/normal
    // captured before that update. Do not manufacture tracking loss for it.
    // Position and identity must still match; the observed/non-observed class
    // used by the original Tracking inlier threshold must also remain valid.
    if ((point.nObs>0)!=(snapshot.observations>0) || point.mWorldPos != snapshot.geometry.position) {
      if (state->found>0) return fail("point_constraint_changed");
      state->discarded=any_discarded=true;
      continue;
    }
    if (statistics == LocalStatistics::Attempted &&
        (state->found < 0 || state->found > state->visible || state->visible > 1))
      return fail("opportunity_invariant");
  }
  // All validation finished. From here there are no KF callbacks or fallible
  // lookups. All MP counters remain protected until the complete batch lands.
  for (auto* state : ordered) {
    auto& point = *state->snapshot.owner;
    if (state->discarded) continue;
    point.mnVisible += state->visible;
    point.mnFound += state->found;
    if (state->mark_reference) point.mnTrackReferenceForFrame = frame.mnId;
    if (state->mark_seen) {
      point.mnLastFrameSeen = frame.mnId;
      point.mbTrackInView = false; point.mbTrackInViewR = false;
    }
    if (state->write_projection) Frame::ApplyLocalProjection(point, state->projection);
  }
  for (auto* key : keyframes) key->mnTrackReferenceForFrame = frame.mnId;
  if (any_discarded) {
    points.erase(std::remove_if(points.begin(),points.end(),[&](const MapPointPtr& point) {
      return ExistingPoint(point.get()).discarded;
    }),points.end());
    for (auto& point : frame.mvpMapPoints)
      if (point && ExistingPoint(point.get()).discarded) point.reset();
  }
  destination.SetPose(frame.GetPose());
  destination.mvpMapPoints.swap(frame.mvpMapPoints);
  destination.mvbOutlier.swap(frame.mvbOutlier);
  destination.mpReferenceKF = frame.mpReferenceKF;
  destination.mmProjectPoints.swap(frame.mmProjectPoints);
  destination.mmMatchedInImage.swap(frame.mmMatchedInImage);
  committed = true;
  commit_ms += Milliseconds(start);
  return true;
}
} // namespace ORB_SLAM3
