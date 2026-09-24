#include "LocalTrackingAttempt.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace ORB_SLAM3 {
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
}

LocalPointDelta& LocalTrackingAttempt::Point(const MapPointPtr& point) {
  auto it = deltas.find(point.get());
  if (it == deltas.end()) {
    LocalPointDelta value;
    value.snapshot = point->SnapshotLocalTracking();
    it = deltas.emplace(point.get(), std::move(value)).first;
  }
  return it->second;
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
  build_ms += Milliseconds(start);
}

void LocalTrackingAttempt::SearchAndOptimize() {
  auto start = Clock::now();
  std::set<MapPoint*> visited;
  // Snapshot slots, release the KF feature lock, then read MP state. No MP/KF
  // callback nesting and no shared deduplication marker writes in an attempt.
  for (auto it = keyframes.rbegin(); it != keyframes.rend(); ++it) {
    const auto slots = (*it)->GetMapPointMatches();
    for (const auto& point : slots) {
      if (!point || !visited.insert(point.get()).second) continue;
      auto& state = Point(point);
      if (!state.snapshot.valid || state.snapshot.map != input.map ||
          state.snapshot.entry_reference == frame.mnId) continue;
      state.mark_reference = true;
      points.push_back(point);
    }
  }
  build_ms += Milliseconds(start);
  start = Clock::now();
  for (auto& point : frame.mvpMapPoints) {
    if (!point) continue;
    auto& state = Point(point);
    if (!state.snapshot.valid || state.snapshot.map != input.map) { point.reset(); continue; }
    if (statistics == LocalStatistics::Legacy) ++state.visible;
    else state.visible = 1;
    state.mark_seen = true;
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
      if (candidate.queried) deltas.at(candidate.snapshot->owner.get()).visible = 1;
  search_ms += Milliseconds(start);
  start = Clock::now();
  std::map<MapPoint*, Eigen::Vector3f> positions;
  for (const auto& point : frame.mvpMapPoints)
    if (point) positions.emplace(point.get(), Point(point).snapshot.geometry.position);
  Optimizer::PoseOptimization(&frame, &positions);
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
  auto fail = [&](const char* reason) {
    rejection = reason; commit_ms += Milliseconds(start); return false;
  };
  if (committed) return fail("duplicate_commit");
  std::vector<KeyFrame*> keys = keyframes;
  if (reference) keys.push_back(reference);
  std::sort(keys.begin(), keys.end(), std::less<KeyFrame*>());
  keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
  std::vector<std::unique_lock<std::mutex>> key_locks;
  // KF retirement takes nested connection locks during spanning-tree updates.
  // Never block while collecting multiple connection locks: failure is wholly
  // before any write, so an attempt can be discarded without lock inversion.
  for (auto* key : keys) {
    key_locks.emplace_back(key->mMutexConnections, std::try_to_lock);
    if (!key_locks.back().owns_lock()) return fail("keyframe_busy");
    key_locks.emplace_back(key->mMutexMap, std::try_to_lock);
    if (!key_locks.back().owns_lock()) return fail("keyframe_map_busy");
    if (key->mbBad || key->mpMap != input.map) return fail("keyframe_invalidated");
  }
  std::vector<LocalPointDelta*> ordered;
  for (auto& entry : deltas) {
    auto& state = entry.second;
    if (state.mark_reference || state.mark_seen || state.write_projection || state.found)
      ordered.push_back(&state);
  }
  std::sort(ordered.begin(), ordered.end(), [](const LocalPointDelta* a, const LocalPointDelta* b) {
    return a->snapshot.id < b->snapshot.id;
  });
  std::vector<std::unique_lock<std::mutex>> point_locks;
  point_locks.reserve(3 * ordered.size());
  for (auto* state : ordered) {
    auto& point = *state->snapshot.owner;
    point_locks.emplace_back(point.mMutexFeatures);
    point_locks.emplace_back(point.mMutexPos);
    point_locks.emplace_back(point.mMutexMap);
    const auto& snapshot = state->snapshot;
    if (point.mbBad || point.mpReplaced || point.mpMap != input.map ||
        point.nObs != snapshot.observations ||
        point.mWorldPos != snapshot.geometry.position ||
        point.mNormalVector != snapshot.geometry.normal ||
        point.mfMinDistance != snapshot.geometry.min_distance ||
        point.mfMaxDistance != snapshot.geometry.max_distance ||
        point.mDescriptor.type() != CV_8UC1 || point.mDescriptor.rows != 1 || point.mDescriptor.cols != 32 ||
        !std::equal(snapshot.descriptor.begin(), snapshot.descriptor.end(), point.mDescriptor.ptr<unsigned char>()))
      return fail("point_invalidated");
    if (statistics == LocalStatistics::Attempted &&
        (state->found < 0 || state->found > state->visible || state->visible > 1))
      return fail("opportunity_invariant");
  }
  // All validation finished. From here there are no KF callbacks or fallible
  // lookups. All MP counters remain protected until the complete batch lands.
  for (auto* state : ordered) {
    auto& point = *state->snapshot.owner;
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
