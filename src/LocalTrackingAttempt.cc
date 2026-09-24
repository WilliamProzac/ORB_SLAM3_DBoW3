#include "LocalTrackingAttempt.h"
#include "LocalTrackingIdentity.h"
#include "LocalMapGrid.h"
#include <array>
#include <ostream>
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

void LocalTrackingAttempt::SearchAndOptimize(size_t point_limit,bool reserve_young) {
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
    if(!state.mark_seen) ++associated_points;
    state.mark_seen = true;
    ++associated_slots;
  }
  std::vector<LocalProjectedPoint> candidates;
  candidates.reserve(points.size());
  for (const auto& point : points) {
    auto& state = Point(point);
    // A point rejected by upstream tracking remains forbidden even if it is
    // absent from Frame; private gather visitation is a different concept.
    if(state.mark_seen) continue;
    if(state.snapshot.entry_seen == frame.mnId) {++entry_skipped;continue;}
    state.projection = frame.ProjectLocalPoint(state.snapshot.geometry, 0.5f);
    state.write_projection = true;
    if (!state.projection.visible) {++frustum_rejected;continue;}
    ++visible_candidates;
    if (statistics == LocalStatistics::Legacy) ++state.visible;
    if (input.record_projections)
      frame.mmProjectPoints[point->mnId] = cv::Point2f(state.projection.x, state.projection.y);
    if (input.far_points && state.projection.distance > input.far_distance) {++far_excluded;continue;}
    candidates.push_back({&state.snapshot, state.projection, false});
  }
  if(input.record_selection) {
    selection_samples_.reserve(candidates.size());
    for(const auto& candidate:candidates) {
      const auto& p=candidate.projection;
      const int grid=LocalMapGridCell(p.x,p.y,frame.mnMinX,frame.mnMaxX,frame.mnMinY,frame.mnMaxY,24,16);
      const int depth=p.z<frame.mThDepth?0:(p.z<3*frame.mThDepth?1:2);
      const auto first=candidate.snapshot->owner->mnFirstFrame;
      const uint64_t age=first>=0 && frame.mnId>=static_cast<unsigned long>(first)?frame.mnId-first:0;
      selection_samples_.push_back({point_index_.at(candidate.snapshot->owner.get()),grid,depth,age<30?0:(age<150?1:2)});
    }
  }
  point_selection.eligible=point_selection.selected=candidates.size();
  point_selection.protected_points=associated_points;
  if(point_limit) {
    if(statistics!=LocalStatistics::Attempted)
      throw std::invalid_argument("MP budget requires attempted opportunity statistics");
    point_selection=SelectLocalMapPoints(*this,candidates,point_limit,reserve_young);
    search_points.reserve(candidates.size());
    for(const auto& candidate:candidates) search_points.push_back(candidate.snapshot->owner);
  }
  ORBmatcher matcher(0.8f);
  new_matches = matcher.SearchLocalProjection(frame, candidates, input.search_threshold);
  if (statistics == LocalStatistics::Attempted || input.record_selection)
    for (const auto& candidate : candidates) {
      auto& state=Point(candidate.snapshot->owner);
      state.selected_search=true;state.queried_search=candidate.queried;
      if(statistics==LocalStatistics::Attempted && candidate.queried) state.visible=1;
    }
  search_ms += Milliseconds(start);
  start = Clock::now();
  std::map<MapPoint*, Eigen::Vector3f> positions;
  for (size_t i=0;i<frame.mvpMapPoints.size();++i) {
    const auto& point=frame.mvpMapPoints[i];
    if (point) {
      auto& state=Point(point);
      state.mark_associated=true;
      positions.emplace(point.get(), state.snapshot.geometry.position);
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

void LocalTrackingAttempt::WriteSelectionDiagnostics(std::ostream& output,uint64_t generation,const char* role) const {
  using Counts=std::array<size_t,4>; // eligible, selected, queried, found
  std::array<Counts,384> grids{};
  std::array<Counts,3> depths{},ages{};
  for(const auto& sample:selection_samples_) {
    const auto& state=deltas[sample.delta];
    if(state.discarded) continue;
    auto add=[&](Counts& n) {++n[0];n[1]+=state.selected_search;n[2]+=state.queried_search;n[3]+=state.found>0;};
    if(sample.grid>=0) add(grids[sample.grid]);
    add(depths[sample.depth]);add(ages[sample.age]);
  }
  auto write=[&](const char* axis,size_t bin,const Counts& n) {
    if(!n[0]) return;
    output<<generation<<','<<frame.mnId<<','<<role<<','<<axis<<','<<bin;
    for(auto value:n) output<<','<<value;
    output<<'\n';
  };
  for(size_t i=0;i<grids.size();++i) write("grid",i,grids[i]);
  for(size_t i=0;i<3;++i) {write("depth",i,depths[i]);write("creation_frame_age",i,ages[i]);}
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
  auto lock_point=[&](std::mutex& mutex) {
    std::unique_lock<std::mutex> lock(mutex,std::defer_lock);
    // Only time genuinely contended acquisitions; two clocks per uncontended
    // point would itself distort the ARM hot path. A miss waits, never fails.
    if(!lock.try_lock()) {
      const auto waiting=Clock::now();lock.lock();
      commit_lock_wait_ms+=Milliseconds(waiting);
    }
    point_locks.emplace_back(std::move(lock));
  };
  for (auto* state : ordered) {
    auto& point = *state->snapshot.owner;
    lock_point(point.mMutexFeatures);
    lock_point(point.mMutexPos);
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
    if(state->mark_associated) {
      point.mnLocalAssociationFrame=frame.mnId;point.mbLocalAssociationKnown=true;
    }
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
