#include "Tracking.h"
#include "LocalMapBudget.h"
#include "LocalTrackingIdentity.h"
#include "FieldObservation.h"
#include <algorithm>
#include <chrono>
#include <set>

namespace ORB_SLAM3 {
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
}

void Tracking::ResetLocalMapBudgetHistory() {
  mLocalMapTemporal.clear();
  mpLocalBudgetMap=nullptr;
  mpLocalBudgetRelocation=nullptr;
}

void Tracking::FinishLocalMapDiagnostics() {
  std::lock_guard<std::mutex> lock(mLocalMapLogMutex);
  if(mLocalMapLog.is_open()) mLocalMapLog.close();
}

bool Tracking::TrackLocalMapBudgeted(Map* locked_map) {
  const auto identity_start=Clock::now();
  LocalTrackingIdentityGuard identity(LocalTrackingIdentityMutex());
  const double identity_wait_ms=elapsed(identity_start);
  // Capture all inputs before changing reference/workset/per-frame counters.
  LocalTrackingInput input{mCurrentFrame};
  input.map=locked_map; input.reference=mpReferenceKF;
  input.last_keyframe=mpLastKeyFrame; input.only_tracking=mbOnlyTracking;
  input.recently_lost=mState==RECENTLY_LOST;
  input.last_relocation=mnLastRelocFrameId; input.max_frames=mMaxFrames;
  input.far_points=mpLocalMapper->mbFarPoints; input.far_distance=mpLocalMapper->mThFarPoints;
  input.record_projections=mRgbdOptimizationLevel<4 || mpViewer;
  input.search_threshold=mCurrentFrame.mnId<mnLastRelocFrameId+2 ? 5.f : 3.f;
  if(mState==LOST || mState==RECENTLY_LOST) input.search_threshold=15.f;
  if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames)
    input.relocalization_keyframe=mpLocalBudgetRelocation;

  std::vector<KeyFrame*> temporal;
  if(mpLastKeyFrame) temporal.push_back(mpLastKeyFrame);
  if(mpLocalBudgetMap==input.map)
    for(auto* key:mLocalMapTemporal)
      if(key!=mpLastKeyFrame) temporal.push_back(key);
  temporal.erase(std::remove_if(temporal.begin(),temporal.end(),[&](KeyFrame* key) {
    return !key || key->isBad() || key->GetMap()!=input.map;
  }),temporal.end());
  // RGB-D does not maintain the inertial mPrevKF chain. Recover temporal
  // predecessors once after loading/reset if the small Tracking history is
  // insufficient; normal frames use at most four retained KF pointers.
  if(temporal.size()<4 && mpLastKeyFrame && input.map->KeyFramesInMap()>=4) {
    auto keys=input.map->GetAllKeyFrames();
    std::sort(keys.begin(),keys.end(),[](KeyFrame* a,KeyFrame* b) {
      if(a->mnFrameId!=b->mnFrameId) return a->mnFrameId>b->mnFrameId;
      return a->mnId>b->mnId;
    });
    for(auto* key:keys) {
      if(temporal.size()>=4) break;
      if(!key->isBad() && key->GetMap()==input.map && key->mnFrameId<=mpLastKeyFrame->mnFrameId &&
          std::find(temporal.begin(),temporal.end(),key)==temporal.end()) temporal.push_back(key);
    }
  }
  if(temporal.size()>4) temporal.resize(4);
  const bool shadow=mLocalMapMode==3;
  // Merge switches Atlas under both maps' update locks. The caller may have
  // acquired the old map's lock after that switch; never commit it as active.
  auto commit=[&](LocalTrackingAttempt& attempt) {
    if(mpAtlas->GetCurrentMap()!=locked_map) {
      attempt.rejection="active_map_changed"; return false;
    }
    return attempt.Commit(mCurrentFrame);
  };
  const bool protected_frame=mState!=OK || mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames ||
                              mCurrentFrame.mnId<mnLastInitFrameId+20;
  bool budget_attempt=mLocalMapMode>=2 && (!protected_frame || shadow);
  LocalKeyFrameSelection selection;
  std::string reason=protected_frame?"protection":"full";
  auto start=Clock::now();
  std::unique_ptr<LocalTrackingAttempt> trial(new LocalTrackingAttempt(input));
  trial->BuildLocalMap();
  if(budget_attempt) {
    selection=SelectLocalKeyFrames(*trial,mLocalMapKeyFrameBudget,temporal);
    budget_attempt=selection.bounded;
    reason=selection.reason;
  }
  trial->SearchAndOptimize();
  const double trial_ms=elapsed(start);
  const int budget_inliers=budget_attempt?trial->inliers:-1;
  const size_t trial_keyframes=trial->keyframes.size(), trial_points=trial->points.size();
  bool retry=budget_attempt && (!trial->TrackingSucceeded() || shadow);
  bool committed=false;
  if(!retry) {
    committed=commit(*trial);
    if(!committed) { reason=trial->rejection; retry=budget_attempt; }
  }
  std::unique_ptr<LocalTrackingAttempt> full;
  LocalTrackingAttempt* final=trial.get();
  double full_ms=0;
  size_t retained=0,total=0,retained_new=0,total_new=0,association=0;
  if(retry) {
    start=Clock::now();
    full.reset(new LocalTrackingAttempt(input));
    full->BuildLocalMap(); full->SearchAndOptimize(); full_ms=elapsed(start);
    final=full.get();
    if(shadow) {
      // Trial membership was fixed before the full match/optimization. Count
      // unique retained MP identities separately from actual feature matches.
      std::set<MapPoint*> selected,existing,seen;
      for(const auto& point:trial->points) selected.insert(point.get());
      for(const auto& point:input.frame.mvpMapPoints)
        if(point) { selected.insert(point.get()); existing.insert(point.get()); }
      for(int i=0;i<full->frame.N;++i) {
        const auto& point=full->frame.mvpMapPoints[i];
        if(!point || full->frame.mvbOutlier[i]) continue;
        if(trial->frame.mvpMapPoints[i]==point && !trial->frame.mvbOutlier[i]) ++association;
        if(!seen.insert(point.get()).second) continue;
        ++total; if(selected.count(point.get())) ++retained;
        if(!existing.count(point.get())) { ++total_new; if(selected.count(point.get())) ++retained_new; }
      }
    }
    committed=commit(*full);
    if(!committed) reason=full->rejection;
    else if(!shadow) reason="full_retry";
  }
  ++mTrackedFr;
  mLocalMapTemporal=std::move(temporal); mpLocalBudgetMap=input.map;
  if(committed) {
    // Preserve the legacy reference-point content: publish the PREVIOUS local
    // point vector. Speculative attempts never publish to Atlas or Mapping.
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
    mpReferenceKF=final->reference;
    mvpLocalKeyFrames=std::move(final->keyframes);
    mvpLocalMapPoints=std::move(final->points);
    mLocalPointIdentities.clear();
    mnMatchesInliers=final->inliers;
  } else {
    // No failed attempt was committed. Do not publish stale point constraints.
    std::fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),MapPointPtr());
    mnMatchesInliers=0;
  }
  mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
  const bool success=committed && final->TrackingSucceeded();
  const double update_ms=trial->build_ms+(full?full->build_ms:0)+selection.milliseconds;
  const double search_ms=trial->search_ms+(full?full->search_ms:0);
  const double pose_ms=trial->optimize_ms+(full?full->optimize_ms:0);
  const double snapshot_ms=trial->snapshot_ms+(full?full->snapshot_ms:0);
  const double commit_ms=trial->commit_ms+(full?full->commit_ms:0);
  if(mpCurrentRgbdTiming) {
    auto& timing=*mpCurrentRgbdTiming;
    timing.local_update_ms+=update_ms; timing.local_search_ms+=search_ms; timing.local_pose_ms+=pose_ms;
    timing.local_keyframes_ms+=trial->keyframes_ms+(full?full->keyframes_ms:0)+selection.milliseconds;
    timing.local_points_ms+=trial->gather_ms+(full?full->gather_ms:0);
    timing.local_keyframes=mvpLocalKeyFrames.size(); timing.local_points=mvpLocalMapPoints.size();
  }
  {
  std::lock_guard<std::mutex> log_lock(mLocalMapLogMutex);
  if(mLocalMapLog.is_open())
    mLocalMapLog<<mCurrentFrame.mnId<<','<<input.map->GetId()<<','<<mLocalMapMode<<','
      <<mLocalMapKeyFrameBudget<<','<<mvpLocalKeyFrames.size()<<','<<mvpLocalMapPoints.size()<<','
      <<selection.mandatory<<','<<selection.candidates<<','<<budget_attempt<<','<<budget_inliers<<','
      <<mnMatchesInliers<<','<<success<<','<<(retry&&!shadow)<<','<<committed<<','<<reason<<','
      <<selection.milliseconds<<','<<snapshot_ms<<','<<update_ms<<','<<search_ms<<','<<pose_ms<<','
      <<commit_ms<<','<<trial_ms<<','<<full_ms<<','<<retained<<','<<total<<','<<retained_new<<','
      <<total_new<<','<<association<<','<<trial_keyframes<<','<<trial_points<<','<<identity_wait_ms<<'\n';
  }
  if(FieldEnabled()) {
    FieldEmit(13,mCurrentFrame.mnId,3,1,mnMatchesInliers);
    if(committed) {
      FieldEmit(14,mCurrentFrame.mnId,0,1,final->pose_inliers);
      FieldEmit(23,mCurrentFrame.mnId,0,mpReferenceKF?31:30,mpReferenceKF?mpReferenceKF->mnId:0,
                mvpLocalKeyFrames.size(),mvpLocalMapPoints.size(),final->visible_candidates,final->associated_slots);
      FieldEmit(24,mCurrentFrame.mnId,1,3,final->new_matches,static_cast<int>(input.search_threshold));
      FieldEmit(25,mCurrentFrame.mnId,0,3,final->pre_associations,final->pre_outliers);
      uint64_t quadrants[4]={}; int count=0,outliers=0;
      for(int i=0;i<mCurrentFrame.N;++i) if(mCurrentFrame.mvpMapPoints[i]) {
        ++count; if(mCurrentFrame.mvbOutlier[i]) { ++outliers; continue; }
        const auto& p=mCurrentFrame.mvKeysUn[i].pt;
        ++quadrants[(p.x>=(mCurrentFrame.mnMinX+mCurrentFrame.mnMaxX)/2?1:0)+
                    (p.y>=(mCurrentFrame.mnMinY+mCurrentFrame.mnMaxY)/2?2:0)];
      }
      FieldEmit(25,mCurrentFrame.mnId,1,3,count,outliers);
      FieldEmit(21,mCurrentFrame.mnId,0,15,quadrants[0],quadrants[1],quadrants[2],quadrants[3]);
    }
  }
  return success;
}
} // namespace ORB_SLAM3
