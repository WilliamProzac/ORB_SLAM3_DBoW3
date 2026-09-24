#pragma once
#include <algorithm>
#include <cstdint>
#include <deque>

namespace ORB_SLAM3 {
struct LocalMapHealth {
  int inliers=0, occupied=0, feature_cells=0;
  bool tracking=false;
  double Coverage() const { return feature_cells ? double(occupied)/feature_cells : 0.; }
  bool Stable() const { return tracking && inliers>=100 && Coverage()>=.45 && occupied>=12; }
  bool Low(double reference) const {
    return !tracking || inliers<60 || (reference>0 && inliers<.65*reference) ||
           Coverage()<.30 || occupied<8;
  }
  bool Severe() const { return Coverage()<.15 || occupied<4; }
};
struct LocalBudgetDecision {
  int level=3; // 0=base, 1=1.25, 2=1.5, 3=full
  bool probe=false;
  bool Bounded() const { return level<3; }
  double Scale() const { return level==0?1.:(level==1?1.25:1.5); }
};

// Pure, deterministic controller. Ordinary BA versions are deliberately absent.
// Begin/Observe are called once per input frame, never once per retry.
class LocalMapController {
 public:
  void Reset() { *this=LocalMapController(); }
  LocalBudgetDecision Begin(uint64_t frame,uint64_t generation,uint64_t event_version,
      bool recently_lost,uint64_t relocation,uint64_t legacy_protection_end) {
    if(initialized_ && frame<=last_frame_) Reset();
    if(initialized_ && frame!=last_frame_+1)
      full_stable_=bounded_stable_=probe_stable_=0;
    const bool new_map=!initialized_ || generation!=generation_;
    const bool event=initialized_ && event_version!=event_version_;
    const bool relocated=initialized_ && relocation!=relocation_;
    const bool entered_lost=recently_lost && !recently_lost_;
    if(new_map) { Reset(); initialized_=true; generation_=generation; }
    if(new_map || event || relocated || entered_lost) {
      Recover(frame); safety_until_=std::max(safety_until_,frame+20);
    }
    if(event || relocated) healthy_reference_=0;
    last_frame_=frame;
    event_version_=event_version; relocation_=relocation; recently_lost_=recently_lost;
    // Existing relocalization safety window may exceed our minimum 20 frames.
    hold_until_=std::max(hold_until_,legacy_protection_end);
    safety_until_=std::max(safety_until_,legacy_protection_end);
    if(recently_lost || frame<hold_until_) return {3,false};
    if(probe_level_>=0) return {probe_level_,true};
    if(frame>=cool_until_) {
      if(level_==3 && full_stable_>=10) StartProbe(2);
      else if(level_>0 && level_<3 && bounded_stable_>=15) StartProbe(level_-1);
    }
    return probe_level_>=0 ? LocalBudgetDecision{probe_level_,true} : LocalBudgetDecision{level_,false};
  }
  void Observe(uint64_t frame,const LocalBudgetDecision& decision,
      bool had_budget,const LocalMapHealth& trial,const LocalMapHealth& final,bool retried) {
    if(!had_budget) {
      full_stable_=final.Stable()?full_stable_+1:0;
      return;
    }
    full_stable_=0;
    retries_.push_back(retried); if(retries_.size()>10) retries_.pop_front();
    const bool low=trial.Low(healthy_reference_);
    const bool failed=retried || !trial.tracking;
    if(decision.probe) {
      if(failed || !trial.Stable() || low) {
        const int previous=level_;
        probe_level_=-1; probe_stable_=0; bounded_stable_=0;
        if(previous==3) Recover(frame);
        cool_until_=std::max(cool_until_,frame+20);
        return;
      }
      if(++probe_stable_>=5) {
        level_=probe_level_; probe_level_=-1; probe_stable_=0; bounded_stable_=0;
      }
    } else if(low || failed) {
      bounded_stable_=0;
      if(decision.level==2 || std::count(retries_.begin(),retries_.end(),true)>=3) Recover(frame);
      else level_=std::min(2,decision.level+1);
      if(failed) cool_until_=std::max(cool_until_,frame+15);
    } else {
      bounded_stable_=trial.Stable()?bounded_stable_+1:0;
    }
    // Successful FULL retry never updates budget-health history.
    if(!failed && !low && trial.Stable())
      healthy_reference_=healthy_reference_==0?trial.inliers:.9*healthy_reference_+.1*trial.inliers;
    if(std::count(retries_.begin(),retries_.end(),true)>=3 && level_!=3) Recover(frame);
  }
  bool ShouldRetry(const LocalBudgetDecision& decision,const LocalMapHealth& trial) const {
    return decision.Bounded() && (!trial.tracking || trial.Severe() ||
      (decision.probe && (!trial.Stable() || trial.Low(healthy_reference_))));
  }
  int Level() const { return level_; }
  uint64_t HoldUntil() const { return hold_until_; }
  uint64_t CoolUntil() const { return cool_until_; }
  uint64_t SafetyUntil() const { return safety_until_; }
  double HealthyReference() const { return healthy_reference_; }
 private:
  void Recover(uint64_t frame) {
    level_=3; probe_level_=-1; probe_stable_=bounded_stable_=full_stable_=0;
    // A new protected interval is a fresh budget-health epoch. Keeping an
    // old stationary-view peak forever would reject healthy lower-texture
    // probes indefinitely. Full retries never update this reference; after
    // protection, five actual bounded successes must establish it again.
    healthy_reference_=0;
    hold_until_=std::max(hold_until_,frame+20);
    cool_until_=std::max(cool_until_,frame+20);
    retries_.clear();
  }
  void StartProbe(int level) { probe_level_=level; probe_stable_=0; }
  bool initialized_=false,recently_lost_=false;
  int level_=3,probe_level_=-1,probe_stable_=0,bounded_stable_=0,full_stable_=0;
  uint64_t last_frame_=0,generation_=0,event_version_=0,relocation_=0,hold_until_=0,cool_until_=0,safety_until_=0;
  double healthy_reference_=0;
  std::deque<bool> retries_;
};
} // namespace ORB_SLAM3
