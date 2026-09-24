#include "LocalMapBudget.h"
#include "KeyFrame.h"
#include "LocalMapGrid.h"
#include <array>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <set>
#include <limits>

namespace ORB_SLAM3 {
LocalKeyFrameSelection SelectLocalKeyFrames(LocalTrackingAttempt& attempt,
    size_t limit, const std::vector<KeyFrame*>& temporal, LocalKeyFrameStrategy strategy) {
  const auto start = std::chrono::steady_clock::now();
  LocalKeyFrameSelection result;
  result.original = attempt.keyframes.size();
  auto finish = [&] {
    result.milliseconds = std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-start).count();
    return result;
  };
  auto valid = [&](KeyFrame* key) {
    return key && !key->isBad() && key->GetMap() == attempt.input.map;
  };
  std::vector<KeyFrame*> candidates;
  std::set<KeyFrame*> visited, mandatory;
  for (auto* key : attempt.keyframes)
    if (valid(key) && visited.insert(key).second) candidates.push_back(key);
  auto protect = [&](KeyFrame* key) {
    if (!valid(key)) return;
    mandatory.insert(key);
    if (visited.insert(key).second) candidates.push_back(key);
  };
  protect(attempt.input.reference);
  protect(attempt.vote_winner);
  protect(attempt.input.last_keyframe);
  for (size_t i=0;i<std::min<size_t>(4,temporal.size());++i) protect(temporal[i]);
  protect(attempt.input.relocalization_keyframe);
  result.mandatory = mandatory.size();
  result.candidates = candidates.size();
  if (!limit || mandatory.size()>limit) {
    result.reason="mandatory_overflow"; return finish();
  }
  std::vector<KeyFrame*> ranked;
  for (auto* key : candidates) if (!mandatory.count(key)) ranked.push_back(key);
  auto vote = [&](KeyFrame* key) {
    const auto it=attempt.votes.find(key); return it==attempt.votes.end()?0:it->second;
  };
  auto age = [&](KeyFrame* key) {
    return key->mnFrameId>attempt.frame.mnId ? key->mnFrameId-attempt.frame.mnId
                                          : attempt.frame.mnId-key->mnFrameId;
  };
  std::sort(ranked.begin(),ranked.end(),[&](KeyFrame* a,KeyFrame* b) {
    if(vote(a)!=vote(b)) return vote(a)>vote(b);
    if(age(a)!=age(b)) return age(a)<age(b);
    return a->mnId<b->mnId;
  });
  if(ranked.size()>4*limit) ranked.resize(4*limit);
  std::set<KeyFrame*> selected=mandatory;
  if(strategy==LocalKeyFrameStrategy::Coverage && selected.size()<limit) {
    // Only the already-associated support points provide coverage proxies.
    // No candidate KF point-slot scan; descriptor matching order stays separate.
    std::vector<KeyFrame*> pool;
    for(auto* key:candidates) if(mandatory.count(key)) pool.push_back(key);
    pool.insert(pool.end(),ranked.begin(),ranked.end());
    std::unordered_map<KeyFrame*,size_t> index;
    for(size_t i=0;i<pool.size();++i) index.emplace(pool[i],i);
    struct Support { MapPointPtr point; int cell; };
    std::vector<Support> support;
    std::unordered_set<MapPoint*> seen;
    std::array<uint64_t,6> all_cells{};
    std::vector<float> depths;
    for(int i=0;i<attempt.frame.N;++i) {
      const auto& point=attempt.frame.mvpMapPoints[i];
      if(!point || !seen.insert(point.get()).second) continue;
      const auto& snapshot=attempt.ExistingPoint(point.get()).snapshot;
      if(!snapshot.valid || snapshot.map!=attempt.input.map) continue;
      const auto& uv=attempt.frame.mvKeysUn[i].pt;
      const int cell=LocalMapGridCell(uv.x,uv.y,attempt.frame.mnMinX,attempt.frame.mnMaxX,
                                     attempt.frame.mnMinY,attempt.frame.mnMaxY,24,16);
      support.push_back({point,cell});
      if(cell>=0) all_cells[cell/64]|=uint64_t(1)<<(cell%64);
      const float z=(attempt.frame.GetPose()*snapshot.geometry.position).z();
      if(std::isfinite(z) && z>0) depths.push_back(z);
    }
    result.support_points=support.size();
    for(auto bits:all_cells) result.support_cells+=__builtin_popcountll(bits);
    if(!support.empty()) {
      const size_t words=(support.size()+63)/64;
      std::vector<uint64_t> point_bits(pool.size()*words,0),covered(words,0);
      std::vector<std::array<uint64_t,6>> cell_bits(pool.size());
      std::array<uint64_t,6> covered_cells{};
      for(size_t j=0;j<support.size();++j) {
        const auto observations=support[j].point->GetObservations();
        for(const auto& observation:observations) {
          const auto found=index.find(observation.first);
          if(found==index.end()) continue;
          const size_t k=found->second;
          point_bits[k*words+j/64]|=uint64_t(1)<<(j%64);
          const int cell=support[j].cell;
          if(cell>=0) cell_bits[k][cell/64]|=uint64_t(1)<<(cell%64);
        }
      }
      float scale=attempt.frame.mThDepth;
      if(!depths.empty()) {
        std::nth_element(depths.begin(),depths.begin()+depths.size()/2,depths.end());
        scale=depths[depths.size()/2];
      }
      if(!std::isfinite(scale) || scale<=0) scale=1.f;
      std::vector<Eigen::Vector3f> centers;
      std::vector<Eigen::Matrix3f> rotations;
      centers.reserve(pool.size()); rotations.reserve(pool.size());
      for(auto* key:pool) {
        const auto pose=key->GetPose();
        centers.push_back(pose.inverse().translation()); rotations.push_back(pose.rotationMatrix());
      }
      std::vector<double> diversity(pool.size(),1.);
      auto add=[&](size_t chosen) {
        for(size_t w=0;w<words;++w) covered[w]|=point_bits[chosen*words+w];
        for(size_t w=0;w<6;++w) covered_cells[w]|=cell_bits[chosen][w];
        for(size_t k=0;k<pool.size();++k) {
          const double translation=std::min(1.,double((centers[k]-centers[chosen]).norm()/scale));
          const double cosine=std::max(-1.,std::min(1.,double(((rotations[k]*rotations[chosen].transpose()).trace()-1)*.5f)));
          const double angle=std::acos(cosine)/3.14159265358979323846;
          diversity[k]=std::min(diversity[k],.5*translation+.5*angle);
        }
      };
      for(auto* key:mandatory) add(index.at(key));
      auto totals=[&]() {
        size_t points=0,cells=0;
        for(auto bits:covered) points+=__builtin_popcountll(bits);
        for(auto bits:covered_cells) cells+=__builtin_popcountll(bits);
        return std::make_pair(points,cells);
      };
      auto initial=totals();
      result.mandatory_support=initial.first;result.mandatory_cells=initial.second;
      int maximum_vote=0;
      for(auto* key:pool) maximum_vote=std::max(maximum_vote,vote(key));
      while(selected.size()<limit && selected.size()<pool.size()) {
        const auto coverage=totals();
        bool saturated=coverage.first==support.size() && coverage.second==result.support_cells;
        size_t best=pool.size(),vote_best=pool.size();
        double best_score=-1.;
        bool any_gain=false;
        for(auto* key:ranked) {
          if(selected.count(key)) continue;
          const size_t k=index.at(key);
          if(vote_best==pool.size()) vote_best=k;
          size_t new_points=0,new_cells=0;
          for(size_t w=0;w<words;++w)
            new_points+=__builtin_popcountll(point_bits[k*words+w]&~covered[w]);
          for(size_t w=0;w<6;++w)
            new_cells+=__builtin_popcountll(cell_bits[k][w]&~covered_cells[w]);
          any_gain=any_gain || new_points || new_cells;
          const double score=.50*new_points/std::max<size_t>(1,support.size())+
            .30*new_cells/std::max<size_t>(1,result.support_cells)+
            .15*vote(key)/std::max(1,maximum_vote)+.05*diversity[k];
          // ranked order supplies deterministic vote/age/ID ties.
          if(score>best_score) {best=k;best_score=score;}
        }
        if(best==pool.size()) break;
        saturated=saturated || !any_gain;
        if(saturated && result.saturation_selected<0) result.saturation_selected=selected.size();
        if(saturated) {
          ++result.saturation_slots;
          if(best!=vote_best) ++result.saturation_diversity;
          else {
            bool same_vote=false,diversity_separates=false;
            for(auto* key:ranked) {
              if(selected.count(key) || key==pool[best] || vote(key)!=vote(pool[best])) continue;
              same_vote=true;
              if(diversity[index.at(key)]<diversity[best]) diversity_separates=true;
            }
            if(!same_vote) ++result.saturation_votes;
            else if(diversity_separates) ++result.saturation_diversity;
            else ++result.saturation_ties;
          }
        }
        selected.insert(pool[best]); add(best);
      }
      result.coverage_used=true;
    }
  }
  for(auto* key:ranked) {
    if(selected.size()==limit) break;
    selected.insert(key);
  }
  std::vector<KeyFrame*> ordered;
  ordered.reserve(selected.size());
  for(auto* key:candidates) if(selected.count(key)) ordered.push_back(key);
  attempt.keyframes.swap(ordered);
  result.bounded=true; result.selected=attempt.keyframes.size(); result.reason=result.coverage_used?"coverage":"votes";
  return finish();
}
} // namespace ORB_SLAM3
