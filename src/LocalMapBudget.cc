#include "LocalMapBudget.h"
#include "KeyFrame.h"
#include <algorithm>
#include <chrono>
#include <set>
#include <limits>

namespace ORB_SLAM3 {
LocalKeyFrameSelection SelectLocalKeyFrames(LocalTrackingAttempt& attempt,
    size_t limit, const std::vector<KeyFrame*>& temporal) {
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
  for(auto* key:ranked) {
    if(selected.size()==limit) break;
    selected.insert(key);
  }
  std::vector<KeyFrame*> ordered;
  ordered.reserve(selected.size());
  for(auto* key:candidates) if(selected.count(key)) ordered.push_back(key);
  attempt.keyframes.swap(ordered);
  result.bounded=true; result.selected=attempt.keyframes.size(); result.reason="votes";
  return finish();
}
} // namespace ORB_SLAM3
