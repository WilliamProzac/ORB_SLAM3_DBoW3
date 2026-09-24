#include "LocalMapPointBudget.h"
#include "LocalTrackingAttempt.h"
#include "LocalMapGrid.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_set>
namespace ORB_SLAM3 {
LocalPointSelection SelectLocalMapPoints(LocalTrackingAttempt& attempt,
    std::vector<LocalProjectedPoint>& candidates,size_t limit,bool reserve_young) {
  using Clock=std::chrono::steady_clock;
  const auto start=Clock::now();
  LocalPointSelection result;result.eligible=candidates.size();result.selected=candidates.size();
  auto finish=[&] {result.milliseconds=std::chrono::duration<double,std::milli>(Clock::now()-start).count();return result;};
  result.protected_points=attempt.associated_points;
  // No observation scoring, grid allocation or candidate reordering when the
  // requested cap cannot bind. Search budget counts post-frustum candidates.
  if(!limit || candidates.size()<=limit) return finish();
  const auto& frame=attempt.frame;
  std::array<size_t,384> occupied{};
  std::array<std::array<size_t,3>,384> layer_occupied{};
  std::unordered_set<MapPoint*> protected_seen;
  const float threshold=frame.mThDepth;
  if(!std::isfinite(threshold) || threshold<=0) {result.valid=false;return finish();}
  auto depth_layer=[&](float z) {return z<threshold?0:(z<3*threshold?1:2);};
  for(int i=0;i<frame.N;++i) {
    const auto& p=frame.mvpMapPoints[i];
    if(!p || !protected_seen.insert(p.get()).second) continue;

    const auto& uv=frame.mvKeysUn[i].pt;
    const int cell=LocalMapGridCell(uv.x,uv.y,frame.mnMinX,frame.mnMaxX,frame.mnMinY,frame.mnMaxY,24,16);
    if(cell<0) continue;
    ++occupied[cell];
    const auto& snap=attempt.ExistingPoint(p.get()).snapshot;
    const float z=(frame.GetPose()*snap.geometry.position).z();
    if(std::isfinite(z) && z>0) ++layer_occupied[cell][depth_layer(z)];
  }
  // Avoid thousands of observation snapshots when the cap cannot remove any
  // candidate. An unchanged candidate list needs no ranking or sorting.
  struct Ranked {size_t index;double score;bool young;};
  std::array<std::vector<Ranked>,1152> buckets;
  for(size_t i=0;i<candidates.size();++i) {
    const auto& c=candidates[i];const auto& s=*c.snapshot;
    const int cell=LocalMapGridCell(c.projection.x,c.projection.y,frame.mnMinX,frame.mnMaxX,
                                   frame.mnMinY,frame.mnMaxY,24,16);
    if(cell<0 || !std::isfinite(c.projection.z) || c.projection.z<=0) {
      result.valid=false;return finish(); // caller retries complete, no partial truncation
    }
    const auto observed_start=Clock::now();
    size_t observed=0;
    const auto observations=s.owner->GetObservations();
    for(const auto& entry:observations) {
      auto* key=entry.first;
      if(key && (std::get<0>(entry.second)>=0 || std::get<1>(entry.second)>=0) &&
         !key->isBad() && key->GetMap()==attempt.input.map && ++observed==8) break;
    }
    result.observation_ms+=std::chrono::duration<double,std::milli>(Clock::now()-observed_start).count();
    const double ratio=std::max(0.,std::min(1.,(double(s.found)+1)/(double(s.visible)+2)));
    const uint64_t age=frame.mnId>=s.association_frame?frame.mnId-s.association_frame:0;
    const double recent=s.association_known?std::exp(-double(age)/30.):0.;
    const double view=std::max(0.,std::min(1.,(c.projection.view_cos-.5)/.5));
    const double score=.50*ratio+.20*std::min<size_t>(observed,8)/8.+.20*recent+.10*view;
    const bool young=observed<3 && frame.mnId>=s.owner->mnFirstFrame &&
                     frame.mnId-s.owner->mnFirstFrame<=30;
    buckets[cell*3+depth_layer(c.projection.z)].push_back({i,score,young});
  }
  for(auto& bucket:buckets)
    std::sort(bucket.begin(),bucket.end(),[](const Ranked& a,const Ranked& b) {
      return a.score!=b.score?a.score>b.score:a.index<b.index;
    });
  std::vector<bool> selected(candidates.size(),false);
  size_t count=0;
  auto take=[&](const Ranked& point,size_t bucket) {
    selected[point.index]=true;++count;++occupied[bucket/3];++layer_occupied[bucket/3][bucket%3];
  };
  // Same fair grid/depth allocator for the optional young reservation and the
  // general pool. A grid with fewer protected/selected points is served first;
  // then its least-covered nonempty depth layer. Empty quotas are recycled.
  auto allocate=[&](size_t target,bool young_only) {
    std::array<size_t,1152> cursor{};
    struct Cell {size_t occupancy,cell;};
    struct Greater {bool operator()(const Cell& a,const Cell& b) const {
      return a.occupancy!=b.occupancy?a.occupancy>b.occupancy:a.cell>b.cell;}};
    std::priority_queue<Cell,std::vector<Cell>,Greater> cells;
    for(size_t cell=0;cell<384;++cell) cells.push({occupied[cell],cell});
    while(count<target && !cells.empty()) {
      const size_t cell=cells.top().cell;cells.pop();
      size_t chosen=1152;
      for(size_t depth=0;depth<3;++depth) {
        const size_t bucket=cell*3+depth;const auto& points=buckets[bucket];
        auto& at=cursor[bucket];
        while(at<points.size() && (selected[points[at].index] || (young_only&&!points[at].young))) ++at;
        if(at==points.size()) continue;
        if(chosen==1152 || layer_occupied[cell][depth]<layer_occupied[cell][chosen%3] ||
           (layer_occupied[cell][depth]==layer_occupied[cell][chosen%3] &&
            points[at].score>buckets[chosen][cursor[chosen]].score)) chosen=bucket;
      }
      if(chosen==1152) continue;
      take(buckets[chosen][cursor[chosen]++],chosen);
      cells.push({occupied[cell],cell});
    }
  };
  if(reserve_young) {allocate(limit/10,true);result.young_reserved=count;}
  allocate(limit,false);
  std::vector<LocalProjectedPoint> retained;retained.reserve(count);
  for(size_t i=0;i<candidates.size();++i) if(selected[i]) retained.push_back(candidates[i]);
  candidates.swap(retained);result.selected=candidates.size();result.truncated=true;
  return finish();
}
}
