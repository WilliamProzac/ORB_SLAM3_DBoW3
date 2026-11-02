#include "StabilityDiagnostics.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace ORB_SLAM3 { namespace Stability {
namespace {
constexpr std::size_t FrameLimit=20000, PointLimit=262144, LifeLimit=100000;
struct Source { std::uint64_t ordinal=0,stamp=0; std::uint32_t rgb=0,depth=0; };
struct Row {
 Source source; InputIdentity input; int before=-1,after=-1,current_valid=0;
 int depths=0,close=0,far=0; FrameCounts counts;
};
struct PointRow {
 Source source; std::uint64_t frame=0,point=0; int stage=0,kind=0,index=-1,outlier=-1;
 float u=0,v=0,x=0,y=0,z=0,depth=0;
};
struct LifeRow {
 std::uint64_t serial=0,generation=0,map=0,point=0;
 int kind=0; std::int64_t key=-1,value=-1,extra=-1,found=-1,visible=-1;
};
std::once_flag configured;
bool enabled=false,eager=false,accepting=false,stopped=false,flush_ok=false;
std::string directory;
std::mutex mutex;
std::vector<Row> frames;
std::vector<PointRow> points;
std::vector<LifeRow> life;
struct Window {std::uint64_t first,last,attempted=0,stored=0;};
std::vector<Window> windows;
std::array<std::uint64_t,13> life_totals{};
std::uint64_t frame_drops=0,point_drops=0,life_drops=0,serial=0,closed_appends=0;
thread_local Source source;
thread_local Row current;
thread_local bool active=false,window=false;
thread_local int window_index=-1;
thread_local Stage stage=None;
void AddPoint(const PointRow& row) {
 std::lock_guard<std::mutex> lock(mutex);
 if(!accepting){++closed_appends;return;}
 if(window_index>=0)++windows[window_index].attempted;
 if(points.size()==PointLimit){++point_drops;return;}
 if(window_index>=0)++windows[window_index].stored;
 points.push_back(row);
}
template<class T> void Cell(std::ostream& stream,T value) {
 if(value<0)stream<<"NA";else stream<<value;
}
}
void Configure() {
 std::call_once(configured,[]{
  const char* b=std::getenv("ORB_RGBD_EAGER_BOW");
  if(b && std::string(b)!="0" && std::string(b)!="1")
    throw std::runtime_error("ORB_RGBD_EAGER_BOW must be 0 or 1");
  eager=b && std::string(b)=="1";
  const char* d=std::getenv("ORB_RGBD_STABILITY_DIR");
  directory=d?d:""; enabled=!directory.empty();
  if(!enabled)return;
  const char* w=std::getenv("ORB_RGBD_STABILITY_WINDOWS");
  if(w && *w) {
   std::istringstream list(w);std::string range;
   while(std::getline(list,range,',')) {
    std::istringstream input(range);std::uint64_t first=0,last=0;char dash=0,extra=0;
    if(!(input>>first>>dash>>last)||dash!='-'||!first||last<first||(input>>extra))
      throw std::runtime_error("invalid ORB_RGBD_STABILITY_WINDOWS");
    if(windows.size()==64)throw std::runtime_error("too many stability windows");
    windows.push_back(Window{first,last,0,0});
   }
  }
  // Fail at startup instead of silently discarding a requested observation.
  std::ofstream probe(directory+"/stability_summary.json",std::ios::trunc);
  if(!probe)throw std::runtime_error("stability output directory is not writable");
  probe<<"{\"schema\":\"rgbd_stability_v1\",\"complete\":false}\n";
  probe.close();if(!probe)throw std::runtime_error("stability startup write failed");
  frames.reserve(FrameLimit);points.reserve(PointLimit);life.reserve(LifeLimit);
  accepting=true;
 });
}
bool Enabled(){return enabled;}
bool EagerBoW(){return eager;}
void PrepareKeyFrame(KeyFrame* key,bool rgbd) {
 if(!rgbd || !eager)return;
 const auto start=enabled?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point();
 key->ComputeBoW(); // exclusively owned: no reference/Atlas/queue publication yet
 if(active){++current.counts.bow_prepares;
  current.counts.bow_prepare_ms+=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-start).count();}
}
InputIdentity CaptureInput(const Frame& f){return {f.mnId,f.N};}
void SetSource(std::uint64_t ordinal,std::uint64_t stamp,std::uint32_t rgb,std::uint32_t depth) {
 if(enabled)source={ordinal,stamp,rgb,depth};
}
void ClearSource(){if(enabled)source=Source();}
void BeginFrame(const Frame& f,int before,float close_depth) {
 if(!enabled)return;
 current=Row();current.source=source;current.input=CaptureInput(f);current.before=before;
 active=true;stage=None;window=false;window_index=-1;
 for(std::size_t i=0;i<windows.size();++i)
  if(source.ordinal>=windows[i].first && source.ordinal<=windows[i].last){
    window=true;window_index=int(i);break;
  }
 for(float depth:f.mvDepth)if(std::isfinite(depth)&&depth>0){
  ++current.depths;if(depth<close_depth)++current.close;else ++current.far;
 }
}
void EndFrame(const Frame& f,int after) {
 if(!active)return;
 current.after=after;
 current.current_valid=f.mnId==current.input.frame_id && f.N==current.input.keypoints;
 std::lock_guard<std::mutex> lock(mutex);
 if(!accepting)++closed_appends;
 else if(frames.size()==FrameLimit)++frame_drops;
 else frames.push_back(current);
 active=false;window=false;stage=None;
}
StageScope::StageScope(Stage next):previous_(stage){if(active)stage=next;}
StageScope::~StageScope(){stage=previous_;}
FrameCounts* Counts(){return active?&current.counts:nullptr;}
void PoseSnapshot(Frame& f,bool after) {
 if(!active)return;
 int edges=0,outliers=0;
 for(int i=0;i<f.N;++i) {
  const auto& point=f.mvpMapPoints[i];if(!point)continue;
  ++edges;if(f.mvbOutlier[i])++outliers;
  if(!window || !f.HasPose() || !f.mpCamera || f.Nleft!=-1)continue;
  const Eigen::Vector3f pc=f.GetPose()*point->GetWorldPos();
  if(!pc.allFinite() || pc.z()<=0)continue;
  const Eigen::Vector2f uv=f.mpCamera->project(pc);
  const auto& xy=f.mvKeysUn[i].pt;
  if(!uv.allFinite())continue;
  PointRow row;row.source=current.source;row.frame=f.mnId;row.point=point->mnId;
  row.stage=stage;row.kind=after?3:2;row.index=i;row.outlier=after?int(f.mvbOutlier[i]):-1;
  row.u=uv.x();row.v=uv.y();row.x=xy.x;row.y=xy.y;
  row.z=pc.z();row.depth=f.mvDepth[i];AddPoint(row);
 }
 if(after){current.counts.pose_after=edges;current.counts.pose_outliers=outliers;}
 else{++current.counts.pose_calls;current.counts.pose_before=edges;}
}
void SearchProjection(std::uint64_t point,float u,float v) {
 if(!active || !window)return;
 PointRow row;row.source=current.source;row.frame=current.input.frame_id;
 row.point=point;row.stage=stage;row.kind=1;row.u=u;row.v=v;AddPoint(row);
}
void Life(Map* map,LifeKind kind,std::uint64_t point,std::int64_t key,
          std::int64_t value,std::int64_t extra,std::int64_t found,std::int64_t visible) {
 if(!enabled || !map)return;
 LifeRow row;row.generation=map->GetLocalTrackingEvent().generation;row.map=map->GetId();
 row.kind=kind;row.point=point;row.key=key;row.value=value;row.extra=extra;
 row.found=found;row.visible=visible;
 if(kind==Birth && active && current.counts.kf_new_points>=0)++current.counts.kf_new_points;
 std::lock_guard<std::mutex> lock(mutex);
 if(!accepting){++closed_appends;return;}
 row.serial=++serial;++life_totals[unsigned(kind)];
 if(life.size()==LifeLimit){++life_drops;return;}
 life.push_back(row);
}
bool StopAndFlush(bool joined) {
 if(!enabled)return true;
 std::lock_guard<std::mutex> lock(mutex);
 if(stopped)return flush_ok;
 accepting=false;stopped=true;
 std::ofstream f(directory+"/stability_frames.csv"),p(directory+"/stability_points.csv"),
               l(directory+"/stability_lifetime.csv"),s(directory+"/stability_summary.json");
 f<<"source_ordinal,source_stamp_ns,rgb_seq,depth_seq,frame_id,input_keypoints,state_before,state_after,current_frame_valid,finite_positive_depths,close_depths,far_depths,motion_first,motion_final,motion_map,reference_coarse,reference_map,local_points,search_seen,search_bad,search_projected,search_visible,search_matches,local_inliers,local_threshold,pose_calls,pose_last_before,pose_last_after,pose_last_outliers,kf_id,kf_depth_slots,kf_new_points,bow_prepares,kf_tracked_close,kf_untracked_close,kf_need_close,kf_ref_matches,kf_c1a,kf_c1b,kf_c1c,kf_c2,kf_mapper_idle,kf_queue,bow_prepare_ms\n";
 f<<std::setprecision(9);
 for(const auto& r:frames){const auto& c=r.counts;
  f<<r.source.ordinal<<','<<r.source.stamp<<','<<r.source.rgb<<','<<r.source.depth<<','
   <<r.input.frame_id<<','<<r.input.keypoints<<','<<r.before<<','<<r.after<<','<<r.current_valid
   <<','<<r.depths<<','<<r.close<<','<<r.far;
  const int values[]={c.motion_first,c.motion_final,c.motion_map,c.reference_coarse,c.reference_map,
   c.local_points,c.search_seen,c.search_bad,c.search_projected,c.search_visible,c.search_matches,
   c.local_inliers,c.local_threshold,c.pose_calls,c.pose_before,c.pose_after,c.pose_outliers,
   c.kf_id,c.kf_depth_slots,c.kf_new_points,c.bow_prepares,c.kf_tracked_close,c.kf_untracked_close,
   c.kf_need_close,c.kf_ref_matches,c.kf_c1a,c.kf_c1b,c.kf_c1c,c.kf_c2,c.kf_mapper_idle,c.kf_queue};
  for(int value:values){f<<',';Cell(f,value);}f<<','<<c.bow_prepare_ms<<'\n';
 }
 p<<"source_ordinal,source_stamp_ns,frame_id,stage,kind,point_id,keypoint_index,outlier,projected_u,projected_v,observed_u,observed_v,camera_z,input_depth\n";
 p<<std::setprecision(9);
 for(const auto& r:points){p<<r.source.ordinal<<','<<r.source.stamp<<','<<r.frame<<','<<r.stage<<','
  <<r.kind<<','<<r.point<<',';Cell(p,r.index);p<<',';Cell(p,r.outlier);
  p<<','<<r.u<<','<<r.v<<',';if(r.kind==1)p<<"NA,NA,NA,NA";
  else p<<r.x<<','<<r.y<<','<<r.z<<','<<r.depth;p<<'\n';}
 l<<"serial,map_generation,map_id,kind,point_id,keyframe_id,value,extra,found,visible\n";
 for(const auto& r:life){l<<r.serial<<','<<r.generation<<','<<r.map<<','<<r.kind<<','<<r.point;
  const std::int64_t values[]={r.key,r.value,r.extra,r.found,r.visible};
  for(auto value:values){l<<',';Cell(l,value);}l<<'\n';}
 f.close();p.close();l.close();flush_ok=bool(f)&&bool(p)&&bool(l);
 s<<"{\"schema\":\"rgbd_stability_v1\",\"complete\":"<<(flush_ok?"true":"false")
  <<",\"workers_joined\":"<<(joined?"true":"false")<<",\"eager_bow\":"<<(eager?"true":"false")
  <<",\"frames\":"<<frames.size()<<",\"points\":"<<points.size()<<",\"lifetime\":"<<life.size()
  <<",\"frame_capacity\":"<<FrameLimit<<",\"point_capacity\":"<<PointLimit<<",\"lifetime_capacity\":"<<LifeLimit
  <<",\"frame_drops\":"<<frame_drops<<",\"point_drops\":"<<point_drops<<",\"lifetime_drops\":"<<life_drops
  <<",\"closed_appends_at_flush\":"<<closed_appends<<",\"retained_capacity_bytes\":"
  <<(frames.capacity()*sizeof(Row)+points.capacity()*sizeof(PointRow)+life.capacity()*sizeof(LifeRow))
  <<",\"life_totals\":[";
 for(std::size_t i=0;i<life_totals.size();++i){if(i)s<<',';s<<life_totals[i];}
 s<<"],\"windows\":[";
 for(std::size_t i=0;i<windows.size();++i){const auto& w=windows[i];if(i)s<<',';
  s<<"{\"start\":"<<w.first<<",\"end\":"<<w.last<<",\"attempted\":"<<w.attempted
   <<",\"stored\":"<<w.stored<<",\"dropped\":"<<(w.attempted-w.stored)<<'}';}
 s<<"],\"pose_scope\":\"last pose call counts; window coordinates are native RGBD monocular-camera projections at each call; other sensors omitted\"}\n";
 s.close();flush_ok=flush_ok&&bool(s);return flush_ok;
}
}}
