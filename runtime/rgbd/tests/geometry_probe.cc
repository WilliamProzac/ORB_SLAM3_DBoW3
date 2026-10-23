// RGBD25: compile ONLY this fixture TU against each matching complete ARM DSO.
// No replacement definition of Frame::isInFrustum or MapPoint methods appears
// here. Static results are compared across the four separate executables.
// All system/Eigen/Boost headers precede project headers; no access macros.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <time.h>
#include "Frame.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"
#include "CameraModels/Pinhole.h"

using namespace ORB_SLAM3;
namespace {
using Clock=std::chrono::steady_clock;
void Require(bool value,const std::string& message) {
  if(!value)throw std::runtime_error(message);
}
std::uint32_t Bits(float value) {
  std::uint32_t result;std::memcpy(&result,&value,sizeof(result));return result;
}
std::int64_t Nanoseconds(clockid_t which) {
  timespec value{};Require(clock_gettime(which,&value)==0,"clock failure");
  return std::int64_t(value.tv_sec)*1000000000LL+value.tv_nsec;
}
struct Hash {
  std::uint64_t value=14695981039346656037ULL;
  void Add(const std::string& s) {for(unsigned char c:s){value^=c;value*=1099511628211ULL;}}
};

// Real constructor + explicit calibrated synthetic observations. No copying of
// partially initialized Frame, raw storage construction or const-field writes.
void InitializeFrame(Frame& f,GeometricCamera* camera,ORBVocabulary* vocabulary) {
  f.mpORBvocabulary=vocabulary;f.mpORBextractorLeft=nullptr;f.mpORBextractorRight=nullptr;
  f.mnId=1000;f.mTimeStamp=0.;f.mnDataset=0;f.mNameFile.clear();
  f.mK=(cv::Mat_<float>(3,3)<<288.13f,0,312.32f,0,288.13f,264.73f,0,0,1);
  f.mK_<<288.13f,0,312.32f,0,288.13f,264.73f,0,0,1;
  f.mDistCoef=cv::Mat::zeros(4,1,CV_32F);
  f.fx=288.13f;f.fy=288.13f;f.cx=312.32f;f.cy=264.73f;
  f.invfx=1.f/f.fx;f.invfy=1.f/f.fy;
  f.mbf=18;f.mb=18.f/f.fx;f.mThDepth=40;
  f.N=1;f.Nleft=-1;f.Nright=-1;f.monoLeft=-1;f.monoRight=-1;
  f.mpCamera=camera;f.mpCamera2=nullptr;
  f.mvKeys.assign(1,cv::KeyPoint(312.32f,264.73f,1,-1,0,2));f.mvKeysUn=f.mvKeys;
  f.mvKeysRight.clear();f.mvuRight={306.32f};f.mvDepth={3.f};
  f.mDescriptors=cv::Mat::zeros(1,32,CV_8U);f.mDescriptorsRight=cv::Mat();
  f.mvpMapPoints.assign(1,MapPointPtr());f.mvbOutlier.assign(1,false);
  f.mnCloseMPs=1;f.mpReferenceKF=nullptr;f.mpLastKeyFrame=nullptr;f.mpPrevFrame=nullptr;
  f.mpImuPreintegrated=nullptr;f.mpImuPreintegratedFrame=nullptr;
  f.mBowVec.clear();f.mFeatVec.clear();
  f.mnScaleLevels=5;f.mfScaleFactor=1.2f;f.mfLogScaleFactor=std::log(1.2f);
  f.mvScaleFactors.resize(5);f.mvInvScaleFactors.resize(5);
  f.mvLevelSigma2.resize(5);f.mvInvLevelSigma2.resize(5);
  f.mvScaleFactors[0]=1;
  for(int i=1;i<5;++i)f.mvScaleFactors[i]=f.mvScaleFactors[i-1]*f.mfScaleFactor;
  for(int i=0;i<5;++i){
    f.mvInvScaleFactors[i]=1/f.mvScaleFactors[i];
    f.mvLevelSigma2[i]=f.mvScaleFactors[i]*f.mvScaleFactors[i];
    f.mvInvLevelSigma2[i]=1/f.mvLevelSigma2[i];
  }
  f.mnMinX=0;f.mnMinY=0;f.mnMaxX=640;f.mnMaxY=544;
  f.mfGridElementWidthInv=float(FRAME_GRID_COLS)/640;
  f.mfGridElementHeightInv=float(FRAME_GRID_ROWS)/544;
  f.mvLeftToRightMatch.clear();f.mvRightToLeftMatch.clear();f.mvStereo3Dpoints.clear();
  f.mImuCalib.mTbc=Sophus::SE3f();f.mImuCalib.mTcb=Sophus::SE3f();
  f.mImuCalib.Cov.diagonal().setZero();f.mImuCalib.CovWalk.diagonal().setZero();
  f.mImuCalib.mbIsSet=false;f.mImuBias=IMU::Bias();f.mPredBias=IMU::Bias();
  f.mRgbdOptimizationLevel=4;f.mUseLocalMapProjection=false;
  for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y){
    f.mGrid[x][y].clear();f.mGridRight[x][y].clear();
  }
  int x=0,y=0;
  if(f.PosInGrid(f.mvKeysUn[0],x,y))f.mGrid[x][y].push_back(0);
  f.SetPose(Sophus::SE3f());
  Require(f.HasPose()&&!f.HasVelocity(),"legal default Frame pose flags");
}

struct BoundaryCamera : Pinhole {
  BoundaryCamera():Pinhole(std::vector<float>{288.13f,288.13f,312.32f,264.73f}){}
  bool fixed=false;
  Eigen::Vector2f uv{312.32f,264.73f};
  std::uint64_t calls=0;
  Eigen::Vector2f project(const Eigen::Vector3f& point) override {
    ++calls;return fixed?uv:Pinhole::project(point);
  }
};
struct FixturePoint : MapPoint {
  FixturePoint(const Eigen::Vector3f& p,KeyFrame* key,Map* map):MapPoint(p,key,map){}
  void Publish(const LocalPointGeometry& g) {
    std::lock_guard<std::mutex> lock(mMutexPos);
    mWorldPos=g.position;mNormalVector=g.normal;
    mfMinDistance=g.min_distance;mfMaxDistance=g.max_distance;
  }
};
struct Cache {
  std::array<std::uint32_t,8> floats;
  std::array<unsigned long,2> markers;
  std::array<int,2> levels;
  bool in_view,in_view_right;
  bool operator==(const Cache& other)const {
    return floats==other.floats&&markers==other.markers&&levels==other.levels&&
           in_view==other.in_view&&in_view_right==other.in_view_right;
  }
};
Cache ReadCache(const MapPoint& p) {
  return {{Bits(p.mTrackProjX),Bits(p.mTrackProjY),Bits(p.mTrackProjXR),Bits(p.mTrackProjYR),
      Bits(p.mTrackDepth),Bits(p.mTrackDepthR),Bits(p.mTrackViewCos),Bits(p.mTrackViewCosR)},
      {p.mnLastFrameSeen,p.mnTrackReferenceForFrame},{p.mnTrackScaleLevel,p.mnTrackScaleLevelR},
      p.mbTrackInView,p.mbTrackInViewR};
}
void Poison(MapPoint& p) {
  p.mTrackProjX=101;p.mTrackProjY=102;p.mTrackProjXR=103;p.mTrackProjYR=104;
  p.mTrackDepth=105;p.mTrackDepthR=106;p.mTrackViewCos=107;p.mTrackViewCosR=108;
  p.mnTrackScaleLevel=109;p.mnTrackScaleLevelR=110;
  p.mbTrackInView=true;p.mbTrackInViewR=true;
  p.mnLastFrameSeen=111;p.mnTrackReferenceForFrame=112;
}
struct Result {
  bool visible;Cache fields;
  bool operator==(const Result& other)const{return visible==other.visible&&fields==other.fields;}
};
using GeometryBits=std::array<std::uint32_t,8>;
GeometryBits ReadGeometry(const LocalPointGeometry& g) {
  return {{Bits(g.position.x()),Bits(g.position.y()),Bits(g.position.z()),
           Bits(g.normal.x()),Bits(g.normal.y()),Bits(g.normal.z()),
           Bits(g.min_distance),Bits(g.max_distance)}};
}
LocalPointGeometry Geometry(Eigen::Vector3f p={0,0,4},
    Eigen::Vector3f normal={0,0,1},float low=1,float high=8) {
  LocalPointGeometry g;g.position=p;g.normal=normal;g.min_distance=low;g.max_distance=high;return g;
}
struct Fixture {
  ORBVocabulary vocabulary;
  KeyFrameDatabase database{vocabulary};
  BoundaryCamera camera;
  Map map{0};
  Frame frame;
  std::unique_ptr<KeyFrame> key;
  std::shared_ptr<FixturePoint> controlled;
  MapPointPtr point;
  Fixture() {
    InitializeFrame(frame,&camera,&vocabulary);
    key.reset(new KeyFrame(frame,&map,&database));map.AddKeyFrame(key.get());
    controlled.reset(new FixturePoint(Eigen::Vector3f(0,0,3),key.get(),&map));
    point=controlled;point->RecordLocalBirth();map.AddMapPoint(point);
    key->AddMapPoint(point,0);point->AddObservation(key.get(),0);point->UpdateNormalAndDepth();
  }
  Result Call(float limit=.5f) {
    Poison(*point);bool visible=frame.isInFrustum(point,limit);return {visible,ReadCache(*point)};
  }
};

struct Cases {
  Fixture& f;std::ofstream csv;Hash hash;unsigned count=0,visible=0;
  explicit Cases(Fixture& value,const std::string& out):f(value),csv(out+"/geometry_cases.csv") {
    Require(bool(csv),"cannot create geometry_cases.csv");
    csv<<"case,name,input_hash,visible,proj_x_bits,proj_y_bits,proj_xr_bits,proj_yr_bits,depth_bits,depth_r_bits,view_cos_bits,view_cos_r_bits,last_seen,track_reference,scale_level,scale_level_r,in_view,in_view_r,predict_scale_actual\n";
  }
  Result Add(const std::string& name,const LocalPointGeometry& geometry,
      int expect=-1,float limit=.5f,int rejection=0) {
    Require(!f.frame.mUseLocalMapProjection&&f.frame.Nleft==-1,"ordinary branch fixture changed");
    f.controlled->Publish(geometry);
    Hash input;
    for(auto v:ReadGeometry(geometry))input.Add(std::to_string(v)+",");
    input.Add(std::to_string(Bits(limit))+","+std::to_string(f.camera.fixed)+",");
    input.Add(std::to_string(Bits(f.camera.uv.x()))+","+std::to_string(Bits(f.camera.uv.y()))+",");
    const auto pose=f.frame.GetPose().matrix();
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)input.Add(std::to_string(Bits(pose(i,j)))+",");
    auto result=f.Call(limit);
    Require(expect<0||result.visible==bool(expect),name+": wrong branch");
    // PredictScale is the unchanged real library oracle, not a copied formula.
    int actual_scale=-999;
    if(result.visible) {
      const float distance=(geometry.position-f.frame.GetCameraCenter()).norm();
      Require(distance>0&&std::isfinite(distance)&&geometry.max_distance>0&&
          std::isfinite(geometry.max_distance),name+": undefined integer-scale test input");
      actual_scale=f.point->PredictScale(distance,&f.frame);
      Require(result.fields.levels[0]==actual_scale,name+": raw-max actual PredictScale differs");
    }
    if(rejection) {
      Require(!result.visible&&!result.fields.in_view,name+": rejected visible");
      const auto& p=result.fields;
      Require(p.floats[2]==Bits(103)&&p.floats[3]==Bits(104)&&p.floats[4]==Bits(105)&&
          p.floats[5]==Bits(106)&&p.floats[6]==Bits(107)&&p.floats[7]==Bits(108)&&
          p.levels==std::array<int,2>{{109,110}}&&p.in_view_right&&
          p.markers==std::array<unsigned long,2>{{111,112}},name+": untouched fields changed");
      if(rejection==1)Require(p.floats[0]==Bits(-1)&&p.floats[1]==Bits(-1),name+": preimage writes differ");
      else Require(p.floats[0]==Bits(f.camera.uv.x())&&p.floats[1]==Bits(f.camera.uv.y()),name+": partial image writes differ");
    }
    std::ostringstream row;row<<count<<','<<name<<','<<input.value<<','<<result.visible;
    for(auto v:result.fields.floats)row<<','<<v;
    for(auto v:result.fields.markers)row<<','<<v;
    for(auto v:result.fields.levels)row<<','<<v;
    row<<','<<result.fields.in_view<<','<<result.fields.in_view_right<<','<<actual_scale<<'\n';
    auto text=row.str();csv<<text;hash.Add(text);++count;visible+=result.visible;
    return result;
  }
};

void StaticCases(Cases& c) {
  auto& f=c.f;const float inf=std::numeric_limits<float>::infinity();
  c.Add("ordinary",Geometry(),1);
  auto off=Geometry({1,1,4},Eigen::Vector3f(1,1,4).normalized());
  auto r=c.Add("depth_is_euclidean_not_z",off,1);
  Require(r.fields.floats[4]==Bits(off.position.norm())&&r.fields.floats[4]!=Bits(off.position.z()),"depth is not Pc.norm");
  f.frame.SetPose(Sophus::SE3f(Eigen::Quaternionf(Eigen::AngleAxisf(.2f,Eigen::Vector3f::UnitY())),Eigen::Vector3f(.2f,-.1f,.3f)));
  c.Add("nonidentity_pose",off,1);f.frame.SetPose(Sophus::SE3f());
  c.Add("negative_depth",Geometry({0,0,-4}),0,.5f,1);
  c.Add("zero_depth_real_pinhole",Geometry({1,0,0},{1,0,0}),0,.5f,1);
  f.camera.fixed=true;f.camera.uv={312.32f,264.73f};
  r=c.Add("zero_depth_legacy_less_than",Geometry({1,0,0},{1,0,0}),1);
  Require(std::isinf(f.point->mTrackProjXR)&&f.point->mTrackProjXR<0,"zero reciprocal semantics changed");
  // Preserve/compare the actual FP environment's handling of subnormal depth;
  // both negative and flushed-to-zero paths keep a finite scale expression.
  c.Add("negative_denormal_depth",Geometry({1,0,-std::numeric_limits<float>::denorm_min()},{1,0,0}));
  for(int axis=0;axis<2;++axis)for(float edge:{0.f,axis?544.f:640.f}) {
    f.camera.uv={312.32f,264.73f};f.camera.uv[axis]=edge;
    c.Add("inclusive_image_edge_"+std::to_string(axis)+"_"+std::to_string(edge),Geometry(),1);
    f.camera.uv[axis]=std::nextafter(edge,edge==0?-inf:inf);
    c.Add("outside_image_edge_"+std::to_string(axis)+"_"+std::to_string(edge),Geometry(),0,.5f,1);
  }
  f.camera.uv={-0.0f,264.73f};r=c.Add("signed_zero_image",Geometry(),1);
  Require(r.fields.floats[0]==Bits(-0.0f),"signed zero project changed");
  f.camera.uv={312.32f,264.73f};
  c.Add("distance_lower_equal",Geometry({0,0,.8f*5},{0,0,1},5,5),1);
  c.Add("distance_lower_outside",Geometry({0,0,std::nextafter(.8f*5,0.f)},{0,0,1},5,5),0,.5f,2);
  c.Add("distance_upper_equal",Geometry({0,0,1.2f*5},{0,0,1},5,5),1);
  c.Add("distance_upper_outside",Geometry({0,0,std::nextafter(1.2f*5,inf)},{0,0,1},5,5),0,.5f,2);
  r=c.Add("scale_uses_uninflated_max",Geometry({0,0,5.5f},{0,0,1},1,5),1);
  Require(r.fields.levels[0]==0,"inflated maximum used in scale");
  r=c.Add("upper_scale_clamp",Geometry({0,0,3},{0,0,1},0,1e9f),1);
  Require(r.fields.levels[0]==4,"upper scale clamp changed");
  r=c.Add("lower_scale_clamp",Geometry({0,0,3},{0,0,1},0,2.5f),1);
  Require(r.fields.levels[0]==0,"lower scale clamp changed");
  c.Add("view_equal",Geometry({0,0,4},{0,0,.5f}),1);
  c.Add("view_below",Geometry({0,0,4},{0,0,std::nextafter(.5f,0.f)}),0,.5f,2);
  const float nan=std::numeric_limits<float>::quiet_NaN();
  c.Add("nan_normal_defined_scale",Geometry({0,0,4},{nan,0,1}),1);
  Require(std::isnan(f.point->mTrackViewCos),"NaN normal was sanitized");
  c.Add("nan_min_range_defined_scale",Geometry({0,0,4},{0,0,1},nan,8),1);
  f.camera.uv={nan,nan};c.Add("nan_project_defined_scale",Geometry(),1);
  Require(std::isnan(f.point->mTrackProjX)&&std::isnan(f.point->mTrackProjXR),"NaN UV sanitized");
  f.camera.uv={312.32f,264.73f};
  c.Add("nan_view_limit_defined_scale",Geometry({0,0,4},{0,0,-1}),1,nan);
  for(int level=-1;level<=8;++level) {
    float distance=5.f/std::pow(1.2f,float(level));
    for(float value:{std::nextafter(distance,0.f),distance,std::nextafter(distance,inf)})
      c.Add("scale_ulp_"+std::to_string(level)+"_"+std::to_string(Bits(value)),Geometry({0,0,value},{0,0,1},0,5));
  }
  f.camera.fixed=false;
  std::mt19937 rng(0x25002026);
  std::uniform_real_distribution<float> xy(-8,8),z(.01f,20),range(.01f,20),normal(-1,1);
  for(unsigned i=0;i<4096;++i) {
    const float px=xy(rng),py=xy(rng),pz=z(rng);
    const float nx=normal(rng),ny=normal(rng),nz=normal(rng);
    const float low=range(rng),high=range(rng);
    c.Add("finite_random_"+std::to_string(i),Geometry({px,py,pz},{nx,ny,nz},low,high));
  }
  // Same finite points under a nonidentity camera pose, preserving cached pose.
  f.frame.SetPose(Sophus::SE3f(Eigen::Quaternionf(Eigen::AngleAxisf(-.17f,Eigen::Vector3f::UnitX())),Eigen::Vector3f(.1f,.2f,-.3f)));
  for(unsigned i=0;i<256;++i) {
    const float px=xy(rng),py=xy(rng),pz=z(rng);
    const float nx=normal(rng),ny=normal(rng),nz=normal(rng);
    const float low=range(rng),high=range(rng);
    c.Add("finite_pose_random_"+std::to_string(i),Geometry({px,py,pz},{nx,ny,nz},low,high));
  }
  f.frame.SetPose(Sophus::SE3f());
}

struct ConcurrentResult {std::uint64_t writes=0,geometry_checks=0,projection_checks=0,projection_hybrids=0;};
ConcurrentResult Concurrent(Fixture& f,bool snapshot_required,bool public_writer) {
  using Geometries=std::vector<LocalPointGeometry,Eigen::aligned_allocator<LocalPointGeometry>>;
  Geometries states;
  const Eigen::Vector3f pa(0,0,3),pb(1,0,4);
  if(public_writer) {
    f.point->SetWorldPos(pa);f.point->UpdateNormalAndDepth();auto a=f.point->GetLocalTrackingGeometry();
    f.point->SetWorldPos(pb);auto ab=f.point->GetLocalTrackingGeometry();
    f.point->UpdateNormalAndDepth();auto b=f.point->GetLocalTrackingGeometry();
    f.point->SetWorldPos(pa);auto ba=f.point->GetLocalTrackingGeometry();
    states={a,ab,b,ba}; // SetWorldPos and UpdateNormalAndDepth publish separately.
  } else states={Geometry(pa,{0,0,1},1,3.1f),Geometry(pb,{0,0,.6f},2,10)};
  std::vector<GeometryBits> allowed_geometry;std::vector<Result> allowed_projection;
  for(const auto& g:states) {
    f.controlled->Publish(g);allowed_geometry.push_back(ReadGeometry(f.point->GetLocalTrackingGeometry()));
    allowed_projection.push_back(f.Call());
  }
  f.controlled->Publish(states.front());
  std::atomic<bool> stop{false},started{false};std::atomic<std::uint64_t> writes{0};
  const auto deadline=Clock::now()+std::chrono::seconds(10);
  std::thread writer([&] {
    started.store(true,std::memory_order_release);
    while(!stop.load(std::memory_order_acquire)&&Clock::now()<deadline) {
      if(public_writer) {
        f.point->SetWorldPos(pa);f.point->UpdateNormalAndDepth();
        f.point->SetWorldPos(pb);f.point->UpdateNormalAndDepth();
      } else {f.controlled->Publish(states[0]);f.controlled->Publish(states[1]);}
      ++writes;std::this_thread::yield();
    }
  });
  ConcurrentResult result;
  try {
    while(!started.load(std::memory_order_acquire)) {
      Require(Clock::now()<deadline,"writer start timeout");std::this_thread::yield();
    }
    while(writes.load()==0) {
      Require(Clock::now()<deadline,"writer made no progress");std::this_thread::yield();
    }
    for(unsigned i=0;i<10000;++i) {
      Require(Clock::now()<deadline,"concurrent fixture timeout");
      auto geometry=ReadGeometry(f.point->GetLocalTrackingGeometry());++result.geometry_checks;
      Require(std::find(allowed_geometry.begin(),allowed_geometry.end(),geometry)!=allowed_geometry.end(),
          "GetLocalTrackingGeometry contains a torn geometry state");
      auto projected=f.Call();++result.projection_checks;
      bool belongs=std::find(allowed_projection.begin(),allowed_projection.end(),projected)!=allowed_projection.end();
      if(!belongs)++result.projection_hybrids;
      if(snapshot_required)Require(belongs,"single-read projection outside a published geometry state");
    }
  } catch(...) {stop.store(true,std::memory_order_release);writer.join();throw;}
  stop.store(true,std::memory_order_release);writer.join();result.writes=writes.load();
  Require(result.writes>0,"writer did not run");return result;
}

void Bench(Fixture& f,const std::string& out,std::uint64_t iterations) {
  if(!iterations)return;
  Require(iterations<=10000000,"benchmark iteration cap");
  Pinhole camera(std::vector<float>{288.13f,288.13f,312.32f,264.73f});
  auto* previous=f.frame.mpCamera;f.frame.mpCamera=&camera;
  f.controlled->Publish(Geometry());
  std::ofstream file(out+"/geometry_benchmark.csv");Require(bool(file),"benchmark CSV open");
  file<<"batch,iterations,wall_ns,cpu_ns,visible\n";
  try {
    for(unsigned i=0;i<1024;++i)f.frame.isInFrustum(f.point,.5f);
    for(unsigned batch=0;batch<5;++batch) {
      std::uint64_t visible=0;
      auto wall=Nanoseconds(CLOCK_MONOTONIC),cpu=Nanoseconds(CLOCK_THREAD_CPUTIME_ID);
      for(std::uint64_t i=0;i<iterations;++i)visible+=f.frame.isInFrustum(f.point,.5f);
      auto cpu_end=Nanoseconds(CLOCK_THREAD_CPUTIME_ID),wall_end=Nanoseconds(CLOCK_MONOTONIC);
      Require(visible==iterations,"benchmark changed visibility");
      file<<batch<<','<<iterations<<','<<wall_end-wall<<','<<cpu_end-cpu<<','<<visible<<'\n';
    }
    Require(bool(file),"benchmark CSV write");
  } catch(...) {f.frame.mpCamera=previous;throw;}
  f.frame.mpCamera=previous;
}
}

int main(int argc,char** argv) {
  try {
    Require(argc==3||argc==4,"usage: geometry_probe OUTPUT_DIR SNAPSHOT_REQUIRED(0|1) [BENCH_ITERATIONS]");
    std::string out=argv[1],mode=argv[2];Require(mode=="0"||mode=="1","snapshot-required must be0or1");
    const bool snapshot_required=mode=="1";
    const std::uint64_t iterations=argc==4?std::stoull(argv[3]):0;
    Fixture f;Cases cases(f,out);StaticCases(cases);cases.csv.flush();Require(bool(cases.csv),"case CSV flush");
    auto controlled=Concurrent(f,snapshot_required,false);
    auto public_writer=Concurrent(f,snapshot_required,true);
    Bench(f,out,iterations);
    std::ofstream summary(out+"/geometry_summary.json");Require(bool(summary),"summary open");
    summary<<"{\n  \"schema\":1,\n  \"actual_library_methods\":true,\n"
       <<"  \"static_cases\":"<<cases.count<<",\n  \"static_visible\":"<<cases.visible<<",\n"
       <<"  \"static_case_csv_fnv1a64\":\""<<cases.hash.value<<"\",\n"
       <<"  \"snapshot_required\":"<<(snapshot_required?"true":"false")<<",\n"
       <<"  \"controlled_writer\":{\"cycles\":"<<controlled.writes<<",\"geometry_checks\":"<<controlled.geometry_checks
       <<",\"projection_checks\":"<<controlled.projection_checks<<",\"projection_hybrids\":"<<controlled.projection_hybrids<<"},\n"
       <<"  \"public_setpos_update_writer\":{\"cycles\":"<<public_writer.writes<<",\"geometry_checks\":"<<public_writer.geometry_checks
       <<",\"projection_checks\":"<<public_writer.projection_checks<<",\"projection_hybrids\":"<<public_writer.projection_hybrids<<"},\n"
       <<"  \"benchmark_iterations_per_batch\":"<<iterations<<",\n"
       <<"  \"scope\":\"Synthetic actual Frame/MapPoint/Pinhole methods, not System/image replay or whole-SLAM performance; concurrent split reads are not required to equal one snapshot\",\n"
       <<"  \"excluded_undefined_inputs\":\"No NaN/infinite/out-of-int-range scale-to-int conversion; selected NaN normal/UV/minimum/view-limit cases keep scale finite\",\n"
       <<"  \"pass\":true\n}\n";
    summary.close();Require(bool(summary),"summary write");
    std::cout<<"GEOMETRY_ACTUAL_LIBRARY_STATIC_PASS cases="<<cases.count<<" visible="<<cases.visible
       <<" fnv1a64="<<cases.hash.value<<"\nGEOMETRY_LOCKED_SNAPSHOT_PASS controlled="<<controlled.writes
       <<" public="<<public_writer.writes<<"\n";
    return 0;
  } catch(const std::exception& error) {std::cerr<<"GEOMETRY_PROBE_FAIL "<<error.what()<<'\n';return 1;}
}
