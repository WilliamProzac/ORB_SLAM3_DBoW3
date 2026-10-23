// RGBD25: compile this TU with each variant's headers and complete matching DSO.
// ATOMIC_CANDIDATE refers to MapPoint, not the pre-existing atomic KeyFrame.
// Production methods are called as linked; no method overrides/interposition.
// Derived fixtures expose protected state only. No private/public macro needed.
#include "MapPoint.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "LocalTrackingIdentity.h"
#include "MemoryStats.h"
#include "CameraModels/Pinhole.h"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>
#include <time.h>

#ifndef ATOMIC_CANDIDATE
#error Use -DATOMIC_CANDIDATE=0 for baseline/geometry, 1 for atomic/both.
#endif

using namespace ORB_SLAM3;
namespace {
using Clock=std::chrono::steady_clock;
using Ms=std::chrono::milliseconds;
void Require(bool okay,const std::string& message) {
  if(!okay) throw std::runtime_error(message);
}
bool Wait(const std::function<bool()>& predicate,int milliseconds=3000) {
  const auto deadline=Clock::now()+Ms(milliseconds);
  while(Clock::now()<deadline) {
    if(predicate()) return true;
    std::this_thread::yield();
  }
  return false;
}

// Build actual immutable KeyFrame fields through its real Frame constructor.
// No const_cast writes to N, NLeft, mvuRight or mDescriptors are performed.
void PrepareFrame(Frame& f,GeometricCamera* camera) {
  f.mnId=1234;f.mTimeStamp=12.5;f.N=4;
  f.mpORBvocabulary=nullptr;f.mpORBextractorLeft=nullptr;f.mpORBextractorRight=nullptr;
  f.mpCamera=camera;f.mpCamera2=nullptr;
  f.Nleft=-1;f.Nright=-1;f.monoLeft=0;f.monoRight=0;
  f.mK=cv::Mat::eye(3,3,CV_32F);f.mK_.setIdentity();
  f.mDistCoef=cv::Mat::zeros(4,1,CV_32F);
  Frame::fx=500;Frame::fy=500;Frame::cx=320;Frame::cy=240;
  Frame::invfx=1.f/500;Frame::invfy=1.f/500;
  Frame::mnMinX=0;Frame::mnMinY=0;Frame::mnMaxX=640;Frame::mnMaxY=480;
  Frame::mfGridElementWidthInv=FRAME_GRID_COLS/640.f;
  Frame::mfGridElementHeightInv=FRAME_GRID_ROWS/480.f;
  f.mbf=40;f.mb=.08f;f.mThDepth=40;
  f.mvKeys.assign(4,cv::KeyPoint(320,240,31));f.mvKeysUn=f.mvKeys;
  f.mvuRight.assign(4,-1);f.mvDepth.assign(4,4);
  f.mDescriptors=cv::Mat(4,32,CV_8UC1);
  for(int row=0;row<4;++row)
    for(int column=0;column<32;++column)
      f.mDescriptors.at<unsigned char>(row,column)=static_cast<unsigned char>(row*17+column);
  f.mvpMapPoints.assign(4,MapPointPtr());f.mvbOutlier.assign(4,false);
  f.mnScaleLevels=3;f.mfScaleFactor=1.2f;f.mfLogScaleFactor=std::log(1.2f);
  f.mvScaleFactors={1,1.2f,1.44f};f.mvInvScaleFactors={1,1/1.2f,1/1.44f};
  f.mvLevelSigma2={1,1.44f,2.0736f};f.mvInvLevelSigma2={1,1/1.44f,1/2.0736f};
  f.mnDataset=0;f.mnCloseMPs=0;f.mpLastKeyFrame=nullptr;
  f.mImuBias=IMU::Bias();f.mImuCalib.Set(Sophus::SE3f(),0,0,0,0);
  f.SetPose(Sophus::SE3f());
}

class ProbeKey:public KeyFrame {
public:
  ProbeKey(Frame& frame,Map* map):KeyFrame(frame,map,nullptr) {mnId=42;}
  std::mutex& Features(){return mMutexFeatures;}
};

class ProbePoint:public MapPoint {
public:
  ProbePoint(KeyFrame* key,Map* map,unsigned long id):MapPoint(Eigen::Vector3f(1.25f,-2.5f,4),key,map) {
    static_assert(std::is_same<decltype(mbBad),
#if ATOMIC_CANDIDATE
      std::atomic<bool>
#else
      bool
#endif
      >::value,"MapPoint fixture/header variant mismatch");
    mnId=id;mNormalVector=Eigen::Vector3f(.25f,.5f,.75f);
    mDescriptor=cv::Mat(1,32,CV_8UC1);
    for(int i=0;i<32;++i)mDescriptor.at<unsigned char>(0,i)=static_cast<unsigned char>(i+11);
    mfMinDistance=.75f;mfMaxDistance=123.5f;
    mBackupRefKFId=42;mBackupReplacedId=-1;
  }
  std::mutex& Features(){return mMutexFeatures;}
  std::mutex& Position(){return mMutexPos;}
  void ArchiveSeed() {
    mnId=77;mnFirstKFid=42;mnFirstFrame=1234;nObs=7;
    mBackupObservationsId1={{42,1}};mBackupObservationsId2={{42,-1}};
    mBackupRefKFId=42;mBackupReplacedId=777;
  }
  void ReadSentinels() {
    mnId=999;mnFirstKFid=-999;mnFirstFrame=-998;nObs=19;
    mWorldPos=Eigen::Vector3f(-8,-9,-10);mNormalVector.setZero();
    mDescriptor=cv::Mat::zeros(1,32,CV_8UC1);
    mBackupObservationsId1={{999,8}};mBackupObservationsId2={{999,9}};
    mBackupRefKFId=666;mBackupReplacedId=-88;
    mfMinDistance=11;mfMaxDistance=12;
  }
  void CheckArchive(bool bad) {
    Require(isBad()==bad,"archive bool");
    Require(mnId==77 && mnFirstKFid==42 && mnFirstFrame==1234 && nObs==7,"archive prefix");
    Require((GetWorldPos()-Eigen::Vector3f(1.25f,-2.5f,4)).norm()==0,"archive position");
    Require((GetNormal()-Eigen::Vector3f(.25f,.5f,.75f)).norm()==0,"archive normal");
    Require(mBackupObservationsId1==std::map<unsigned long,int>{{42,1}} &&
      mBackupObservationsId2==std::map<unsigned long,int>{{42,-1}},"archive observation backups");
    const cv::Mat descriptor=GetDescriptor();
    Require(descriptor.type()==CV_8UC1 && descriptor.rows==1 && descriptor.cols==32,"archive descriptor shape");
    for(int i=0;i<32;++i)Require(descriptor.at<unsigned char>(0,i)==i+11,"archive descriptor bytes");
    Require(mBackupRefKFId==42,"archive field immediately before bool");
    Require(mBackupReplacedId==777 && mfMinDistance==.75f && mfMaxDistance==123.5f,"archive fields after bool");
  }
  void BackupEdge(long long id) {mBackupRefKFId=42;mBackupReplacedId=id;}
};

struct Fixture {
  Map map{0};
  Pinhole camera{std::vector<float>{500,500,320,240}};
  Frame frame;
  std::unique_ptr<ProbeKey> key;
  std::shared_ptr<ProbePoint> point;
  Fixture() {
    PrepareFrame(frame,&camera);key.reset(new ProbeKey(frame,&map));
    point=Make(77);
  }
  std::shared_ptr<ProbePoint> Make(unsigned long id) {
    auto p=std::shared_ptr<ProbePoint>(new ProbePoint(key.get(),&map,id));
    map.AddMapPoint(p);return p;
  }
  void Attach(const std::shared_ptr<ProbePoint>& p,int index) {
    key->AddMapPoint(p,index);p->AddObservation(key.get(),index);
  }
  bool Contains(const MapPointPtr& p) {
    for(const auto& q:map.GetAllMapPoints())if(q==p)return true;
    return false;
  }
};

void DefaultAndSample() {
  auto detached=std::make_shared<MapPoint>();
  Require(!detached->isBad(),"unpublished archive constructor initializes false");
  Fixture f;LocalMapLockSample sample;
  sample.site=LocalMapLockSite::VoteMapPointIsBadFeatures;sample.call_ordinal=73;
  sample.clock_failures=7;sample.request_ns=sample.acquired_ns=sample.before_unlock_ns=123;
  // Real baseline spans OR flags, so start its valid measurement at zero.
#if !ATOMIC_CANDIDATE
  sample.clock_failures=0;
#endif
  Require(!f.point->isBad(&sample),"new point is not bad");
  Require(sample.object_id==77 && sample.object_frame_id==1234 && sample.call_ordinal==73,"sample identity and ordinal");
#if ATOMIC_CANDIDATE
  Require(sample.site==LocalMapLockSite::NotCollected && !sample.clock_failures &&
    !sample.request_ns && !sample.acquired_ns && !sample.before_unlock_ns,"no fictional atomic lock measurement");
#else
  Require(sample.site==LocalMapLockSite::VoteMapPointIsBadFeatures && !sample.clock_failures &&
    sample.request_ns>0 && sample.acquired_ns>=sample.request_ns &&
    sample.before_unlock_ns>=sample.acquired_ns,"baseline real lock endpoints");
#endif
}

void FeaturesLockedQuery() {
  Fixture f;std::unique_lock<std::mutex> held(f.point->Features());
  std::promise<void> entered;auto started=entered.get_future();
  auto query=std::async(std::launch::async,[&]{entered.set_value();return f.point->isBad();});
  started.wait();
  auto getter=std::async(std::launch::async,[&]{return f.point->GetObservations();});
  const bool query_ready=query.wait_for(Ms(250))==std::future_status::ready;
  const bool getter_ready=getter.wait_for(Ms(30))==std::future_status::ready;
  held.unlock();const bool bad=query.get();const auto observations=getter.get();
  Require(!bad && observations.empty() && !getter_ready,"feature getter retains its lock");
  Require(query_ready==bool(ATOMIC_CANDIDATE),"only candidate query completes under another thread's feature lock");
}

void RetirementPositionWindow() {
  Fixture f;std::unique_lock<std::mutex> held(f.point->Position());
  auto retiring=std::async(std::launch::async,[&]{f.point->SetBadFlag();});
  const bool writer_has_features=Wait([&]{
    if(!f.point->Features().try_lock())return true;
    f.point->Features().unlock();return false;
  });
  auto query=std::async(std::launch::async,[&]{return f.point->isBad();});
  const bool ready=query.wait_for(Ms(250))==std::future_status::ready;
  // Save candidate false before allowing writer mutation. Do not inspect plain
  // retirement fields concurrently or use GetReplaced as a publication probe.
  const bool during=ready?query.get():false;
  held.unlock();retiring.get();const bool after=ready?during:query.get();
  Require(writer_has_features,"writer reached feature->position wait");
  Require(ready==bool(ATOMIC_CANDIDATE),"document controlled pre-transition query behavior");
  Require(ATOMIC_CANDIDATE?!after:after,"candidate false before transition; baseline waits for true");
  Require(f.point->isBad() && f.point->GetObservations().empty() && !f.Contains(f.point),"retirement finishes after release");
}

void RetirementCleanupBoundary() {
  Fixture f;f.Attach(f.point,0);
  std::unique_lock<std::mutex> held(f.key->Features());
  auto retiring=std::async(std::launch::async,[&]{f.point->SetBadFlag();});
  const bool local_transition=Wait([&]{return f.point->GetObservations().empty();});
  const bool bad=f.point->isBad();const bool still_in_map=f.Contains(f.point);
  const bool writer_finished=retiring.wait_for(Ms(30))==std::future_status::ready;
  held.unlock();retiring.get();
  Require(local_transition && bad && still_in_map && !writer_finished,"true does not claim complete KF/map cleanup");
  Require(!f.key->GetMapPoint(0) && !f.Contains(f.point),"real keyframe and map removal");
}

void ReplacementTransfer() {
  Fixture f;auto target=f.Make(78);f.Attach(f.point,1);
  f.point->IncreaseVisible(4);f.point->IncreaseFound(2);
  const auto before=MemoryStats::Read(1).retired;
  std::unique_lock<std::mutex> held(f.key->Features());
  auto replacing=std::async(std::launch::async,[&]{f.point->Replace(target);});
  const bool local_transition=Wait([&]{return f.point->GetObservations().empty();});
  const bool bad=f.point->isBad();const auto edge=f.point->GetReplaced();
  const bool map_owned=f.Contains(f.point);
  const bool writer_finished=replacing.wait_for(Ms(30))==std::future_status::ready;
  held.unlock();replacing.get();
  Require(local_transition && bad && edge==target && map_owned && !writer_finished,"replacement edge available via locked getter before downstream migration completes");
  Require(f.point->GetReplaced()==target && f.key->GetMapPoint(1)==target && target->IsInKeyFrame(f.key.get()),"real target edge and observation migration");
  Require(target->GetFound()==4 && target->Observations()==1,"found and observation transfer");
  Require(!f.Contains(f.point) && f.Contains(target),"map source removed; target retained");
  f.point->SetBadFlag();f.point->Replace(f.Make(79));
  Require(f.point->GetReplaced()==target && MemoryStats::Read(1).retired==before+1,"retired source edge immutable and counted once");
}

void DuplicateRetirementRace() {
  Fixture f;auto target=f.Make(78);const auto before=MemoryStats::Read(1).retired;
  std::promise<void> go;auto signal=go.get_future().share();
  auto erase=std::async(std::launch::async,[&]{signal.wait();f.point->SetBadFlag();});
  auto replace=std::async(std::launch::async,[&]{signal.wait();f.point->Replace(target);});
  go.set_value();erase.get();replace.get();
  const auto edge=f.point->GetReplaced();
  Require(f.point->isBad() && (!edge || edge==target) && !f.Contains(f.point),"retirement/replace race has one terminal result");
  Require(MemoryStats::Read(1).retired==before+1,"race retires source once");
  for(int i=0;i<3;++i){f.point->SetBadFlag();f.point->Replace(target);}
  Require(f.point->GetReplaced()==edge && MemoryStats::Read(1).retired==before+1,"repeat calls cannot rewrite terminal edge");
}

void OppositeReplacement() {
  Fixture f;auto b=f.Make(78);const auto before=MemoryStats::Read(1).retired;
  std::promise<void> go;auto signal=go.get_future().share();
  auto first=std::async(std::launch::async,[&]{signal.wait();f.point->Replace(b);});
  auto second=std::async(std::launch::async,[&]{signal.wait();b->Replace(f.point);});
  go.set_value();first.get();second.get();
  const auto a_to=f.point->GetReplaced(),b_to=b->GetReplaced();
  Require(bool(a_to)!=bool(b_to) && ((a_to==b && !b_to)||(!a_to && b_to==f.point)),"opposite replacements do not create a strong cycle");
  Require(MemoryStats::Read(1).retired==before+1,"opposite replacement commits one edge");
}

void ReplacementEarlyReturns() {
  Fixture f;const auto before=MemoryStats::Read(1).retired;
  f.point->Replace(MapPointPtr());f.point->Replace(f.point);
  auto same_id=f.Make(77);f.point->Replace(same_id);
  Require(!f.point->isBad() && !f.point->GetReplaced() && MemoryStats::Read(1).retired==before,"null/self/same-id replacement does not retire source");
  auto retired_target=f.Make(78);retired_target->SetBadFlag();
  f.point->Replace(retired_target);
  Require(!f.point->isBad() && !f.point->GetReplaced() && f.Contains(f.point),"bad target replacement rejected");
  Require(MemoryStats::Read(1).retired==before+1,"only explicitly retired target counted");
}

void KeepAliveAndChain() {
  Fixture f;const auto before=MemoryStats::Read(1);std::weak_ptr<MapPoint> weak=f.point;
  auto retained=f.point;f.point->SetBadFlag();f.point.reset();
  Require(!weak.expired() && retained->isBad() && MemoryStats::Read(1).destroyed==before.destroyed,"caller strong owner survives map erase");
  retained.reset();
  Require(weak.expired() && MemoryStats::Read(1).destroyed==before.destroyed+1 &&
    MemoryStats::Read(1).retired_destroyed==before.retired_destroyed+1,"last strong owner controls actual destruction");
  auto a=f.Make(80),b=f.Make(81),c=f.Make(82);
  std::weak_ptr<MapPoint> wb=b;a->Replace(b);b->Replace(c);b.reset();
  Require(!wb.expired() && a->GetReplaced()->GetReplaced()==c,"strong replacement chain retains intermediate node");
  a.reset();Require(wb.expired(),"acyclic chain releases intermediate node with final source owner");
}

void IdentityBarrier() {
  Fixture f;std::unique_lock<std::recursive_mutex> barrier(LocalTrackingIdentityMutex());
  auto writer=std::async(std::launch::async,[&]{f.point->SetBadFlag();});
  const bool finished=writer.wait_for(Ms(50))==std::future_status::ready;
  const bool bad=f.point->isBad();barrier.unlock();writer.get();
  Require(!finished && !bad && f.point->isBad(),"retirement still respects existing identity barrier");
}

void UnpublishedPostLoad() {
  Fixture f;auto target=f.Make(78);f.point->SetBadFlag();
  f.point->BackupEdge(78);
  std::map<unsigned long,KeyFrame*> keys{{42,f.key.get()}};
  std::map<unsigned long,MapPointPtr> points{{77,f.point},{78,target}};
  f.point->PostLoad(keys,points);
  Require(f.point->isBad() && f.point->GetReplaced()==target,"unpublished load restores strong replacement identity");
  target->BackupEdge(77);bool rejected=false;
  try{target->PostLoad(keys,points);}catch(const std::runtime_error&){rejected=true;}
  Require(rejected && !target->GetReplaced(),"unpublished cycle rejected without strong cyclic edge");
}

using Case=std::pair<const char*,void(*)()>;
const std::vector<Case>& Cases() {
  static const std::vector<Case> cases={{"default_sample",DefaultAndSample},{"features_lock",FeaturesLockedQuery},
    {"retirement_position_window",RetirementPositionWindow},{"retirement_cleanup_boundary",RetirementCleanupBoundary},
    {"replacement_transfer",ReplacementTransfer},{"duplicate_retirement_race",DuplicateRetirementRace},
    {"opposite_replacement",OppositeReplacement},{"replacement_early_returns",ReplacementEarlyReturns},{"keepalive_chain",KeepAliveAndChain},
    {"identity_barrier",IdentityBarrier},{"unpublished_postload",UnpublishedPostLoad}};
  return cases;
}

template<class Archive>void Save(std::ostream& out,MapPoint& p){Archive ar(out);ar<<p;}
template<class Archive>void Load(std::istream& in,MapPoint& p){Archive ar(in);ar>>p;}
void ArchiveCase(const std::string& mode,const std::string& format,const std::string& path,bool bad) {
  Fixture f;
  if(mode=="write") {
    f.point->ArchiveSeed();if(bad)f.point->SetBadFlag();
    std::ofstream out(path,std::ios::binary);Require(bool(out),"archive open output");
    if(format=="text")Save<boost::archive::text_oarchive>(out,*f.point);
    else Save<boost::archive::binary_oarchive>(out,*f.point);
    out.close();Require(bool(out),"archive write complete");f.point->CheckArchive(bad);
  } else if(mode=="reject") {
    f.point->ReadSentinels();const auto before=MemoryStats::Read(1).retired;bool rejected=false;
    try {
      std::ifstream in(path,std::ios::binary);Require(bool(in),"archive open input");
      if(format=="text")Load<boost::archive::text_iarchive>(in,*f.point);
      else Load<boost::archive::binary_iarchive>(in,*f.point);
    } catch(const boost::archive::archive_exception&) {rejected=true;}
    Require(rejected,"truncated archive must throw; never publish a partial object");
    std::cout<<"MAPPOINT_RESULT {\"pass\":true,\"mode\":\"reject\",\"format\":\""<<format
      <<"\",\"bad_after_exception\":"<<(f.point->isBad()?"true":"false")
      <<",\"retire_delta\":"<<MemoryStats::Read(1).retired-before<<"}\n";
    return;
  } else {
    f.point->ReadSentinels();const auto before=MemoryStats::Read(1).retired;
    for(int repeat=0;repeat<2;++repeat) {
      std::ifstream in(path,std::ios::binary);Require(bool(in),"archive open input");
      if(format=="text")Load<boost::archive::text_iarchive>(in,*f.point);
      else Load<boost::archive::binary_iarchive>(in,*f.point);
      f.point->CheckArchive(bad);
      Require(MemoryStats::Read(1).retired==before+(bad?1:0),"load Retire is counted once, including repeated load");
    }
  }
  std::cout<<"MAPPOINT_RESULT {\"pass\":true,\"mode\":\""<<mode<<"\",\"format\":\""<<format
    <<"\",\"bad\":"<<(bad?"true":"false")<<"}\n";
}

std::uint64_t Nanoseconds(clockid_t clock) {
  timespec value{};
  Require(clock_gettime(clock,&value)==0,"benchmark clock_gettime");
  return static_cast<std::uint64_t>(value.tv_sec)*1000000000ULL+value.tv_nsec;
}
void Bench() {
  Fixture f;
  const MapPointPtr owner=f.point;MapPoint* const point=owner.get();
  constexpr std::uint64_t warmup=100000,iterations=1000000;
  // External, non-pure DSO calls remain observable to the compiler. Accumulate
  // every return value and emit it; the live point intentionally stays false.
  std::uint64_t checksum=0;
  for(std::uint64_t i=0;i<warmup;++i)checksum+=point->isBad()?1:0;
  Require(checksum==0,"benchmark warmup state");
  for(unsigned batch=0;batch<5;++batch) {
    checksum=0;
    const auto wall_begin=Nanoseconds(CLOCK_MONOTONIC);
    const auto cpu_begin=Nanoseconds(CLOCK_THREAD_CPUTIME_ID);
    for(std::uint64_t i=0;i<iterations;++i)checksum+=point->isBad()?1:0;
    const auto cpu_end=Nanoseconds(CLOCK_THREAD_CPUTIME_ID);
    const auto wall_end=Nanoseconds(CLOCK_MONOTONIC);
    Require(cpu_end>=cpu_begin && wall_end>=wall_begin && checksum==0,"benchmark clocks/state");
    std::cout<<"MAPPOINT_BENCH {\"atomic_candidate\":"<<ATOMIC_CANDIDATE
      <<",\"batch\":"<<batch<<",\"iterations\":"<<iterations<<",\"warmup_iterations\":"<<warmup
      <<",\"checksum\":"<<checksum<<",\"wall_ns\":"<<wall_end-wall_begin
      <<",\"thread_cpu_ns\":"<<cpu_end-cpu_begin
      <<",\"scope\":\"uncontended live isBad only; one fixed strong owner; no background updater; not a SLAM speedup estimate\"}\n";
  }
}
} // namespace

int main(int argc,char** argv) {
  setenv("ORB_MEMORY_STATS","1",1);
  try {
    Require(MemoryStats::Enabled(),"memory counters must be enabled before fixture construction");
    if(argc==2 && std::string(argv[1])=="contract") {
      for(const auto& c:Cases()){c.second();std::cout<<"MAPPOINT_CASE "<<c.first<<" PASS\n";}
    } else if(argc==2 && std::string(argv[1])=="bench") {
      Bench();return 0;
    } else if(argc==3 && std::string(argv[1])=="case") {
      bool found=false;for(const auto& c:Cases())if(c.first==std::string(argv[2])){c.second();found=true;break;}
      Require(found,"unknown case");
    } else if(argc==5 && (std::string(argv[1])=="write"||std::string(argv[1])=="read"||std::string(argv[1])=="reject") &&
        (std::string(argv[2])=="text"||std::string(argv[2])=="binary") &&
        (std::string(argv[4])=="0"||std::string(argv[4])=="1")) {
      ArchiveCase(argv[1],argv[2],argv[3],std::string(argv[4])=="1");return 0;
    } else {
      std::cerr<<"usage: probe contract | bench | case NAME | (write|read|reject) (text|binary) PATH BAD\n";return 2;
    }
    std::cout<<"MAPPOINT_RESULT {\"pass\":true,\"atomic_candidate\":"<<ATOMIC_CANDIDATE<<"}\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"MAPPOINT_PROBE_FAILURE: "<<e.what()<<'\n';return 1;
  }
}
