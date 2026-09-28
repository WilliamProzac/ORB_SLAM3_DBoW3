#include "System.h"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

// Access only after Track returns; no algorithm or per-frame timer instrumentation.
namespace ORB_SLAM3 {
class ControlsAccess {
public:
 static void row(System& s, std::ostream& o) {
  const auto& f=s.mpTracker->mCurrentFrame;
  size_t associations=0;
  for(size_t j=0;j<f.mvpMapPoints.size();++j)
   if(f.mvpMapPoints[j] && !f.mvbOutlier[j]) ++associations;
  auto* map=s.mpAtlas->GetCurrentMap();
  o<<','<<f.N<<','<<associations<<','<<map->GetId()<<','
   <<s.mpAtlas->GetAllMaps().size()<<','<<map->KeyFramesInMap()<<','<<map->MapPointsInMap();
 }
 static bool finish(System& s) {
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(60);
  // The official Shutdown only requests finish. Wait outside the measured region.
  while(!s.mpLocalMapper->isFinished() || !s.mpLoopCloser->isFinished() || s.mpLoopCloser->isRunningGBA()) {
   if(std::chrono::steady_clock::now()>deadline)return false;
   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if(s.mptLocalMapping->joinable())s.mptLocalMapping->join();
  if(s.mptLoopClosing->joinable())s.mptLoopClosing->join();
  return true;
 }
};
}

int main(int argc,char** argv) {
 if(argc!=7) {std::cerr<<"vocabulary settings mono|rgbd manifest output max_frames(0=all)\n";return 2;}
 cv::setNumThreads(1);
 const bool rgbd=std::string(argv[3])=="rgbd";
 struct Input {double t;std::string im,depth;};
 std::vector<Input> inputs;std::ifstream f(argv[4]);std::string line;
 while(std::getline(f,line)) {
  if(line.empty()||line[0]=='#')continue;
  std::istringstream s(line);Input i;
  if(!(s>>i.t>>i.im)||(rgbd&&!(s>>i.depth)))return 3;
  if(!inputs.empty()&&i.t<=inputs.back().t)return 3;
  inputs.push_back(i);
 }
 if(inputs.empty())return 3;
 size_t limit=std::stoul(argv[6]);if(limit&&inputs.size()>limit)inputs.resize(limit);
 std::string out=argv[5];
 ORB_SLAM3::System slam(argv[1],argv[2],rgbd?ORB_SLAM3::System::RGBD:ORB_SLAM3::System::MONOCULAR,false);
 std::ofstream csv(out+"/frames.csv"),poses(out+"/online.txt");
 if(!csv||!poses)return 5;
 csv<<"index,timestamp,state,track_ms,input_lateness_ms,features,valid_associations,map_id,maps,kfs,mps\n";
 csv<<std::setprecision(17);poses<<std::fixed<<std::setprecision(9);
 using Clock=std::chrono::steady_clock;auto begin=Clock::now();
 for(size_t k=0;k<inputs.size();++k) {
  auto &i=inputs[k];auto due=begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(i.t-inputs.front().t));
  std::this_thread::sleep_until(due);
  cv::Mat im=cv::imread(i.im,cv::IMREAD_UNCHANGED),depth;
  if(rgbd)depth=cv::imread(i.depth,cv::IMREAD_UNCHANGED);
  if(im.empty()||(rgbd&&depth.empty()))return 4;
  auto start=Clock::now();
  const double late=std::max(0.0,std::chrono::duration<double,std::milli>(start-due).count());
  Sophus::SE3f T=rgbd?slam.TrackRGBD(im,depth,i.t):slam.TrackMonocular(im,i.t);
  double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
  int state=slam.GetTrackingState();
  csv<<k<<','<<i.t<<','<<state<<','<<ms<<','<<late;
  ORB_SLAM3::ControlsAccess::row(slam,csv);csv<<'\n';
  if(state==2) {
   auto W=T.inverse();auto p=W.translation();auto q=W.unit_quaternion();
   if(!W.matrix().allFinite())return 6;
   poses<<i.t<<' '<<p.x()<<' '<<p.y()<<' '<<p.z()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';
  }
  if(k%100==0){csv.flush();poses.flush();std::cerr<<"REPRO_FRAME "<<k<<" state="<<state<<" ms="<<ms<<'\n';}
 }
 csv.close();poses.close();slam.Shutdown();
 if(!ORB_SLAM3::ControlsAccess::finish(slam))return 7;
 // Online poses are primary: a single alignment, never align separate Atlas maps.
 slam.SaveKeyFrameTrajectoryTUM(out+"/keyframes.txt");
 std::cerr<<"REPRO_COMPLETE "<<inputs.size()<<'\n';return 0;
}
