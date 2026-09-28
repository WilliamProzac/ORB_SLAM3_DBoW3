#include "System.h"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

// manifest: timestamp_seconds absolute_image_path [absolute_depth_path]
int main(int argc,char** argv) {
 if(argc!=7) { std::cerr<<"vocabulary settings mono|rgbd manifest output max_frames(0=all)\n"; return 2; }
 cv::setNumThreads(1);
 const bool rgbd=std::string(argv[3])=="rgbd";
 struct Input {double t; std::string im,depth;};
 std::vector<Input> inputs; std::ifstream f(argv[4]); std::string line;
 while(std::getline(f,line)) {if(line.empty()||line[0]=='#')continue; std::istringstream s(line); Input i; if(!(s>>i.t>>i.im)) return 3; if(rgbd && !(s>>i.depth))return 3; inputs.push_back(i);}
 if(inputs.empty())return 3;
 size_t limit=std::stoul(argv[6]); if(limit && inputs.size()>limit)inputs.resize(limit);
 std::string out=argv[5];
 ORB_SLAM2::System slam(argv[1],argv[2],rgbd?ORB_SLAM2::System::RGBD:ORB_SLAM2::System::MONOCULAR,false);
 // Budget only used by upstream optional routines; unchanged image extraction count.
 slam.SetBudgetPerFrame(1.0/slam.mpTracker->camera_fps);
 slam.SetConstrPerFrame(2000);
 slam.SetRealTimeFileStream(out+"/upstream_online.txt");
 std::ofstream csv(out+"/frames.csv"), poses(out+"/online.txt");
 csv<<"index,timestamp,state,track_ms,local_ms,match_ms,opt_ms,hash_field_ms,local_mp,covis_mp,hash_mp,inliers,features,kfs,mps,pose_stage_sum_ms,post_ms,pose_ready_ms\n";
 csv<<std::setprecision(17); poses<<std::fixed<<std::setprecision(9);
 using Clock=std::chrono::steady_clock; auto begin=Clock::now();
 for(size_t k=0;k<inputs.size();++k) {
  auto &i=inputs[k]; auto due=begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(i.t-inputs.front().t));
  std::this_thread::sleep_until(due);
  cv::Mat im=cv::imread(i.im,cv::IMREAD_UNCHANGED),depth;
  if(rgbd)depth=cv::imread(i.depth,cv::IMREAD_UNCHANGED);
  if(im.empty()||(rgbd&&depth.empty()))return 4;
  auto start=Clock::now();
  cv::Mat T=rgbd?slam.TrackRGBD(im,depth,i.t):slam.TrackMonocular(im,i.t);
  double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
  const auto &l=slam.mpTracker->logCurrentFrame;
  int state=slam.GetTrackingState();
  csv<<k<<','<<i.t<<','<<state<<','<<ms<<','<<l.time_track_map*1000<<','<<l.time_match*1000<<','<<l.time_optim*1000<<','<<l.time_hash_query*1000<<','<<l.lmk_localmap_comb<<','<<l.lmk_localmap_covis<<','<<l.lmk_localmap_hash<<','<<l.lmk_num_map<<','<<slam.mpTracker->mCurrentFrame.N<<','<<slam.mpMap->KeyFramesInMap()<<','<<slam.mpMap->MapPointsInMap()<<','<<1000*(l.time_ORB_extraction+l.time_track_motion+l.time_track_frame+l.time_track_map)<<','<<1000*l.time_post_proc<<','<<slam.mpTracker->mReproPoseReadyMs<<'\n';
  if(state==2 && !T.empty()) {cv::Mat R=T.rowRange(0,3).colRange(0,3).t(); cv::Mat t=-R*T.rowRange(0,3).col(3);auto q=ORB_SLAM2::Converter::toQuaternion(R);poses<<i.t<<' '<<t.at<float>(0)<<' '<<t.at<float>(1)<<' '<<t.at<float>(2)<<' '<<q[0]<<' '<<q[1]<<' '<<q[2]<<' '<<q[3]<<'\n';}
  if(k%100==0){csv.flush();poses.flush();std::cerr<<"REPRO_FRAME "<<k<<" state="<<state<<" ms="<<ms<<'\n';}
 }
 csv.close();poses.close();slam.Shutdown();
#ifdef LOCAL_SEARCH_USING_HASHING
 auto d=slam.mpTracker->HashIndexDiagnostics();
 std::ofstream indexlog(out+"/index_diagnostics.json");
 indexlog<<"{\"entries\":"<<d.entries<<",\"max_bucket\":"<<d.max_bucket<<",\"indexed_points\":"<<d.indexed_points
 <<",\"insertions\":"<<d.insertions<<",\"refreshes\":"<<d.refreshes<<",\"evictions\":"<<d.evictions
 <<",\"migrations\":"<<d.migrations<<",\"queries\":"<<d.queries<<"}\n";
#endif
 slam.SaveTrackingLog(out+"/upstream_tracking.txt");
 slam.SaveMappingLog(out+"/upstream_mapping.txt");
 slam.SaveKeyFrameTrajectoryTUM(out+"/keyframes.txt");
 if(rgbd)slam.SaveTrajectoryTUM(out+"/corrected.txt");
 std::cerr<<"REPRO_COMPLETE "<<inputs.size()<<'\n';
 return 0;
}
