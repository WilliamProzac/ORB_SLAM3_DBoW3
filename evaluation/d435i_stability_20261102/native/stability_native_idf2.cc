// Link this TU to the matching complete ORB_SLAM3 DSO. No replacement core
// method, private-access macro, or descriptor/keypoint reconstruction is used.
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include "Frame.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "StabilityDiagnostics.h"
#include "CameraModels/Pinhole.h"
using namespace ORB_SLAM3;
namespace {
void Require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
std::string Read(const std::string& path){std::ifstream f(path);return std::string(
    std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>());}
}
int main(int argc,char** argv) {
 try {
  if(argc!=3)throw std::runtime_error("usage: stability_native disabled|off|on|flush-failure EXISTING_OUTPUT_DIR");
  const std::string mode=argv[1],dir=argv[2];
  Require(mode=="disabled"||mode=="off"||mode=="on"||mode=="flush-failure","bad mode");
  const bool on=mode=="on"||mode=="flush-failure";
  setenv("ORB_RGBD_EAGER_BOW",on?"1":"0",1);
  setenv("ORB_RGBD_STABILITY_DIR",dir.c_str(),1);
  if(mode=="disabled")unsetenv("ORB_RGBD_STABILITY_DIR");
  setenv("ORB_RGBD_STABILITY_WINDOWS","1-1",1);
  Stability::Configure();
  if(mode=="disabled") {
   Require(!Stability::Enabled()&&!Stability::Counts(),"diagnostics disabled state");
   Frame frame;Stability::BeginFrame(frame,0,2.f);Stability::EndFrame(frame,0);
   Require(!Stability::Counts()&&Stability::StopAndFlush(false),"disabled path acquired records");
   Require(!std::ifstream(dir+"/stability_summary.json"),"disabled path created output");
   std::cout<<"PASS disabled actual Frame + disabled sink\n";return 0;
  }
  Require(Stability::Enabled()&&Stability::EagerBoW()==on,"fixed startup configuration");
  // Later environment changes cannot silently change an active System.
  setenv("ORB_RGBD_EAGER_BOW",on?"0":"1",1);Stability::Configure();
  Require(Stability::EagerBoW()==on,"configuration must be immutable");
  ORBVocabulary vocabulary(2,6);
  cv::Mat train(256,32,CV_8U);cv::RNG rng(72491);rng.fill(train,cv::RNG::UNIFORM,0,256);
  // Eight independent documents preserve the production TF_IDF weighting.
  // A single training document gives every seen word log(1/1)==0 and makes
  // transform omit both BowVector and FeatureVector, invalidating readiness.
  std::vector<cv::Mat> training_documents;
  for(int document=0;document<8;++document)
    training_documents.push_back(train.rowRange(document*32,(document+1)*32).clone());
  vocabulary.create(training_documents);
  ORBextractor extractor(700,1.2f,5,20,7);
  Pinhole camera(std::vector<float>{288,288,320,240});
  cv::Mat k=(cv::Mat_<float>(3,3)<<288,0,320,0,288,240,0,0,1);
  cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F),image(480,640,CV_8U);
  rng.fill(image,cv::RNG::UNIFORM,0,256);
  cv::Mat depth(480,640,CV_32F,cv::Scalar(3));
  Frame frame(image,depth,1.,&extractor,&vocabulary,k,distortion,14.4f,2.f,&camera);
  frame.mnDataset=0;frame.Nright=-1;frame.SetPose(Sophus::SE3f());
  Require(frame.N>0,"real RGBD extraction produced no features");
  // Validate the vocabulary with the actual extracted descriptors, without
  // filling Frame::mBowVec/mFeatVec and bypassing the publication switch.
  std::vector<cv::Mat> extracted_descriptors;
  for(int i=0;i<frame.mDescriptors.rows;++i)
    extracted_descriptors.push_back(frame.mDescriptors.row(i));
  DBoW3::BowVector preflight_bow;
  DBoW3::FeatureVector preflight_features;
  vocabulary.transform(extracted_descriptors,preflight_bow,preflight_features,4);
  Require(!preflight_bow.empty()&&!preflight_features.empty(),
      "TF_IDF fixture actual extracted descriptors have no weighted words");
  Map map;KeyFrameDatabase database(vocabulary);
  std::unique_ptr<KeyFrame> key(new KeyFrame(frame,&map,&database));
  const auto identity=Stability::CaptureInput(frame);
  Stability::SetSource(1,1234567890123456789ULL,11,12);
  Stability::BeginFrame(frame,2,2.f);
  Stability::PrepareKeyFrame(key.get(),true);
  Require((!key->mBowVec.empty()&&!key->mFeatVec.empty())==on,"publication readiness switch");
  if(!on)key->ComputeBoW(); // backend baseline, exclusively owned in this fixture
  Require(!key->mBowVec.empty()&&!key->mFeatVec.empty(),
      "TF_IDF fixture requires nonempty BoW/features before concurrent readers");
  const auto expected_bow=key->mBowVec;
  const auto expected_features=key->mFeatVec;
  std::atomic<KeyFrame*> published{nullptr};std::atomic<bool> valid{true};
  std::vector<std::thread> readers;
  for(int n=0;n<3;++n)readers.emplace_back([&]{
   KeyFrame* kf=nullptr;while(!(kf=published.load(std::memory_order_acquire))){}
   for(int i=0;i<1000;++i)if(kf->mBowVec!=expected_bow||kf->mFeatVec!=expected_features)valid=false;
  });
  published.store(key.get(),std::memory_order_release);
  // Actual backend's repeated ComputeBoW is read-only once ready.
  std::thread backend([&]{for(int i=0;i<1000;++i)key->ComputeBoW();});
  backend.join();for(auto& t:readers)t.join();Require(valid,"published BoW changed");
  Frame empty(image,depth,2.,&extractor,&vocabulary,k,distortion,14.4f,2.f,&camera);
  empty.mnDataset=0;empty.Nright=-1;empty.SetPose(Sophus::SE3f());empty.mDescriptors.release();
  KeyFrame empty_key(empty,&map,&database);empty_key.ComputeBoW();
  Require(empty_key.mBowVec.empty()&&empty_key.mFeatVec.empty(),"empty descriptors semantics");
  Frame floating(image,depth,3.,&extractor,&vocabulary,k,distortion,14.4f,2.f,&camera);
  floating.mnDataset=0;floating.Nright=-1;floating.SetPose(Sophus::SE3f());
  floating.mDescriptors.convertTo(floating.mDescriptors,CV_32F);
  KeyFrame floating_key(floating,&map,&database);floating_key.ComputeBoW();
  Require(floating_key.mBowVec.empty()&&floating_key.mFeatVec.empty(),"float descriptors must skip ORB vocabulary");
  auto point=MapPoint::Create(Eigen::Vector3f(0,0,3),key.get(),&map);
  point->AddObservation(key.get(),0);key->AddMapPoint(point,0);map.AddMapPoint(point);
  point->SetBadFlag(LocalPointExit::AgeLowObservation,key->mnId+2);
  // Overflow and concurrent producers exercise the actual bounded sink.
  std::thread producer1([&]{for(int i=0;i<60000;++i)Stability::Life(&map,Stability::Birth,i,0);});
  std::thread producer2([&]{for(int i=0;i<60000;++i)Stability::Life(&map,Stability::RetireRatio,i,0);});
  producer1.join();producer2.join();
  for(int i=0;i<262150;++i)Stability::SearchProjection(i,12.f,14.f);
  // This is the exact capture API used before Tracking::Track. Reset to a real
  // default Frame; no reads of its many unrelated uninitialized scalars.
  Frame reset;
  Require(reset.N==0&&reset.mnId==static_cast<unsigned long>(-1),"default reset identity");
  Require(identity.frame_id==frame.mnId&&identity.keypoints==frame.N,"captured input changed");
  Stability::EndFrame(reset,0);Stability::ClearSource();
  if(mode=="flush-failure"){
   // Caller creates a directory named stability_frames.csv after Configure,
   // so stream-open failure is real and the sink must return false.
   Require(::mkdir((dir+"/stability_frames.csv").c_str(),0755)==0,"create flush failure directory");
   Require(!Stability::StopAndFlush(true),"flush failure was silently accepted");
  }else{
   std::atomic<bool> ready{false},finish{false};
   std::thread late_producer([&]{ready=true;
    while(!finish.load())Stability::Life(&map,Stability::Birth,999999,0);
    Stability::Life(&map,Stability::Birth,999999,0);
   });
   while(!ready.load()){}
   const bool flushed=Stability::StopAndFlush(false);
   finish=true;late_producer.join();Require(flushed,"flush failed");
   const auto csv=Read(dir+"/stability_frames.csv");
   const auto expected=std::string("1,1234567890123456789,11,12,")+std::to_string(identity.frame_id)+","+
       std::to_string(identity.keypoints)+",2,0,0,";
   Require(csv.find(expected)!=std::string::npos,"CSV did not retain input identity/current invalid flag");
   const auto summary=Read(dir+"/stability_summary.json");
   Require(summary.find("\"point_drops\":6")!=std::string::npos,"point capacity drops");
   Require(summary.find("\"lifetime_drops\":0")==std::string::npos,"life capacity did not report drops");
   const auto before=Read(dir+"/stability_lifetime.csv");
   Stability::Life(&map,Stability::Birth,999999,0);
   Require(Stability::StopAndFlush(true),"idempotent flush failed");
   Require(before==Read(dir+"/stability_lifetime.csv"),"append after stop changed sealed output");
  }
  std::cout<<"PASS "<<mode<<" actual Frame/KeyFrame/MapPoint + bounded sink\n";return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
