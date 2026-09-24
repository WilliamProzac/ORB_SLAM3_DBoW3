// Fixed-state component equivalence: no live map thread or timing-dependent inputs.
#include "FeatureDescriptor.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "CameraModels/Pinhole.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <algorithm>
#include <cmath>

using namespace ORB_SLAM3;
void require(bool value, const char *what) {
  if (!value) throw std::runtime_error(what);
}
int main() {
  try {
    cv::RNG rng(713903);
    int distances=0, queries=0, matches=0, poses=0;
    for (int width : {1, 16, 31, 32, 33, 64}) {
      cv::Mat a(3,width+3,CV_8UC1), b(3,width+5,CV_8UC1);
      for (int n=0;n<1000;++n) {
        rng.fill(a,cv::RNG::UNIFORM,0,256); rng.fill(b,cv::RNG::UNIFORM,0,256);
        cv::Mat x=a(cv::Rect(1,1,width,1)), y=b(cv::Rect(2,1,width,1));
        require(FeatureDescriptorDistance(x,y)==FeatureDescriptorDistanceExact(x,y),"Hamming/ROI mismatch");
        ++distances;
      }
    }
    cv::Mat zeros=cv::Mat::zeros(1,32,CV_8UC1), ones(1,32,CV_8UC1,cv::Scalar(255));
    require(FeatureDescriptorDistanceExact(zeros,ones)==256,"wide reduction overflow");
    require(FeatureDescriptorDistanceExact(ones,ones)==0,"zero distance");
    for(int n=0;n<100;++n) {
      cv::Mat a(1,64,CV_32FC1),b(1,64,CV_32FC1);
      rng.fill(a,cv::RNG::UNIFORM,-1,1);rng.fill(b,cv::RNG::UNIFORM,-1,1);
      require(FeatureDescriptorDistance(a,b)==FeatureDescriptorDistanceExact(a,b),"float fallback mismatch");
    }
    bool rejected=false;
    try {FeatureDescriptorDistanceExact(zeros,cv::Mat());}
    catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,"empty descriptor accepted");
    cv::Mat K=(cv::Mat_<float>(3,3)<<288.13,0,312.32,0,288.13,264.73,0,0,1);
    cv::Mat dist=cv::Mat::zeros(4,1,CV_32FC1);
    Pinhole camera({288.13f,288.13f,312.32f,264.73f});
    ORBextractor extractor(700,1.2f,5,20,7,3);
    for(int scene=0;scene<12;++scene) {
      cv::Mat gray(544,640,CV_8UC1),depth(544,640,CV_32FC1,cv::Scalar(2.0));
      rng.fill(gray,cv::RNG::UNIFORM,0,256);
      Frame base(gray,depth,1.0+scene,&extractor,nullptr,K,dist,17.419f,2.418f,&camera);
      base.SetPose(Sophus::SE3f());
      require(base.N>100,"insufficient synthetic features");
      std::vector<size_t> scratch;
      for(int n=0;n<2000;++n) {
        float x=rng.uniform(-100.f,740.f),y=rng.uniform(-100.f,644.f),radius=rng.uniform(0.f,90.f);
        int min=rng.uniform(-1,5),max=rng.uniform(-1,5);
        auto reference=base.GetFeaturesInArea(x,y,radius,min,max);
        base.GetFeaturesInArea(x,y,radius,scratch,min,max);
        require(reference==scratch,"candidate sequence mismatch");++queries;
      }
      // Exact boundaries and empty windows also retain strict comparisons.
      for (const auto &kp : base.mvKeysUn) {
        for(float radius : {0.f,1.f,8.f}) {
          auto reference=base.GetFeaturesInArea(kp.pt.x+radius,kp.pt.y,radius,-1,-1);
          base.GetFeaturesInArea(kp.pt.x+radius,kp.pt.y,radius,scratch,-1,-1);
          require(reference==scratch,"boundary mismatch");++queries;
        }
      }
      Map map(0);
      KeyFrame key(base,&map,nullptr);
      std::vector<MapPointPtr> owned;
      std::vector<MapPointPtr> points;
      for(int i=0;i<base.N;++i) {
        Eigen::Vector3f position;
        require(base.UnprojectStereo(i,position),"unprojection failed");
        owned.emplace_back(MapPoint::Create(position,&map,&base,i));
        MapPointPtr p=owned.back();p->AddObservation(&key,i);
        p->mbTrackInView=true;p->mbTrackInViewR=false;
        p->mTrackProjX=base.mvKeysUn[i].pt.x;p->mTrackProjY=base.mvKeysUn[i].pt.y;
        p->mTrackProjXR=base.mvuRight[i];p->mnTrackScaleLevel=base.mvKeysUn[i].octave;
        p->mTrackViewCos=1;p->mTrackDepth=2;
        unsigned char snapshot[32];
        require(p->CopyOrbDescriptor(snapshot),"snapshot refused ORB");
        cv::Mat view(1,32,CV_8UC1,snapshot);
        require(cv::norm(view,p->GetDescriptor(),cv::NORM_INF)==0,"snapshot bytes mismatch");
        snapshot[0]^=255;
        require(cv::norm(view,p->GetDescriptor(),cv::NORM_INF)>0,"snapshot aliases map point");
        points.push_back(p);
      }
      Frame last(base);last.mvpMapPoints=points;
      for(int level=1;level<=4;++level) for(int mode=0;mode<2;++mode) {
        Frame ref(base),fast(base);
        ref.mRgbdOptimizationLevel=0;fast.mRgbdOptimizationLevel=level;
        // Deterministic perturbation and existing assignments exercise filtering.
        Sophus::SE3f pose(Eigen::Quaternionf::Identity(),Eigen::Vector3f(.003f*scene,0,0));
        ref.SetPose(pose);fast.SetPose(pose);
        for(int i=0;i<base.N;i+=7)ref.mvpMapPoints[i]=fast.mvpMapPoints[i]=points[i];
        ORBmatcher matcher(.8f);
        int nr=mode?matcher.SearchByProjection(ref,last,3,false):matcher.SearchByProjection(ref,points,3,false,0);
        int nf=mode?matcher.SearchByProjection(fast,last,3,false):matcher.SearchByProjection(fast,points,3,false,0);
        require(nr==nf && ref.mvpMapPoints==fast.mvpMapPoints,"match assignment mismatch");
        require(nr>10,"trivial matcher parity");++matches;
        int ir=Optimizer::PoseOptimization(&ref), iff=Optimizer::PoseOptimization(&fast);
        require(ir==iff && ref.mvbOutlier==fast.mvbOutlier,"pose inlier mismatch");
        require((ref.GetPose().matrix()-fast.GetPose().matrix()).cwiseAbs().maxCoeff()==0,"pose bits differ");
        require(cv::norm(ref.mDescriptors,fast.mDescriptors,cv::NORM_INF)==0,"descriptors changed");
        ++poses;
      }
    }
    std::cout<<"{\"passed\":true,\"distance_cases\":"<<distances
             <<",\"query_cases\":"<<queries<<",\"matcher_cases\":"<<matches
             <<",\"exact_pose_cases\":"<<poses<<"}"<<std::endl;
    return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
}
