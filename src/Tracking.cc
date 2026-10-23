#include "MapScaleObservation.h"
#include "FieldObservation.h"
#include "PointTrace.h"
/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Tracking.h"
#include "ImuParameterValidation.h"
#include "LocalMapTiming.h"

#include "Converter.h"
#include "FrameDrawer.h"
#include "G2oTypes.h"
#include "GeometricTools.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Pinhole.h"

#include <iostream>
#include <memory>

#include <chrono>
#include <atomic>
#include <ctime>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <sys/syscall.h>
#include <unistd.h>

using namespace std;

namespace ORB_SLAM3 {
namespace {
std::atomic<bool> rgbd_light_diagnostics{false};
std::atomic<std::size_t> rgbd_observation_capacity{0};
std::atomic<std::uint64_t> rgbd_reset_count{0}, rgbd_active_reset_count{0};
thread_local RgbdObservationContext rgbd_observation_context;
}
void SetRgbdLightDiagnostics(bool enabled) {
  if (enabled && GetRgbdObservationCapacity() == 0)
    throw std::logic_error("set frozen RGB-D observation capacity before enabling L0");
  rgbd_light_diagnostics.store(enabled);
}
bool RgbdLightDiagnostics() { return rgbd_light_diagnostics.load(); }
void SetRgbdObservationCapacity(std::size_t expected_inputs) {
  if (expected_inputs == 0 || expected_inputs > kMaxRgbdObservationInputs)
    throw std::invalid_argument("RGB-D observation capacity must be in 1..8192");
  rgbd_observation_capacity.store(expected_inputs);
}
std::size_t GetRgbdObservationCapacity() { return rgbd_observation_capacity.load(); }
void SetRgbdObservationContext(std::int64_t stamp, std::uint64_t ordinal,
                               bool detail, std::uint64_t run) {
  rgbd_observation_context = {stamp, ordinal, run, true, detail};
}
void ClearRgbdObservationContext() { rgbd_observation_context = {}; }
RgbdObservationContext GetRgbdObservationContext() { return rgbd_observation_context; }
RgbdResetCounters GetRgbdResetCounters() {
  return {rgbd_reset_count.load(), rgbd_active_reset_count.load()};
}
std::uint64_t RgbdLinuxThreadId() {
  const auto tid = syscall(SYS_gettid);
  return tid > 0 ? static_cast<std::uint64_t>(tid) : 0;
}
std::uint64_t RgbdSteadyNanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
double RgbdThreadCpuMilliseconds() {
#if defined(CLOCK_THREAD_CPUTIME_ID)
  timespec now;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now) == 0)
    return double(now.tv_sec) * 1000.0 + double(now.tv_nsec) / 1e6;
#endif
  return -1.0;
}
namespace { FieldSink field_sink=nullptr; void* field_context=nullptr; bool heavy_diagnostics=false; }
void SetFieldSink(FieldSink sink,void*context){field_sink=sink;field_context=context;}
bool FieldEnabled(){return field_sink!=nullptr;}
void SetHeavyDiagnostics(bool enabled){heavy_diagnostics=enabled;}
bool HeavyDiagnostics(){return heavy_diagnostics;}
void FieldEmit(uint16_t kind,uint64_t frame,uint32_t id,uint16_t valid,uint64_t a,uint64_t b,uint64_t c,uint64_t d,uint64_t e){if(field_sink)field_sink(field_context,kind,frame,id,valid,a,b,c,d,e);}
// POINT_TRACE_DRAFT_BEGIN
namespace {
uint64_t point_trace_frame=0,point_trace_ids[32]={};
size_t point_trace_count=0;
uint32_t point_trace_projection=0;
bool point_trace_aborted=false;
}
bool ConfigurePointTrace(uint64_t frame,const uint64_t* ids,size_t count) {
 if(!frame||!ids||!count||count>32)return false;
 for(size_t i=0;i<count;++i)if(i&&ids[i]<=ids[i-1])return false;
 point_trace_frame=frame;point_trace_count=count;point_trace_projection=0;
 point_trace_aborted=false;
 for(size_t i=0;i<count;++i)point_trace_ids[i]=ids[i];
 return true;
}
bool PointTraceEnabled(uint64_t frame) {
 return point_trace_count&&frame==point_trace_frame&&!point_trace_aborted&&FieldEnabled();
}
bool PointTraceSelected(uint64_t id) {
 for(size_t i=0;i<point_trace_count;++i)if(point_trace_ids[i]==id)return true;
 return false;
}
void PointTraceAbort(uint64_t frame,uint64_t reason) {
 if(PointTraceEnabled(frame))FieldEmit(84,frame,0,1,reason);
 point_trace_aborted=true;
}
uint32_t PointTraceProjection(uint64_t frame) {
 if(!PointTraceEnabled(frame))return 0;
 if(point_trace_projection==2){PointTraceAbort(frame,3);return 0;}
 return ++point_trace_projection;
}
void PointTraceSnapshot(uint64_t source,uint32_t phase,const Frame& frame) {
 if(!PointTraceEnabled(source))return;
 if(frame.N<0||frame.N>2048||frame.mvpMapPoints.size()!=size_t(frame.N)||
    frame.mvbOutlier.size()!=size_t(frame.N)){PointTraceAbort(source,2);return;}
 bool found[32]={};size_t records=0,unique=0;
 for(int slot=0;slot<frame.N;++slot) {
  const auto& point=frame.mvpMapPoints[slot]; // borrowed from existing owner
  if(!point)continue;
  for(size_t w=0;w<point_trace_count;++w)if(point->mnId==point_trace_ids[w]) {
   if(!found[w]){found[w]=true;++unique;}
   FieldEmit(70,source,phase,31,point->mnId,slot,
      static_cast<uint64_t>(point->Observations()),frame.mvbOutlier[slot],frame.mnId);
   // Scalar-only pose residual diagnostic. It is emitted only for the explicitly
   // selected points while the test trace is enabled; it never feeds tracking.
   if (frame.HasPose() && frame.mpCamera && static_cast<size_t>(slot) < frame.mvKeysUn.size()) {
    const Eigen::Vector3f x3dc = frame.GetPose() * point->GetWorldPos();
    if (x3dc.z() > 0.0f) {
     const Eigen::Vector2f uv = frame.mpCamera->project(x3dc);
     const cv::Point2f measured = frame.mvKeysUn[slot].pt;
     const float du = uv.x() - measured.x;
     const float dv = uv.y() - measured.y;
     float dr = 0.0f;
     if (static_cast<size_t>(slot) < frame.mvuRight.size() && frame.mvuRight[slot] >= 0.0f)
      dr = uv.x() - frame.mbf / x3dc.z() - frame.mvuRight[slot];
     const int octave = frame.mvKeysUn[slot].octave;
     const float inv_sigma2 = octave >= 0 &&
             static_cast<size_t>(octave) < frame.mvInvLevelSigma2.size()
         ? frame.mvInvLevelSigma2[octave] : 1.0f;
     const float chi2 = (du * du + dv * dv + dr * dr) * inv_sigma2;
     FieldEmit(85,source,phase,31,point->mnId,PointTraceFloat(du),
         PointTraceFloat(dv),PointTraceFloat(dr),PointTraceFloat(chi2));
    }
   }
   ++records;break;
  }
 }
 for(size_t w=0;w<point_trace_count;++w)if(!found[w]) {
  FieldEmit(70,source,phase,17,point_trace_ids[w],0,0,0,frame.mnId);++records;
 }
 FieldEmit(83,source,phase,31,frame.mnId,frame.N,records,unique,point_trace_count);
}
// POINT_TRACE_DRAFT_END

namespace {
int FieldPoseOptimization(Frame* frame, PoseOptimizationObservation* observation = nullptr) {
 const int n=Optimizer::PoseOptimization(frame, nullptr, observation);
 if(FieldEnabled()) {
  FieldEmit(14,frame->mnId,0,1,n);
  uint64_t q[4]={};
  for(int i=0;i<frame->N;++i) if(frame->mvpMapPoints[i]&&!frame->mvbOutlier[i]) {
   const auto &p=frame->mvKeysUn[i].pt;
   ++q[(p.x>=(frame->mnMinX+frame->mnMaxX)/2?1:0)+(p.y>=(frame->mnMinY+frame->mnMaxY)/2?2:0)];
  }
  FieldEmit(21,frame->mnId,0,15,q[0],q[1],q[2],q[3]);
 }
 return n;
}
}


namespace {
class ScopedThreadCpuAccumulator {
public:
  explicit ScopedThreadCpuAccumulator(double* destination)
      : destination_(destination), start_(destination ? RgbdThreadCpuMilliseconds() : -1) {}
  ~ScopedThreadCpuAccumulator() {
    if (!destination_) return;
    const double end = RgbdThreadCpuMilliseconds();
    if (start_ < 0 || end < start_) *destination_ = -1;
    else if (*destination_ >= 0) *destination_ += end - start_;
  }
private:
  double* destination_;
  double start_;
};
class ScopedTimingAccumulator {
public:
  explicit ScopedTimingAccumulator(double *destination)
      : destination_(destination), start_(destination ? std::chrono::steady_clock::now()
                                                     : std::chrono::steady_clock::time_point()) {}
  ~ScopedTimingAccumulator() {
    if (destination_) {
      *destination_ +=
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              std::chrono::steady_clock::now() - start_)
              .count();
    }
  }
private:
  double *destination_;
  std::chrono::steady_clock::time_point start_;
};

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point &start) {
  return std::chrono::duration_cast<
             std::chrono::duration<double, std::milli>>(
             std::chrono::steady_clock::now() - start)
      .count();
}
} // namespace

Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
                   MapDrawer *pMapDrawer, Atlas *pAtlas,
                   KeyFrameDatabase *pKFDB, const string &strSettingPath,
                   const int sensor, Settings *settings, const string &_nameSeq)
    : mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
      mbOnlyTracking(false), mbMapUpdated(false), mbVO(false),
      mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB), mbReadyToInitializate(false),
      mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
      mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas),
      mnLastRelocFrameId(0), time_recently_lost(5.0), mnInitialFrameId(0),
      mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr),
      mpLastKeyFrame(static_cast<KeyFrame *>(NULL)), mbHasLastImuMeas(false),
      mLastImuAcc(Eigen::Vector3f::Zero()),
      mLastImuGyro(Eigen::Vector3f::Zero()), mLastImuStamp(0.0),
      mbCurrentFrameHasValidIMU(false) {
  const bool usesImu = mSensor == System::IMU_MONOCULAR ||
                       mSensor == System::IMU_STEREO ||
                       mSensor == System::IMU_RGBD;
  // Load camera parameters from settings file
  if (settings) {
    newParameterLoader(settings);
  } else {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if (!b_parse_cam) {
      std::cout << "*Error with the camera parameters in the config file*"
                << std::endl;
    }

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if (!b_parse_orb) {
      std::cout << "*Error with the ORB parameters in the config file*"
                << std::endl;
    }

    bool b_parse_imu = true;
    if (usesImu) {
      b_parse_imu = ParseIMUParamFile(fSettings);
      if (!b_parse_imu) {
        std::cout << "*Error with the IMU parameters in the config file*"
                  << std::endl;
      }

    }

    if (!b_parse_cam || !b_parse_orb || !b_parse_imu) {
      std::cerr << "**ERROR in the config file, the format is not correct**"
                << std::endl;
      try {
        throw -1;
      } catch (exception &e) {
      }
    }
  }

  mnFramesToResetIMU = usesImu ? mMaxFrames : 0;

  // Read in both legacy and Settings-based loaders; no effect on other sensors.
  if (mSensor == System::RGBD) {
    cv::FileStorage options(strSettingPath, cv::FileStorage::READ);
    const cv::FileNode level = options["RGBD.ExactOptimizationLevel"];
    if (!level.empty()) {
      if (!level.isInt() || int(level) < 0 || int(level) > 4)
        throw std::invalid_argument("RGBD.ExactOptimizationLevel must be integer 0..4");
      mRgbdOptimizationLevel = int(level);
    }
    std::cout << "RGB-D exact optimization level: " << mRgbdOptimizationLevel
              << " (0=reference,1=grid,2=distance,3=snapshot,4=headless)" << std::endl;
    const auto local_mode = options["RGBD.LocalMapMode"];
    if (!local_mode.empty()) {
      if (!local_mode.isString()) throw std::invalid_argument("RGBD.LocalMapMode must be a string");
      const std::string name = static_cast<std::string>(local_mode);
      if (name == "off") mLocalMapMode = 0;
      else if (name == "full") mLocalMapMode = 1;
      else if (name == "kf") mLocalMapMode = 2;
      else if (name == "shadow") mLocalMapMode = 3;
      else throw std::invalid_argument("RGBD.LocalMapMode must be off/full/kf/shadow");
    }
    const auto local_limit = options["RGBD.LocalMapKFs"];
    if (!local_limit.empty()) {
      if (!local_limit.isInt() || int(local_limit)<1 || int(local_limit)>512)
        throw std::invalid_argument("RGBD.LocalMapKFs must be integer 1..512");
      mLocalMapKeyFrameBudget = int(local_limit);
    }
    const auto local_points = options["RGBD.LocalMapMPs"];
    if (!local_points.empty()) {
      if (!local_points.isInt() || int(local_points)<0 || int(local_points)>100000)
        throw std::invalid_argument("RGBD.LocalMapMPs must be integer 0..100000");
      mLocalMapPointBudget=int(local_points);
    }
    const auto local_statistics = options["RGBD.LocalMapStatistics"];
    if (!local_statistics.empty()) {
      if (!local_statistics.isString()) throw std::invalid_argument("RGBD.LocalMapStatistics must be legacy/attempted");
      const std::string policy=local_statistics;
      if (policy!="legacy" && policy!="attempted") throw std::invalid_argument("RGBD.LocalMapStatistics must be legacy/attempted");
      mLocalMapAttempted=policy=="attempted";
    }
    if (mLocalMapPointBudget && !mLocalMapAttempted)
      throw std::invalid_argument("MP budget requires attempted opportunity statistics");
    const auto local_young = options["RGBD.LocalMapReserveYoung"];
    if (!local_young.empty()) {
      if (!local_young.isInt() || (int(local_young)!=0 && int(local_young)!=1))
        throw std::invalid_argument("RGBD.LocalMapReserveYoung must be 0 or 1");
      mLocalMapReserveYoung=int(local_young)!=0;
    }
    const auto local_strategy = options["RGBD.LocalMapKFStrategy"];
    if (!local_strategy.empty()) {
      if (!local_strategy.isString()) throw std::invalid_argument("RGBD.LocalMapKFStrategy must be votes/coverage");
      const std::string strategy=local_strategy;
      if (strategy!="votes" && strategy!="coverage") throw std::invalid_argument("RGBD.LocalMapKFStrategy must be votes/coverage");
      mLocalMapCoverage=strategy=="coverage";
    }
    const auto local_detailed = options["RGBD.LocalMapDiagnostics"];
    if (!local_detailed.empty()) {
      if (!local_detailed.isInt() || (int(local_detailed)!=0 && int(local_detailed)!=1))
        throw std::invalid_argument("RGBD.LocalMapDiagnostics must be 0 or 1");
      mLocalMapDetailed=int(local_detailed)!=0;
    }
    const auto local_adaptive = options["RGBD.LocalMapAdaptive"];
    if (!local_adaptive.empty()) {
      if (!local_adaptive.isInt() || (int(local_adaptive)!=0 && int(local_adaptive)!=1))
        throw std::invalid_argument("RGBD.LocalMapAdaptive must be 0 or 1");
      mLocalMapAdaptive = int(local_adaptive)!=0;
    }
    const auto selection_log = options["RGBD.LocalMapSelectionLog"];
    if (!selection_log.empty()) {
      if (!selection_log.isString()) throw std::invalid_argument("RGBD.LocalMapSelectionLog must be a path");
      mLocalMapSelectionLog.open(static_cast<std::string>(selection_log));
      if (!mLocalMapSelectionLog) throw std::runtime_error("cannot open RGBD.LocalMapSelectionLog");
      mLocalMapSelectionEnabled=true;
      mLocalMapSelectionLog<<"generation,frame_id,role,axis,bin,eligible,selected,queried,found\n";
    }
    const auto local_log = options["RGBD.LocalMapLog"];
    if (mLocalMapMode && !local_log.empty()) {
      if (!local_log.isString()) throw std::invalid_argument("RGBD.LocalMapLog must be a path string");
      mLocalMapLog.open(static_cast<std::string>(local_log));
      if (!mLocalMapLog) throw std::runtime_error("cannot open RGBD.LocalMapLog");
      mLocalMapLog << "frame_id,map_id,mode,budget_kf,active_kf,active_mp,mandatory,candidate_kf,budget_attempt,budget_inliers,final_inliers,tracking_ok,retry,commit_ok,reason,selection_ms,snapshot_ms,update_ms,search_ms,pose_ms,commit_ms,trial_ms,full_ms,selected_inliers,total_inliers,selected_new_inliers,total_new_inliers,association_inliers,trial_kf,trial_mp,identity_wait_ms,controller_level,attempt_level,probe,budget_low,trial_cells,trial_feature_cells,trial_coverage,final_cells,final_feature_cells,final_coverage,hold_until,cool_until,safety_until,map_generation,event_version,event_kind,budget_failure,health_ms,kf_strategy,coverage_used,support_points,support_cells,mandatory_support,mandatory_cells,coverage_saturation_selected,saturation_slots,saturation_votes,saturation_diversity,saturation_ties,association_lost,association_changed,association_added,pose_delta_m,pose_delta_deg,trial_residual_rms,full_residual_rms,trial_residual_p95,full_residual_p95,trial_information_min,trial_information_max,full_information_min,full_information_max,trial_rank,full_rank,diagnostic_ms,statistics,budget_mp,eligible_mp,selected_mp,protected_mp,mp_truncated,mp_partition_valid,young_reserved,mp_selection_ms,mp_observation_ms,entry_skipped,frustum_rejected,far_excluded,visible_candidates,commit_lock_wait_ms\n";
    }
  }

  initID = 0;
  lastID = 0;
  mbInitWith3KFs = false;
  mnNumDataset = 0;

  vector<GeometricCamera *> vpCams = mpAtlas->GetAllCameras();
  std::cout << "There are " << vpCams.size() << " cameras in the atlas"
            << std::endl;
  for (GeometricCamera *pCam : vpCams) {
    std::cout << "Camera " << pCam->GetId();
    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      std::cout << " is pinhole" << std::endl;
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      std::cout << " is fisheye" << std::endl;
    } else {
      std::cout << " is unknown" << std::endl;
    }
  }

#ifdef REGISTER_TIMES
  vdRectStereo_ms.clear();
  vdResizeImage_ms.clear();
  vdORBExtract_ms.clear();
  vdStereoMatch_ms.clear();
  vdIMUInteg_ms.clear();
  vdPosePred_ms.clear();
  vdLMTrack_ms.clear();
  vdNewKF_ms.clear();
  vdTrackTotal_ms.clear();
#endif
}

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times) {
  double accum = 0;
  for (double value : v_times) {
    accum += value;
  }

  return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average) {
  double accum = 0;
  for (double value : v_times) {
    accum += pow(value - average, 2);
  }
  return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values) {
  double accum = 0;
  int total = 0;
  for (double value : v_values) {
    if (value == 0)
      continue;
    accum += value;
    total++;
  }

  return accum / total;
}

double calcDeviation(vector<int> v_values, double average) {
  double accum = 0;
  int total = 0;
  for (double value : v_values) {
    if (value == 0)
      continue;
    accum += pow(value - average, 2);
    total++;
  }
  return sqrt(accum / total);
}

void Tracking::LocalMapStats2File() {
  ofstream f;
  f.open("LocalMapTimeStats.txt");
  f << fixed << setprecision(6);
  f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF "
       "culling[ms], Total[ms]"
    << endl;
  for (int i = 0; i < mpLocalMapper->vdLMTotal_ms.size(); ++i) {
    f << mpLocalMapper->vdKFInsert_ms[i] << ","
      << mpLocalMapper->vdMPCulling_ms[i] << ","
      << mpLocalMapper->vdMPCreation_ms[i] << ","
      << mpLocalMapper->vdLBASync_ms[i] << ","
      << mpLocalMapper->vdKFCullingSync_ms[i] << ","
      << mpLocalMapper->vdLMTotal_ms[i] << endl;
  }

  f.close();

  f.open("LBA_Stats.txt");
  f << fixed << setprecision(6);
  f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
  for (int i = 0; i < mpLocalMapper->vdLBASync_ms.size(); ++i) {
    f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i]
      << "," << mpLocalMapper->vnLBA_KFfixed[i] << ","
      << mpLocalMapper->vnLBA_MPs[i] << "," << mpLocalMapper->vnLBA_edges[i]
      << endl;
  }

  f.close();
}

void Tracking::TrackStats2File() {
  ofstream f;
  f.open("SessionInfo.txt");
  f << fixed;
  f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
  f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

  f << "OpenCV version: " << CV_VERSION << endl;

  f.close();

  f.open("TrackingTimeStats.txt");
  f << fixed << setprecision(6);

  f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU "
       "preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]"
    << endl;

  for (int i = 0; i < vdTrackTotal_ms.size(); ++i) {
    double stereo_rect = 0.0;
    if (!vdRectStereo_ms.empty()) {
      stereo_rect = vdRectStereo_ms[i];
    }

    double resize_image = 0.0;
    if (!vdResizeImage_ms.empty()) {
      resize_image = vdResizeImage_ms[i];
    }

    double stereo_match = 0.0;
    if (!vdStereoMatch_ms.empty()) {
      stereo_match = vdStereoMatch_ms[i];
    }

    double imu_preint = 0.0;
    if (!vdIMUInteg_ms.empty()) {
      imu_preint = vdIMUInteg_ms[i];
    }

    f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << ","
      << stereo_match << "," << imu_preint << "," << vdPosePred_ms[i] << ","
      << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i]
      << endl;
  }

  f.close();
}

void Tracking::PrintTimeStats() {
  // Save data in files
  TrackStats2File();
  LocalMapStats2File();

  ofstream f;
  f.open("ExecMean.txt");
  f << fixed;
  // Report the mean and std of each one
  std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
  f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
  cout << "OpenCV version: " << CV_VERSION << endl;
  f << "OpenCV version: " << CV_VERSION << endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
  f << "---------------------------" << std::endl;
  f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
  double average, deviation;
  if (!vdRectStereo_ms.empty()) {
    average = calcAverage(vdRectStereo_ms);
    deviation = calcDeviation(vdRectStereo_ms, average);
    std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Stereo Rectification: " << average << "$\\pm$" << deviation
      << std::endl;
  }

  if (!vdResizeImage_ms.empty()) {
    average = calcAverage(vdResizeImage_ms);
    deviation = calcDeviation(vdResizeImage_ms, average);
    std::cout << "Image Resize: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
  }

  average = calcAverage(vdORBExtract_ms);
  deviation = calcDeviation(vdORBExtract_ms, average);
  std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

  if (!vdStereoMatch_ms.empty()) {
    average = calcAverage(vdStereoMatch_ms);
    deviation = calcDeviation(vdStereoMatch_ms, average);
    std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
  }

  if (!vdIMUInteg_ms.empty()) {
    average = calcAverage(vdIMUInteg_ms);
    deviation = calcDeviation(vdIMUInteg_ms, average);
    std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "IMU Preintegration: " << average << "$\\pm$" << deviation
      << std::endl;
  }

  average = calcAverage(vdPosePred_ms);
  deviation = calcDeviation(vdPosePred_ms, average);
  std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdLMTrack_ms);
  deviation = calcDeviation(vdLMTrack_ms, average);
  std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
  f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdNewKF_ms);
  deviation = calcDeviation(vdNewKF_ms, average);
  std::cout << "New KF decision: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdTrackTotal_ms);
  deviation = calcDeviation(vdTrackTotal_ms, average);
  std::cout << "Total Tracking: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

  // Local Mapping time stats
  std::cout << std::endl << std::endl << std::endl;
  std::cout << "Local Mapping" << std::endl << std::endl;
  f << std::endl << "Local Mapping" << std::endl << std::endl;

  average = calcAverage(mpLocalMapper->vdKFInsert_ms);
  deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
  std::cout << "KF Insertion: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdMPCulling_ms);
  deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
  std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
  f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdMPCreation_ms);
  deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
  std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
  f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdLBA_ms);
  deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
  std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
  f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdKFCulling_ms);
  deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
  std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
  f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdLMTotal_ms);
  deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
  std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

  // Local Mapping LBA complexity
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
  f << "---------------------------" << std::endl;
  f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

  average = calcAverage(mpLocalMapper->vnLBA_edges);
  deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
  std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
  f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vnLBA_KFopt);
  deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
  std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
  deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
  std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vnLBA_MPs);
  deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
  std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
            << std::endl;
  f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

  std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
  std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
  f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
  f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

  // Map complexity
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "Map complexity" << std::endl;
  std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
  std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
  f << "---------------------------" << std::endl;
  f << std::endl << "Map complexity" << std::endl;
  vector<Map *> vpMaps = mpAtlas->GetAllMaps();
  Map *pBestMap = vpMaps[0];
  for (int i = 1; i < vpMaps.size(); ++i) {
    if (pBestMap->GetAllKeyFrames().size() <
        vpMaps[i]->GetAllKeyFrames().size()) {
      pBestMap = vpMaps[i];
    }
  }

  f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
  f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

  f << "---------------------------" << std::endl;
  f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
  average = calcAverage(mpLoopClosing->vdDataQuery_ms);
  deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
  f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Database Query: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vdEstSim3_ms);
  deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
  f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vdPRTotal_ms);
  deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
  f << "Total Place Recognition: " << average << "$\\pm$" << deviation
    << std::endl
    << std::endl;
  std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
  average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
  deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
  f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
  deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
  f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Essential Graph: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
  deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
  f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
  std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
  average = calcAverage(mpLoopClosing->vnLoopKFs);
  deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;

  f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
  average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
  deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
  f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
  deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
  f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
  deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
  f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
  deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
  f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
  std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
  average = calcAverage(mpLoopClosing->vnMergeKFs);
  deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vnMergeMPs);
  deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
  f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
            << std::endl;

  f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
  average = calcAverage(mpLoopClosing->vdGBA_ms);
  deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
  f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
  deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
  f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
  deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
  f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
  std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
  f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
  std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
  average = calcAverage(mpLoopClosing->vnGBAKFs);
  deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vnGBAMPs);
  deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
  f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
            << std::endl;

  f.close();
}

#endif

Tracking::~Tracking() {
  StopAndJoinExtractorWorkers();
  // f_track_stats.close();
}

void Tracking::StopAndJoinExtractorWorkers() {
  if (mpORBextractorLeft) mpORBextractorLeft->StopAndJoinWorkers();
  if (mpORBextractorRight) mpORBextractorRight->StopAndJoinWorkers();
  if (mpIniORBextractor) mpIniORBextractor->StopAndJoinWorkers();
}

void Tracking::newParameterLoader(Settings *settings) {
  mpCamera = settings->camera1();
  mpCamera = mpAtlas->AddCamera(mpCamera);

  if (settings->needToUndistort()) {
    mDistCoef = settings->camera1DistortionCoef();
  } else {
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  }

  // TODO: missing image scaling and rectification
  mImageScale = 1.0f;

  mK = cv::Mat::eye(3, 3, CV_32F);
  mK.at<float>(0, 0) = mpCamera->getParameter(0);
  mK.at<float>(1, 1) = mpCamera->getParameter(1);
  mK.at<float>(0, 2) = mpCamera->getParameter(2);
  mK.at<float>(1, 2) = mpCamera->getParameter(3);

  mK_.setIdentity();
  mK_(0, 0) = mpCamera->getParameter(0);
  mK_(1, 1) = mpCamera->getParameter(1);
  mK_(0, 2) = mpCamera->getParameter(2);
  mK_(1, 2) = mpCamera->getParameter(3);

  if ((mSensor == System::STEREO || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      settings->cameraType() == Settings::KannalaBrandt) {
    mpCamera2 = settings->camera2();
    mpCamera2 = mpAtlas->AddCamera(mpCamera2);

    mTlr = settings->Tlr();

    mpFrameDrawer->both = true;
  }

  if (mSensor == System::STEREO || mSensor == System::RGBD ||
      mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
    mbf = settings->bf();
    mThDepth = settings->b() * settings->thDepth();
  }

  if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
    mDepthMapFactor = settings->depthMapFactor();
    if (fabs(mDepthMapFactor) < 1e-5)
      mDepthMapFactor = 1;
    else
      mDepthMapFactor = 1.0f / mDepthMapFactor;
  }

  mMinFrames = 0;
  mMaxFrames = settings->fps();
  mbRGB = settings->rgb();

  // ORB parameters
  int nFeatures = settings->nFeatures();
  int nLevels = settings->nLevels();
  int fIniThFAST = settings->initThFAST();
  int fMinThFAST = settings->minThFAST();
  float fScaleFactor = settings->scaleFactor();
  int orbParallelWorkers = settings->orbParallelWorkers();

  mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                        fIniThFAST, fMinThFAST,
                                        orbParallelWorkers);

  if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                           fIniThFAST, fMinThFAST,
                                           orbParallelWorkers);

  if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels,
                                         fIniThFAST, fMinThFAST,
                                         orbParallelWorkers);

  // IMU settings are only populated for inertial sensors.
  if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
      mSensor == System::IMU_RGBD) {
    const Sophus::SE3f Tbc = settings->Tbc();
    const bool insertKFsLost = settings->insertKFsWhenLost();
    const float imuFrequency = settings->imuFrequency();
    const double imuPeriod = (imuFrequency > 1e-3f)
        ? (1.0 / static_cast<double>(imuFrequency)) : 0.001;
    imu_parameter_validation::ScaledNoise noise;
    std::string error;
    if (!imu_parameter_validation::ScaleNoise(
            imuFrequency, settings->noiseGyro(), settings->noiseAcc(),
            settings->gyroWalk(), settings->accWalk(), noise, error))
      throw std::invalid_argument(error);

    std::unique_ptr<IMU::Calib> calib(new IMU::Calib(
        Tbc, noise.gyro, noise.acc, noise.gyroWalk, noise.accWalk));
    std::unique_ptr<IMU::Preintegrated> preintegrated(
        new IMU::Preintegrated(IMU::Bias(), *calib));
    // Publish only a complete pair; preserve the existing runtime ownership.
    mInsertKFsLost = insertKFsLost;
    mImuFreq = imuFrequency;
    mImuPer = imuPeriod;
    mpImuCalib = calib.release();
    mpImuPreintegratedFromLastKF = preintegrated.release();
  }
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings) {
  mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  cout << endl << "Camera Parameters: " << endl;
  bool b_miss_params = false;

  string sCameraName = fSettings["Camera.type"];
  if (sCameraName == "PinHole") {
    float fx, fy, cx, cy;
    mImageScale = 1.f;

    // Camera calibration parameters
    cv::FileNode node = fSettings["Camera.fx"];
    if (!node.empty() && node.isReal()) {
      fx = node.real();
    } else {
      std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.fy"];
    if (!node.empty() && node.isReal()) {
      fy = node.real();
    } else {
      std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.cx"];
    if (!node.empty() && node.isReal()) {
      cx = node.real();
    } else {
      std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.cy"];
    if (!node.empty() && node.isReal()) {
      cy = node.real();
    } else {
      std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    // Distortion parameters
    node = fSettings["Camera.k1"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.at<float>(0) = node.real();
    } else {
      std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.k2"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.at<float>(1) = node.real();
    } else {
      std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.p1"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.at<float>(2) = node.real();
    } else {
      std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.p2"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.at<float>(3) = node.real();
    } else {
      std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.k3"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.resize(5);
      mDistCoef.at<float>(4) = node.real();
    }

    node = fSettings["Camera.imageScale"];
    if (!node.empty() && node.isReal()) {
      mImageScale = node.real();
    }

    if (b_miss_params) {
      return false;
    }

    if (mImageScale != 1.f) {
      // K matrix parameters must be scaled.
      fx = fx * mImageScale;
      fy = fy * mImageScale;
      cx = cx * mImageScale;
      cy = cy * mImageScale;
    }

    vector<float> vCamCalib{fx, fy, cx, cy};

    mpCamera = new Pinhole(vCamCalib);

    mpCamera = mpAtlas->AddCamera(mpCamera);

    std::cout << "- Camera: Pinhole" << std::endl;
    std::cout << "- Image scale: " << mImageScale << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
    std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;

    std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
    std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

    if (mDistCoef.rows == 5)
      std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

    mK = cv::Mat::eye(3, 3, CV_32F);
    mK.at<float>(0, 0) = fx;
    mK.at<float>(1, 1) = fy;
    mK.at<float>(0, 2) = cx;
    mK.at<float>(1, 2) = cy;

    mK_.setIdentity();
    mK_(0, 0) = fx;
    mK_(1, 1) = fy;
    mK_(0, 2) = cx;
    mK_(1, 2) = cy;
  } else if (sCameraName == "KannalaBrandt8") {
    float fx, fy, cx, cy;
    float k1, k2, k3, k4;
    mImageScale = 1.f;

    // Camera calibration parameters
    cv::FileNode node = fSettings["Camera.fx"];
    if (!node.empty() && node.isReal()) {
      fx = node.real();
    } else {
      std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }
    node = fSettings["Camera.fy"];
    if (!node.empty() && node.isReal()) {
      fy = node.real();
    } else {
      std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.cx"];
    if (!node.empty() && node.isReal()) {
      cx = node.real();
    } else {
      std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.cy"];
    if (!node.empty() && node.isReal()) {
      cy = node.real();
    } else {
      std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    // Distortion parameters
    node = fSettings["Camera.k1"];
    if (!node.empty() && node.isReal()) {
      k1 = node.real();
    } else {
      std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }
    node = fSettings["Camera.k2"];
    if (!node.empty() && node.isReal()) {
      k2 = node.real();
    } else {
      std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.k3"];
    if (!node.empty() && node.isReal()) {
      k3 = node.real();
    } else {
      std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.k4"];
    if (!node.empty() && node.isReal()) {
      k4 = node.real();
    } else {
      std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.imageScale"];
    if (!node.empty() && node.isReal()) {
      mImageScale = node.real();
    }

    if (!b_miss_params) {
      if (mImageScale != 1.f) {
        // K matrix parameters must be scaled.
        fx = fx * mImageScale;
        fy = fy * mImageScale;
        cx = cx * mImageScale;
        cy = cy * mImageScale;
      }

      vector<float> vCamCalib{fx, fy, cx, cy, k1, k2, k3, k4};
      mpCamera = new KannalaBrandt8(vCamCalib);
      mpCamera = mpAtlas->AddCamera(mpCamera);
      std::cout << "- Camera: Fisheye" << std::endl;
      std::cout << "- Image scale: " << mImageScale << std::endl;
      std::cout << "- fx: " << fx << std::endl;
      std::cout << "- fy: " << fy << std::endl;
      std::cout << "- cx: " << cx << std::endl;
      std::cout << "- cy: " << cy << std::endl;
      std::cout << "- k1: " << k1 << std::endl;
      std::cout << "- k2: " << k2 << std::endl;
      std::cout << "- k3: " << k3 << std::endl;
      std::cout << "- k4: " << k4 << std::endl;

      mK = cv::Mat::eye(3, 3, CV_32F);
      mK.at<float>(0, 0) = fx;
      mK.at<float>(1, 1) = fy;
      mK.at<float>(0, 2) = cx;
      mK.at<float>(1, 2) = cy;

      mK_.setIdentity();
      mK_(0, 0) = fx;
      mK_(1, 1) = fy;
      mK_(0, 2) = cx;
      mK_(1, 2) = cy;
    }

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD) {
      // Right camera
      // Camera calibration parameters
      cv::FileNode node = fSettings["Camera2.fx"];
      if (!node.empty() && node.isReal()) {
        fx = node.real();
      } else {
        std::cerr
            << "*Camera2.fx parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }
      node = fSettings["Camera2.fy"];
      if (!node.empty() && node.isReal()) {
        fy = node.real();
      } else {
        std::cerr
            << "*Camera2.fy parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      node = fSettings["Camera2.cx"];
      if (!node.empty() && node.isReal()) {
        cx = node.real();
      } else {
        std::cerr
            << "*Camera2.cx parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      node = fSettings["Camera2.cy"];
      if (!node.empty() && node.isReal()) {
        cy = node.real();
      } else {
        std::cerr
            << "*Camera2.cy parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      // Distortion parameters
      node = fSettings["Camera2.k1"];
      if (!node.empty() && node.isReal()) {
        k1 = node.real();
      } else {
        std::cerr
            << "*Camera2.k1 parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }
      node = fSettings["Camera2.k2"];
      if (!node.empty() && node.isReal()) {
        k2 = node.real();
      } else {
        std::cerr
            << "*Camera2.k2 parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      node = fSettings["Camera2.k3"];
      if (!node.empty() && node.isReal()) {
        k3 = node.real();
      } else {
        std::cerr
            << "*Camera2.k3 parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      node = fSettings["Camera2.k4"];
      if (!node.empty() && node.isReal()) {
        k4 = node.real();
      } else {
        std::cerr
            << "*Camera2.k4 parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      int leftLappingBegin = -1;
      int leftLappingEnd = -1;

      int rightLappingBegin = -1;
      int rightLappingEnd = -1;

      node = fSettings["Camera.lappingBegin"];
      if (!node.empty() && node.isInt()) {
        leftLappingBegin = node.operator int();
      } else {
        std::cout << "WARNING: Camera.lappingBegin not correctly defined"
                  << std::endl;
      }
      node = fSettings["Camera.lappingEnd"];
      if (!node.empty() && node.isInt()) {
        leftLappingEnd = node.operator int();
      } else {
        std::cout << "WARNING: Camera.lappingEnd not correctly defined"
                  << std::endl;
      }
      node = fSettings["Camera2.lappingBegin"];
      if (!node.empty() && node.isInt()) {
        rightLappingBegin = node.operator int();
      } else {
        std::cout << "WARNING: Camera2.lappingBegin not correctly defined"
                  << std::endl;
      }
      node = fSettings["Camera2.lappingEnd"];
      if (!node.empty() && node.isInt()) {
        rightLappingEnd = node.operator int();
      } else {
        std::cout << "WARNING: Camera2.lappingEnd not correctly defined"
                  << std::endl;
      }

      node = fSettings["Tlr"];
      cv::Mat cvTlr;
      if (!node.empty()) {
        cvTlr = node.mat();
        if (cvTlr.rows != 3 || cvTlr.cols != 4) {
          std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*"
                    << std::endl;
          b_miss_params = true;
        }
      } else {
        std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
        b_miss_params = true;
      }

      if (!b_miss_params) {
        if (mImageScale != 1.f) {
          // K matrix parameters must be scaled.
          fx = fx * mImageScale;
          fy = fy * mImageScale;
          cx = cx * mImageScale;
          cy = cy * mImageScale;

          leftLappingBegin = leftLappingBegin * mImageScale;
          leftLappingEnd = leftLappingEnd * mImageScale;
          rightLappingBegin = rightLappingBegin * mImageScale;
          rightLappingEnd = rightLappingEnd * mImageScale;
        }

        static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[0] =
            leftLappingBegin;
        static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[1] =
            leftLappingEnd;

        mpFrameDrawer->both = true;

        vector<float> vCamCalib2{fx, fy, cx, cy, k1, k2, k3, k4};
        mpCamera2 = new KannalaBrandt8(vCamCalib2);
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = Converter::toSophus(cvTlr);

        static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[0] =
            rightLappingBegin;
        static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[1] =
            rightLappingEnd;

        std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", "
                  << leftLappingEnd << std::endl;

        std::cout << std::endl << "Camera2 Parameters:" << std::endl;
        std::cout << "- Camera: Fisheye" << std::endl;
        std::cout << "- Image scale: " << mImageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << k1 << std::endl;
        std::cout << "- k2: " << k2 << std::endl;
        std::cout << "- k3: " << k3 << std::endl;
        std::cout << "- k4: " << k4 << std::endl;

        std::cout << "- mTlr: \n" << cvTlr << std::endl;

        std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", "
                  << rightLappingEnd << std::endl;
      }
    }

    if (b_miss_params) {
      return false;
    }

  } else {
    std::cerr << "*Not Supported Camera Sensor*" << std::endl;
    std::cerr << "Check an example configuration file with the desired sensor"
              << std::endl;
  }

  if (mSensor == System::STEREO || mSensor == System::RGBD ||
      mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
    cv::FileNode node = fSettings["Camera.bf"];
    if (!node.empty() && node.isReal()) {
      mbf = node.real();
      if (mImageScale != 1.f) {
        mbf *= mImageScale;
      }
    } else {
      std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }
  }

  float fps = fSettings["Camera.fps"];
  if (fps == 0)
    fps = 30;

  // Max/Min Frames to insert keyframes and to check relocalisation
  mMinFrames = 0;
  mMaxFrames = fps;

  cout << "- fps: " << fps << endl;

  int nRGB = fSettings["Camera.RGB"];
  mbRGB = nRGB;

  if (mbRGB)
    cout << "- color order: RGB (ignored if grayscale)" << endl;
  else
    cout << "- color order: BGR (ignored if grayscale)" << endl;

  if (mSensor == System::STEREO || mSensor == System::RGBD ||
      mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
    float fx = mpCamera->getParameter(0);
    cv::FileNode node = fSettings["ThDepth"];
    if (!node.empty() && node.isReal()) {
      mThDepth = node.real();
      mThDepth = mbf * mThDepth / fx;
      cout << endl
           << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    } else {
      std::cerr << "*ThDepth parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }
  }

  if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
    cv::FileNode node = fSettings["DepthMapFactor"];
    if (!node.empty() && node.isReal()) {
      mDepthMapFactor = node.real();
      if (fabs(mDepthMapFactor) < 1e-5)
        mDepthMapFactor = 1;
      else
        mDepthMapFactor = 1.0f / mDepthMapFactor;
    } else {
      std::cerr
          << "*DepthMapFactor parameter doesn't exist or is not a real number*"
          << std::endl;
      b_miss_params = true;
    }
  }

  if (b_miss_params) {
    return false;
  }

  return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings) {
  bool b_miss_params = false;
  int nFeatures, nLevels, fIniThFAST, fMinThFAST;
  int orbParallelWorkers = 1;
  float fScaleFactor;

  cv::FileNode node = fSettings["ORBextractor.nFeatures"];
  if (!node.empty() && node.isInt()) {
    nFeatures = node.operator int();
  } else {
    std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an "
                 "integer*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.scaleFactor"];
  if (!node.empty() && node.isReal()) {
    fScaleFactor = node.real();
  } else {
    std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not "
                 "a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.nLevels"];
  if (!node.empty() && node.isInt()) {
    nLevels = node.operator int();
  } else {
    std::cerr
        << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.iniThFAST"];
  if (!node.empty() && node.isInt()) {
    fIniThFAST = node.operator int();
  } else {
    std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an "
                 "integer*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.minThFAST"];
  if (!node.empty() && node.isInt()) {
    fMinThFAST = node.operator int();
  } else {
    std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an "
                 "integer*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.parallelWorkers"];
  if (!node.empty()) {
    if (node.isInt()) {
      orbParallelWorkers = node.operator int();
      if (orbParallelWorkers < 1 || orbParallelWorkers > 4) {
        std::cerr << "*ORBextractor.parallelWorkers must be in [1, 4]*"
                  << std::endl;
        b_miss_params = true;
      }
    } else {
      std::cerr << "*ORBextractor.parallelWorkers is not an integer*"
                << std::endl;
      b_miss_params = true;
    }
  }

  if (b_miss_params) {
    return false;
  }

  mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                        fIniThFAST, fMinThFAST,
                                        orbParallelWorkers);

  if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                           fIniThFAST, fMinThFAST,
                                           orbParallelWorkers);

  if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels,
                                         fIniThFAST, fMinThFAST,
                                         orbParallelWorkers);

  cout << endl << "ORB Extractor Parameters: " << endl;
  cout << "- Number of Features: " << nFeatures << endl;
  cout << "- Scale Levels: " << nLevels << endl;
  cout << "- Scale Factor: " << fScaleFactor << endl;
  cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
  cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
  cout << "- Parallel Workers: " << orbParallelWorkers << endl;

  return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings) {
  bool b_miss_params = false;
  std::string error;
  float imuFrequency = 0.0f;
  bool insertKFsLost = true;
  bool fastInit = false;

  cv::Mat cvTbc;
  cv::FileNode node = fSettings["Tbc"];
  if (!node.empty()) {
    cvTbc = node.mat();
    if (cvTbc.rows != 4 || cvTbc.cols != 4) {
      std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*"
                << std::endl;
      b_miss_params = true;
    }
  } else {
    std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
    b_miss_params = true;
  }
  // Reject invalid storage or geometry before ptr<float>() / Sophus conversion.
  if (b_miss_params ||
      !imu_parameter_validation::ValidateTransform(cvTbc, error)) {
    if (!error.empty()) std::cerr << "*" << error << "*" << std::endl;
    return false;
  }

  node = fSettings["InsertKFsWhenLost"];
  if (!node.empty() && node.isInt())
    insertKFsLost = static_cast<bool>(static_cast<int>(node));

  float Ng, Na, Ngw, Naw;

  node = fSettings["IMU.Frequency"];
  if (!node.empty() && node.isInt()) {
    imuFrequency = node.operator int();
  } else {
    std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.NoiseGyro"];
  if (!node.empty() && node.isReal()) {
    Ng = node.real();
  } else {
    std::cerr
        << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.NoiseAcc"];
  if (!node.empty() && node.isReal()) {
    Na = node.real();
  } else {
    std::cerr
        << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.GyroWalk"];
  if (!node.empty() && node.isReal()) {
    Ngw = node.real();
  } else {
    std::cerr
        << "*IMU.GyroWalk parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.AccWalk"];
  if (!node.empty() && node.isReal()) {
    Naw = node.real();
  } else {
    std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.fastInit"];
  if (!node.empty()) {
    fastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
  }

  if (b_miss_params) {
    return false;
  }

  imu_parameter_validation::ScaledNoise noise;
  if (!imu_parameter_validation::ScaleNoise(
          imuFrequency, Ng, Na, Ngw, Naw, noise, error)) {
    std::cerr << "*" << error << "*" << std::endl;
    return false;
  }
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
  const Sophus::SE3f Tbc(eigTbc);
  const double imuPeriod = (imuFrequency > 1e-3f)
      ? (1.0 / static_cast<double>(imuFrequency)) : 0.001;

  cout << endl;
  cout << "Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
  if (!insertKFsLost)
    cout << "Do not insert keyframes when lost visual tracking " << endl;
  if (fastInit)
    cout << "Fast IMU initialization. Acceleration is not checked \n";
  cout << "IMU frequency: " << imuFrequency << " Hz" << endl;
  cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
  cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
  cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
  cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

  std::unique_ptr<IMU::Calib> calib(new IMU::Calib(
      Tbc, noise.gyro, noise.acc, noise.gyroWalk, noise.accWalk));
  std::unique_ptr<IMU::Preintegrated> preintegrated(
      new IMU::Preintegrated(IMU::Bias(), *calib));
  mInsertKFsLost = insertKFsLost;
  mImuFreq = imuFrequency;
  mImuPer = imuPeriod;
  mFastInit = fastInit;
  mpImuCalib = calib.release();
  mpImuPreintegratedFromLastKF = preintegrated.release();
  return true;
}

bool Tracking::ShouldCreateNewKeyFrame(bool bNeedKF, bool bOK) const {
  const bool usesImu = mSensor == System::IMU_MONOCULAR ||
                       mSensor == System::IMU_STEREO ||
                       mSensor == System::IMU_RGBD;
  return bNeedKF &&
         (bOK || (usesImu && mInsertKFsLost && mState == RECENTLY_LOST));
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer) { mpViewer = pViewer; }

void Tracking::SetStereoDepthProvider(
    std::shared_ptr<StereoDepthProvider> provider) {
  mpStereoDepthProvider = std::move(provider);
}

void Tracking::SetStereoFeatureProvider(
    std::shared_ptr<StereoFeatureProvider> provider) {
  mpStereoFeatureProvider = std::move(provider);
}

void Tracking::SetReferenceKeyFrame(KeyFrame *pKF) {
  if (pKF && !pKF->isBad()) {
    mpReferenceKF = pKF;
    mpLastKeyFrame = pKF;

    if (mbOnlyTracking) {
      // In localization mode, force LOST so the system relocalizes immediately
      // instead of blindly assuming it starts at the reference KF.
      (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
      cout << "Tracking: Initialized with reference KF " << pKF->mnId
           << " but force-set state to LOST to trigger Relocalization" << endl;
    } else {
      // Set state to OK so tracking can start directly with loaded map (normal
      // mapping continuation)
      (mState = OK, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
      cout << "Tracking: Initialized with reference KF " << pKF->mnId << endl;
    }
  }
}

void Tracking::SetStepByStep(bool bSet) { bStepByStep = bSet; }

bool Tracking::GetStepByStep() { return bStepByStep; }

Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft,
                                       const cv::Mat &imRectRight,
                                       const double &timestamp,
                                       string filename) {
  // cout << "GrabImageStereo" << endl;

  mImGray = imRectLeft;
  cv::Mat imGrayRight = imRectRight;
  mImRight = imRectRight;

  if (mImGray.channels() == 3) {
    // cout << "Image with 3 channels" << endl;
    if (mbRGB) {
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
    } else {
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
    }
  } else if (mImGray.channels() == 4) {
    // cout << "Image with 4 channels" << endl;
    if (mbRGB) {
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
    } else {
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
    }
  }

  // cout << "Incoming frame creation" << endl;

  if (mSensor == System::STEREO && !mpCamera2)
    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft,
                          mpORBextractorRight, mpORBVocabulary, mK, mDistCoef,
                          mbf, mThDepth, mpCamera,
                          mpStereoDepthProvider.get(), nullptr, IMU::Calib(),
                          mpStereoFeatureProvider.get());
  else if (mSensor == System::STEREO && mpCamera2)
    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft,
                          mpORBextractorRight, mpORBVocabulary, mK, mDistCoef,
                          mbf, mThDepth, mpCamera, mpCamera2, mTlr);
  else if (mSensor == System::IMU_STEREO && !mpCamera2)
    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft,
                          mpORBextractorRight, mpORBVocabulary, mK, mDistCoef,
                          mbf, mThDepth, mpCamera,
                          mpStereoDepthProvider.get(), &mLastFrame,
                          *mpImuCalib, mpStereoFeatureProvider.get());
  else if (mSensor == System::IMU_STEREO && mpCamera2)
    mCurrentFrame =
        Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft,
              mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf,
              mThDepth, mpCamera, mpCamera2, mTlr, &mLastFrame, *mpImuCalib);

  // cout << "Incoming frame ended" << endl;

  mCurrentFrame.mNameFile = filename;
  mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
  vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

  // cout << "Tracking start" << endl;
  Track();
  // cout << "Tracking end" << endl;

  return mCurrentFrame.GetPose();
}

Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD,
                                     const double &timestamp, string filename,
                                     RgbdTimingRecord *timing) {
  mpCurrentRgbdTiming = timing;
  if (timing && RgbdLightDiagnostics()) {
    timing->l0_collected = true;
    timing->observation = GetRgbdObservationContext();
    timing->tracking_thread_id = RgbdLinuxThreadId();
  }
  if (timing) {
    timing->image_timestamp = timestamp;
    timing->state_before = static_cast<int>(mState);
    timing->local_mapping_queue_before = mpLocalMapper->KeyframesInQueue();
  }

  const std::chrono::steady_clock::time_point gray_start =
      std::chrono::steady_clock::now();
  mImGray = imRGB;
  cv::Mat imDepth = imD;

  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }
  if (timing)
    timing->gray_convert_ms = ElapsedMilliseconds(gray_start);

  const std::chrono::steady_clock::time_point depth_start =
      std::chrono::steady_clock::now();
  if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);
  if (timing)
    timing->depth_convert_ms = ElapsedMilliseconds(depth_start);

  const double frame_cpu_start = timing && timing->l0_collected
      ? RgbdThreadCpuMilliseconds() : -1.0;
  const std::chrono::steady_clock::time_point frame_start =
      std::chrono::steady_clock::now();
  if (mSensor == System::RGBD)
    mCurrentFrame =
        Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary,
              mK, mDistCoef, mbf, mThDepth, mpCamera);
  else if (mSensor == System::IMU_RGBD)
    mCurrentFrame =
        Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary,
              mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
  mCurrentFrame.mRgbdOptimizationLevel = mRgbdOptimizationLevel;
  mCurrentFrame.mUseLocalMapProjection = mSensor == System::RGBD && mLocalMapMode != 0;
  if (timing) {
    timing->frame_construct_ms = ElapsedMilliseconds(frame_start);
    if (timing->l0_collected) {
      const double frame_cpu_end = RgbdThreadCpuMilliseconds();
      timing->frame_construct_cpu_ms = frame_cpu_start < 0.0 || frame_cpu_end < frame_cpu_start
          ? -1.0 : frame_cpu_end - frame_cpu_start;
    }
    timing->orb = mCurrentFrame.mRgbdOrbTiming;
    timing->frame_id = mCurrentFrame.mnId;
    timing->keypoints = mCurrentFrame.N;
    timing->valid_depths = static_cast<int>(std::count_if(
        mCurrentFrame.mvDepth.begin(), mCurrentFrame.mvDepth.end(),
        [](float depth) { return depth > 0.0f; }));
    timing->orb_extract_ms = mCurrentFrame.mTimeRgbdOrbExtract;
    timing->orb_pyramid_ms = mCurrentFrame.mTimeRgbdOrbPyramid;
    timing->orb_keypoints_octree_ms =
        mCurrentFrame.mTimeRgbdOrbKeypointsOctree;
    timing->orb_orientation_ms = mCurrentFrame.mTimeRgbdOrbOrientation;
    timing->orb_blur_ms = mCurrentFrame.mTimeRgbdOrbBlur;
    timing->orb_descriptor_ms = mCurrentFrame.mTimeRgbdOrbDescriptor;
    timing->orb_result_assembly_ms =
        mCurrentFrame.mTimeRgbdOrbResultAssembly;
    timing->orb_fast_initial_ms = mCurrentFrame.mTimeRgbdOrbFastInitial;
    timing->orb_fast_fallback_ms = mCurrentFrame.mTimeRgbdOrbFastFallback;
    timing->orb_octree_ms = mCurrentFrame.mTimeRgbdOrbOctree;
    timing->orb_copy_ms = mCurrentFrame.mTimeRgbdOrbCopy;
    timing->orb_gaussian_ms = mCurrentFrame.mTimeRgbdOrbGaussian;
    timing->orb_output_hash = mCurrentFrame.mRgbdOrbOutputHash;
    timing->orb_mono_index = mCurrentFrame.mRgbdOrbMonoIndex;
    timing->orb_level_count = mCurrentFrame.mRgbdOrbLevelCount;
    timing->orb_level_fast_initial_ms =
        mCurrentFrame.mRgbdOrbLevelFastInitial;
    timing->orb_level_fast_fallback_ms =
        mCurrentFrame.mRgbdOrbLevelFastFallback;
    timing->orb_level_octree_ms = mCurrentFrame.mRgbdOrbLevelOctree;
    timing->orb_level_orientation_ms =
        mCurrentFrame.mRgbdOrbLevelOrientation;
    timing->orb_level_copy_ms = mCurrentFrame.mRgbdOrbLevelCopy;
    timing->orb_level_gaussian_ms = mCurrentFrame.mRgbdOrbLevelGaussian;
    timing->orb_level_descriptor_ms = mCurrentFrame.mRgbdOrbLevelDescriptor;
    timing->orb_level_candidates = mCurrentFrame.mRgbdOrbLevelCandidates;
    timing->orb_level_fallback_cells =
        mCurrentFrame.mRgbdOrbLevelFallbackCells;
    timing->orb_level_octree_splits =
        mCurrentFrame.mRgbdOrbLevelOctreeSplits;
    timing->rgbd_depth_assoc_ms = mCurrentFrame.mTimeRgbdDepthAssoc;
  }

  if(FieldEnabled()) {
    uint64_t valid=0,near=0,far=0;
    for(float d:mCurrentFrame.mvDepth) if(std::isfinite(d)&&d>0) {++valid;if(d<mThDepth)++near;else ++far;}
    FieldEmit(20,mCurrentFrame.mnId,0,31,mCurrentFrame.N,valid,near,far,mState);
  }
  mCurrentFrame.mNameFile = filename;
  mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

  const std::chrono::steady_clock::time_point track_start =
      std::chrono::steady_clock::now();
  Track();
  if (auto* scale = ActiveMapScaleObservation()) {
    scale->frame_id = mCurrentFrame.mnId;
    scale->frame_keypoints = mCurrentFrame.N;
    scale->tracking_state = static_cast<int>(mState);
  }
  if (timing) {
    timing->track_internal_ms = ElapsedMilliseconds(track_start);
    timing->state_after = static_cast<int>(mState);
    timing->tracked_inliers = mnMatchesInliers;
    timing->local_mapping_queue_after = mpLocalMapper->KeyframesInQueue();
  }
  mpCurrentRgbdTiming = nullptr;

  return mCurrentFrame.GetPose();
}

Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im,
                                          const double &timestamp,
                                          string filename) {
  mImGray = im;
  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  if (mSensor == System::MONOCULAR) {
    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET ||
        (lastID - initID) < mMaxFrames)
      mCurrentFrame =
          Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary,
                mpCamera, mDistCoef, mbf, mThDepth);
    else
      mCurrentFrame =
          Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary,
                mpCamera, mDistCoef, mbf, mThDepth);
  } else if (mSensor == System::IMU_MONOCULAR) {
    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET) {
      mCurrentFrame =
          Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary,
                mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
    } else
      mCurrentFrame =
          Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary,
                mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
  }

  if (mState == NO_IMAGES_YET)
    t0 = timestamp;

  mCurrentFrame.mNameFile = filename;
  mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

  lastID = mCurrentFrame.mnId;
  Track();

  return mCurrentFrame.GetPose();
}

void Tracking::GrabImuData(const IMU::Point &imuMeasurement) {
  unique_lock<mutex> lock(mMutexImuQueue);
  mlQueueImuData.push_back(imuMeasurement);
  mbHasLastImuMeas = true;
  mLastImuAcc = imuMeasurement.a;
  mLastImuGyro = imuMeasurement.w;
  mLastImuStamp = imuMeasurement.t;
}

void Tracking::PreintegrateIMU() {
  mbCurrentFrameHasValidIMU = false;

  if (!mCurrentFrame.mpPrevFrame) {
    Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
    mCurrentFrame.setIntegrated();
    return;
  }

  mvImuFromLastFrame.clear();
  mvImuFromLastFrame.reserve(mlQueueImuData.size());
  if (mlQueueImuData.size() == 0) {
    Verbose::PrintMess("Not IMU data in mlQueueImuData!!",
                       Verbose::VERBOSITY_NORMAL);
  }

  while (true) {
    bool bSleep = false;
    {
      unique_lock<mutex> lock(mMutexImuQueue);
      if (!mlQueueImuData.empty()) {
        IMU::Point *m = &mlQueueImuData.front();
        cout.precision(17);
        if (m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer) {
          mlQueueImuData.pop_front();
        } else if (m->t < mCurrentFrame.mTimeStamp - mImuPer) {
          mvImuFromLastFrame.push_back(*m);
          mlQueueImuData.pop_front();
        } else {
          mvImuFromLastFrame.push_back(*m);
          break;
        }
      } else {
        break;
        bSleep = true;
      }
    }
    if (bSleep)
      usleep(500);
  }

  const int n = static_cast<int>(mvImuFromLastFrame.size()) - 1;
  if (n <= 0) {
    IMU::Preintegrated *pImuPreintegratedFromLastFrame =
        new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

    if (!mpImuPreintegratedFromLastKF) {
      mpImuPreintegratedFromLastKF =
          new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);
    }

    Eigen::Vector3f acc = Eigen::Vector3f::Zero();
    Eigen::Vector3f angVel = Eigen::Vector3f::Zero();
    bool bHasFallbackMeas = false;

    if (!mvImuFromLastFrame.empty()) {
      acc = mvImuFromLastFrame.back().a;
      angVel = mvImuFromLastFrame.back().w;
      bHasFallbackMeas = true;
    } else {
      unique_lock<mutex> lock(mMutexImuQueue);
      if (mbHasLastImuMeas) {
        acc = mLastImuAcc;
        angVel = mLastImuGyro;
        bHasFallbackMeas = true;
      }
    }

    float tstep =
        mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
    if (tstep > 0.f && bHasFallbackMeas) {
      mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
      pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel,
                                                              tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;
    mCurrentFrame.setIntegrated();
    mbCurrentFrameHasValidIMU = false;

    cout << "Empty IMU measurements vector -> fallback preintegration used.\n";
    return;
  }

  IMU::Preintegrated *pImuPreintegratedFromLastFrame =
      new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

  for (int i = 0; i < n; i++) {
    float tstep;
    Eigen::Vector3f acc, angVel;
    if ((i == 0) && (i < (n - 1))) {
      float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
      float tini =
          mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
      acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
             (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) *
                 (tini / tab)) *
            0.5f;
      angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) *
                    (tini / tab)) *
               0.5f;
      tstep =
          mvImuFromLastFrame[i + 1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
    } else if (i < (n - 1)) {
      acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
      angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
      tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
    } else if ((i > 0) && (i == (n - 1))) {
      float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
      float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
      acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
             (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) *
                 (tend / tab)) *
            0.5f;
      angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) *
                    (tend / tab)) *
               0.5f;
      tstep = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
    } else if ((i == 0) && (i == (n - 1))) {
      acc = mvImuFromLastFrame[i].a;
      angVel = mvImuFromLastFrame[i].w;
      tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
    }

    if (!mpImuPreintegratedFromLastKF)
      cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
    mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
    pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
  }

  mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
  mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
  mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

  mCurrentFrame.setIntegrated();
  mbCurrentFrameHasValidIMU = true;

  // Verbose::PrintMess("Preintegration is finished!! ",
  // Verbose::VERBOSITY_DEBUG);
}

bool Tracking::PredictStateIMU() {
  if (!mbCurrentFrameHasValidIMU) {
    Verbose::PrintMess(
        "PredictStateIMU skipped: fallback IMU for current frame",
        Verbose::VERBOSITY_DEBUG);
    return false;
  }

  if (!mCurrentFrame.mpPrevFrame) {
    Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
    return false;
  }

  if (mbMapUpdated && mpLastKeyFrame) {
    const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const float t12 = mpImuPreintegratedFromLastKF->dT;

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(
                   mpLastKeyFrame->GetImuBias()));
    Eigen::Vector3f twb2 =
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(
                   mpLastKeyFrame->GetImuBias());
    Eigen::Vector3f Vwb2 =
        Vwb1 + t12 * Gz +
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(
                   mpLastKeyFrame->GetImuBias());
    mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

    mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
    mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
    return true;
  } else if (!mbMapUpdated) {
    const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
    const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(
                   mLastFrame.mImuBias));
    Eigen::Vector3f twb2 =
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(
                   mLastFrame.mImuBias);
    Eigen::Vector3f Vwb2 =
        Vwb1 + t12 * Gz +
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(
                   mLastFrame.mImuBias);

    mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

    mCurrentFrame.mImuBias = mLastFrame.mImuBias;
    mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
    return true;
  } else
    cout << "not IMU prediction!!" << endl;

  return false;
}

void Tracking::ResetFrameIMU() {
  // TODO To implement...
}

void Tracking::Track() {
  // Debug: Track entry state
  // cout << "Track: mState=" << mState << ", mbOnlyTracking=" << mbOnlyTracking
  //      << ", Features=" << mCurrentFrame.mvKeys.size() << endl;

  if (bStepByStep) {
    std::cout << "Tracking: Waiting to the next step" << std::endl;
    while (!mbStep && bStepByStep)
      usleep(500);
    mbStep = false;
  }

  if (mpLocalMapper->mbBadImu) {
    cout << "TRACK: Reset map because local mapper set the bad imu flag "
         << endl;
    mpSystem->ResetActiveMap();
    return;
  }

  Map *pCurrentMap = mpAtlas->GetCurrentMap();
  if (!pCurrentMap) {
    cout << "ERROR: There is not an active map in the atlas" << endl;
  }

  if (mState != NO_IMAGES_YET) {
    if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp) {
      cerr
          << "ERROR: Frame with a timestamp older than previous frame detected!"
          << endl;
      unique_lock<mutex> lock(mMutexImuQueue);
      mlQueueImuData.clear();
      CreateMapInAtlas();
      return;
    } else if (mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0) {
      // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp <<
      // endl; cout << "id last: " << mLastFrame.mnId << "    id curr: " <<
      // mCurrentFrame.mnId << endl;
      if (mpAtlas->isInertial()) {
        // In localization mode, don't reset map on timestamp jump
        // Just continue trying to relocalize
        if (mbOnlyTracking) {
          static bool bFirstJump = true;
          if (bFirstJump) {
            cout << "Timestamp jump detected in localization mode. Will "
                    "attempt relocalization."
                 << endl;
            bFirstJump = false;
          }
          (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
          // Update mLastFrame timestamp to current so we don't keep detecting
          // jumps
          mLastFrame.mTimeStamp = mCurrentFrame.mTimeStamp;
          // Don't return - continue to try relocalization
        } else if (mpAtlas->isImuInitialized()) {
          cout << "Timestamp jump detected. State set to LOST. Reseting IMU "
                  "integration..."
               << endl;
          if (!pCurrentMap->GetIniertialBA2()) {
            mpSystem->ResetActiveMap();
          } else {
            CreateMapInAtlas();
          }
          return;
        } else {
          cout << "Timestamp jump detected, before IMU initialization. "
                  "Reseting..."
               << endl;
          mpSystem->ResetActiveMap();
          return;
        }
      }
    }
  }

  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      mpLastKeyFrame)
    mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

  if (mState == NO_IMAGES_YET) {
    (mState = NOT_INITIALIZED, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
  }

  mLastProcessedState = mState;

  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      !mbCreatedMap) {
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartPreIMU =
        std::chrono::steady_clock::now();
#endif
    PreintegrateIMU();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndPreIMU =
        std::chrono::steady_clock::now();

    double timePreImu =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndPreIMU - time_StartPreIMU)
            .count();
    vdIMUInteg_ms.push_back(timePreImu);
#endif
  }
  mbCreatedMap = false;

  // Get Map Mutex -> Map cannot be changed
  const bool field_measure_map_wait = FieldEnabled();
  const std::chrono::steady_clock::time_point map_lock_start =
      std::chrono::steady_clock::now();
  MapScaleSpan scale_map_wait(MapScaleStage::MapUpdateWait);
  unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);
  scale_map_wait.Finish();
  // Capture immediately after lock acquisition: later map reads are excluded.
  // Reuse this endpoint for both sinks; diagnostics-off adds no clock read.
  const std::chrono::steady_clock::time_point map_lock_acquired =
      (mpCurrentRgbdTiming || field_measure_map_wait)
          ? std::chrono::steady_clock::now()
          : std::chrono::steady_clock::time_point{};
  if (mpCurrentRgbdTiming)
    mpCurrentRgbdTiming->map_mutex_wait_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            map_lock_acquired - map_lock_start).count();

  if (auto* scale = ActiveMapScaleObservation()) {
    MapScaleSpan snapshot(MapScaleStage::ScaleSnapshot);
    long unsigned int keyframes = 0, points = 0;
    scale->scale_snapshot_mono_ns = MapScaleMonotonicNanoseconds();
    pCurrentMap->GetScaleObservationCounts(keyframes, points);
    scale->global_kf = keyframes;
    scale->global_mp = points;
    scale->map_id = pCurrentMap->GetId();
  }

  mbMapUpdated = false;

  int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
  int nMapChangeIndex = pCurrentMap->GetLastMapChange();
  if (nCurMapChangeIndex > nMapChangeIndex) {
    pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
    mbMapUpdated = true;
  }
// FIELD_MAP_BEGIN
  if(field_measure_map_wait) FieldEmit(33,mCurrentFrame.mnId,0,15,
      pCurrentMap->GetId(),nCurMapChangeIndex,mbMapUpdated,
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
          map_lock_acquired - map_lock_start).count()));
// FIELD_MAP_END


  if (mState == NOT_INITIALIZED) {
    if (mSensor == System::STEREO || mSensor == System::RGBD ||
        mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
      StereoInitialization();
    } else {
      MonocularInitialization();
    }

    // mpFrameDrawer->Update(this);

    if (mState != OK) // If rightly initialized, mState=OK
    {
      mLastFrame = Frame(mCurrentFrame);
      return;
    }

    if (mpAtlas->GetAllMaps().size() == 1) {
      mnFirstFrameId = mCurrentFrame.mnId;
    }
  } else {
    // System is initialized. Track Frame.
    bool bOK;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartPosePred =
        std::chrono::steady_clock::now();
#endif

    // Initial camera pose estimation using motion model or relocalization (if
    // tracking is lost)
    if (!mbOnlyTracking) {

      // State OK
      // Local Mapping is activated. This is the normal behaviour, unless
      // you explicitly activate the "only tracking" mode.
      if (mState == OK) {

        // Local Mapping might have changed some MapPoints tracked in last frame
        CheckReplacedInLastFrame();

        if ((!mbVelocity && !pCurrentMap->isImuInitialized()) ||
            mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
          Verbose::PrintMess("TRACK: Track with respect to the reference KF ",
                             Verbose::VERBOSITY_DEBUG);
          bOK = TrackReferenceKeyFrame();
        } else {
          Verbose::PrintMess("TRACK: Track with motion model",
                             Verbose::VERBOSITY_DEBUG);
          bOK = TrackWithMotionModel();
          if (!bOK)
            bOK = TrackReferenceKeyFrame();
        }

        if (!bOK) {
          if ((mSensor == System::IMU_MONOCULAR ||
               mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
              mCurrentFrame.mnId <= (mnLastRelocFrameId + mnFramesToResetIMU)) {
            (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
          } else if (pCurrentMap->KeyFramesInMap() > 10) {
            // cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
            (mState = RECENTLY_LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
            mTimeStampLost = mCurrentFrame.mTimeStamp;
          } else {
            (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
          }
        }
      } else {

        if (mState == RECENTLY_LOST) {
          Verbose::PrintMess("Lost for a short time",
                             Verbose::VERBOSITY_NORMAL);

          bOK = true;
          if ((mSensor == System::IMU_MONOCULAR ||
               mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)) {
            if (pCurrentMap->isImuInitialized())
              bOK = PredictStateIMU();
            else
              bOK = false;

            if (mCurrentFrame.mTimeStamp - mTimeStampLost >
                time_recently_lost) {
              (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
              Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
              bOK = false;
            }
          } else {
            // F23 float descriptors have no vocabulary until F24. During the
            // short RECENTLY_LOST window, recover against the existing
            // reference keyframe with direct descriptor matching and Fine
            // refinement instead of entering the ORB BoW relocalizer, which
            // cannot produce candidates for an empty float-descriptor BoW.
            if (mCurrentFrame.mDescriptors.type() == CV_32FC1 &&
                mpStereoFeatureProvider) {
              bOK = TrackReferenceKeyFrame();
            } else {
              bOK = Relocalization();
            }
            // std::cout << "mCurrentFrame.mTimeStamp:" <<
            // to_string(mCurrentFrame.mTimeStamp) << std::endl; std::cout <<
            // "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
            if (mCurrentFrame.mTimeStamp - mTimeStampLost > 3.0f && !bOK) {
              (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
              Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
              bOK = false;
            }
          }
        } else if (mState == LOST) {

          Verbose::PrintMess("A new map is started...",
                             Verbose::VERBOSITY_NORMAL);

          if (pCurrentMap->KeyFramesInMap() < 10) {
            mpSystem->ResetActiveMap();
            Verbose::PrintMess("Reseting current map...",
                               Verbose::VERBOSITY_NORMAL);
          } else
            CreateMapInAtlas();

          if (mpLastKeyFrame)
            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

          Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

          return;
        }
      }

    } else {
      // Localization Mode: Local Mapping is deactivated (TODO Not available in
      // inertial mode)
      if (mState == LOST) {
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
            mSensor == System::IMU_RGBD)
          Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
        bOK = Relocalization();
      } else {
        if (!mbVO) {
          // In last frame we tracked enough MapPoints in the map
          if (mbVelocity) {
            bOK = TrackWithMotionModel();
          } else {
            bOK = TrackReferenceKeyFrame();
          }
        } else {
          // In last frame we tracked mainly "visual odometry" points.

          // We compute two camera poses, one from motion model and one doing
          // relocalization. If relocalization is sucessfull we choose that
          // solution, otherwise we retain the "visual odometry" solution.

          bool bOKMM = false;
          bool bOKReloc = false;
          vector<MapPointPtr > vpMPsMM;
          vector<bool> vbOutMM;
          Sophus::SE3f TcwMM;
          if (mbVelocity) {
            bOKMM = TrackWithMotionModel();
            vpMPsMM = mCurrentFrame.mvpMapPoints;
            vbOutMM = mCurrentFrame.mvbOutlier;
            TcwMM = mCurrentFrame.GetPose();
          }
          bOKReloc = Relocalization();

          if (bOKMM && !bOKReloc) {
            mCurrentFrame.SetPose(TcwMM);
            mCurrentFrame.mvpMapPoints = vpMPsMM;
            mCurrentFrame.mvbOutlier = vbOutMM;

            if (mbVO) {
              for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] &&
                    !mCurrentFrame.mvbOutlier[i]) {
                  mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                }
              }
            }
          } else if (bOKReloc) {
            mbVO = false;
          }

          bOK = bOKReloc || bOKMM;
        }
      }
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndPosePred =
        std::chrono::steady_clock::now();

    double timePosePred =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndPosePred - time_StartPosePred)
            .count();
    vdPosePred_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartLMTrack =
        std::chrono::steady_clock::now();
#endif
    // If we have an initial estimation of the camera pose and matching. Track
    // the local map.
    if (!mbOnlyTracking) {
      if (bOK) {
        bOK = TrackLocalMap(pCurrentMap);
      }
      if (!bOK)
        cout << "Fail to track local map!" << endl;
    } else {
      // mbVO true means that there are few matches to MapPoints in the map. We
      // cannot retrieve a local map and therefore we do not perform
      // TrackLocalMap(). Once the system relocalizes the camera we will use the
      // local map again.
      if (bOK && !mbVO)
        bOK = TrackLocalMap(pCurrentMap);
    }

    if (bOK)
      (mState = OK, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
    else if (mState == OK) {
      if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
          mSensor == System::IMU_RGBD) {
        Verbose::PrintMess("Track lost for less than one second...",
                           Verbose::VERBOSITY_NORMAL);
        if (!pCurrentMap->isImuInitialized() ||
            !pCurrentMap->GetIniertialBA2()) {
          cout << "IMU is not or recently initialized. Reseting active map..."
               << endl;
          mpSystem->ResetActiveMap();
        }

        (mState = RECENTLY_LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
      } else
        (mState = RECENTLY_LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState)); // visual to lost

      /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
      {*/
      mTimeStampLost = mCurrentFrame.mTimeStamp;
      //}
    }

    // Save frame if recent relocalization, since they are used for IMU reset
    // (as we are making copy, it shluld be once mCurrFrame is completely
    // modified)
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD) &&
        !mbOnlyTracking &&
        (mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) &&
        (mCurrentFrame.mnId > mnFramesToResetIMU) &&
        pCurrentMap->isImuInitialized() && !mLastFrame.mK.empty()) {
      // TODO check this situation
      Verbose::PrintMess("Saving pointer to frame. imu needs reset...",
                         Verbose::VERBOSITY_NORMAL);
      Frame *pF = new Frame(mCurrentFrame);
      pF->mpPrevFrame = new Frame(mLastFrame);

      // Load preintegration if available.
      if (mCurrentFrame.mpImuPreintegratedFrame) {
        pF->mpImuPreintegratedFrame =
            new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
      } else {
        pF->mpImuPreintegratedFrame = nullptr;
      }
    }

    if (pCurrentMap->isImuInitialized()) {
      if (bOK) {
        if (mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU)) {
          // In localization mode, skip IMU reset as we use the loaded map's IMU
          // state
          if (!mbOnlyTracking) {
            cout << "RESETING FRAME!!!" << endl;
            ResetFrameIMU();
          } else {
            cout << "Localization mode: Skipping IMU frame reset after "
                    "relocalization"
                 << endl;
          }
        } else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
          mLastBias = mCurrentFrame.mImuBias;
      }
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndLMTrack =
        std::chrono::steady_clock::now();

    double timeLMTrack =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndLMTrack - time_StartLMTrack)
            .count();
    vdLMTrack_ms.push_back(timeLMTrack);
#endif

    // Update drawer
    if (mRgbdOptimizationLevel < 4 || mpViewer)
      mpFrameDrawer->Update(this);
    if (mCurrentFrame.isSet())
      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

    if (bOK || mState == RECENTLY_LOST) {
      // Update motion model
      if (mLastFrame.isSet() && mCurrentFrame.isSet()) {
        // Right after relocalization, the pose jump can make a stale/large
        // velocity model that destabilizes pure localization.
        if (mCurrentFrame.mnId <= mnLastRelocFrameId + 5) {
          if (mbOnlyTracking) {
            // Pure localization needs projection matching immediately after
            // relocalization. Keep an identity model for the accepted frame,
            // then update it from consecutive successful poses.
            if (mCurrentFrame.mnId == mnLastRelocFrameId)
              mVelocity = Sophus::SE3f();
            else
              mVelocity =
                  mCurrentFrame.GetPose() * mLastFrame.GetPose().inverse();
            mbVelocity = true;
          } else {
            mbVelocity = false;
          }
        } else {
          Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
          mVelocity = mCurrentFrame.GetPose() * LastTwc;
          mbVelocity = true;
        }
      } else {
        mbVelocity = false;
      }

      if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
          mSensor == System::IMU_RGBD)
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

      if(PointTraceEnabled(mCurrentFrame.mnId))
        PointTraceSnapshot(mCurrentFrame.mnId,6,mCurrentFrame);
      // Clean VO matches
      for (int i = 0; i < mCurrentFrame.N; i++) {
        MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
          if (pMP->Observations() < 1) {
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.mvpMapPoints[i] = MapPointPtr();
          }
      }

      // Drop temporary ownership; current/last frame snapshots can still use
      // these points until their own handles are released.
      mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_StartNewKF =
          std::chrono::steady_clock::now();
#endif
      bool bNeedKF = NeedNewKeyFrame();

      // Check if we need to insert a new keyframe
      // if(bNeedKF && bOK)
      if (ShouldCreateNewKeyFrame(bNeedKF, bOK))
        CreateNewKeyFrame();
      else if (bNeedKF && RgbdLightDiagnostics() && mpCurrentRgbdTiming)
        mpCurrentRgbdTiming->keyframe_decision.create_result =
            KeyFrameCreateResult::RejectedByTrackingState;

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_EndNewKF =
          std::chrono::steady_clock::now();

      double timeNewKF =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndNewKF - time_StartNewKF)
              .count();
      vdNewKF_ms.push_back(timeNewKF);
#endif

      // We allow points with high innovation (considererd outliers by the Huber
      // Function) pass to the new keyframe, so that bundle adjustment will
      // finally decide if they are outliers or not. We don't want next frame to
      // estimate its position with those points so we discard them in the
      // frame. Only has effect if lastframe is tracked
      for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
          mCurrentFrame.mvpMapPoints[i] = MapPointPtr();
      }
    }

    // Reset if the camera get lost soon after initialization
    if (mState == LOST) {
      if (pCurrentMap->KeyFramesInMap() <= 10) {
        mpSystem->ResetActiveMap();
        return;
      }
      if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
          mSensor == System::IMU_RGBD)
        if (!pCurrentMap->isImuInitialized()) {
          Verbose::PrintMess(
              "Track lost before IMU initialisation, reseting...",
              Verbose::VERBOSITY_QUIET);
          mpSystem->ResetActiveMap();
          return;
        }

      CreateMapInAtlas();

      return;
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    if(PointTraceEnabled(mCurrentFrame.mnId))
      PointTraceSnapshot(mCurrentFrame.mnId,7,mCurrentFrame);
    mLastFrame = Frame(mCurrentFrame);
    if(PointTraceEnabled(mCurrentFrame.mnId))
      PointTraceSnapshot(mCurrentFrame.mnId,8,mLastFrame);
  }

  if (mState == OK || mState == RECENTLY_LOST) {
    // Store frame pose information to retrieve the complete camera trajectory
    // afterwards.
    if (mCurrentFrame.isSet()) {
      Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() *
                          mCurrentFrame.mpReferenceKF->GetPoseInverse();
      mlRelativeFramePoses.push_back(Tcr_);
      mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
      mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
      mlbLost.push_back(mState == LOST);
    } else {
      // This can happen if tracking is lost
      mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
      mlpReferences.push_back(mlpReferences.back());
      mlFrameTimes.push_back(mlFrameTimes.back());
      mlbLost.push_back(mState == LOST);
    }
  }

#ifdef REGISTER_LOOP
  if (Stop()) {

    // Safe area to stop
    while (isStopped()) {
      usleep(3000);
    }
  }
#endif
}

void Tracking::StereoInitialization() {
  ScopedTimingAccumulator detailed_timing(
      mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->initialization_ms : nullptr);
  // In pure localization mode, skip initialization entirely so that the
  // Relocalization() path is triggered on the very first frame.  Without
  // this guard the system forcibly assigns the first frame as the world
  // origin (Tcw = Identity), which only works when the camera happens to
  // be at the map's original starting point.
  if (mbOnlyTracking) {
    cout << "Localization mode: Skipping StereoInitialization, setting LOST "
            "to trigger Relocalization."
         << endl;
    (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
    return;
  }

  if (mCurrentFrame.N > 500) {
    // RGB-D initialization publishes KF/MP references before enqueueing.
    // Protect this producer like CreateNewKeyFrame, including new-map startup.
    LocalMapping* rgbd_init_mapper = mSensor == System::RGBD ? mpLocalMapper : nullptr;
    if (rgbd_init_mapper && !rgbd_init_mapper->SetNotStop(true)) return;
    struct RgbdInitializationStopGuard {
      LocalMapping* mapper;
      ~RgbdInitializationStopGuard() { if (mapper) mapper->SetNotStop(false); }
    } rgbd_init_stop_guard{rgbd_init_mapper};
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
      if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated) {
        cout << "not IMU meas" << endl;
        return;
      }

      if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA -
                         mLastFrame.mpImuPreintegratedFrame->avgA)
                                .norm() < 0.5) {
        cout << "not enough acceleration" << endl;
        return;
      }

      if (mpImuPreintegratedFromLastKF)
        delete mpImuPreintegratedFromLastKF;

      mpImuPreintegratedFromLastKF =
          new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
      mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    }

    // Set Frame pose to the origin (In case of inertial SLAM to imu)
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
      Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
      Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
      Eigen::Vector3f Vwb0;
      Vwb0.setZero();
      mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
    } else
      mCurrentFrame.SetPose(Sophus::SE3f());

    // Create KeyFrame
    KeyFrame *pKFini =
        new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    // Insert KeyFrame in the map
    mpAtlas->AddKeyFrame(pKFini);

    // Create MapPoints and asscoiate to KeyFrame
    if (!mpCamera2) {
      for (int i = 0; i < mCurrentFrame.N; i++) {
        float z = mCurrentFrame.mvDepth[i];
        if (z > 0) {
          Eigen::Vector3f x3D;
          mCurrentFrame.UnprojectStereo(i, x3D);
          MapPointPtr pNewMP =
              MapPoint::Create(x3D, pKFini, mpAtlas->GetCurrentMap());
          pNewMP->AddObservation(pKFini, i);
          pKFini->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpAtlas->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
        }
      }
    } else {
      for (int i = 0; i < mCurrentFrame.Nleft; i++) {
        int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
        if (rightIndex != -1) {
          Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

          MapPointPtr pNewMP =
              MapPoint::Create(x3D, pKFini, mpAtlas->GetCurrentMap());

          pNewMP->AddObservation(pKFini, i);
          pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

          pKFini->AddMapPoint(pNewMP, i);
          pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpAtlas->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
          mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
        }
      }
    }

    Verbose::PrintMess("New Map created with " +
                           to_string(mpAtlas->MapPointsInMap()) + " points",
                       Verbose::VERBOSITY_QUIET);

    // cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

    mpLocalMapper->InsertKeyFrame(pKFini);

    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;
    // mnLastRelocFrameId = mCurrentFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

    (mState = OK, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
  }
}

void Tracking::MonocularInitialization() {
  // Same guard as StereoInitialization: in pure localization mode skip
  // monocular initialization and go directly to LOST so Relocalization()
  // is triggered on the first frame.
  if (mbOnlyTracking) {
    cout << "Localization mode: Skipping MonocularInitialization, setting LOST "
            "to trigger Relocalization."
         << endl;
    (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));
    return;
  }

  if (!mbReadyToInitializate) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size() > 100) {

      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

      if (mSensor == System::IMU_MONOCULAR) {
        if (mpImuPreintegratedFromLastKF) {
          delete mpImuPreintegratedFromLastKF;
        }
        mpImuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
      }

      mbReadyToInitializate = true;

      return;
    }
  } else {
    if (((int)mCurrentFrame.mvKeys.size() <= 100) ||
        ((mSensor == System::IMU_MONOCULAR) &&
         (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0))) {
      mbReadyToInitializate = false;

      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9, true);
    int nmatches = matcher.SearchForInitialization(
        mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Check if there are enough correspondences
    if (nmatches < 100) {
      mbReadyToInitializate = false;
      return;
    }

    Sophus::SE3f Tcw;
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if (mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,
                                          mCurrentFrame.mvKeysUn, mvIniMatches,
                                          Tcw, mvIniP3D, vbTriangulated)) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i] = -1;
          nmatches--;
        }
      }

      // Set Frame Poses
      mInitialFrame.SetPose(Sophus::SE3f());
      mCurrentFrame.SetPose(Tcw);

      CreateInitialMapMonocular();
    }
  }
}

void Tracking::CreateInitialMapMonocular() {
  // Create KeyFrames
  KeyFrame *pKFini =
      new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
  KeyFrame *pKFcur =
      new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  if (mSensor == System::IMU_MONOCULAR)
    pKFini->mpImuPreintegrated = (IMU::Preintegrated *)(NULL);

  pKFini->ComputeBoW();
  pKFcur->ComputeBoW();

  // Insert KFs in the map
  mpAtlas->AddKeyFrame(pKFini);
  mpAtlas->AddKeyFrame(pKFcur);

  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0)
      continue;

    // Create MapPoint.
    Eigen::Vector3f worldPos;
    worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
    MapPointPtr pMP = MapPoint::Create(worldPos, pKFcur, mpAtlas->GetCurrentMap());

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, mvIniMatches[i]);

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    // Fill Current Frame structure
    mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    // Add to Map
    mpAtlas->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  std::set<MapPointPtr > sMPs;
  sMPs = pKFini->GetMapPoints();

  // Bundle Adjustment
  Verbose::PrintMess("New Map created with " +
                         to_string(mpAtlas->MapPointsInMap()) + " points",
                     Verbose::VERBOSITY_QUIET);
  Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth;
  if (mSensor == System::IMU_MONOCULAR)
    invMedianDepth = 4.0f / medianDepth; // 4.0f
  else
    invMedianDepth = 1.0f / medianDepth;

  if (medianDepth < 0 ||
      pKFcur->TrackedMapPoints(1) < 50) // TODO Check, originally 100 tracks
  {
    Verbose::PrintMess("Wrong initialization, reseting...",
                       Verbose::VERBOSITY_QUIET);
    mpSystem->ResetActiveMap();
    return;
  }

  // Scale initial baseline
  Sophus::SE3f Tc2w = pKFcur->GetPose();
  Tc2w.translation() *= invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  vector<MapPointPtr > vpAllMapPoints = pKFini->GetMapPointMatches();
  for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
    if (vpAllMapPoints[iMP]) {
      MapPointPtr pMP = vpAllMapPoints[iMP];
      pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
      pMP->UpdateNormalAndDepth();
    }
  }

  if (mSensor == System::IMU_MONOCULAR) {
    pKFcur->mPrevKF = pKFini;
    pKFini->mNextKF = pKFcur;
    pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(
        pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
  }

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);
  mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFcur;
  // mnLastRelocFrameId = mInitialFrame.mnId;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  // Compute here initial velocity
  vector<KeyFrame *> vKFs = mpAtlas->GetAllKeyFrames();

  Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
  mbVelocity = false;
  Eigen::Vector3f phi = deltaT.so3().log();

  double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) /
               (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
  phi *= aux;

  mLastFrame = Frame(mCurrentFrame);

  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

  mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

  mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

  (mState = OK, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));

  initID = pKFcur->mnId;
}

void Tracking::CreateMapInAtlas() {
  ResetLocalMapBudgetHistory();
  // In localization mode, don't create new maps - use the loaded map
  if (mbOnlyTracking) {
    // cout << "CreateMapInAtlas: Skipping in localization mode" << endl;
    (mState = LOST, FieldEmit(15,mCurrentFrame.mnId,0,1,mState)); // Set to LOST to trigger relocalization
    return;
  }

  mnLastInitFrameId = mCurrentFrame.mnId;
  mpAtlas->CreateNewMap();
  if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR ||
      mSensor == System::IMU_RGBD)
    mpAtlas->SetInertialSensor();
  mbSetInit = false;

  mnInitialFrameId = mCurrentFrame.mnId + 1;
  (mState = NO_IMAGES_YET, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));

  // Restart the variable with information about the last KF
  mbVelocity = false;
  // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the
  // current id, because it is the new starting point for new map
  Verbose::PrintMess("First frame id in map: " +
                         to_string(mnLastInitFrameId + 1),
                     Verbose::VERBOSITY_NORMAL);
  mbVO =
      false; // Init value for know if there are enough MapPoints in the last KF
  if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR) {
    mbReadyToInitializate = false;
  }

  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      mpImuPreintegratedFromLastKF) {
    delete mpImuPreintegratedFromLastKF;
    mpImuPreintegratedFromLastKF =
        new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
  }

  if (mpLastKeyFrame)
    mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

  if (mpReferenceKF)
    mpReferenceKF = static_cast<KeyFrame *>(NULL);

  mLastFrame = Frame();
  mCurrentFrame = Frame();
  mvIniMatches.clear();

  mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame() {
  for (int i = 0; i < mLastFrame.N; i++) {
    MapPointPtr pMP = mLastFrame.mvpMapPoints[i];

    if (pMP) {
      MapPointPtr pRep = pMP->GetReplaced();
      if (pRep) {
        mLastFrame.mvpMapPoints[i] = pRep;
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  FieldStage field_stage(mCurrentFrame.mnId,2);
  ScopedTimingAccumulator detailed_timing(
      mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->reference_kf_ms : nullptr);
  const bool uses_float_descriptors =
      mCurrentFrame.mDescriptors.type() == CV_32FC1;
  if (!uses_float_descriptors) {
    mCurrentFrame.ComputeBoW();
  }

  ORBmatcher matcher(uses_float_descriptors ? 0.9F : 0.7F,
                     !uses_float_descriptors);
  vector<MapPointPtr > vpMapPointMatches;

  // Debug: Check reference KF
  if (!mpReferenceKF) {
    cout << "TRACK_REF_KF: mpReferenceKF is NULL!\n";
    return field_stage.Result(false,__LINE__);
  }

  vector<ReferenceFeatureMatch> coarse_reference_matches;
  int nmatches = uses_float_descriptors
      ? matcher.SearchByDescriptor(mpReferenceKF, mCurrentFrame,
                                   vpMapPointMatches,
                                   &coarse_reference_matches)
      : matcher.SearchByBoW(mpReferenceKF, mCurrentFrame,
                            vpMapPointMatches);

  if (uses_float_descriptors && mpStereoFeatureProvider && nmatches > 0) {
    const vector<ReferenceFeatureMatch> refined_matches =
        mpStereoFeatureProvider->RefineReferenceMatches(
            mpReferenceKF->mvKeysUn, mpReferenceKF->mDescriptors,
            mCurrentFrame.mvKeysUn, mCurrentFrame.mDescriptors,
            coarse_reference_matches, mImGray.cols, mImGray.rows);
    const vector<MapPointPtr > reference_map_points =
        mpReferenceKF->GetMapPointMatches();
    vpMapPointMatches.assign(mCurrentFrame.N,
                             MapPointPtr());
    vector<pair<int, cv::Point2f>> keypoint_refinements;
    keypoint_refinements.reserve(refined_matches.size());
    nmatches = 0;
    for (const ReferenceFeatureMatch &match : refined_matches) {
      if (match.reference_index < 0 ||
          match.reference_index >= static_cast<int>(reference_map_points.size()) ||
          match.current_index < 0 || match.current_index >= mCurrentFrame.N) {
        continue;
      }
      MapPointPtr map_point = reference_map_points[match.reference_index];
      if (!map_point || map_point->isBad() ||
          vpMapPointMatches[match.current_index]) {
        continue;
      }
      vpMapPointMatches[match.current_index] = map_point;
      keypoint_refinements.emplace_back(match.current_index,
                                        match.refined_current_point);
      ++nmatches;
    }
    mCurrentFrame.ApplyKeypointRefinements(keypoint_refinements);
  }

  // Only Tracking-owned BoW sizes: reference-KF BoW has no synchronized
  // publication contract, so its size is deliberately absent (e invalid).
  if(FieldEnabled()) FieldEmit(12,mCurrentFrame.mnId,2,15,nmatches,
      mpReferenceKF->mnId,mCurrentFrame.mBowVec.size(),mCurrentFrame.mFeatVec.size());
  if (nmatches < 10) {
    cout << "TRACK_REF_KF: Less than 10 matches!! (got " << nmatches
         << ", RefKF=" << mpReferenceKF->mnId
         << ", RefKF_MPs=" << mpReferenceKF->GetMapPointMatches().size()
         << ", RefKF_BowEmpty=" << mpReferenceKF->mBowVec.empty()
         << ", CurFrame_BowEmpty=" << mCurrentFrame.mBowVec.empty() << ")\n";
    return field_stage.Result(false,__LINE__,nmatches,10,
        FieldDecisionRule::ReferenceCoarseMatchesMin10);
  }

  mCurrentFrame.mvpMapPoints = vpMapPointMatches;
  mCurrentFrame.SetPose(mLastFrame.GetPose());

  // mCurrentFrame.PrintPointDistribution();

  // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw <<
  // endl;
  FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    // if(i >= mCurrentFrame.Nleft) break;
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = MapPointPtr();
        mCurrentFrame.mvbOutlier[i] = false;
        if (i < mCurrentFrame.Nleft) {
          pMP->mbTrackInView = false;
        } else {
          pMP->mbTrackInViewR = false;
        }
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
      mSensor == System::IMU_RGBD) {
    if (mbOnlyTracking)
      return field_stage.Result(nmatchesMap >= 15,__LINE__);
    return field_stage.Result(true,__LINE__);
  } else
    return field_stage.Result(nmatchesMap >= 10,__LINE__,nmatchesMap,10,
        FieldDecisionRule::ReferenceMapMatchesMin10);
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  KeyFrame *pRef = mLastFrame.mpReferenceKF;
  Sophus::SE3f Tlr = mlRelativeFramePoses.back();
  mLastFrame.SetPose(Tlr * pRef->GetPose());

  if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR ||
      mSensor == System::IMU_MONOCULAR || !mbOnlyTracking)
    return;

  // Create "visual odometry" MapPoints
  // We sort points according to their measured depth by the stereo/RGB-D sensor
  vector<pair<float, int>> vDepthIdx;
  const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
  vDepthIdx.reserve(Nfeat);
  for (int i = 0; i < Nfeat; i++) {
    float z = mLastFrame.mvDepth[i];
    if (z > 0) {
      vDepthIdx.push_back(make_pair(z, i));
    }
  }

  if (vDepthIdx.empty())
    return;

  sort(vDepthIdx.begin(), vDepthIdx.end());

  // We insert all close points (depth<mThDepth)
  // If less than 100 close points, we insert the 100 closest ones.
  int nPoints = 0;
  for (size_t j = 0; j < vDepthIdx.size(); j++) {
    int i = vDepthIdx[j].second;

    bool bCreateNew = false;

    MapPointPtr pMP = mLastFrame.mvpMapPoints[i];

    if (!pMP)
      bCreateNew = true;
    else if (pMP->Observations() < 1)
      bCreateNew = true;

    if (bCreateNew) {
      Eigen::Vector3f x3D;

      if (mLastFrame.Nleft == -1) {
        mLastFrame.UnprojectStereo(i, x3D);
      } else {
        x3D = mLastFrame.UnprojectStereoFishEye(i);
      }

      MapPointPtr pNewMP =
          MapPoint::Create(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
      mLastFrame.mvpMapPoints[i] = pNewMP;

      mlpTemporalPoints.push_back(pNewMP);
      nPoints++;
    } else {
      nPoints++;
    }

    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
      break;
  }
}

bool Tracking::TrackWithMotionModel() {
  FieldStage field_stage(mCurrentFrame.mnId,1);
  ScopedTimingAccumulator detailed_timing(
      mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->motion_model_ms : nullptr);
  const bool uses_float_descriptors =
      mCurrentFrame.mDescriptors.type() == CV_32FC1 &&
      mLastFrame.mDescriptors.type() == CV_32FC1;
  ORBmatcher matcher(0.9, !uses_float_descriptors);

  // Update last frame pose according to its reference keyframe
  // Create "visual odometry" points if in Localization Mode
  bool point_trace = PointTraceEnabled(mCurrentFrame.mnId);
  if(point_trace && (!mbOnlyTracking || mSensor!=System::RGBD ||
      mCurrentFrame.Nleft!=-1 || mLastFrame.Nleft!=-1 ||
      mCurrentFrame.N>2048 || mLastFrame.N>2048)) {
    PointTraceAbort(mCurrentFrame.mnId,1);point_trace=false;
  }
  if(point_trace)PointTraceSnapshot(mCurrentFrame.mnId,1,mLastFrame);
  UpdateLastFrame();
  if(point_trace)PointTraceSnapshot(mCurrentFrame.mnId,2,mLastFrame);

  bool bPredictedWithIMU = false;
  if (mpAtlas->isImuInitialized() && mbCurrentFrameHasValidIMU &&
      (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU)) {
    // Predict state with IMU if it is initialized and it doesnt need reset
    PredictStateIMU();
    bPredictedWithIMU = true;
    // In pure localization mode we still require visual correspondences to
    // avoid drifting away after relocalization.
    if (!mbOnlyTracking)
      return field_stage.Result(true,__LINE__);
  } else {
    mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
  }

  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(),
       MapPointPtr());

  // Project points seen in previous frame
  int th;

  if (mSensor == System::STEREO)
    th = 7;
  else
    th = 15;

  int nmatches = 0;
  if (uses_float_descriptors && mpStereoFeatureProvider) {
    const auto temporal_start = std::chrono::steady_clock::now();
    vector<cv::DMatch> descriptor_matches;
    cv::BFMatcher descriptor_matcher(cv::NORM_L2, true);
    descriptor_matcher.match(mLastFrame.mDescriptors,
                             mCurrentFrame.mDescriptors,
                             descriptor_matches);

    vector<ReferenceFeatureMatch> coarse_matches;
    coarse_matches.reserve(descriptor_matches.size());
    // The frozen temporal matcher uses cosine >= 0.82. FeatureDescriptor maps
    // that boundary to round(256 * (1 - 0.82)) = 46.
    const int kXFeatTemporalMaximumDistance = 46;
    for (const cv::DMatch &descriptor_match : descriptor_matches) {
      if (descriptor_match.queryIdx < 0 ||
          descriptor_match.queryIdx >= mLastFrame.N ||
          descriptor_match.trainIdx < 0 ||
          descriptor_match.trainIdx >= mCurrentFrame.N) {
        continue;
      }
      MapPointPtr map_point =
          mLastFrame.mvpMapPoints[descriptor_match.queryIdx];
      if (!map_point || map_point->isBad() ||
          mLastFrame.mvbOutlier[descriptor_match.queryIdx]) {
        continue;
      }
      const int descriptor_distance = ORBmatcher::DescriptorDistance(
          mLastFrame.mDescriptors.row(descriptor_match.queryIdx),
          mCurrentFrame.mDescriptors.row(descriptor_match.trainIdx));
      if (descriptor_distance > kXFeatTemporalMaximumDistance) {
        continue;
      }

      ReferenceFeatureMatch match;
      match.reference_index = descriptor_match.queryIdx;
      match.current_index = descriptor_match.trainIdx;
      match.descriptor_distance = descriptor_distance;
      match.refined_current_point =
          mCurrentFrame.mvKeysUn[descriptor_match.trainIdx].pt;
      coarse_matches.push_back(match);
    }

    if (!coarse_matches.empty()) {
      const vector<ReferenceFeatureMatch> refined_matches =
          mpStereoFeatureProvider->RefineReferenceMatches(
              mLastFrame.mvKeysUn, mLastFrame.mDescriptors,
              mCurrentFrame.mvKeysUn, mCurrentFrame.mDescriptors,
              coarse_matches, mImGray.cols, mImGray.rows);
      vector<pair<int, cv::Point2f>> keypoint_refinements;
      keypoint_refinements.reserve(refined_matches.size());
      for (const ReferenceFeatureMatch &match : refined_matches) {
        if (match.reference_index < 0 ||
            match.reference_index >= mLastFrame.N ||
            match.current_index < 0 ||
            match.current_index >= mCurrentFrame.N ||
            mCurrentFrame.mvpMapPoints[match.current_index]) {
          continue;
        }
        MapPointPtr map_point = mLastFrame.mvpMapPoints[match.reference_index];
        if (!map_point || map_point->isBad()) {
          continue;
        }
        mCurrentFrame.mvpMapPoints[match.current_index] = map_point;
        keypoint_refinements.emplace_back(match.current_index,
                                          match.refined_current_point);
        ++nmatches;
      }
      mCurrentFrame.ApplyKeypointRefinements(keypoint_refinements);
    }
    const auto temporal_end = std::chrono::steady_clock::now();
    mCurrentFrame.mStereoFeatureExtractionStats.fine_match_ms +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            temporal_end - temporal_start)
            .count();
  } else {
    nmatches = matcher.SearchByProjection(
        mCurrentFrame, mLastFrame, th,
        mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);
  }

  // Preserve the initial count to identify which search produced each
  // existing diagnostic row; no extra row, match search or point traversal.
  const int field_first_nmatches = nmatches;
  // If few matches, uses a wider window search
  if(FieldEnabled()) {
    const bool projection = !(uses_float_descriptors && mpStereoFeatureProvider);
    FieldEmit(12,mCurrentFrame.mnId,1,projection?31:27,nmatches,1,
              projection?th:0,0,20);
  }
  if (nmatches < 20) {
    Verbose::PrintMess("Not enough matches, wider window search!!",
                       Verbose::VERBOSITY_NORMAL);
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(),
         MapPointPtr());

    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th,
                                          mSensor == System::MONOCULAR ||
                                              mSensor == System::IMU_MONOCULAR);
    Verbose::PrintMess("Matches with wider search: " + to_string(nmatches),
                       Verbose::VERBOSITY_NORMAL);
  }

  if(FieldEnabled()) {
    const bool retried = field_first_nmatches < 20;
    const bool projection = retried ||
        !(uses_float_descriptors && mpStereoFeatureProvider);
    FieldEmit(12,mCurrentFrame.mnId,1,projection?31:27,nmatches,retried?2:1,
              projection?(retried?2*th:th):0,1,20);
  }
  if (nmatches < 20) {
    Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD) {
      if (mbOnlyTracking || bPredictedWithIMU)
        return field_stage.Result(false,__LINE__);
      return field_stage.Result(true,__LINE__);
    } else
      return field_stage.Result(false,__LINE__,nmatches,20,
          FieldDecisionRule::MotionCoarseMatchesMin20);
  }

  // Optimize frame pose with all matches
  if(point_trace)PointTraceSnapshot(mCurrentFrame.mnId,3,mCurrentFrame);
  FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
  if(point_trace)PointTraceSnapshot(mCurrentFrame.mnId,4,mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = MapPointPtr();
        mCurrentFrame.mvbOutlier[i] = false;
        if (i < mCurrentFrame.Nleft) {
          pMP->mbTrackInView = false;
        } else {
          pMP->mbTrackInViewR = false;
        }
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  if(point_trace)PointTraceSnapshot(mCurrentFrame.mnId,5,mCurrentFrame);
  if (mbOnlyTracking) {
    mbVO = nmatchesMap < 10;
    return field_stage.ResultMotionLocalization(nmatches > 20 && nmatchesMap >= 10,
                                                __LINE__,nmatches,nmatchesMap);
  }

  if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
      mSensor == System::IMU_RGBD)
    return field_stage.Result(true,__LINE__);
  else
    return field_stage.Result(nmatchesMap >= 10,__LINE__,nmatchesMap,10,
        FieldDecisionRule::MotionMapMatchesMin10);
}

bool Tracking::TrackLocalMap(Map* locked_map) {
  MapScaleSpan scale_local(MapScaleStage::TrackLocalMap);
  FieldStage field_stage(mCurrentFrame.mnId,3);
  ScopedTimingAccumulator detailed_timing(
      mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->local_map_ms : nullptr);
  if (mLocalMapMode && mSensor == System::RGBD && mCurrentFrame.Nleft == -1 &&
      !mCurrentFrame.mpCamera2 && mCurrentFrame.mDescriptors.type() == CV_8UC1)
    return field_stage.Result(TrackLocalMapBudgeted(locked_map),__LINE__);

  // We have an estimation of the camera pose and some map points tracked in the
  // frame. We retrieve the local map and try to find matches to points in the
  // local map.
  mTrackedFr++;

  {
    ScopedTimingAccumulator timer(mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->local_update_ms : nullptr);
    UpdateLocalMap();
  }
  {
    ScopedTimingAccumulator timer(mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->local_search_ms : nullptr);
    SearchLocalPoints();
  }
  if (mpCurrentRgbdTiming) {
    mpCurrentRgbdTiming->local_keyframes = mvpLocalKeyFrames.size();
    mpCurrentRgbdTiming->local_points = mvpLocalMapPoints.size();
  }

  // TOO check outliers before PO
  int aux1 = 0, aux2 = 0;
  for (int i = 0; i < mCurrentFrame.N; i++)
    if (mCurrentFrame.mvpMapPoints[i]) {
      aux1++;
      if (mCurrentFrame.mvbOutlier[i])
        aux2++;
    }

// FIELD_MAP_BEGIN
  FieldEmit(25,mCurrentFrame.mnId,0,3,aux1,aux2);
// FIELD_MAP_END
  int inliers;
  const auto local_pose_start = mpCurrentRgbdTiming ? std::chrono::steady_clock::now()
                                                   : std::chrono::steady_clock::time_point();
  MapScaleSpan scale_pose(MapScaleStage::LocalPoseOptimization);
  if (!mpAtlas->isImuInitialized())
    FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
  else {
    if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU) {
      Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
      FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
    } else {
      // Check if IMU preintegration data is valid before using IMU-based
      // optimization
      bool bImuDataValid = mCurrentFrame.mpImuPreintegratedFrame != nullptr &&
                           mCurrentFrame.mpImuPreintegrated != nullptr &&
                           mCurrentFrame.mpPrevFrame != nullptr &&
                           mbCurrentFrameHasValidIMU;

      // In localization mode after relocalization, IMU data may be stale
      // Fall back to vision-only optimization in that case
      if (!bImuDataValid || (mbOnlyTracking && mState == RECENTLY_LOST)) {
        Verbose::PrintMess(
            "TLM: Falling back to PoseOptimization (IMU data unavailable)",
            Verbose::VERBOSITY_DEBUG);
        FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
      } else if (!mbMapUpdated) {
        const bool bHasPrevPrior = mCurrentFrame.mpPrevFrame &&
                                   mCurrentFrame.mpPrevFrame->mpcpi != nullptr;

        if (bHasPrevPrior) {
          Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ",
                             Verbose::VERBOSITY_DEBUG);
          inliers =
              Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame);
        } else {
          // In localization mode after relocalization, previous-frame IMU prior
          // may be missing. Bootstrap with last-keyframe inertial optimization.
          Verbose::PrintMess(
              "TLM: Bootstrap with LastKeyFrame inertial optimization",
              Verbose::VERBOSITY_DEBUG);
          inliers =
              Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame);
        }

        // If IMU optimization failed, fall back to vision-only.
        if (inliers < 0) {
          cout << "TLM: IMU optimization failed, falling back to "
                  "PoseOptimization"
               << endl;
          FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
        }
      } else {
        Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ",
                           Verbose::VERBOSITY_DEBUG);
        inliers =
            Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame);
        // If IMU optimization failed, fall back to vision-only
        if (inliers < 0) {
          cout << "TLM: IMU optimization failed, falling back to "
                  "PoseOptimization"
               << endl;
          FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
        }
      }
    }
  }

  scale_pose.Finish();
  if (mpCurrentRgbdTiming)
    mpCurrentRgbdTiming->local_pose_ms += ElapsedMilliseconds(local_pose_start);
  aux1 = 0, aux2 = 0;
  for (int i = 0; i < mCurrentFrame.N; i++)
    if (mCurrentFrame.mvpMapPoints[i]) {
      aux1++;
      if (mCurrentFrame.mvbOutlier[i])
        aux2++;
    }

// FIELD_MAP_BEGIN
  FieldEmit(25,mCurrentFrame.mnId,1,3,aux1,aux2);
// FIELD_MAP_END
  mnMatchesInliers = 0;

  // Update MapPoints Statistics
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (!mCurrentFrame.mvbOutlier[i]) {
        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        if (!mbOnlyTracking) {
          if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
            mnMatchesInliers++;
        } else
          mnMatchesInliers++;
      } else if (mSensor == System::STEREO)
        mCurrentFrame.mvpMapPoints[i] = MapPointPtr();
    }
  }

  // Decide if the tracking was succesful
  // More restrictive if there was a relocalization recently
  if (auto* scale = ActiveMapScaleObservation())
    scale->tracking_inliers = mnMatchesInliers;
  mpLocalMapper->mnMatchesInliers = mnMatchesInliers;
  FieldEmit(13,mCurrentFrame.mnId,3,1,mnMatchesInliers);
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames &&
      mnMatchesInliers < 50)
    return field_stage.Result(false,__LINE__);

  // In pure localization + IMU mode, keep a stricter gate to prevent drift
  // when visual constraints become weak.
  if (mbOnlyTracking &&
      (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      mnMatchesInliers < 25)
    return field_stage.Result(false,__LINE__);

  if ((mnMatchesInliers > 10) && (mState == RECENTLY_LOST))
    return field_stage.Result(true,__LINE__);

  if (mSensor == System::IMU_MONOCULAR) {
    if ((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) ||
        (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized())) {
      return field_stage.Result(false,__LINE__);
    } else
      return field_stage.Result(true,__LINE__);
  } else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
    if (mnMatchesInliers < 15) {
      return field_stage.Result(false,__LINE__);
    } else
      return field_stage.Result(true,__LINE__);
  } else {
    if (mnMatchesInliers < 30)
      return field_stage.Result(false,__LINE__);
    else
      return field_stage.Result(true,__LINE__);
  }
}

bool Tracking::NeedNewKeyFrame() {
  ScopedTimingAccumulator detailed_timing(
      mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->need_new_kf_ms : nullptr);
  auto* decision = (RgbdLightDiagnostics() && mpCurrentRgbdTiming)
      ? &mpCurrentRgbdTiming->keyframe_decision : nullptr;
  auto note = [decision](unsigned bit, bool value) {
    if (decision) {
      decision->evaluated_mask |= (1u << bit);
      if (value) decision->value_mask |= (1u << bit);
    }
    return value;
  };
  auto finish = [decision](bool result, KeyFrameDecisionReason reason) {
    if (decision) { decision->need_result = result; decision->reason = reason; }
    return result;
  };
  if (decision) {
    decision->frames_since_keyframe = mCurrentFrame.mnId - mnLastKeyFrameId;
    decision->min_frames = mMinFrames;
    decision->max_frames = mMaxFrames;
    decision->matches_inliers = mnMatchesInliers;
  }
  if (note(KfImuStartup, (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      !mpAtlas->GetCurrentMap()->isImuInitialized())) {
    if (mSensor == System::IMU_MONOCULAR &&
        (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
      return finish(true, KeyFrameDecisionReason::ImuStartupInsert);
    else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
             (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
      return finish(true, KeyFrameDecisionReason::ImuStartupInsert);
    else
      return finish(false, KeyFrameDecisionReason::ImuStartupWait);
  }

  if (note(KfOnlyTracking, mbOnlyTracking))
    return finish(false, KeyFrameDecisionReason::OnlyTracking);

  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  const bool mapping_stopped = note(KfMappingStopped, mpLocalMapper->isStopped());
  if (mapping_stopped || note(KfStopRequested, mpLocalMapper->stopRequested())) {
    /*if(mSensor == System::MONOCULAR)
    {
        std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
    }*/
    return finish(false, mapping_stopped ? KeyFrameDecisionReason::MappingStopped
                                        : KeyFrameDecisionReason::MappingStopRequested);
  }

  const int nKFs = mpAtlas->KeyFramesInMap();
  if (decision) decision->map_keyframes = nKFs;

  // Do not insert keyframes if not enough frames have passed from last
  // relocalisation
  if (note(KfRelocalizationGuard, mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames &&
      nKFs > mMaxFrames)) {
    return finish(false, KeyFrameDecisionReason::RelocalizationGuard);
  }

  // Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  if (nKFs <= 2)
    nMinObs = 2;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);
  if (decision) decision->reference_matches = nRefMatches;

  // Local Mapping accept keyframes?
  bool bLocalMappingIdle = note(KfMappingIdle, mpLocalMapper->AcceptKeyFrames());

  // Check how many "close" points are being tracked and how many could be
  // potentially created.
  int nNonTrackedClose = 0;
  int nTrackedClose = 0;

  if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
    int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
    for (int i = 0; i < N; i++) {
      if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
          nTrackedClose++;
        else
          nNonTrackedClose++;
      }
    }
    // Verbose::PrintMess("[NEEDNEWKF]-> closed points: " +
    // to_string(nTrackedClose) + "; non tracked closed points: " +
    // to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);//
    // Verbose::VERBOSITY_DEBUG);
  }

  bool bNeedToInsertClose;
  bNeedToInsertClose = note(KfNeedClose, (nTrackedClose < 100) && (nNonTrackedClose > 70));
  if (decision) {
    decision->tracked_close = nTrackedClose;
    decision->untracked_close = nNonTrackedClose;
  }

  // Thresholds
  float thRefRatio = 0.75f;
  if (nKFs < 2)
    thRefRatio = 0.4f;

  /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
  const int thStereoClosedPoints = 15;
  if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO ||
  mSensor==System::IMU_STEREO))
  {
      //Pseudo-monocular, there are not enough close points to be confident
  about the stereo observations. thRefRatio = 0.9f;
  }*/

  if (mSensor == System::MONOCULAR)
    thRefRatio = 0.9f;

  if (mpCamera2)
    thRefRatio = 0.75f;

  if (mSensor == System::IMU_MONOCULAR) {
    if (mnMatchesInliers > 350) // Points tracked from the local map
      thRefRatio = 0.75f;
    else
      thRefRatio = 0.90f;
  }

  // Condition 1a: More than "MaxFrames" have passed from last keyframe
  // insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b =
      ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) &&
       bLocalMappingIdle); // mpLocalMapper->KeyframesInQueue() < 2);
  // Condition 1c: tracking is weak
  const bool c1c =
      mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR &&
      mSensor != System::IMU_STEREO && mSensor != System::IMU_RGBD &&
      (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
  // Condition 2: Few tracked points compared to reference keyframe. Lots of
  // visual odometry compared to map matches.
  const bool c2 =
      (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) &&
       mnMatchesInliers > 15);

  // std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c
  // << "; c2=" << c2 << std::endl;
  //  Temporal condition for Inertial cases
  bool c3 = false;
  if (mpLastKeyFrame) {
    if (mSensor == System::IMU_MONOCULAR) {
      if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
        c3 = true;
    } else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
      if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
        c3 = true;
    }
  }

  bool c4 = false;
  if ((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) ||
       mState == RECENTLY_LOST) &&
      (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally
                                          // ((((mnMatchesInliers<75) &&
                                          // (mnMatchesInliers>15)) ||
                                          // mState==RECENTLY_LOST) && ((mSensor
                                          // == System::IMU_MONOCULAR)))
    c4 = true;
  else
    c4 = false;

  if (decision) decision->reference_ratio = thRefRatio;
  note(KfC1a, c1a); note(KfC1b, c1b); note(KfC1c, c1c);
  note(KfC2, c2); note(KfC3, c3); note(KfC4, c4);
  if (note(KfCombinedConditions, ((c1a || c1b || c1c) && c2) || c3 || c4)) {
    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle || note(KfMappingInitializing, mpLocalMapper->IsInitializing())) {
      return finish(true, bLocalMappingIdle ? KeyFrameDecisionReason::MappingAccepts
                                           : KeyFrameDecisionReason::MappingInitializing);
    } else {
      LocalMappingCancellation::RequestRecord request;
      mpLocalMapper->InterruptBA(decision ? &request : nullptr);
      if (decision) {
        decision->interrupt_called = true;
        decision->interrupt_request_id = request.request_id;
        decision->interrupt_attempt_id = request.attempt_id;
        decision->interrupt_request_ns = request.request_ns;
      }
      if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
        const int queue_depth = mpLocalMapper->KeyframesInQueue();
        if (decision) decision->queue_depth = queue_depth;
        if (note(KfQueueBelowThree, queue_depth < 3))
          return finish(true, KeyFrameDecisionReason::BusyQueueAccepts);
        else
          return finish(false, KeyFrameDecisionReason::BusyQueueFull);
      } else {
        // std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
        return finish(false, KeyFrameDecisionReason::BusyMonocular);
      }
    }
  } else
    return finish(false, KeyFrameDecisionReason::ConditionsFalse);
}

void Tracking::CreateNewKeyFrame() {
  ScopedTimingAccumulator detailed_timing(
      mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->create_new_kf_ms : nullptr);
  auto* decision = (RgbdLightDiagnostics() && mpCurrentRgbdTiming)
      ? &mpCurrentRgbdTiming->keyframe_decision : nullptr;
  if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized()) {
    if (decision) decision->create_result = KeyFrameCreateResult::Initializing;
    return;
  }

  if (!mpLocalMapper->SetNotStop(true)) {
    if (decision) decision->create_result = KeyFrameCreateResult::StopGuardRejected;
    return;
  }

  KeyFrame *pKF =
      new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  if (mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
    pKF->bImu = true;

  pKF->SetNewBias(mCurrentFrame.mImuBias);
  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;

  if (mpLastKeyFrame) {
    pKF->mPrevKF = mpLastKeyFrame;
    mpLastKeyFrame->mNextKF = pKF;
  } else
    Verbose::PrintMess("No last KF in KF creation!!",
                       Verbose::VERBOSITY_NORMAL);

  // Reset preintegration from last KF (Create new object)
  if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
      mSensor == System::IMU_RGBD) {
    mpImuPreintegratedFromLastKF =
        new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
  }

  if (mSensor != System::MONOCULAR &&
      mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
  {
    mCurrentFrame.UpdatePoseMatrices();
    // cout << "create new MPs" << endl;
    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    int maxPoint = 100;
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
      maxPoint = 100;

    vector<pair<float, int>> vDepthIdx;
    int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
    vDepthIdx.reserve(mCurrentFrame.N);
    for (int i = 0; i < N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
        vDepthIdx.push_back(make_pair(z, i));
      }
    }

    if (!vDepthIdx.empty()) {
      sort(vDepthIdx.begin(), vDepthIdx.end());

      int nPoints = 0;
      for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP)
          bCreateNew = true;
        else if (pMP->Observations() < 1) {
          bCreateNew = true;
          mCurrentFrame.mvpMapPoints[i] = MapPointPtr();
        }

        if (bCreateNew) {
          Eigen::Vector3f x3D;

          if (mCurrentFrame.Nleft == -1) {
            mCurrentFrame.UnprojectStereo(i, x3D);
          } else {
            x3D = mCurrentFrame.UnprojectStereoFishEye(i);
          }

          MapPointPtr pNewMP = MapPoint::Create(x3D, pKF, mpAtlas->GetCurrentMap());
          pNewMP->AddObservation(pKF, i);

          // Check if it is a stereo observation in order to not
          // duplicate mappoints
          if (mCurrentFrame.Nleft != -1 &&
              mCurrentFrame.mvLeftToRightMatch[i] >= 0) {
            mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft +
                                       mCurrentFrame.mvLeftToRightMatch[i]] =
                pNewMP;
            pNewMP->AddObservation(
                pKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
            pKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft +
                                         mCurrentFrame.mvLeftToRightMatch[i]);
          }

          pKF->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpAtlas->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
          nPoints++;
        } else {
          nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
          break;
        }
      }
      // Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints),
      // Verbose::VERBOSITY_NORMAL);
    }
  }

  mpLocalMapper->InsertKeyFrame(pKF);
  if (decision) decision->create_result = KeyFrameCreateResult::Enqueued;
  if (mpCurrentRgbdTiming)
    mpCurrentRgbdTiming->inserted_keyframe = 1;

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
  MapScaleSpan scale_search(MapScaleStage::SearchLocalPoints);
  auto* light = RgbdLightDiagnostics() ? mpCurrentRgbdTiming : nullptr;
  ScopedThreadCpuAccumulator cpu(light ? &light->local_search_cpu_ms : nullptr);
// FIELD_MAP_BEGIN
  uint64_t field_associated=0;
// FIELD_MAP_END
  // Do not search map points already matched
  for (vector<MapPointPtr >::iterator vit = mCurrentFrame.mvpMapPoints.begin(),
                                    vend = mCurrentFrame.mvpMapPoints.end();
       vit != vend; vit++) {
    if (light) ++light->search_associated_slots;
    MapPointPtr pMP = *vit;
    if (pMP) {
      if (pMP->isBad()) {
        *vit = MapPointPtr();
      } else {
// FIELD_MAP_BEGIN
        if(FieldEnabled()) ++field_associated;
// FIELD_MAP_END
        pMP->IncreaseVisible();
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
      }
    }
  }

  int nToMatch = 0;

  // Project points in frame and check its visibility
  for (vector<MapPointPtr >::iterator vit = mvpLocalMapPoints.begin(),
                                    vend = mvpLocalMapPoints.end();
       vit != vend; vit++) {
    // Tracking retains this vector unchanged throughout projection.
    const MapPointPtr& pMP = *vit;

    if (light) ++light->search_local_slots;
    if (pMP->mnLastFrameSeen == mCurrentFrame.mnId) {
      if (light) ++light->search_seen_skips;
      continue;
    }
    if (pMP->isBad()) {
      if (light) ++light->search_bad_skips;
      continue;
    }
    if (light) ++light->search_projected;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
      if (light) ++light->search_visible;
    } else if (light) ++light->search_rejected;
    if (pMP->mbTrackInView && (mRgbdOptimizationLevel < 4 || mpViewer)) {
      mCurrentFrame.mmProjectPoints[pMP->mnId] =
          cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
    }
  }

// FIELD_MAP_BEGIN
  if(FieldEnabled()) {
    FieldEmit(23,mCurrentFrame.mnId,0,mpReferenceKF?31:30,mpReferenceKF?mpReferenceKF->mnId:0,mvpLocalKeyFrames.size(),mvpLocalMapPoints.size(),nToMatch,field_associated);
    if(nToMatch==0) FieldEmit(24,mCurrentFrame.mnId,0,1,0);
  }
// FIELD_MAP_END
  if (nToMatch > 0) {
    const bool uses_float_descriptors =
        mCurrentFrame.mDescriptors.type() == CV_32FC1;
    ORBmatcher matcher(uses_float_descriptors ? 0.9F : 0.8F);
    int th = 1;
    if (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
      th = 3;
    if (mpAtlas->isImuInitialized()) {
      if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
        th = 2;
      else
        th = 6;
    } else if (!mpAtlas->isImuInitialized() &&
               (mSensor == System::IMU_MONOCULAR ||
                mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)) {
      th = 10;
    }

    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
      th = 5;

    if (mState == LOST ||
        mState == RECENTLY_LOST) // Lost for less than 1 second
      th = 15;                   // 15

    int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints,
                                             th, mpLocalMapper->mbFarPoints,
                                             mpLocalMapper->mThFarPoints,
                                             light ? &light->projection_match : nullptr);
    if (light) light->search_matches += matches;
// FIELD_MAP_BEGIN
    FieldEmit(24,mCurrentFrame.mnId,1,3,matches,th);
// FIELD_MAP_END

  }
}

void Tracking::UpdateLocalMap() {
  // This is for visualization
  {
    MapScaleSpan scale_reference(MapScaleStage::ReferenceCopy);
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
  }

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  MapScaleSpan scale_a3(MapScaleStage::A3);
  auto* scale = ActiveMapScaleObservation();
  if (scale) {
    scale->gather_slots = 0;
    scale->visited_slots = 0;
    scale->accepted_points = 0;
  }
  ScopedTimingAccumulator timer(mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->local_points_ms : nullptr);
  auto* light = RgbdLightDiagnostics() ? mpCurrentRgbdTiming : nullptr;
  ScopedThreadCpuAccumulator cpu(light ? &light->local_points_cpu_ms : nullptr);
  auto* detail = light && light->observation.detail_window
      ? &light->local_map_detail : nullptr;
  InitializeLocalMapDetail(detail, light ? light->tracking_thread_id : 0);
  if (detail) detail->old_owner_count = mvpLocalMapPoints.size();
  MapScaleSpan scale_clear(MapScaleStage::A3Clear);
  {
    LocalMapClearSpan span(detail ? &detail->hint_clear : nullptr);
    mLocalPointIdentities.clear();
  }
  {
    LocalMapClearSpan span(detail ? &detail->owner_clear : nullptr);
    mvpLocalMapPoints.clear();
  }

  scale_clear.Finish();
  MapScaleSpan scale_gather(MapScaleStage::A3Gather);
  for (vector<KeyFrame *>::const_reverse_iterator
           itKF = mvpLocalKeyFrames.rbegin(),
           itEndKF = mvpLocalKeyFrames.rend();
       itKF != itEndKF; ++itKF) {
    KeyFrame *pKF = *itKF;
    pKF->AppendLocalMapPoints(mCurrentFrame.mnId, mvpLocalMapPoints,
        mLocalPointIdentities, light ? &light->gather : nullptr,
        detail ? detail->gather_features.Next(LocalMapLockSite::GatherKeyFrameFeatures)
               : nullptr);
  }
  scale_gather.Finish();
  if (scale) {
    scale->local_mp = mvpLocalMapPoints.size();
    scale->accepted_points = mvpLocalMapPoints.size();
  }
}

void Tracking::UpdateLocalKeyFrames() {
  MapScaleSpan scale_a1(MapScaleStage::A1);
  ScopedTimingAccumulator timer(mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->local_keyframes_ms : nullptr);
  auto* light = RgbdLightDiagnostics() ? mpCurrentRgbdTiming : nullptr;
  ScopedThreadCpuAccumulator cpu(light ? &light->local_keyframes_cpu_ms : nullptr);
  auto* local_detail = light && light->observation.detail_window
      ? &light->local_map_detail : nullptr;
  InitializeLocalMapDetail(local_detail, light ? light->tracking_thread_id : 0);
  // Each map point vote for the keyframes in which it has been observed
  LocalMapClearSpan total_span(local_detail ? &local_detail->a1_total : nullptr);
  map<KeyFrame *, int> keyframeCounter;
  MapScaleSpan scale_vote(MapScaleStage::A1Vote);
  {
  LocalMapClearSpan voting_span(local_detail ? &local_detail->a1_voting : nullptr);
  if (!mpAtlas->isImuInitialized() ||
      (mCurrentFrame.mnId < mnLastRelocFrameId + 2)) {
    for (int i = 0; i < mCurrentFrame.N; i++) {
      if (light) ++light->voting_slots;
      MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];
      if (pMP) {
        if (light) ++light->voting_associated;
        if (!pMP->isBad(local_detail ? local_detail->voting_is_bad.NextRotating(
            LocalMapLockSite::VoteMapPointIsBadFeatures, light->observation.source_ordinal) : nullptr)) {
          const map<KeyFrame *, tuple<int, int>> observations = [&] {
            const bool detail = light && light->observation.detail_window;
            ScopedTimingAccumulator detail_wall(detail
                ? &light->voting_observation_copy_ms : nullptr);
            ScopedThreadCpuAccumulator detail_cpu(detail
                ? &light->voting_observation_copy_cpu_ms : nullptr);
            return pMP->GetObservations(local_detail ? local_detail->voting_features.Next(
                LocalMapLockSite::VoteMapPointFeatures) : nullptr);
          }();
          if (light) light->voting_observation_entries += observations.size();
          for (map<KeyFrame *, tuple<int, int>>::const_iterator
                   it = observations.begin(),
                   itend = observations.end();
               it != itend; it++)
            keyframeCounter[it->first]++;
        } else {
          mCurrentFrame.mvpMapPoints[i] = NULL;
        }
      }
    }
  } else {
    for (int i = 0; i < mLastFrame.N; i++) {
      if (light) ++light->voting_slots;
      // Using lastframe since current frame has not matches yet
      if (mLastFrame.mvpMapPoints[i]) {
        if (light) ++light->voting_associated;
        MapPointPtr pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP)
          continue;
        if (!pMP->isBad(local_detail ? local_detail->voting_is_bad.NextRotating(
            LocalMapLockSite::VoteMapPointIsBadFeatures, light->observation.source_ordinal) : nullptr)) {
          const map<KeyFrame *, tuple<int, int>> observations = [&] {
            const bool detail = light && light->observation.detail_window;
            ScopedTimingAccumulator detail_wall(detail
                ? &light->voting_observation_copy_ms : nullptr);
            ScopedThreadCpuAccumulator detail_cpu(detail
                ? &light->voting_observation_copy_cpu_ms : nullptr);
            return pMP->GetObservations(local_detail ? local_detail->voting_features.Next(
                LocalMapLockSite::VoteMapPointFeatures) : nullptr);
          }();
          if (light) light->voting_observation_entries += observations.size();
          for (map<KeyFrame *, tuple<int, int>>::const_iterator
                   it = observations.begin(),
                   itend = observations.end();
               it != itend; it++)
            keyframeCounter[it->first]++;
        } else {
          // MODIFICATION
          mLastFrame.mvpMapPoints[i] = NULL;
        }
      }
    }
  }

  if (light) light->voting_unique_keyframes = keyframeCounter.size();
  } // complete voting loop, including observation copies and vote insertion
  scale_vote.Finish();
  if (auto* scale = ActiveMapScaleObservation())
    scale->voting_kf = keyframeCounter.size();
  int max = 0;
  KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also
  // check which keyframe shares most points
  {
  LocalMapClearSpan filter_span(local_detail ? &local_detail->a1_filter : nullptr);
  MapScaleSpan scale_filter(MapScaleStage::A1Filter);
  for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(),
                                            itEnd = keyframeCounter.end();
       it != itEnd; it++) {
    KeyFrame *pKF = it->first;

    if (pKF->isBad(local_detail ? local_detail->filter_is_bad.NextRotating(
        LocalMapLockSite::FilterKeyFrameIsBadConnections, light->observation.source_ordinal) : nullptr))
      continue;

    if (it->second > max) {
      max = it->second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(pKF);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }

  scale_filter.Finish();

  // Include also some not-already-included keyframes that are neighbors to
  } // complete filter loop; allocation/clear and neighbor work remain remainder
  // already-included keyframes
  for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                          itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80) // 80
      break;

    KeyFrame *pKF = *itKF;

    const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10,
        local_detail ? local_detail->keyframe_connections.Next(
            LocalMapLockSite::CovisibleConnections) : nullptr);

    for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(),
                                            itEndNeighKF = vNeighs.end();
         itNeighKF != itEndNeighKF; itNeighKF++) {
      KeyFrame *pNeighKF = *itNeighKF;
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    const set<KeyFrame *> spChilds = pKF->GetChilds(local_detail
        ? local_detail->keyframe_connections.Next(LocalMapLockSite::ChildrenConnections)
        : nullptr);
    for (set<KeyFrame *>::const_iterator sit = spChilds.begin(),
                                         send = spChilds.end();
         sit != send; sit++) {
      KeyFrame *pChildKF = *sit;
      if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    KeyFrame *pParent = pKF->GetParent(local_detail
        ? local_detail->keyframe_connections.Next(LocalMapLockSite::ParentConnections)
        : nullptr);
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }
  }

  // Add 10 last temporal KFs (mainly for IMU)
  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      mvpLocalKeyFrames.size() < 80) {
    KeyFrame *tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

    const int Nd = 20;
    for (int i = 0; i < Nd; i++) {
      if (!tempKeyFrame)
        break;
      if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(tempKeyFrame);
        tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        tempKeyFrame = tempKeyFrame->mPrevKF;
      }
    }
  }

  if (auto* scale = ActiveMapScaleObservation())
    scale->local_kf = mvpLocalKeyFrames.size();

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  FieldStage field_stage(mCurrentFrame.mnId,4);
  ScopedTimingAccumulator detailed_timing(
      mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->relocalization_ms : nullptr);
  Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
  SetLatestRelocalizationStatus(RelocalizationRunning);
  // Compute Bag of Words Vector
  mCurrentFrame.ComputeBoW();

  // Relocalization is performed when tracking is lost
  // Track Lost: Query KeyFrame Database for keyframe candidates for
  // relocalisation
  vector<KeyFrame *> vpCandidateKFs =
      mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame,
                                                   mpAtlas->GetCurrentMap());

  if (vpCandidateKFs.empty()) {
    Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
    cout << "Relocalization: No candidates found!" << endl;
    SetLatestRelocalizationStatus(RelocalizationFailed);
    return field_stage.Result(false,__LINE__);
  }
  cout << "Relocalization: Found " << vpCandidateKFs.size() << " candidates"
       << endl;

  const int nKFs = vpCandidateKFs.size();

  // We perform first an ORB matching with each candidate
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.75, true);

  vector<std::unique_ptr<MLPnPsolver>> vpMLPnPsolvers;
  vpMLPnPsolvers.resize(nKFs);

  vector<vector<MapPointPtr >> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame *pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches =
          matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
      // Lowered from 15 to 10: mid-map positions visited once may have fewer
      // overlapping features with the stored KF.
      cout << "Relocalization candidate KF " << pKF->mnId
           << ": BoW matches=" << nmatches << endl;
      if (nmatches < 10) {
        vbDiscarded[i] = true;
        continue;
      } else {
        vpMLPnPsolvers[i].reset(
            new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]));
        // minInliers lowered from 10 to 6 (minimal solver size) so that RANSAC
        // isn't blocked before even attempting a pose.
        vpMLPnPsolvers[i]->SetRansacParameters(
            0.99, 6, 300, 6, 0.5,
            5.991); // This solver needs at least 6 points
        nCandidates++;
      }
    }
  }

  // Alternatively perform some iterations of P4P RANSAC
  // Until we found a camera pose supported by enough inliers
  bool bMatch = false;
  KeyFrame *pMatchedRelocKF = nullptr;
  ORBmatcher matcher2(0.9, true);

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i])
        continue;

      // Perform 5 Ransac Iterations
      vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      MLPnPsolver *pSolver = vpMLPnPsolvers[i].get();
      Eigen::Matrix4f eigTcw;
      bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (bTcw) {
        Sophus::SE3f Tcw(eigTcw);
        mCurrentFrame.SetPose(Tcw);
        // Tcw.copyTo(mCurrentFrame.mTcw);

        set<MapPointPtr > sFound;

        const int np = vbInliers.size();

        for (int j = 0; j < np; j++) {
          if (vbInliers[j]) {
            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
            sFound.insert(vvpMapPointMatches[i][j]);
          } else
            mCurrentFrame.mvpMapPoints[j] = NULL;
        }

        int nGood = FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
        cout << "Relocalization KF " << vpCandidateKFs[i]->mnId
             << ": inliers after 1st PoseOpt=" << nGood << endl;

        if (nGood < 10)
          continue;

        for (int io = 0; io < mCurrentFrame.N; io++)
          if (mCurrentFrame.mvbOutlier[io])
            mCurrentFrame.mvpMapPoints[io] = MapPointPtr();

        // If few inliers, search by projection in a coarse window and optimize
        // again.
        // Strategy for mid-map single-visit locations:
        //   1. Wider initial search radius (15px)
        //   2. Also search covisible KFs' MapPoints to expand the pool
        //   3. Final threshold lowered to 20 (stereo depth gives accurate 3D)
        if (nGood < 50) {
          // Primary: wider search of candidate KF's own MapPoints
          int nadditional = matcher2.SearchByProjection(
              mCurrentFrame, vpCandidateKFs[i], sFound, 15, 100);

          // Secondary: also pull in MapPoints from the top-5 covisible KFs.
          vector<KeyFrame *> vpCovis =
              vpCandidateKFs[i]->GetBestCovisibilityKeyFrames(5);
          for (KeyFrame *pKFcov : vpCovis) {
            if (!pKFcov || pKFcov->isBad())
              continue;
            int nFromCov = matcher2.SearchByProjection(mCurrentFrame, pKFcov,
                                                       sFound, 15, 100);
            nadditional += nFromCov;
          }
          cout << "Relocalization KF " << vpCandidateKFs[i]->mnId
               << ": additional via projection+covis=" << nadditional << endl;

          if (nadditional + nGood >= 20) {
            nGood = FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
            cout << "Relocalization KF " << vpCandidateKFs[i]->mnId
                 << ": inliers after 2nd PoseOpt=" << nGood << endl;

            // Tight second-pass search if still borderline
            if (nGood > 10 && nGood < 50) {
              sFound.clear();
              for (int ip = 0; ip < mCurrentFrame.N; ip++)
                if (mCurrentFrame.mvpMapPoints[ip])
                  sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
              nadditional = matcher2.SearchByProjection(
                  mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);
              // Second-pass on covisible KFs
              for (KeyFrame *pKFcov : vpCovis) {
                if (!pKFcov || pKFcov->isBad())
                  continue;
                nadditional += matcher2.SearchByProjection(
                    mCurrentFrame, pKFcov, sFound, 3, 64);
              }

              if (nGood + nadditional >= 20) {
                nGood = FieldPoseOptimization(&mCurrentFrame,
        RgbdLightDiagnostics() && mpCurrentRgbdTiming ? &mpCurrentRgbdTiming->pose : nullptr);
                cout << "Relocalization KF " << vpCandidateKFs[i]->mnId
                     << ": inliers after 3rd PoseOpt=" << nGood << endl;

                for (int io = 0; io < mCurrentFrame.N; io++)
                  if (mCurrentFrame.mvbOutlier[io])
                    mCurrentFrame.mvpMapPoints[io] = NULL;
              }
            }
          }
        }

        // Accept if >= 20 inliers (was 50, then 30).
        // Stereo depth gives accurate 3D → 20 reliable inliers is sufficient.
        if (nGood >= 20) {
          bMatch = true;
          pMatchedRelocKF = vpCandidateKFs[i];
          cout << "Relocalization: accepted with " << nGood << " inliers"
               << endl;
          break;
        } else {
          cout << "Relocalization KF " << vpCandidateKFs[i]->mnId
               << ": rejected, only " << nGood << " inliers" << endl;
        }
      }
    }
  }

  if (!bMatch) {
    SetLatestRelocalizationStatus(RelocalizationFailed);
    return field_stage.Result(false,__LINE__);
  } else {
    mnLastRelocFrameId = mCurrentFrame.mnId;
    mpLocalBudgetRelocation = pMatchedRelocKF;
    if (pMatchedRelocKF && !pMatchedRelocKF->isBad()) {
      mpReferenceKF = pMatchedRelocKF;
      mpLastKeyFrame = pMatchedRelocKF;
      mCurrentFrame.mpReferenceKF = pMatchedRelocKF;
    }
    // After relocalization, enable TrackWithMotionModel (projection-based) for
    // the next frame instead of TrackReferenceKeyFrame (BoW-based).
    // Reason: BoW matching of D435i infrared images against an RGB-trained
    // vocabulary yields only 3-12 matches, which is too few to maintain
    // tracking. Projection-based search uses the known pose to reproject all
    // visible MapPoints into the new frame within a pixel search radius,
    // returning 100+ matches regardless of vocabulary quality.
    // Use identity velocity (zero motion assumption) as a conservative
    // starting point; the next frame can then use projection matching around
    // the relocalized pose instead of falling back to weak infrared BoW
    // reference-keyframe matching.
    mVelocity = Sophus::SE3f();
    mbVelocity = true;

    // Re-bootstrap IMU preintegration chain from the matched keyframe.
    // Do not delete the previous object here: current/previous frame copies
    // can still reference it in this tracking iteration.
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD) {
      KeyFrame *pBaseKF = pMatchedRelocKF ? pMatchedRelocKF : mpLastKeyFrame;
      if (pBaseKF) {
        mpImuPreintegratedFromLastKF =
            new IMU::Preintegrated(pBaseKF->GetImuBias(), pBaseKF->mImuCalib);
      } else {
        mpImuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
      }
      mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
      mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;
    }

    cout << "Relocalized!!" << endl;
    SetLatestRelocalizationStatus(RelocalizationSucceed);
    return field_stage.Result(true,__LINE__);
  }
}

void Tracking::Reset(bool bLocMap) {
  ++rgbd_reset_count;
  ResetLocalMapBudgetHistory();
  Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

  if (mpViewer) {
    mpViewer->RequestStop();
    while (!mpViewer->isStopped())
      usleep(3000);
  }

  // Reset Local Mapping
  if (!bLocMap) {
    Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
    mpLocalMapper->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
  }

  // Reset Loop Closing
  Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
  mpLoopClosing->RequestReset();
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear BoW Database
  Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
  mpKeyFrameDB->clear();
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpAtlas->clearAtlas();
  mpAtlas->CreateNewMap();
  if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR ||
      mSensor == System::IMU_RGBD)
    mpAtlas->SetInertialSensor();
  mnInitialFrameId = 0;

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  (mState = NO_IMAGES_YET, FieldEmit(15,mCurrentFrame.mnId,0,1,mState));

  mbReadyToInitializate = false;
  mbSetInit = false;

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();
  mCurrentFrame = Frame();
  mnLastRelocFrameId = 0;
  mLastFrame = Frame();
  mpReferenceKF = static_cast<KeyFrame *>(NULL);
  mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
  mvIniMatches.clear();

  if (mpViewer)
    mpViewer->Release();

  Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap) {
  ++rgbd_active_reset_count;
  ResetLocalMapBudgetHistory();
  Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
  if (mpViewer) {
    mpViewer->RequestStop();
    while (!mpViewer->isStopped())
      usleep(3000);
  }

  Map *pMap = mpAtlas->GetCurrentMap();

  if (!bLocMap) {
    Verbose::PrintMess("Reseting Local Mapper...",
                       Verbose::VERBOSITY_VERY_VERBOSE);
    mpLocalMapper->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
  }

  // Reset Loop Closing
  Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
  mpLoopClosing->RequestResetActiveMap(pMap);
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear BoW Database
  Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
  mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpAtlas->clearMap();

  // KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
  // Frame::nNextId = mnLastInitFrameId;
  mnLastInitFrameId = Frame::nNextId;
  // mnLastRelocFrameId = mnLastInitFrameId;
  (mState = NO_IMAGES_YET, FieldEmit(15,mCurrentFrame.mnId,0,1,mState)); // NOT_INITIALIZED;

  mbReadyToInitializate = false;

  list<bool> lbLost;
  // lbLost.reserve(mlbLost.size());
  unsigned int index = mnFirstFrameId;
  cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
  for (Map *pMap : mpAtlas->GetAllMaps()) {
    if (pMap->GetAllKeyFrames().size() > 0) {
      if (index > pMap->GetLowerKFID())
        index = pMap->GetLowerKFID();
    }
  }

  // cout << "First Frame id: " << index << endl;
  int num_lost = 0;
  cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

  for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end();
       ilbL++) {
    if (index < mnInitialFrameId)
      lbLost.push_back(*ilbL);
    else {
      lbLost.push_back(true);
      num_lost += 1;
    }

    index++;
  }
  cout << num_lost << " Frames set to lost" << endl;

  mlbLost = lbLost;

  mnInitialFrameId = mCurrentFrame.mnId;
  mnLastRelocFrameId = mCurrentFrame.mnId;

  mCurrentFrame = Frame();
  mLastFrame = Frame();
  mpReferenceKF = static_cast<KeyFrame *>(NULL);
  mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
  mvIniMatches.clear();

  mbVelocity = false;

  if (mpViewer)
    mpViewer->Release();

  Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPointPtr > Tracking::GetLocalMapMPS() { return mvpLocalMapPoints; }

void Tracking::ChangeCalibration(const string &strSettingPath) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];

  mK_.setIdentity();
  mK_(0, 0) = fx;
  mK_(1, 1) = fy;
  mK_(0, 2) = cx;
  mK_(1, 2) = cy;

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  K.copyTo(mK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = fSettings["Camera.bf"];

  Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag) {
  mbOnlyTracking = flag;
  if (flag) {
    SetLatestRelocalizationStatus(RelocalizationRunning);
  }
}

bool Tracking::TryGetLatestRelocalizationStatus(
    eRelocalizationStatus &status) const {
  unique_lock<mutex> lock(mMutexRelocalizationStatus);
  if (!mbHasLatestRelocalizationStatus) {
    return false;
  }

  status = mLatestRelocalizationStatus;
  return true;
}

bool Tracking::IsOnlyTrackingEnabled() const { return mbOnlyTracking; }

void Tracking::SetLatestRelocalizationStatus(eRelocalizationStatus status) {
  unique_lock<mutex> lock(mMutexRelocalizationStatus);
  mLatestRelocalizationStatus = status;
  mbHasLatestRelocalizationStatus = true;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b,
                              KeyFrame *pCurrentKeyFrame) {
  Map *pMap = pCurrentKeyFrame->GetMap();
  unsigned int index = mnFirstFrameId;
  list<ORB_SLAM3::KeyFrame *>::iterator lRit = mlpReferences.begin();
  list<bool>::iterator lbL = mlbLost.begin();
  for (auto lit = mlRelativeFramePoses.begin(),
            lend = mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lbL++) {
    if (*lbL)
      continue;

    KeyFrame *pKF = *lRit;

    while (pKF->isBad()) {
      pKF = pKF->GetParent();
    }

    if (pKF->GetMap() == pMap) {
      (*lit).translation() *= s;
    }
  }

  mLastBias = b;

  mpLastKeyFrame = pCurrentKeyFrame;

  mLastFrame.SetNewBias(mLastBias);
  mCurrentFrame.SetNewBias(mLastBias);

  while (!mCurrentFrame.imuIsPreintegrated()) {
    usleep(500);
  }

  if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId) {
    mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                  mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                  mLastFrame.mpLastKeyFrame->GetVelocity());
  } else {
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
    float t12 = mLastFrame.mpImuPreintegrated->dT;

    mLastFrame.SetImuPoseVelocity(
        IMU::NormalizeRotation(
            Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
        Vwb1 + Gz * t12 +
            Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
  }

  if (mCurrentFrame.mpImuPreintegrated) {
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

    const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
    float t12 = mCurrentFrame.mpImuPreintegrated->dT;

    mCurrentFrame.SetImuPoseVelocity(
        IMU::NormalizeRotation(
            Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
        Vwb1 + Gz * t12 +
            Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
  }

  mnFirstImuFrameId = mCurrentFrame.mnId;
}

void Tracking::NewDataset() { mnNumDataset++; }

int Tracking::GetNumberDataset() { return mnNumDataset; }

int Tracking::GetMatchesInliers() { return mnMatchesInliers; }

void Tracking::SaveSubTrajectory(string strNameFile_frames,
                                 string strNameFile_kf, string strFolder) {
  mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
  // mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames,
                                 string strNameFile_kf, Map *pMap) {
  mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
  if (!strNameFile_kf.empty())
    mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::GetImageScale() { return mImageScale; }

#ifdef REGISTER_LOOP
void Tracking::RequestStop() {
  unique_lock<mutex> lock(mMutexStop);
  mbStopRequested = true;
}

bool Tracking::Stop() {
  unique_lock<mutex> lock(mMutexStop);
  if (mbStopRequested && !mbNotStop) {
    mbStopped = true;
    cout << "Tracking STOP" << endl;
    return true;
  }

  return false;
}

bool Tracking::stopRequested() {
  unique_lock<mutex> lock(mMutexStop);
  return mbStopRequested;
}

bool Tracking::isStopped() {
  unique_lock<mutex> lock(mMutexStop);
  return mbStopped;
}

void Tracking::Release() {
  unique_lock<mutex> lock(mMutexStop);
  mbStopped = false;
  mbStopRequested = false;
}
#endif

} // namespace ORB_SLAM3
