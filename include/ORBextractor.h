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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H
#include "RgbdTiming.h"
#include "FastGridKernel.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <list>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

using namespace std;

namespace ORB_SLAM3 {

class ExtractorNode {
public:
  ExtractorNode() : bNoMore(false) {}

  void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3,
                  ExtractorNode &n4);

  std::vector<cv::KeyPoint> vKeys;
  cv::Point2i UL, UR, BL, BR;
  std::list<ExtractorNode>::iterator lit;
  bool bNoMore;
};

class ORBextractor {
public:
  enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

  static constexpr std::size_t kMaxTimedLevels = 16;

  struct LevelTiming {
    double fast_initial_ms = 0.0;
    double fast_fallback_ms = 0.0;
    double merge_ms = 0.0, merge_cpu_ms = 0.0;
    double octree_ms = 0.0;
    double orientation_ms = 0.0;
    double copy_ms = 0.0;
    double gaussian_ms = 0.0;
    double descriptor_ms = 0.0;
    std::uint32_t candidate_count = 0;
    std::uint32_t fallback_cells = 0;
    std::uint32_t octree_splits = 0;
  };

  struct StageTiming {
    OrbTimingRecord observation;
    double pyramid_ms = 0.0;
    double keypoints_octree_ms = 0.0;
    double orientation_ms = 0.0;
    double blur_ms = 0.0;
    double descriptor_ms = 0.0;
    double result_assembly_ms = 0.0;
    double fast_initial_ms = 0.0;
    double fast_fallback_ms = 0.0;
    double octree_ms = 0.0;
    double copy_ms = 0.0;
    double gaussian_ms = 0.0;
    std::uint64_t output_hash = 0;
    int mono_index = 0;
    int level_count = 0;
    std::array<LevelTiming, kMaxTimedLevels> levels{};
  };

  ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST,
               int minThFAST, int parallelWorkers = 1);

  ~ORBextractor();
  // Terminal operation after the caller has finished its last extraction.
  // Does not delete extractor/image storage; not concurrent with operator().
  void StopAndJoinWorkers();

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  int operator()(cv::InputArray _image, cv::InputArray _mask,
                 std::vector<cv::KeyPoint> &_keypoints,
                 cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

  int inline GetLevels() { return nlevels; }

  const StageTiming &GetLastStageTiming() const { return mLastStageTiming; }

  std::size_t GetFastWorkspaceBytes() const {
    std::size_t bytes = 0;
    for (const auto &workspace : mFastKernelWorkspaces)
      bytes += workspace.CapacityBytes();
    return bytes;
  }

  // Set only between complete extractions on their calling thread.
  void SetObservationTiming(bool light, bool detail) {
    mCollectLight = light;
    mCollectDetail = light && detail;
  }

  float inline GetScaleFactor() { return scaleFactor; }

  std::vector<float> inline GetScaleFactors() { return mvScaleFactor; }

  std::vector<float> inline GetInverseScaleFactors() {
    return mvInvScaleFactor;
  }

  std::vector<float> inline GetScaleSigmaSquares() { return mvLevelSigma2; }

  std::vector<float> inline GetInverseScaleSigmaSquares() {
    return mvInvLevelSigma2;
  }

  std::vector<cv::Mat> mvImagePyramid;

protected:
  StageTiming mLastStageTiming;

  void EnsureWorkspace(const cv::Mat &image);
  void ComputePyramid(cv::Mat image);
  void
  ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
  void ComputeKeyPointsOctTreeLevel(int level,
                                    std::vector<cv::KeyPoint> &keypoints,
                                    LevelTiming &timing);
  void PrepareFastRowTasks(const cv::Size &imageSize);
  void ComputeFastRow(int taskIndex, int workerSlot,
                      std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
  void FinishKeyPointsOctTreeLevel(int level,
                                    const std::vector<cv::KeyPoint> &candidates,
                                    std::vector<cv::KeyPoint> &keypoints,
                                    LevelTiming &timing);
  void DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys,
                         const int &minX, const int &maxX, const int &minY,
                         const int &maxY, const int &nFeatures,
                         const int &level,
                         std::vector<cv::KeyPoint> &resultKeys,
                         std::uint32_t *splitCount = nullptr);

  void RunLevelTasks(int taskCount, const std::function<void(int, int)> &task);
  void WorkerLoop(int workerSlot);
  void ExecuteAvailableTasks(int workerSlot);

  void
  ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
  std::vector<cv::Point> pattern;

  int nfeatures;
  double scaleFactor;
  int nlevels;
  int iniThFAST;
  int minThFAST;
  int mParallelWorkers;

  std::vector<int> mnFeaturesPerLevel;

  std::vector<int> umax;

  std::vector<float> mvScaleFactor;
  std::vector<float> mvInvScaleFactor;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;

  std::vector<cv::Mat> mPyramidStorage;
  std::vector<cv::Mat> mBlurredPyramid;
  std::vector<cv::Mat> mLevelDescriptors;
  std::vector<std::vector<cv::KeyPoint>> mAllKeypoints;
  std::vector<std::vector<cv::KeyPoint>> mCellKeypoints;
  std::vector<std::vector<cv::KeyPoint>> mCandidateKeypoints;

  // Built/resized only by the caller before a dispatch. Each row task owns its
  // cells and timing; the last completed row merges the level in original order.
  struct FastRowWorkspace {
    int level = 0, row = 0;
    std::vector<std::vector<cv::KeyPoint>> cells;
    LevelTiming timing;
    double initial_cpu_ms = 0.0, fallback_cpu_ms = 0.0;
  };
  struct FastLevelWorkspace {
    int first_row = 0, row_count = 0, col_count = 0;
    int cell_width = 0, cell_height = 0;
    int max_border_x = 0, max_border_y = 0;
  };
  std::vector<FastRowWorkspace> mFastRows;
  std::array<FastLevelWorkspace, kMaxTimedLevels> mFastLevels{};
  std::array<std::atomic<int>, kMaxTimedLevels> mFastRowsRemaining{};
  cv::Size mFastWorkspaceImageSize;
  bool mUseFastRows = false;

  std::array<FastGridWorkspace, kMaxRgbdOrbWorkers> mFastKernelWorkspaces;

  std::vector<std::thread> mWorkerThreads;
  std::mutex mWorkerMutex;
  std::condition_variable mWorkerCv;
  std::condition_variable mWorkerDoneCv;
  std::function<void(int, int)> mLevelTask;
  std::atomic<int> mNextTask{0};
  int mTaskCount = 0;
  int mCompletedHelpers = 0;
  std::size_t mTaskGeneration = 0;
  bool mStopWorkers = false;
  std::exception_ptr mWorkerException;
  struct WorkerTiming {
    double cpu_ms = 0.0;
    std::uint32_t tasks = 0;
    // Keep slots apart without requiring over-aligned C++14 allocation.
    std::array<unsigned char, 52> padding{};
  };
  std::array<WorkerTiming, kMaxRgbdOrbWorkers> mWorkerTiming{};
  bool mCollectLight = false;
#ifdef ORB_SLAM3_ORB_FINE_TIMING
  bool mCollectDetail = true;
#else
  bool mCollectDetail = false;
#endif
  unsigned mObservationBatch = 0;
  double mLastBatchHelperCpu = 0.0;
};

} // namespace ORB_SLAM3

#endif
