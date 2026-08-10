/**
 * Optional stereo-depth refinement hook used by feature-front-end experiments.
 *
 * A provider may add or replace depth for existing left-image observations,
 * but must not reorder keypoints or descriptors. This preserves the original
 * ORB tracking, MapPoint, BoW and loop-closing semantics in Hybrid mode.
 */
#ifndef STEREO_DEPTH_PROVIDER_H
#define STEREO_DEPTH_PROVIDER_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <vector>

namespace ORB_SLAM3 {

struct StereoDepthRefinementStats {
  int candidates = 0;
  int fine_matches = 0;
  int accepted = 0;
  int added = 0;
  int replaced = 0;
  int rejected = 0;
  double total_ms = 0.0;
};

class StereoDepthProvider {
public:
  virtual ~StereoDepthProvider() = default;

  virtual StereoDepthRefinementStats Refine(
      const cv::Mat &left, const cv::Mat &right,
      const std::vector<cv::KeyPoint> &left_keypoints, float bf,
      std::vector<float> &right_coordinates,
      std::vector<float> &depths) = 0;
};

} // namespace ORB_SLAM3

#endif // STEREO_DEPTH_PROVIDER_H
