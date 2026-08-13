#ifndef STEREO_FEATURE_PROVIDER_H
#define STEREO_FEATURE_PROVIDER_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <vector>

namespace ORB_SLAM3 {

struct StereoFeatureExtractionStats {
  double feature_extract_ms = 0.0;
  double stereo_match_ms = 0.0;
  double fine_match_ms = 0.0;
  int stereo_candidates = 0;
  int stereo_matches = 0;
};

struct StereoFeatureOutput {
  std::vector<cv::KeyPoint> left_keypoints;
  std::vector<cv::KeyPoint> right_keypoints;
  cv::Mat left_descriptors;
  cv::Mat right_descriptors;
  std::vector<float> right_coordinates;
  std::vector<float> depths;
  StereoFeatureExtractionStats stats;
};

struct ReferenceFeatureMatch {
  int reference_index = -1;
  int current_index = -1;
  int descriptor_distance = 0;
  cv::Point2f refined_current_point;
  float confidence = 0.0F;
};

// Full stereo front-end replacement seam. Unlike StereoDepthProvider, this
// provider owns detector keypoints and descriptors as well as stereo depth.
// F23 implementations return CV_32FC1 XFeat descriptors. Vocabulary and Atlas
// compatibility remain F24 responsibilities.
class StereoFeatureProvider {
public:
  virtual ~StereoFeatureProvider() = default;

  virtual StereoFeatureOutput Extract(const cv::Mat &left,
                                      const cv::Mat &right, float bf) = 0;

  virtual std::vector<ReferenceFeatureMatch> RefineReferenceMatches(
      const std::vector<cv::KeyPoint> &reference_keypoints,
      const cv::Mat &reference_descriptors,
      const std::vector<cv::KeyPoint> &current_keypoints,
      const cv::Mat &current_descriptors,
      const std::vector<ReferenceFeatureMatch> &coarse_matches,
      int image_width, int image_height) = 0;
};

} // namespace ORB_SLAM3

#endif // STEREO_FEATURE_PROVIDER_H
