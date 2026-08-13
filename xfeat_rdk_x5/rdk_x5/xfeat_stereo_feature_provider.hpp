#pragma once

#include "StereoFeatureProvider.h"
#include "xfeat_frontend.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace ORB_SLAM3 {

struct XFeatStereoFeatureOptions {
  std::string backbone_model;
  std::string fine_model;
  int max_features = 600;
  int fine_batch_size = 384;
  float minimum_cosine = 0.82F;
  float ratio_threshold = 0.90F;
  float fine_confidence = 0.20F;
  float maximum_disparity = 160.0F;
  float vertical_tolerance = 2.0F;
  float minimum_patch_ncc = 0.50F;
  int grid_columns = 8;
  int grid_rows = 6;
  int extraction_grid_maximum_per_cell = -1;
  int match_grid_maximum_per_cell = 16;
  int bpu_core = 0;
};

class XFeatStereoFeatureProvider final : public StereoFeatureProvider {
public:
  explicit XFeatStereoFeatureProvider(
      const XFeatStereoFeatureOptions &options);

  StereoFeatureOutput Extract(const cv::Mat &left, const cv::Mat &right,
                              float bf) override;

  std::vector<ReferenceFeatureMatch> RefineReferenceMatches(
      const std::vector<cv::KeyPoint> &reference_keypoints,
      const cv::Mat &reference_descriptors,
      const std::vector<cv::KeyPoint> &current_keypoints,
      const cv::Mat &current_descriptors,
      const std::vector<ReferenceFeatureMatch> &coarse_matches,
      int image_width, int image_height) override;

private:
  XFeatStereoFeatureOptions options_;
  std::mutex bpu_mutex_;
  std::unique_ptr<XFeatBpuFrontend> left_frontend_;
  std::unique_ptr<XFeatBpuFrontend> right_frontend_;
  std::unique_ptr<FixedFineMatcherBpu> fine_matcher_;
};

} // namespace ORB_SLAM3
