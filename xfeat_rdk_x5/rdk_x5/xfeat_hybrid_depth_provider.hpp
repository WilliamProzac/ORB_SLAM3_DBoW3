#pragma once

#include "StereoDepthProvider.h"
#include "xfeat_frontend.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace ORB_SLAM3 {

struct XFeatHybridDepthOptions {
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
  float association_radius = 5.0F;
  float replacement_ncc_margin = 0.05F;
  int grid_columns = 8;
  int grid_rows = 6;
  int extraction_grid_maximum_per_cell = -1;
  int match_grid_maximum_per_cell = 16;
  int bpu_core = 0;
};

class XFeatHybridDepthProvider final : public StereoDepthProvider {
public:
  explicit XFeatHybridDepthProvider(const XFeatHybridDepthOptions &options);

  StereoDepthRefinementStats Refine(
      const cv::Mat &left, const cv::Mat &right,
      const std::vector<cv::KeyPoint> &left_keypoints, float bf,
      std::vector<float> &right_coordinates,
      std::vector<float> &depths) override;

private:
  XFeatHybridDepthOptions options_;
  std::mutex bpu_mutex_;
  std::unique_ptr<XFeatBpuFrontend> left_frontend_;
  std::unique_ptr<XFeatBpuFrontend> right_frontend_;
  std::unique_ptr<FixedFineMatcherBpu> fine_matcher_;
};

} // namespace ORB_SLAM3
