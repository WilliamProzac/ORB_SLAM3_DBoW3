#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "hb_dnn.h"

struct StageTiming {
  double preprocess_ms = 0.0;
  double bpu_queue_ms = 0.0;
  double bpu_ms = 0.0;
  double postprocess_ms = 0.0;
  double total_ms = 0.0;
};

struct FeatureSet {
  std::vector<cv::Point2f> points;
  std::vector<float> scores;
  std::vector<float> scales;
  cv::Mat descriptors;
  int image_width = 0;
  int image_height = 0;
  StageTiming timing;
};

struct FineMatchResult {
  std::vector<cv::DMatch> matches;
  // One refined train-image coordinate for each element in matches. The query
  // coordinate remains the detector output, as required by rectified stereo.
  std::vector<cv::Point2f> refined_train_points;
  std::vector<float> confidences;
  StageTiming timing;
};

class XFeatBpuFrontend {
 public:
  XFeatBpuFrontend(const std::string &model_path, int top_k,
                   float detection_threshold, int bpu_core = 0,
                   std::mutex *bpu_mutex = nullptr, bool fast_decode = false,
                   bool semi_dense_single = false, int grid_columns = 8,
                   int grid_rows = 6,
                   int extraction_grid_maximum_per_cell = -1);
  ~XFeatBpuFrontend();

  XFeatBpuFrontend(const XFeatBpuFrontend &) = delete;
  XFeatBpuFrontend &operator=(const XFeatBpuFrontend &) = delete;

  FeatureSet extract(const cv::Mat &mono8);
  const std::string &model_name() const { return model_name_; }
  int input_width() const { return input_width_; }
  int input_height() const { return input_height_; }

 private:
  float output_value(int output_index, int channel, int y, int x) const;
  void allocate_tensors();
  void release_tensors();

  hbPackedDNNHandle_t packed_ = nullptr;
  hbDNNHandle_t model_ = nullptr;
  std::vector<hbDNNTensor> inputs_;
  std::vector<hbDNNTensor> outputs_;
  std::string model_name_;
  int top_k_ = 600;
  float detection_threshold_ = 0.05F;
  int bpu_core_ = 0;
  std::mutex *bpu_mutex_ = nullptr;
  bool fast_decode_ = false;
  bool semi_dense_single_ = false;
  int grid_columns_ = 8;
  int grid_rows_ = 6;
  int extraction_grid_maximum_per_cell_ = -1;
  int input_width_ = 0;
  int input_height_ = 0;
  int feature_width_ = 0;
  int feature_height_ = 0;
  int dense_index_ = -1;
  int logits_index_ = -1;
  int reliability_index_ = -1;
};

class FixedFineMatcherBpu {
 public:
  FixedFineMatcherBpu(const std::string &model_path, int bpu_core = 0,
                      std::mutex *bpu_mutex = nullptr);
  ~FixedFineMatcherBpu();

  FixedFineMatcherBpu(const FixedFineMatcherBpu &) = delete;
  FixedFineMatcherBpu &operator=(const FixedFineMatcherBpu &) = delete;

  FineMatchResult refine(const FeatureSet &query, const FeatureSet &train,
                         const std::vector<cv::DMatch> &coarse_matches,
                         float minimum_confidence, bool refine_y,
                         int image_width, int image_height);
  int capacity() const { return capacity_; }
  const std::string &model_name() const { return model_name_; }

 private:
  void release_tensors();
  float output_value(int match, int bin) const;

  hbPackedDNNHandle_t packed_ = nullptr;
  hbDNNHandle_t model_ = nullptr;
  hbDNNTensor input_{};
  hbDNNTensor output_{};
  std::string model_name_;
  int capacity_ = 0;
  int bpu_core_ = 0;
  std::mutex *bpu_mutex_ = nullptr;
};

std::vector<cv::DMatch> match_xfeat(const FeatureSet &a, const FeatureSet &b,
                                    float minimum_cosine,
                                    double *latency_ms = nullptr);

std::vector<cv::DMatch> match_xfeat_stereo_guided(
    const FeatureSet &left, const FeatureSet &right, float minimum_cosine,
    float ratio_threshold, float vertical_tolerance, float maximum_disparity,
    size_t maximum_matches, double *latency_ms = nullptr, int grid_columns = 8,
    int grid_rows = 6, size_t grid_maximum_per_cell = 8);

std::vector<cv::DMatch> match_xfeat_temporal_guided(
    const FeatureSet &reference, const FeatureSet &current, float minimum_cosine,
    float ratio_threshold, float search_radius, size_t maximum_matches,
    double *latency_ms = nullptr, int grid_columns = 8, int grid_rows = 6,
    size_t grid_maximum_per_cell = 8);
