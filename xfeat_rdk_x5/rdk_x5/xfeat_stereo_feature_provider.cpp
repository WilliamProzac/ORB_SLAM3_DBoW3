#include "xfeat_stereo_feature_provider.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace ORB_SLAM3 {
namespace {

using Clock = std::chrono::steady_clock;

double ElapsedMilliseconds(const Clock::time_point &start,
                           const Clock::time_point &end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

bool PatchNcc(const cv::Mat &left, const cv::Mat &right,
              const cv::Point2f &left_point,
              const cv::Point2f &right_point, float &ncc) {
  constexpr int kPatchSize = 9;
  constexpr float kHalfPatch = (kPatchSize - 1) * 0.5F;
  if (left_point.x < kHalfPatch || left_point.y < kHalfPatch ||
      right_point.x < kHalfPatch || right_point.y < kHalfPatch ||
      left_point.x >= left.cols - kHalfPatch ||
      left_point.y >= left.rows - kHalfPatch ||
      right_point.x >= right.cols - kHalfPatch ||
      right_point.y >= right.rows - kHalfPatch) {
    return false;
  }

  cv::Mat left_patch;
  cv::Mat right_patch;
  cv::getRectSubPix(left, cv::Size(kPatchSize, kPatchSize), left_point,
                    left_patch);
  cv::getRectSubPix(right, cv::Size(kPatchSize, kPatchSize), right_point,
                    right_patch);
  left_patch.convertTo(left_patch, CV_32F);
  right_patch.convertTo(right_patch, CV_32F);
  left_patch -= cv::mean(left_patch)[0];
  right_patch -= cv::mean(right_patch)[0];
  const double left_norm = cv::norm(left_patch);
  const double right_norm = cv::norm(right_patch);
  if (left_norm < 1e-6 || right_norm < 1e-6) {
    return false;
  }
  ncc = static_cast<float>(
      left_patch.dot(right_patch) / (left_norm * right_norm));
  return std::isfinite(ncc);
}

std::vector<cv::KeyPoint> ToKeyPoints(const FeatureSet &features) {
  if (features.points.size() != features.scores.size() ||
      features.points.size() != features.scales.size()) {
    throw std::runtime_error("XFeat point/score/scale arrays do not align");
  }
  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(features.points.size());
  for (std::size_t index = 0; index < features.points.size(); ++index) {
    const float scale = std::max(features.scales[index], 1.0F);
    const int octave = std::max(
        0, static_cast<int>(std::lround(std::log(scale) / std::log(1.2F))));
    keypoints.emplace_back(features.points[index], 8.0F * scale, -1.0F,
                           features.scores[index], octave);
  }
  return keypoints;
}

FeatureSet ToFeatureSet(const std::vector<cv::KeyPoint> &keypoints,
                        const cv::Mat &descriptors, int image_width,
                        int image_height) {
  if (descriptors.type() != CV_32FC1 ||
      descriptors.rows != static_cast<int>(keypoints.size())) {
    throw std::runtime_error(
        "Fine refinement requires one float descriptor per keypoint");
  }
  FeatureSet features;
  features.image_width = image_width;
  features.image_height = image_height;
  features.descriptors = descriptors;
  features.points.reserve(keypoints.size());
  features.scores.reserve(keypoints.size());
  features.scales.reserve(keypoints.size());
  for (const cv::KeyPoint &keypoint : keypoints) {
    features.points.push_back(keypoint.pt);
    features.scores.push_back(keypoint.response);
    features.scales.push_back(std::max(keypoint.size / 8.0F, 1.0F));
  }
  return features;
}

} // namespace

XFeatStereoFeatureProvider::XFeatStereoFeatureProvider(
    const XFeatStereoFeatureOptions &options)
    : options_(options) {
  if (options_.backbone_model.empty() || options_.fine_model.empty()) {
    throw std::invalid_argument(
        "XFeat mode requires backbone and Fine model paths");
  }
  if (options_.fine_batch_size != 384) {
    throw std::invalid_argument("F23 freezes the Fine capacity at 384");
  }
  left_frontend_.reset(new XFeatBpuFrontend(
      options_.backbone_model, options_.max_features, 0.05F,
      options_.bpu_core, &bpu_mutex_, false, true, options_.grid_columns,
      options_.grid_rows, options_.extraction_grid_maximum_per_cell));
  right_frontend_.reset(new XFeatBpuFrontend(
      options_.backbone_model, options_.max_features, 0.05F,
      options_.bpu_core, &bpu_mutex_, false, true, options_.grid_columns,
      options_.grid_rows, options_.extraction_grid_maximum_per_cell));
  fine_matcher_.reset(new FixedFineMatcherBpu(
      options_.fine_model, options_.bpu_core, &bpu_mutex_));
  if (fine_matcher_->capacity() != options_.fine_batch_size) {
    throw std::runtime_error("loaded Fine model capacity does not equal 384");
  }
}

StereoFeatureOutput XFeatStereoFeatureProvider::Extract(
    const cv::Mat &left, const cv::Mat &right, float bf) {
  const Clock::time_point start = Clock::now();
  if (left.type() != CV_8UC1 || right.type() != CV_8UC1 ||
      left.size() != right.size()) {
    throw std::runtime_error(
        "XFeat mode requires equal-size rectified mono8 images");
  }
  if (left.cols != left_frontend_->input_width() ||
      left.rows != left_frontend_->input_height()) {
    throw std::runtime_error(
        "XFeat image size does not match the frozen backbone");
  }

  std::future<FeatureSet> left_future =
      std::async(std::launch::async, [&]() { return left_frontend_->extract(left); });
  std::future<FeatureSet> right_future =
      std::async(std::launch::async, [&]() { return right_frontend_->extract(right); });
  FeatureSet left_features = left_future.get();
  FeatureSet right_features = right_future.get();
  const Clock::time_point extracted = Clock::now();

  double coarse_ms = 0.0;
  const std::vector<cv::DMatch> coarse = match_xfeat_stereo_guided(
      left_features, right_features, options_.minimum_cosine,
      options_.ratio_threshold, options_.vertical_tolerance,
      options_.maximum_disparity,
      static_cast<std::size_t>(options_.fine_batch_size), &coarse_ms,
      options_.grid_columns, options_.grid_rows,
      static_cast<std::size_t>(options_.match_grid_maximum_per_cell));
  const Clock::time_point coarse_finished = Clock::now();
  const FineMatchResult fine = fine_matcher_->refine(
      left_features, right_features, coarse, options_.fine_confidence, false,
      left.cols, left.rows);

  StereoFeatureOutput output;
  output.left_keypoints = ToKeyPoints(left_features);
  output.right_keypoints = ToKeyPoints(right_features);
  output.left_descriptors = left_features.descriptors.clone();
  output.right_descriptors = right_features.descriptors.clone();
  output.right_coordinates.assign(output.left_keypoints.size(), -1.0F);
  output.depths.assign(output.left_keypoints.size(), -1.0F);
  output.stats.feature_extract_ms = ElapsedMilliseconds(start, extracted);
  output.stats.stereo_match_ms =
      ElapsedMilliseconds(extracted, coarse_finished);
  output.stats.fine_match_ms = fine.timing.total_ms;
  output.stats.stereo_candidates = static_cast<int>(coarse.size());

  for (std::size_t index = 0; index < fine.matches.size(); ++index) {
    const cv::DMatch &match = fine.matches[index];
    if (match.queryIdx < 0 ||
        match.queryIdx >= static_cast<int>(left_features.points.size())) {
      continue;
    }
    const cv::Point2f left_point =
        left_features.points[static_cast<std::size_t>(match.queryIdx)];
    const cv::Point2f right_point = fine.refined_train_points[index];
    const float disparity = left_point.x - right_point.x;
    if (!std::isfinite(disparity) || disparity <= 0.0F ||
        disparity > options_.maximum_disparity) {
      continue;
    }
    float ncc = -1.0F;
    if (!PatchNcc(left, right, left_point, right_point, ncc) ||
        ncc < options_.minimum_patch_ncc) {
      continue;
    }
    const std::size_t query_index = static_cast<std::size_t>(match.queryIdx);
    output.right_coordinates[query_index] = right_point.x;
    output.depths[query_index] = bf / disparity;
    ++output.stats.stereo_matches;
  }
  return output;
}

std::vector<ReferenceFeatureMatch>
XFeatStereoFeatureProvider::RefineReferenceMatches(
    const std::vector<cv::KeyPoint> &reference_keypoints,
    const cv::Mat &reference_descriptors,
    const std::vector<cv::KeyPoint> &current_keypoints,
    const cv::Mat &current_descriptors,
    const std::vector<ReferenceFeatureMatch> &coarse_matches,
    int image_width, int image_height) {
  if (coarse_matches.empty()) {
    return std::vector<ReferenceFeatureMatch>();
  }

  std::vector<ReferenceFeatureMatch> selected = coarse_matches;
  std::sort(selected.begin(), selected.end(),
            [](const ReferenceFeatureMatch &a,
               const ReferenceFeatureMatch &b) {
              return a.descriptor_distance < b.descriptor_distance;
            });
  if (selected.size() > static_cast<std::size_t>(fine_matcher_->capacity())) {
    selected.resize(static_cast<std::size_t>(fine_matcher_->capacity()));
  }

  std::vector<cv::DMatch> coarse;
  coarse.reserve(selected.size());
  for (const ReferenceFeatureMatch &match : selected) {
    if (match.reference_index < 0 ||
        match.reference_index >= static_cast<int>(reference_keypoints.size()) ||
        match.current_index < 0 ||
        match.current_index >= static_cast<int>(current_keypoints.size())) {
      throw std::out_of_range("reference Fine match index is invalid");
    }
    coarse.emplace_back(match.reference_index, match.current_index,
                        static_cast<float>(match.descriptor_distance));
  }

  const FeatureSet reference = ToFeatureSet(
      reference_keypoints, reference_descriptors, image_width, image_height);
  const FeatureSet current = ToFeatureSet(
      current_keypoints, current_descriptors, image_width, image_height);
  const FineMatchResult fine = fine_matcher_->refine(
      reference, current, coarse, options_.fine_confidence, true,
      image_width, image_height);

  std::vector<ReferenceFeatureMatch> refined;
  refined.reserve(fine.matches.size());
  for (std::size_t index = 0; index < fine.matches.size(); ++index) {
    ReferenceFeatureMatch match;
    match.reference_index = fine.matches[index].queryIdx;
    match.current_index = fine.matches[index].trainIdx;
    match.descriptor_distance = static_cast<int>(
        std::lround(fine.matches[index].distance));
    match.refined_current_point = fine.refined_train_points[index];
    match.confidence = fine.confidences[index];
    refined.push_back(match);
  }
  return refined;
}

} // namespace ORB_SLAM3
