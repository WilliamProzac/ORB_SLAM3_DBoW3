#include "xfeat_hybrid_depth_provider.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace ORB_SLAM3 {
namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(const Clock::time_point &start,
                  const Clock::time_point &end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

bool patch_ncc(const cv::Mat &left, const cv::Mat &right,
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
  if (left_norm < 1e-6 || right_norm < 1e-6) return false;
  ncc = static_cast<float>(left_patch.dot(right_patch) /
                           (left_norm * right_norm));
  return std::isfinite(ncc);
}

struct ProposedDepth {
  bool valid = false;
  float right_coordinate = -1.0F;
  float depth = -1.0F;
  float ncc = -1.0F;
};

} // namespace

XFeatHybridDepthProvider::XFeatHybridDepthProvider(
    const XFeatHybridDepthOptions &options)
    : options_(options) {
  if (options_.backbone_model.empty() || options_.fine_model.empty()) {
    throw std::invalid_argument(
        "Hybrid XFeat requires backbone and Fine model paths");
  }
  if (options_.fine_batch_size != 384) {
    throw std::invalid_argument("F22 freezes the Fine capacity at 384");
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
    throw std::runtime_error("Loaded Fine model capacity does not equal 384");
  }
}

StereoDepthRefinementStats XFeatHybridDepthProvider::Refine(
    const cv::Mat &left, const cv::Mat &right,
    const std::vector<cv::KeyPoint> &left_keypoints, float bf,
    std::vector<float> &right_coordinates, std::vector<float> &depths) {
  const auto start = Clock::now();
  StereoDepthRefinementStats stats;
  if (left.type() != CV_8UC1 || right.type() != CV_8UC1 ||
      left.size() != right.size()) {
    throw std::runtime_error(
        "Hybrid XFeat requires equal-size rectified mono8 images");
  }
  if (left.cols != left_frontend_->input_width() ||
      left.rows != left_frontend_->input_height()) {
    throw std::runtime_error(
        "Hybrid XFeat input size does not match the frozen backbone");
  }
  if (right_coordinates.size() != left_keypoints.size() ||
      depths.size() != left_keypoints.size()) {
    throw std::runtime_error("ORB stereo arrays do not match keypoint count");
  }

  auto left_future = std::async(std::launch::async, [&]() {
    return left_frontend_->extract(left);
  });
  auto right_future = std::async(std::launch::async, [&]() {
    return right_frontend_->extract(right);
  });
  FeatureSet left_features = left_future.get();
  FeatureSet right_features = right_future.get();

  double coarse_ms = 0.0;
  const std::vector<cv::DMatch> coarse = match_xfeat_stereo_guided(
      left_features, right_features, options_.minimum_cosine,
      options_.ratio_threshold, options_.vertical_tolerance,
      options_.maximum_disparity,
      static_cast<size_t>(options_.fine_batch_size), &coarse_ms,
      options_.grid_columns, options_.grid_rows,
      static_cast<size_t>(options_.match_grid_maximum_per_cell));
  (void)coarse_ms;
  stats.candidates = static_cast<int>(coarse.size());

  const FineMatchResult fine = fine_matcher_->refine(
      left_features, right_features, coarse, options_.fine_confidence, false,
      left.cols, left.rows);
  stats.fine_matches = static_cast<int>(fine.matches.size());

  std::vector<ProposedDepth> proposals(left_keypoints.size());
  const float radius_squared =
      options_.association_radius * options_.association_radius;
  for (size_t index = 0; index < fine.matches.size(); ++index) {
    const cv::DMatch &match = fine.matches[index];
    const cv::Point2f xfeat_left =
        left_features.points[static_cast<size_t>(match.queryIdx)];
    const cv::Point2f xfeat_right = fine.refined_train_points[index];
    const float disparity = xfeat_left.x - xfeat_right.x;
    if (!std::isfinite(disparity) || disparity <= 0.0F ||
        disparity > options_.maximum_disparity) {
      ++stats.rejected;
      continue;
    }

    int nearest = -1;
    float nearest_distance = radius_squared;
    for (size_t orb_index = 0; orb_index < left_keypoints.size(); ++orb_index) {
      const cv::Point2f delta = left_keypoints[orb_index].pt - xfeat_left;
      const float distance = delta.dot(delta);
      if (distance <= nearest_distance) {
        nearest_distance = distance;
        nearest = static_cast<int>(orb_index);
      }
    }
    if (nearest < 0) {
      ++stats.rejected;
      continue;
    }

    const cv::Point2f orb_left =
        left_keypoints[static_cast<size_t>(nearest)].pt;
    const cv::Point2f orb_right(orb_left.x - disparity, orb_left.y);
    float candidate_ncc = -1.0F;
    if (!patch_ncc(left, right, orb_left, orb_right, candidate_ncc) ||
        candidate_ncc < options_.minimum_patch_ncc) {
      ++stats.rejected;
      continue;
    }

    ProposedDepth &proposal = proposals[static_cast<size_t>(nearest)];
    if (!proposal.valid || candidate_ncc > proposal.ncc) {
      proposal.valid = true;
      proposal.right_coordinate = orb_right.x;
      proposal.depth = bf / disparity;
      proposal.ncc = candidate_ncc;
    }
  }

  for (size_t index = 0; index < proposals.size(); ++index) {
    const ProposedDepth &proposal = proposals[index];
    if (!proposal.valid) continue;
    if (depths[index] <= 0.0F || right_coordinates[index] < 0.0F) {
      right_coordinates[index] = proposal.right_coordinate;
      depths[index] = proposal.depth;
      ++stats.added;
      ++stats.accepted;
      continue;
    }

    float baseline_ncc = -1.0F;
    const cv::Point2f left_point = left_keypoints[index].pt;
    const cv::Point2f baseline_right(right_coordinates[index], left_point.y);
    if (patch_ncc(left, right, left_point, baseline_right, baseline_ncc) &&
        proposal.ncc >= baseline_ncc + options_.replacement_ncc_margin) {
      right_coordinates[index] = proposal.right_coordinate;
      depths[index] = proposal.depth;
      ++stats.replaced;
      ++stats.accepted;
    }
  }
  stats.total_ms = elapsed_ms(start, Clock::now());
  return stats;
}

} // namespace ORB_SLAM3
