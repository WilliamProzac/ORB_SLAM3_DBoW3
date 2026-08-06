#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>

#include "xfeat_frontend.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

struct PairMetrics {
  int matches = 0;
  int ransac_inliers = 0;
  int strict_inliers = 0;
  double strict_ratio = 0.0;
  double strict_coverage = 0.0;
  double median_vertical_error = std::numeric_limits<double>::quiet_NaN();
  double median_disparity = std::numeric_limits<double>::quiet_NaN();
  double geometry_ms = 0.0;
  int photometric_inliers = 0;
  double photometric_inlier_ratio = 0.0;
  double photometric_coverage = 0.0;
  double patch_ncc_median = std::numeric_limits<double>::quiet_NaN();
  double patch_ncc_p10 = std::numeric_limits<double>::quiet_NaN();
  double photometric_ms = 0.0;
};

struct StereoFeatures {
  FeatureSet left;
  FeatureSet right;
  double wall_ms = 0.0;
};

double median(std::vector<float> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  if (values.size() % 2) return values[middle];
  const float upper = values[middle];
  std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
  return 0.5 * (upper + values[middle - 1]);
}

double percentile(std::vector<float> values, double fraction) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double position = fraction * (values.size() - 1);
  const size_t low = static_cast<size_t>(std::floor(position));
  const size_t high = static_cast<size_t>(std::ceil(position));
  if (low == high) return values[low];
  return values[low] * (high - position) + values[high] * (position - low);
}

double grid_coverage(const std::vector<cv::Point2f> &points,
                     const std::vector<uint8_t> &mask, int width, int height) {
  bool occupied[6][8] = {};
  int count = 0;
  for (size_t index = 0; index < points.size(); ++index) {
    if (!mask.empty() && !mask[index]) continue;
    const int column = std::clamp(static_cast<int>(points[index].x * 8 / width), 0, 7);
    const int row = std::clamp(static_cast<int>(points[index].y * 6 / height), 0, 5);
    if (!occupied[row][column]) {
      occupied[row][column] = true;
      ++count;
    }
  }
  return count / 48.0;
}

PairMetrics evaluate_pair(const FeatureSet &a, const FeatureSet &b,
                          const std::vector<cv::DMatch> &matches, bool stereo,
                          int width, int height,
                          const std::vector<cv::Point2f> *refined_train_points = nullptr,
                          const cv::Mat *image_a = nullptr,
                          const cv::Mat *image_b = nullptr,
                          float minimum_patch_ncc = 0.5F) {
  const auto start = Clock::now();
  PairMetrics result;
  result.matches = static_cast<int>(matches.size());
  std::vector<cv::Point2f> points_a;
  std::vector<cv::Point2f> points_b;
  points_a.reserve(matches.size());
  points_b.reserve(matches.size());
  if (refined_train_points && refined_train_points->size() != matches.size()) {
    throw std::runtime_error("Refined coordinate count does not match match count");
  }
  for (size_t index = 0; index < matches.size(); ++index) {
    const auto &match = matches[index];
    points_a.push_back(a.points[match.queryIdx]);
    points_b.push_back(refined_train_points ? (*refined_train_points)[index]
                                            : b.points[match.trainIdx]);
  }

  std::vector<uint8_t> ransac(matches.size(), 0);
  if (matches.size() >= 8) {
    cv::Mat mask;
    cv::setRNGSeed(0x58464541);
    cv::findFundamentalMat(points_a, points_b, cv::FM_RANSAC, 1.5, 0.999, mask);
    if (!mask.empty() && mask.total() == matches.size()) {
      for (size_t index = 0; index < matches.size(); ++index) {
        ransac[index] = mask.at<uint8_t>(static_cast<int>(index));
      }
    }
  }
  std::vector<uint8_t> strict = ransac;
  std::vector<float> vertical_errors;
  std::vector<float> disparities;
  vertical_errors.reserve(matches.size());
  disparities.reserve(matches.size());
  for (size_t index = 0; index < matches.size(); ++index) {
    const float vertical = std::abs(points_a[index].y - points_b[index].y);
    const float disparity = points_a[index].x - points_b[index].x;
    vertical_errors.push_back(vertical);
    if (stereo) {
      const bool rectified = vertical <= 2.0F && disparity >= 0.0F && disparity <= 160.0F;
      strict[index] = static_cast<uint8_t>(strict[index] && rectified);
      if (rectified) disparities.push_back(disparity);
    }
  }
  result.ransac_inliers = std::accumulate(ransac.begin(), ransac.end(), 0);
  result.strict_inliers = std::accumulate(strict.begin(), strict.end(), 0);
  result.strict_ratio = matches.empty() ? 0.0 : result.strict_inliers / double(matches.size());
  result.strict_coverage = grid_coverage(points_a, strict, width, height);
  result.median_vertical_error = median(vertical_errors);
  result.median_disparity = stereo ? median(disparities)
                                   : std::numeric_limits<double>::quiet_NaN();
  const auto geometry_end = Clock::now();
  result.geometry_ms = elapsed_ms(start, geometry_end);

  if (image_a && image_b && !image_a->empty() && !image_b->empty()) {
    constexpr int patch_size = 9;
    constexpr int half = patch_size / 2;
    std::vector<uint8_t> photometric(matches.size(), 0);
    std::vector<float> ncc_values;
    ncc_values.reserve(matches.size());
    for (size_t index = 0; index < matches.size(); ++index) {
      const auto &pa = points_a[index];
      const auto &pb = points_b[index];
      if (pa.x < half || pa.x >= image_a->cols - half || pa.y < half ||
          pa.y >= image_a->rows - half || pb.x < half ||
          pb.x >= image_b->cols - half || pb.y < half ||
          pb.y >= image_b->rows - half) {
        continue;
      }
      cv::Mat patch_a;
      cv::Mat patch_b;
      cv::getRectSubPix(*image_a, cv::Size(patch_size, patch_size), pa, patch_a,
                        CV_32F);
      cv::getRectSubPix(*image_b, cv::Size(patch_size, patch_size), pb, patch_b,
                        CV_32F);
      patch_a -= cv::mean(patch_a)[0];
      patch_b -= cv::mean(patch_b)[0];
      const double denominator = cv::norm(patch_a) * cv::norm(patch_b);
      if (denominator <= 1e-6) continue;
      const float ncc = static_cast<float>(patch_a.dot(patch_b) / denominator);
      ncc_values.push_back(ncc);
      photometric[index] =
          static_cast<uint8_t>(strict[index] && ncc >= minimum_patch_ncc);
    }
    result.photometric_inliers =
        std::accumulate(photometric.begin(), photometric.end(), 0);
    result.photometric_inlier_ratio =
        matches.empty() ? 0.0 : result.photometric_inliers / double(matches.size());
    result.photometric_coverage =
        grid_coverage(points_a, photometric, width, height);
    result.patch_ncc_median = median(ncc_values);
    result.patch_ncc_p10 = percentile(ncc_values, 0.10);
    result.photometric_ms = elapsed_ms(geometry_end, Clock::now());
  }
  return result;
}

FeatureSet extract_orb(const cv::Ptr<cv::ORB> &orb, const cv::Mat &image) {
  const auto start = Clock::now();
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
  FeatureSet result;
  result.image_width = image.cols;
  result.image_height = image.rows;
  result.points.reserve(keypoints.size());
  result.scores.reserve(keypoints.size());
  for (const auto &keypoint : keypoints) {
    result.points.push_back(keypoint.pt);
    result.scores.push_back(keypoint.response);
  }
  result.descriptors = descriptors;
  result.timing.total_ms = elapsed_ms(start, Clock::now());
  return result;
}

std::vector<cv::DMatch> match_orb(const FeatureSet &a, const FeatureSet &b,
                                  float maximum_hamming, double *latency_ms) {
  const auto start = Clock::now();
  std::vector<cv::DMatch> matches;
  if (!a.descriptors.empty() && !b.descriptors.empty()) {
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    matcher.match(a.descriptors, b.descriptors, matches);
    matches.erase(std::remove_if(matches.begin(), matches.end(),
                                 [maximum_hamming](const cv::DMatch &match) {
                                   return match.distance > maximum_hamming;
                                 }),
                  matches.end());
    std::sort(matches.begin(), matches.end(),
              [](const cv::DMatch &left, const cv::DMatch &right) {
                return left.distance < right.distance;
              });
  }
  *latency_ms = elapsed_ms(start, Clock::now());
  return matches;
}

class BenchmarkNode {
 public:
  BenchmarkNode() : nh_(), private_nh_("~") {
    std::string model_path;
    std::string output_csv;
    std::string left_topic;
    std::string right_topic;
    std::string tracking_topic;
    private_nh_.param<std::string>("model_path", model_path,
                                   "/userdata/xfeat_rdk_x5/xfeat_backbone_544x640_bayes_e.bin");
    private_nh_.param<std::string>("output_csv", output_csv,
                                   "/userdata/xfeat_rdk_x5/benchmark.csv");
    private_nh_.param<std::string>("left_topic", left_topic,
                                   "/camera/infra1/image_raw");
    private_nh_.param<std::string>("right_topic", right_topic,
                                   "/camera/infra2/image_raw");
    private_nh_.param<std::string>("tracking_topic", tracking_topic,
                                   "/robot_pose_tracking_ok");
    private_nh_.param<std::string>("fine_model_path", fine_model_path_, "");
    private_nh_.param("top_k", top_k_, 600);
    private_nh_.param("anchor_stride", anchor_stride_, 10);
    private_nh_.param("temporal_gap", temporal_gap_, 1);
    private_nh_.param("max_anchors", max_anchors_, 0);
    private_nh_.param("xfeat_min_cosine", xfeat_min_cosine_, 0.82);
    private_nh_.param("xfeat_ratio", xfeat_ratio_, 0.9);
    private_nh_.param("xfeat_guided_stereo", xfeat_guided_stereo_, true);
    private_nh_.param("xfeat_vertical_tolerance", xfeat_vertical_tolerance_, 2.0);
    private_nh_.param("xfeat_max_disparity", xfeat_max_disparity_, 160.0);
    private_nh_.param("xfeat_temporal_radius", xfeat_temporal_radius_, 16.0);
    private_nh_.param("xfeat_fixed_matches", xfeat_fixed_matches_, 600);
    private_nh_.param("xfeat_grid_columns", xfeat_grid_columns_, 8);
    private_nh_.param("xfeat_grid_rows", xfeat_grid_rows_, 6);
    private_nh_.param("xfeat_grid_maximum_per_cell",
                      xfeat_grid_maximum_per_cell_, 0);
    private_nh_.param("xfeat_extraction_grid_maximum_per_cell",
                      xfeat_extraction_grid_maximum_per_cell_, 0);
    private_nh_.param("xfeat_fine_confidence", xfeat_fine_confidence_, 0.25);
    private_nh_.param("xfeat_fine_refine_y", xfeat_fine_refine_y_, false);
    private_nh_.param("minimum_patch_ncc", minimum_patch_ncc_, 0.5);
    private_nh_.param("xfeat_fast_decode", xfeat_fast_decode_, false);
    private_nh_.param("xfeat_semidense_single", xfeat_semidense_single_, false);
    private_nh_.param("orb_max_hamming", orb_max_hamming_, 64.0);
    private_nh_.param("cpu_threads", cpu_threads_, 8);
    if (anchor_stride_ < 1 ||
        (anchor_stride_ > 1 &&
         (temporal_gap_ <= 0 || temporal_gap_ >= anchor_stride_))) {
      throw std::runtime_error("Invalid anchor_stride/temporal_gap");
    }
    if (xfeat_grid_columns_ <= 0 || xfeat_grid_rows_ <= 0 ||
        xfeat_grid_maximum_per_cell_ < 0 ||
        xfeat_extraction_grid_maximum_per_cell_ < -1) {
      throw std::runtime_error("Invalid XFeat grid quota configuration");
    }

    cv::setNumThreads(std::max(1, cpu_threads_));
    xfeat_left_ = std::make_unique<XFeatBpuFrontend>(
        model_path, top_k_, 0.05F, 0, &bpu_mutex_, xfeat_fast_decode_,
        xfeat_semidense_single_, xfeat_grid_columns_, xfeat_grid_rows_,
        xfeat_extraction_grid_maximum_per_cell_);
    xfeat_right_ = std::make_unique<XFeatBpuFrontend>(
        model_path, top_k_, 0.05F, 0, &bpu_mutex_, xfeat_fast_decode_,
        xfeat_semidense_single_, xfeat_grid_columns_, xfeat_grid_rows_,
        xfeat_extraction_grid_maximum_per_cell_);
    if (!fine_model_path_.empty()) {
      fine_matcher_ = std::make_unique<FixedFineMatcherBpu>(
          fine_model_path_, 0, &bpu_mutex_);
      if (fine_matcher_->capacity() != xfeat_fixed_matches_) {
        ROS_WARN_STREAM("Fine capacity " << fine_matcher_->capacity()
                        << " overrides xfeat_fixed_matches="
                        << xfeat_fixed_matches_);
        xfeat_fixed_matches_ = fine_matcher_->capacity();
      }
    }
    orb_left_ = cv::ORB::create(top_k_);
    orb_right_ = cv::ORB::create(top_k_);
    output_.open(output_csv, std::ios::out | std::ios::trunc);
    if (!output_) throw std::runtime_error("Cannot open output CSV: " + output_csv);
    output_ << "frame_index,stamp_ns,tracking_state,method,pair_type,"
               "extract_a_ms,extract_b_ms,preprocess_a_ms,preprocess_b_ms,"
               "bpu_queue_a_ms,bpu_queue_b_ms,bpu_a_ms,bpu_b_ms,"
               "postprocess_a_ms,postprocess_b_ms,keypoints_a,keypoints_b,"
               "extraction_wall_ms,match_ms,geometry_ms,end_to_end_pair_ms,"
               "matches,ransac_inliers,strict_inliers,strict_inlier_ratio,"
               "strict_grid_coverage,median_vertical_error_px,median_disparity_px,"
               "fine_total_ms,fine_preprocess_ms,fine_bpu_queue_ms,fine_bpu_ms,"
               "fine_postprocess_ms,photometric_inliers,photometric_inlier_ratio,"
               "photometric_grid_coverage,patch_ncc_median,patch_ncc_p10,"
               "photometric_ms\n";
    output_.flush();

    tracking_subscriber_ = nh_.subscribe(tracking_topic, 50,
                                         &BenchmarkNode::tracking_callback, this);
    left_subscriber_.subscribe(nh_, left_topic, 50, ros::TransportHints().tcpNoDelay());
    right_subscriber_.subscribe(nh_, right_topic, 50, ros::TransportHints().tcpNoDelay());
    synchronizer_ = std::make_unique<Synchronizer>(SyncPolicy(50), left_subscriber_,
                                                   right_subscriber_);
    synchronizer_->registerCallback(
        boost::bind(&BenchmarkNode::stereo_callback, this, _1, _2));
    ROS_INFO_STREAM("XFeat/ORB benchmark ready: model=" << xfeat_left_->model_name()
                    << " input=" << xfeat_left_->input_width() << 'x'
                    << xfeat_left_->input_height() << " BPU0=serialized"
                    << " top_k=" << top_k_
                    << " semidense_single=" << xfeat_semidense_single_
                    << " grid=" << xfeat_grid_columns_ << 'x'
                    << xfeat_grid_rows_ << " max_per_cell="
                    << xfeat_grid_maximum_per_cell_
                    << " extraction_max_per_cell="
                    << xfeat_extraction_grid_maximum_per_cell_
                    << " fine="
                    << (fine_matcher_ ? fine_matcher_->model_name() : "disabled")
                    << " cpu_threads=" << cpu_threads_ << " output=" << output_csv);
  }

  ~BenchmarkNode() {
    output_.flush();
    ROS_INFO_STREAM("Benchmark stopped: source_frames=" << source_index_
                    << " sampled_frames=" << sampled_frames_
                    << " rows=" << rows_written_);
  }

 private:
  using SyncPolicy =
      message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void tracking_callback(const std_msgs::Bool::ConstPtr &message) {
    tracking_state_ = message->data ? 1 : 0;
  }

  StereoFeatures extract_xfeat_stereo(const cv::Mat &left, const cv::Mat &right) {
    const auto start = Clock::now();
    auto left_future = std::async(std::launch::async, [&] {
      return xfeat_left_->extract(left);
    });
    auto right_future = std::async(std::launch::async, [&] {
      return xfeat_right_->extract(right);
    });
    StereoFeatures result;
    result.left = left_future.get();
    result.right = right_future.get();
    result.wall_ms = elapsed_ms(start, Clock::now());
    return result;
  }

  StereoFeatures extract_orb_stereo(const cv::Mat &left, const cv::Mat &right) {
    const auto start = Clock::now();
    auto left_future = std::async(std::launch::async, [&] {
      return extract_orb(orb_left_, left);
    });
    auto right_future = std::async(std::launch::async, [&] {
      return extract_orb(orb_right_, right);
    });
    StereoFeatures result;
    result.left = left_future.get();
    result.right = right_future.get();
    result.wall_ms = elapsed_ms(start, Clock::now());
    return result;
  }

  void write_pair(int frame_index, uint64_t stamp_ns, const std::string &method,
                  const std::string &pair_type, const FeatureSet &a,
                  const FeatureSet &b, const std::vector<cv::DMatch> &matches,
                  double extraction_wall_ms, double match_ms, bool stereo,
                  int width, int height,
                  const std::vector<cv::Point2f> *refined_train_points = nullptr,
                  const StageTiming *fine_timing = nullptr,
                  const cv::Mat *image_a = nullptr,
                  const cv::Mat *image_b = nullptr) {
    const PairMetrics metrics = evaluate_pair(a, b, matches, stereo, width, height,
                                              refined_train_points, image_a, image_b,
                                              static_cast<float>(minimum_patch_ncc_));
    const double fine_total_ms = fine_timing ? fine_timing->total_ms : 0.0;
    const double pair_ms =
        extraction_wall_ms + match_ms + fine_total_ms + metrics.geometry_ms;
    output_ << frame_index << ',' << stamp_ns << ',' << tracking_state_ << ','
            << method << ',' << pair_type << ',' << std::fixed << std::setprecision(6)
            << a.timing.total_ms << ',' << b.timing.total_ms << ','
            << a.timing.preprocess_ms << ',' << b.timing.preprocess_ms << ','
            << a.timing.bpu_queue_ms << ',' << b.timing.bpu_queue_ms << ','
            << a.timing.bpu_ms << ',' << b.timing.bpu_ms << ','
            << a.timing.postprocess_ms << ',' << b.timing.postprocess_ms << ','
            << a.points.size() << ',' << b.points.size() << ','
            << extraction_wall_ms << ',' << match_ms << ',' << metrics.geometry_ms
            << ',' << pair_ms << ',' << metrics.matches << ','
            << metrics.ransac_inliers << ',' << metrics.strict_inliers << ','
            << metrics.strict_ratio << ',' << metrics.strict_coverage << ','
            << metrics.median_vertical_error << ',' << metrics.median_disparity << ','
            << fine_total_ms << ','
            << (fine_timing ? fine_timing->preprocess_ms : 0.0) << ','
            << (fine_timing ? fine_timing->bpu_queue_ms : 0.0) << ','
            << (fine_timing ? fine_timing->bpu_ms : 0.0) << ','
            << (fine_timing ? fine_timing->postprocess_ms : 0.0) << ','
            << metrics.photometric_inliers << ','
            << metrics.photometric_inlier_ratio << ','
            << metrics.photometric_coverage << ',' << metrics.patch_ncc_median
            << ',' << metrics.patch_ncc_p10 << ',' << metrics.photometric_ms
            << '\n';
    output_.flush();
    ++rows_written_;
  }

  void stereo_callback(const sensor_msgs::Image::ConstPtr &left_message,
                       const sensor_msgs::Image::ConstPtr &right_message) {
    const int frame_index = source_index_++;
    const int phase = frame_index % anchor_stride_;
    const int anchor = frame_index / anchor_stride_;
    if (max_anchors_ > 0 && anchor >= max_anchors_) return;
    if (phase != 0 && phase != temporal_gap_) return;
    try {
      const cv::Mat left = cv_bridge::toCvShare(left_message, "mono8")->image;
      const cv::Mat right = cv_bridge::toCvShare(right_message, "mono8")->image;
      if (!warmed_) {
        (void)extract_xfeat_stereo(left, right);
        (void)extract_orb_stereo(left, right);
        warmed_ = true;
        ROS_INFO("Frontend warmup complete");
      }

      StereoFeatures xfeat_stereo;
      StereoFeatures orb_stereo;
      // Balance schedutil/frequency-ramp and cache-order effects. Without this,
      // XFeat always pays the first-workload cost after eight skipped frames
      // while ORB always runs on an already active CPU.
      if (anchor % 2 == 0) {
        xfeat_stereo = extract_xfeat_stereo(left, right);
        orb_stereo = extract_orb_stereo(left, right);
      } else {
        orb_stereo = extract_orb_stereo(left, right);
        xfeat_stereo = extract_xfeat_stereo(left, right);
      }
      FeatureSet &xfeat_left = xfeat_stereo.left;
      FeatureSet &xfeat_right = xfeat_stereo.right;
      FeatureSet &orb_left = orb_stereo.left;
      FeatureSet &orb_right = orb_stereo.right;
      const uint64_t stamp_ns = left_message->header.stamp.toNSec();

      double xfeat_match_ms = 0.0;
      auto xfeat_matches = xfeat_guided_stereo_
                               ? match_xfeat_stereo_guided(
                                     xfeat_left, xfeat_right,
                                     static_cast<float>(xfeat_min_cosine_),
                                     static_cast<float>(xfeat_ratio_),
                                     static_cast<float>(xfeat_vertical_tolerance_),
                                     static_cast<float>(xfeat_max_disparity_),
                                     static_cast<size_t>(xfeat_fixed_matches_),
                                     &xfeat_match_ms, xfeat_grid_columns_,
                                     xfeat_grid_rows_,
                                     static_cast<size_t>(
                                         xfeat_grid_maximum_per_cell_))
                               : match_xfeat(
                                     xfeat_left, xfeat_right,
                                     static_cast<float>(xfeat_min_cosine_),
                                     &xfeat_match_ms);
      write_pair(frame_index, stamp_ns,
                 xfeat_guided_stereo_
                     ? (xfeat_semidense_single_ ? "xfeat_star_coarse"
                                                : "xfeat_guided")
                     : "xfeat_bpu",
                 "stereo", xfeat_left,
                 xfeat_right, xfeat_matches, xfeat_stereo.wall_ms,
                 xfeat_match_ms, true, left.cols, left.rows, nullptr, nullptr,
                 &left, &right);
      if (fine_matcher_ && xfeat_guided_stereo_) {
        const FineMatchResult fine = fine_matcher_->refine(
            xfeat_left, xfeat_right, xfeat_matches,
            static_cast<float>(xfeat_fine_confidence_), xfeat_fine_refine_y_,
            left.cols, left.rows);
        write_pair(frame_index, stamp_ns,
                   xfeat_semidense_single_ ? "xfeat_star_fine"
                                           : "xfeat_guided_fine",
                   "stereo",
                   xfeat_left, xfeat_right, fine.matches,
                   xfeat_stereo.wall_ms, xfeat_match_ms, true, left.cols,
                   left.rows, &fine.refined_train_points, &fine.timing, &left,
                   &right);
      }

      double orb_match_ms = 0.0;
      auto orb_matches = match_orb(orb_left, orb_right,
                                   static_cast<float>(orb_max_hamming_), &orb_match_ms);
      write_pair(frame_index, stamp_ns, "orb_cpu", "stereo", orb_left, orb_right,
                 orb_matches, orb_stereo.wall_ms, orb_match_ms, true, left.cols,
                 left.rows, nullptr, nullptr, &left, &right);

      if (phase == temporal_gap_ && previous_valid_) {
        xfeat_matches = match_xfeat_temporal_guided(
            previous_xfeat_left_, xfeat_left,
            static_cast<float>(xfeat_min_cosine_), static_cast<float>(xfeat_ratio_),
            static_cast<float>(xfeat_temporal_radius_),
            static_cast<size_t>(xfeat_fixed_matches_), &xfeat_match_ms,
            xfeat_grid_columns_, xfeat_grid_rows_,
            static_cast<size_t>(xfeat_grid_maximum_per_cell_));
        write_pair(frame_index, stamp_ns, "xfeat_guided", "temporal",
                   previous_xfeat_left_, xfeat_left, xfeat_matches,
                   previous_xfeat_left_.timing.total_ms + xfeat_left.timing.total_ms,
                   xfeat_match_ms, false, left.cols, left.rows);
        orb_matches = match_orb(previous_orb_left_, orb_left,
                                static_cast<float>(orb_max_hamming_), &orb_match_ms);
        write_pair(frame_index, stamp_ns, "orb_cpu", "temporal", previous_orb_left_,
                   orb_left, orb_matches,
                   previous_orb_left_.timing.total_ms + orb_left.timing.total_ms,
                   orb_match_ms, false, left.cols, left.rows);
      }
      if (phase == 0) {
        previous_xfeat_left_ = std::move(xfeat_left);
        previous_orb_left_ = std::move(orb_left);
        previous_valid_ = true;
      }
      ++sampled_frames_;
      if (sampled_frames_ % 20 == 0) {
        ROS_INFO_STREAM("Processed " << sampled_frames_ << " sampled stereo frames; rows="
                                      << rows_written_);
      }
    } catch (const std::exception &error) {
      ROS_ERROR_STREAM("Frame " << frame_index << " failed: " << error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  message_filters::Subscriber<sensor_msgs::Image> left_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> right_subscriber_;
  std::unique_ptr<Synchronizer> synchronizer_;
  ros::Subscriber tracking_subscriber_;
  std::mutex bpu_mutex_;
  std::unique_ptr<XFeatBpuFrontend> xfeat_left_;
  std::unique_ptr<XFeatBpuFrontend> xfeat_right_;
  std::unique_ptr<FixedFineMatcherBpu> fine_matcher_;
  cv::Ptr<cv::ORB> orb_left_;
  cv::Ptr<cv::ORB> orb_right_;
  std::ofstream output_;
  FeatureSet previous_xfeat_left_;
  FeatureSet previous_orb_left_;
  bool previous_valid_ = false;
  bool warmed_ = false;
  int tracking_state_ = -1;
  int top_k_ = 600;
  int anchor_stride_ = 10;
  int temporal_gap_ = 1;
  int max_anchors_ = 0;
  double xfeat_min_cosine_ = 0.82;
  double xfeat_ratio_ = 0.9;
  bool xfeat_guided_stereo_ = true;
  double xfeat_vertical_tolerance_ = 2.0;
  double xfeat_max_disparity_ = 160.0;
  double xfeat_temporal_radius_ = 16.0;
  int xfeat_fixed_matches_ = 600;
  int xfeat_grid_columns_ = 8;
  int xfeat_grid_rows_ = 6;
  int xfeat_grid_maximum_per_cell_ = 0;
  int xfeat_extraction_grid_maximum_per_cell_ = 0;
  std::string fine_model_path_;
  double xfeat_fine_confidence_ = 0.25;
  bool xfeat_fine_refine_y_ = false;
  double minimum_patch_ncc_ = 0.5;
  bool xfeat_fast_decode_ = false;
  bool xfeat_semidense_single_ = false;
  double orb_max_hamming_ = 64.0;
  int cpu_threads_ = 8;
  int source_index_ = 0;
  int sampled_frames_ = 0;
  int rows_written_ = 0;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "xfeat_orb_benchmark");
  try {
    BenchmarkNode node;
    ros::spin();
    return 0;
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
}
