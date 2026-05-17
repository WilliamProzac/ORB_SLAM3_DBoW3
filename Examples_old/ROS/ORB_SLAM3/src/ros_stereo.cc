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

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/core/core.hpp>

#include "../../../include/KeyFrame.h"
#include "../../../include/MapPoint.h"
#include "../../../include/System.h"
#include "../../../include/Tracking.h"

using namespace std;

namespace {

struct ProjectionState {
  bool use_gravity = false;
  Eigen::Vector3f gravity_world = Eigen::Vector3f::Zero();
  std::string fallback_reason;
};

class ImuGravityEstimator {
public:
  explicit ImuGravityEstimator(const std::string &settings_path) {
    LoadImuCalibration(settings_path);
  }

  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_buffer_.push_back(imu_msg);
    if (imu_buffer_.size() > kMaxBufferedSamples) {
      imu_buffer_.pop_front();
    }
  }

  ProjectionState Estimate(const Sophus::SE3f &Tcw, const ros::Time &stamp) {
    ProjectionState state;
    if (!has_imu_calibration_) {
      state.fallback_reason = calibration_status_;
      return state;
    }

    std::vector<Eigen::Vector3f> accel_samples_body;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!imu_buffer_.empty() && imu_buffer_.front()->header.stamp <= stamp) {
        const sensor_msgs::ImuConstPtr &imu_msg = imu_buffer_.front();
        accel_samples_body.emplace_back(
            static_cast<float>(imu_msg->linear_acceleration.x),
            static_cast<float>(imu_msg->linear_acceleration.y),
            static_cast<float>(imu_msg->linear_acceleration.z));
        last_imu_stamp_sec_ = imu_msg->header.stamp.toSec();
        imu_buffer_.pop_front();
      }
    }

    for (const Eigen::Vector3f &accel_body : accel_samples_body) {
      const float accel_norm = accel_body.norm();
      if (accel_norm < kAccelNormMin || accel_norm > kAccelNormMax) {
        continue;
      }

      if (!has_smoothed_accel_body_) {
        smoothed_accel_body_ = accel_body;
        has_smoothed_accel_body_ = true;
      } else {
        smoothed_accel_body_ =
            kGravitySmoothingAlpha * accel_body +
            (1.0f - kGravitySmoothingAlpha) * smoothed_accel_body_;
      }
      ++valid_sample_count_;
    }

    if (!has_smoothed_accel_body_ || valid_sample_count_ < kMinGravitySamples) {
      state.fallback_reason = "insufficient IMU samples for gravity alignment";
      return state;
    }

    if (last_imu_stamp_sec_ < 0.0 ||
        stamp.toSec() - last_imu_stamp_sec_ > kMaxImuAgeSeconds) {
      state.fallback_reason = "stale IMU samples for gravity alignment";
      return state;
    }

    if (smoothed_accel_body_.norm() < 1e-4f) {
      state.fallback_reason = "degenerate IMU acceleration estimate";
      return state;
    }

    const Eigen::Vector3f gravity_camera =
        rotation_cb_ * smoothed_accel_body_.normalized();
    Eigen::Vector3f gravity_world =
        Tcw.inverse().rotationMatrix() * gravity_camera;
    if (!gravity_world.allFinite() || gravity_world.norm() < 1e-4f) {
      state.fallback_reason = "invalid gravity direction in map frame";
      return state;
    }

    gravity_world.normalize();
    if (has_previous_gravity_world_ &&
        gravity_world.dot(previous_gravity_world_) < 0.0f) {
      gravity_world = -gravity_world;
    }

    previous_gravity_world_ = gravity_world;
    has_previous_gravity_world_ = true;

    state.use_gravity = true;
    state.gravity_world = gravity_world;
    return state;
  }

private:
  static constexpr size_t kMaxBufferedSamples = 4000;
  static constexpr int kMinGravitySamples = 20;
  static constexpr float kGravitySmoothingAlpha = 0.15f;
  static constexpr float kAccelNormMin = 1.0f;
  static constexpr float kAccelNormMax = 30.0f;
  static constexpr double kMaxImuAgeSeconds = 0.10;

  void LoadImuCalibration(const std::string &settings_path) {
    cv::FileStorage fs_settings(settings_path, cv::FileStorage::READ);
    if (!fs_settings.isOpened()) {
      calibration_status_ = "unable to open stereo settings for IMU.T_b_c1";
      return;
    }

    cv::Mat cv_tbc;
    fs_settings["IMU.T_b_c1"] >> cv_tbc;
    if (cv_tbc.empty() || cv_tbc.rows != 4 || cv_tbc.cols != 4) {
      calibration_status_ = "IMU.T_b_c1 missing or invalid in stereo settings";
      return;
    }

    cv::Mat cv_tbc_32f;
    cv_tbc.convertTo(cv_tbc_32f, CV_32F);

    Eigen::Matrix3f rotation_bc = Eigen::Matrix3f::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        rotation_bc(row, col) = cv_tbc_32f.at<float>(row, col);
      }
    }

    rotation_cb_ = rotation_bc.transpose();
    has_imu_calibration_ = true;
    calibration_status_ = "ok";
  }

  std::deque<sensor_msgs::ImuConstPtr> imu_buffer_;
  std::mutex mutex_;

  bool has_imu_calibration_ = false;
  std::string calibration_status_ =
      "IMU.T_b_c1 unavailable in stereo settings";
  Eigen::Matrix3f rotation_cb_ = Eigen::Matrix3f::Identity();

  bool has_smoothed_accel_body_ = false;
  Eigen::Vector3f smoothed_accel_body_ = Eigen::Vector3f::Zero();
  int valid_sample_count_ = 0;
  double last_imu_stamp_sec_ = -1.0;

  bool has_previous_gravity_world_ = false;
  Eigen::Vector3f previous_gravity_world_ = Eigen::Vector3f::Zero();
};

class GridMapBuilder {
public:
  explicit GridMapBuilder(ros::NodeHandle &nh) {
    ros::NodeHandle private_nh("~");
    private_nh.param("grid_resolution", resolution_, 0.20f);
    private_nh.param("grid_max_point_distance", max_point_distance_, 5.0f);
    private_nh.param("grid_min_x", initial_min_u_, -10.0f);
    private_nh.param("grid_max_x", initial_max_u_, 10.0f);
    private_nh.param("grid_min_z", initial_min_v_, -5.0f);
    private_nh.param("grid_max_z", initial_max_v_, 16.0f);
    private_nh.param("grid_free_thresh", free_thresh_, 0.55f);
    private_nh.param("grid_occupied_thresh", occupied_thresh_, 0.50f);
    private_nh.param("grid_visit_thresh", visit_thresh_, 0);

    if (resolution_ <= 0.0f) {
      resolution_ = 0.20f;
    }
    if (max_point_distance_ < 0.0f) {
      max_point_distance_ = 0.0f;
    }

    plane_axis_u_ = Eigen::Vector3f::UnitX();
    plane_axis_v_ = Eigen::Vector3f::UnitZ();
    plane_normal_ = plane_axis_u_.cross(plane_axis_v_).normalized();

    ResetGridToInitialBounds();

    publisher_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    ROS_INFO_STREAM("Grid map initialized at resolution " << resolution_
                    << " m with initial size " << width_ << " x " << height_
                    << " and plane origin (" << min_u_ << ", " << min_v_
                    << "), max point distance "
                    << FormatMaxPointDistanceForLog() << ", approx storage "
                    << FormatGridStorageMegabytes() << " MiB");
  }

  bool UpdateProjectionState(const ProjectionState &state) {
    Eigen::Vector3f candidate_u = Eigen::Vector3f::UnitX();
    Eigen::Vector3f candidate_v = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f candidate_n =
        candidate_u.cross(candidate_v).normalized();
    bool candidate_uses_gravity = state.use_gravity;
    std::string next_status =
        candidate_uses_gravity ? "gravity"
                               : (state.fallback_reason.empty()
                                      ? "unknown fallback reason"
                                      : state.fallback_reason);

    if (candidate_uses_gravity &&
        !BuildGravityAlignedBasis(state.gravity_world, candidate_u, candidate_v,
                                  candidate_n)) {
      candidate_uses_gravity = false;
      next_status = "invalid gravity-aligned basis from IMU estimate";
      candidate_u = Eigen::Vector3f::UnitX();
      candidate_v = Eigen::Vector3f::UnitZ();
      candidate_n = candidate_u.cross(candidate_v).normalized();
    }

    bool projection_changed = false;
    if (candidate_uses_gravity != using_gravity_projection_) {
      projection_changed = true;
    } else if (candidate_uses_gravity &&
               (candidate_u.dot(plane_axis_u_) < kProjectionChangeCosThreshold ||
                candidate_n.dot(plane_normal_) <
                    kProjectionChangeCosThreshold)) {
      projection_changed = true;
    }

    if (candidate_uses_gravity) {
      if (!using_gravity_projection_) {
        ROS_INFO("Grid map gravity-aligned projection enabled");
      } else if (projection_changed) {
        ROS_INFO("Grid map gravity-aligned projection basis updated");
      }
    } else if (using_gravity_projection_ || last_projection_status_ != next_status) {
      ROS_WARN_STREAM("Grid map falling back to map-frame x-z projection: "
                      << next_status);
    }

    if (projection_changed) {
      plane_axis_u_ = candidate_u;
      plane_axis_v_ = candidate_v;
      plane_normal_ = candidate_n;
      using_gravity_projection_ = candidate_uses_gravity;
      last_projection_status_ = next_status;
      ResetGridToInitialBounds();
      ROS_INFO("Grid map projection changed; accumulation reset before rebuild");
    } else {
      plane_axis_u_ = candidate_u;
      plane_axis_v_ = candidate_v;
      plane_normal_ = candidate_n;
      using_gravity_projection_ = candidate_uses_gravity;
      last_projection_status_ = next_status;
      SyncGridMetadata();
    }

    return projection_changed;
  }

  void UpdateIncremental(const Sophus::SE3f &Tcw,
                         const std::vector<ORB_SLAM3::MapPoint *> &map_points,
                         const ros::Time &stamp) {
    AccumulateFromPose(Tcw.inverse().translation(), map_points);
    Publish(stamp);
  }

  void RebuildFromKeyFrames(const std::vector<ORB_SLAM3::KeyFrame *> &keyframes,
                            const ros::Time &stamp) {
    occupied_counter_.setTo(cv::Scalar(0));
    visit_counter_.setTo(cv::Scalar(0));

    std::vector<ORB_SLAM3::KeyFrame *> sorted_keyframes = keyframes;
    std::sort(sorted_keyframes.begin(), sorted_keyframes.end(),
              [](const ORB_SLAM3::KeyFrame *lhs,
                 const ORB_SLAM3::KeyFrame *rhs) { return lhs->mnId < rhs->mnId; });

    size_t processed_keyframes = 0;
    size_t processed_points = 0;
    for (ORB_SLAM3::KeyFrame *keyframe : sorted_keyframes) {
      if (!keyframe || keyframe->isBad()) {
        continue;
      }

      const std::set<ORB_SLAM3::MapPoint *> map_points =
          keyframe->GetMapPoints();
      if (map_points.empty()) {
        continue;
      }

      processed_points += AccumulateFromPose(
          keyframe->GetPoseInverse().translation(), map_points);
      ++processed_keyframes;
    }

    ROS_INFO_STREAM("Grid map full rebuild completed from " << processed_keyframes
                    << " keyframes and " << processed_points
                    << " unique keyframe map points");
    Publish(stamp);
  }

private:
  static constexpr float kProjectionChangeCosThreshold = 0.9961947f; // 5 deg.

  template <typename MapPointContainer>
  size_t AccumulateFromPose(const Eigen::Vector3f &camera_world,
                            const MapPointContainer &map_points) {
    if (map_points.empty()) {
      return 0;
    }

    const Eigen::Vector2f camera_plane = ProjectToPlane(camera_world);
    float min_plane_u = camera_plane.x();
    float max_plane_u = camera_plane.x();
    float min_plane_v = camera_plane.y();
    float max_plane_v = camera_plane.y();
    bool has_valid_point = false;
    size_t filtered_distant_points = 0;
    float furthest_filtered_distance = 0.0f;

    for (ORB_SLAM3::MapPoint *map_point : map_points) {
      if (!map_point || map_point->isBad()) {
        continue;
      }

      const Eigen::Vector2f point_plane = ProjectToPlane(map_point->GetWorldPos());
      if (FilterDistantPoint(camera_plane, point_plane, filtered_distant_points,
                             furthest_filtered_distance)) {
        continue;
      }

      min_plane_u = std::min(min_plane_u, point_plane.x());
      max_plane_u = std::max(max_plane_u, point_plane.x());
      min_plane_v = std::min(min_plane_v, point_plane.y());
      max_plane_v = std::max(max_plane_v, point_plane.y());
      has_valid_point = true;
    }

    if (!has_valid_point) {
      LogDistanceFilterStats(filtered_distant_points, furthest_filtered_distance);
      return 0;
    }

    EnsureBounds(min_plane_u, max_plane_u, min_plane_v, max_plane_v);

    int camera_grid_x = 0;
    int camera_grid_y = 0;
    if (!ToGrid(camera_plane.x(), camera_plane.y(), camera_grid_x,
                camera_grid_y)) {
      return 0;
    }

    std::unordered_set<unsigned long> seen_points;
    seen_points.reserve(map_points.size());

    size_t processed_points = 0;
    for (ORB_SLAM3::MapPoint *map_point : map_points) {
      if (!map_point || map_point->isBad()) {
        continue;
      }
      if (!seen_points.insert(map_point->mnId).second) {
        continue;
      }

      const Eigen::Vector2f point_plane = ProjectToPlane(map_point->GetWorldPos());
      if (FilterDistantPoint(camera_plane, point_plane)) {
        continue;
      }

      int point_grid_x = 0;
      int point_grid_y = 0;
      if (!ToGrid(point_plane.x(), point_plane.y(), point_grid_x,
                  point_grid_y)) {
        continue;
      }

      ++occupied_counter_.at<int>(point_grid_y, point_grid_x);
      TraceRay(camera_grid_x, camera_grid_y, point_grid_x, point_grid_y);
      ++processed_points;
    }

    LogDistanceFilterStats(filtered_distant_points, furthest_filtered_distance);
    return processed_points;
  }

  Eigen::Vector2f ProjectToPlane(const Eigen::Vector3f &world_pos) const {
    return Eigen::Vector2f(plane_axis_u_.dot(world_pos),
                           plane_axis_v_.dot(world_pos));
  }

  bool ToGrid(float plane_u, float plane_v, int &grid_x, int &grid_y) const {
    grid_x = static_cast<int>(std::floor((plane_u - min_u_) / resolution_));
    grid_y = static_cast<int>(std::floor((plane_v - min_v_) / resolution_));
    if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
      return false;
    }
    return true;
  }

  void EnsureBounds(float min_plane_u, float max_plane_u, float min_plane_v,
                    float max_plane_v) {
    const float current_max_u = min_u_ + static_cast<float>(width_) * resolution_;
    const float current_max_v =
        min_v_ + static_cast<float>(height_) * resolution_;

    const int add_left =
        min_plane_u < min_u_
            ? static_cast<int>(std::ceil((min_u_ - min_plane_u) / resolution_))
            : 0;
    const int add_right =
        max_plane_u >= current_max_u
            ? static_cast<int>(
                  std::floor((max_plane_u - current_max_u) / resolution_)) +
                  1
            : 0;
    const int add_top =
        min_plane_v < min_v_
            ? static_cast<int>(std::ceil((min_v_ - min_plane_v) / resolution_))
            : 0;
    const int add_bottom =
        max_plane_v >= current_max_v
            ? static_cast<int>(
                  std::floor((max_plane_v - current_max_v) / resolution_)) +
                  1
            : 0;

    if (add_left == 0 && add_right == 0 && add_top == 0 && add_bottom == 0) {
      return;
    }

    cv::copyMakeBorder(occupied_counter_, occupied_counter_, add_top, add_bottom,
                       add_left, add_right, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::copyMakeBorder(visit_counter_, visit_counter_, add_top, add_bottom,
                       add_left, add_right, cv::BORDER_CONSTANT, cv::Scalar(0));

    min_u_ -= static_cast<float>(add_left) * resolution_;
    min_v_ -= static_cast<float>(add_top) * resolution_;
    width_ += add_left + add_right;
    height_ += add_top + add_bottom;
    SyncGridMetadata();

    ROS_INFO_STREAM("Grid map expanded to " << width_ << " x " << height_
                    << " cells at " << resolution_
                    << " m resolution; plane origin (" << min_u_ << ", "
                    << min_v_ << "), approx storage "
                    << FormatGridStorageMegabytes() << " MiB");
  }

  void ResetGridToInitialBounds() {
    min_u_ = initial_min_u_;
    max_u_ = initial_max_u_;
    min_v_ = initial_min_v_;
    max_v_ = initial_max_v_;

    width_ = std::max(
        1, static_cast<int>(std::ceil((max_u_ - min_u_) / resolution_)));
    height_ = std::max(
        1, static_cast<int>(std::ceil((max_v_ - min_v_) / resolution_)));

    occupied_counter_ = cv::Mat::zeros(height_, width_, CV_32SC1);
    visit_counter_ = cv::Mat::zeros(height_, width_, CV_32SC1);
    SyncGridMetadata();
  }

  void SyncGridMetadata() {
    grid_map_msg_.info.width = width_;
    grid_map_msg_.info.height = height_;
    grid_map_msg_.info.resolution = resolution_;

    const Eigen::Vector3f plane_origin =
        min_u_ * plane_axis_u_ + min_v_ * plane_axis_v_;
    grid_map_msg_.info.origin.position.x = plane_origin.x();
    grid_map_msg_.info.origin.position.y = plane_origin.y();
    grid_map_msg_.info.origin.position.z = plane_origin.z();

    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    rotation.col(0) = plane_axis_u_;
    rotation.col(1) = plane_axis_v_;
    rotation.col(2) = plane_normal_;
    Eigen::Quaternionf orientation(rotation);
    orientation.normalize();
    grid_map_msg_.info.origin.orientation.x = orientation.x();
    grid_map_msg_.info.origin.orientation.y = orientation.y();
    grid_map_msg_.info.origin.orientation.z = orientation.z();
    grid_map_msg_.info.origin.orientation.w = orientation.w();

    grid_map_msg_.data.assign(width_ * height_, -1);
  }

  size_t EstimateGridStorageBytes() const {
    const size_t occupied_bytes =
        occupied_counter_.total() * occupied_counter_.elemSize();
    const size_t visit_bytes =
        visit_counter_.total() * visit_counter_.elemSize();
    const size_t msg_bytes = grid_map_msg_.data.size() * sizeof(
                                                          nav_msgs::OccupancyGrid::
                                                              _data_type::value_type);
    return occupied_bytes + visit_bytes + msg_bytes;
  }

  std::string FormatGridStorageMegabytes() const {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << static_cast<double>(EstimateGridStorageBytes()) /
                  (1024.0 * 1024.0);
    return stream.str();
  }

  bool FilterDistantPoint(const Eigen::Vector2f &camera_plane,
                          const Eigen::Vector2f &point_plane) const {
    if (max_point_distance_ <= 0.0f) {
      return false;
    }

    const Eigen::Vector2f delta = point_plane - camera_plane;
    return delta.squaredNorm() >
           max_point_distance_ * max_point_distance_;
  }

  bool FilterDistantPoint(const Eigen::Vector2f &camera_plane,
                          const Eigen::Vector2f &point_plane,
                          size_t &filtered_distant_points,
                          float &furthest_filtered_distance) const {
    if (!FilterDistantPoint(camera_plane, point_plane)) {
      return false;
    }

    ++filtered_distant_points;
    furthest_filtered_distance =
        std::max(furthest_filtered_distance,
                 static_cast<float>((point_plane - camera_plane).norm()));
    return true;
  }

  std::string FormatMaxPointDistanceForLog() const {
    if (max_point_distance_ <= 0.0f) {
      return "disabled";
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << max_point_distance_ << " m";
    return stream.str();
  }

  void LogDistanceFilterStats(size_t filtered_distant_points,
                              float furthest_filtered_distance) const {
    if (filtered_distant_points == 0 || max_point_distance_ <= 0.0f) {
      return;
    }

    ROS_INFO_STREAM_THROTTLE(
        2.0, "Grid map ignored " << filtered_distant_points
                                 << " distant map-point references beyond "
                                 << max_point_distance_
                                 << " m planar range; furthest rejected distance "
                                 << furthest_filtered_distance << " m");
  }

  bool BuildGravityAlignedBasis(const Eigen::Vector3f &gravity_world,
                                Eigen::Vector3f &basis_u,
                                Eigen::Vector3f &basis_v,
                                Eigen::Vector3f &basis_n) const {
    if (!gravity_world.allFinite() || gravity_world.norm() < 1e-4f) {
      return false;
    }

    basis_n = gravity_world.normalized();
    if (basis_n.dot(plane_normal_) < 0.0f) {
      basis_n = -basis_n;
    }

    Eigen::Vector3f seed_u = Eigen::Vector3f::UnitX();
    basis_u = seed_u - basis_n.dot(seed_u) * basis_n;
    if (basis_u.squaredNorm() < 1e-4f) {
      seed_u = Eigen::Vector3f::UnitZ();
      basis_u = seed_u - basis_n.dot(seed_u) * basis_n;
    }

    if (basis_u.squaredNorm() < 1e-4f) {
      return false;
    }

    basis_u.normalize();
    if (basis_u.dot(plane_axis_u_) < 0.0f) {
      basis_u = -basis_u;
    }

    basis_v = basis_n.cross(basis_u);
    if (basis_v.squaredNorm() < 1e-4f) {
      return false;
    }

    basis_v.normalize();
    if (basis_v.dot(plane_axis_v_) < 0.0f) {
      basis_u = -basis_u;
      basis_v = -basis_v;
    }

    return true;
  }

  void TraceRay(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
      ++visit_counter_.at<int>(y0, x0);
      if (x0 == x1 && y0 == y1) {
        break;
      }

      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
    }
  }

  void Publish(const ros::Time &stamp) {
    grid_map_msg_.header.stamp = stamp;
    grid_map_msg_.header.frame_id = "map";
    grid_map_msg_.info.map_load_time = stamp;

    for (int row = 0; row < height_; ++row) {
      for (int col = 0; col < width_; ++col) {
        const int visits = visit_counter_.at<int>(row, col);
        const int occupied = occupied_counter_.at<int>(row, col);
        const int data_index = row * width_ + col;

        if (visits <= visit_thresh_) {
          grid_map_msg_.data[data_index] = -1;
          continue;
        }

        const float occupied_ratio =
            static_cast<float>(occupied) / static_cast<float>(visits);
        const float free_ratio = 1.0f - occupied_ratio;

        if (free_ratio >= free_thresh_) {
          grid_map_msg_.data[data_index] = 0;
        } else if (free_ratio >= occupied_thresh_) {
          grid_map_msg_.data[data_index] = -1;
        } else {
          grid_map_msg_.data[data_index] = 100;
        }
      }
    }

    publisher_.publish(grid_map_msg_);
  }

  ros::Publisher publisher_;
  nav_msgs::OccupancyGrid grid_map_msg_;
  cv::Mat occupied_counter_;
  cv::Mat visit_counter_;

  float initial_min_u_ = -10.0f;
  float initial_max_u_ = 10.0f;
  float initial_min_v_ = -5.0f;
  float initial_max_v_ = 16.0f;

  float min_u_ = 0.0f;
  float max_u_ = 0.0f;
  float min_v_ = 0.0f;
  float max_v_ = 0.0f;

  float resolution_ = 0.50f;
  float max_point_distance_ = 0.0f;
  float free_thresh_ = 0.55f;
  float occupied_thresh_ = 0.50f;
  int visit_thresh_ = 0;
  int width_ = 0;
  int height_ = 0;

  Eigen::Vector3f plane_axis_u_ = Eigen::Vector3f::UnitX();
  Eigen::Vector3f plane_axis_v_ = Eigen::Vector3f::UnitZ();
  Eigen::Vector3f plane_normal_ =
      plane_axis_u_.cross(plane_axis_v_).normalized();

  bool using_gravity_projection_ = false;
  std::string last_projection_status_ = "initial fallback";
};

class ImageGrabber {
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh,
               const std::string &settings_path)
      : mpSLAM(pSLAM),
        grid_map_builder_(nh),
        gravity_estimator_(settings_path) {
    pose_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("grid_map/pose", 1);
  }

  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    gravity_estimator_.GrabImu(imu_msg);
  }

  void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft,
                  const sensor_msgs::ImageConstPtr &msgRight);

  void PublishPose(const Sophus::SE3f &Tcw, const ros::Time &stamp) {
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f translation = Twc.translation();
    Eigen::Quaternionf orientation(Twc.rotationMatrix());
    orientation.normalize();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    pose_publisher_.publish(pose_msg);
  }

  ORB_SLAM3::System *mpSLAM;
  GridMapBuilder grid_map_builder_;
  ImuGravityEstimator gravity_estimator_;
  ros::Publisher pose_publisher_;
  bool do_rectify = false;
  cv::Mat M1l, M2l, M1r, M2r;
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "Stereo");
  ros::start();

  if (argc < 4) {
    cerr << endl
         << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary "
            "path_to_settings do_rectify [enable_viewer]"
         << endl;
    ros::shutdown();
    return 1;
  }

  bool bUseViewer = true;
  if (argc >= 5) {
    stringstream ss(argv[4]);
    ss >> boolalpha >> bUseViewer;
  }

  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO,
                         bUseViewer);

  ros::NodeHandle nh;
  ImageGrabber igb(&SLAM, nh, argv[2]);

  stringstream ss(argv[3]);
  ss >> boolalpha >> igb.do_rectify;

  if (igb.do_rectify) {
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;
    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;
    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;
    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
        R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
      cerr << "ERROR: Calibration parameters to rectify stereo are missing!"
           << endl;
      return -1;
    }

    cv::initUndistortRectifyMap(
        K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
        cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
    cv::initUndistortRectifyMap(
        K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
        cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
  }

  ros::Subscriber imu_sub =
      nh.subscribe("/camera/imu", 1000, &ImageGrabber::GrabImu, &igb);
  message_filters::Subscriber<sensor_msgs::Image> left_sub(
      nh, "/camera/infra1/image_rect_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> right_sub(
      nh, "/camera/infra2/image_rect_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,
                                               right_sub);
  sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  SLAM.Shutdown();
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

  ros::shutdown();
  return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft,
                              const sensor_msgs::ImageConstPtr &msgRight) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
    cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
    cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Sophus::SE3f Tcw;
  if (do_rectify) {
    cv::Mat imLeft, imRight;
    cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
    Tcw = mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
  } else {
    Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image,
                              cv_ptrLeft->header.stamp.toSec());
  }

  const bool map_changed = mpSLAM->MapChanged();
  const int tracking_state = mpSLAM->GetTrackingState();
  const bool tracking_ok = tracking_state == ORB_SLAM3::Tracking::OK ||
                           tracking_state == ORB_SLAM3::Tracking::OK_KLT;

  bool projection_changed = false;
  if (tracking_ok) {
    PublishPose(Tcw, cv_ptrLeft->header.stamp);
    projection_changed = grid_map_builder_.UpdateProjectionState(
        gravity_estimator_.Estimate(Tcw, cv_ptrLeft->header.stamp));
  }

  if (map_changed || projection_changed) {
    if (map_changed && projection_changed) {
      ROS_INFO("Grid map rebuild triggered by System::MapChanged() and projection update");
    } else if (map_changed) {
      ROS_INFO("Grid map rebuild triggered by System::MapChanged()");
    } else {
      ROS_INFO("Grid map rebuild triggered by projection update");
    }
    grid_map_builder_.RebuildFromKeyFrames(mpSLAM->GetAllKeyFrames(),
                                           cv_ptrLeft->header.stamp);
  }

  if (tracking_ok) {
    grid_map_builder_.UpdateIncremental(Tcw, mpSLAM->GetTrackedMapPoints(),
                                        cv_ptrLeft->header.stamp);
  }
}
