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
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ORB_SLAM3/RelocalizationStatus.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

namespace {

constexpr double kUnknownCovariance = 1e6;
constexpr double kLowConfidenceTwistCovariance = 1e3;
constexpr double kMinVelocityDtSec = 1e-3;
constexpr double kMaxVelocityDtSec = 0.5;
constexpr double kMaxLinearVelocityMps = 10.0;
constexpr double kMaxAngularVelocityRadPerSec = 8.0;
constexpr char kContinuousMapFrame[] = "continuous_map";
constexpr char kPlanningMapFrame[] = "planning_map";
constexpr char kSlamMapFrame[] = "map";
constexpr char kOdomFrame[] = "odom";
constexpr char kRosCameraFrame[] = "left_camera_link";
constexpr char kOpticalCameraFrame[] = "left_camera";

struct PoseCorrectionConfig {
  bool enable_corrected_map_pose = false;
  int correction_horizon_frames = 30;
  bool auto_enable_on_map_event = true;
};

template <typename CovarianceArray>
void SetDiagonalCovariance(CovarianceArray &covariance, double diagonal_value) {
  std::fill(covariance.begin(), covariance.end(), 0.0);
  covariance[0] = diagonal_value;
  covariance[7] = diagonal_value;
  covariance[14] = diagonal_value;
  covariance[21] = diagonal_value;
  covariance[28] = diagonal_value;
  covariance[35] = diagonal_value;
}

std::string RelocalizationStatusToString(
    ORB_SLAM3::Tracking::eRelocalizationStatus status) {
  switch (status) {
  case ORB_SLAM3::Tracking::RelocalizationRunning:
    return "RelocalizationRunning";
  case ORB_SLAM3::Tracking::RelocalizationSucceed:
    return "RelocalizationSucceed";
  case ORB_SLAM3::Tracking::RelocalizationFailed:
    return "RelocalizationFailed";
  }

  return "RelocalizationFailed";
}

Sophus::SE3f CameraLinkToOpticalTransform() {
  static const Eigen::Matrix3f rotation = [] {
    Eigen::Matrix3f matrix;
    matrix << 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f;
    return matrix;
  }();
  return Sophus::SE3f(rotation, Eigen::Vector3f::Zero());
}

class ImageGrabber {
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh,
         const PoseCorrectionConfig &pose_correction_config)
      : mpSLAM(pSLAM),
    pose_correction_config_(pose_correction_config),
        publish_relocalization_status_(pSLAM &&
                                       pSLAM->IsOnlyTrackingEnabled()) {
    robot_pose_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
    slam_pose_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/robot_pose_slam", 1);
    corrected_map_pose_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/robot_pose_map", 1);
    odometry_publisher_ = nh.advertise<nav_msgs::Odometry>("/odometry", 1);
    tracking_status_publisher_ =
        nh.advertise<std_msgs::Bool>("/robot_pose_tracking_ok", 1);

    if (publish_relocalization_status_) {
      relocalization_status_publisher_ =
          nh.advertise<ORB_SLAM3::RelocalizationStatus>(
              "/relocalization_status", 1);
      relocalization_status_timer_ = nh.createTimer(
          ros::Duration(0.1),
          &ImageGrabber::PublishRelocalizationStatusTimer, this);
    }
    pose_correction_service_ =
        nh.advertiseService("/pose_correction/set_enabled",
                            &ImageGrabber::SetPoseCorrectionEnabled, this);
  }

  void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft,
                  const sensor_msgs::ImageConstPtr &msgRight);

  void PublishTrackingStatus(bool tracking_ok) {
    std_msgs::Bool status_msg;
    status_msg.data = tracking_ok;
    tracking_status_publisher_.publish(status_msg);
  }

  void PublishPoseMessage(ros::Publisher &publisher, const Sophus::SE3f &Twc,
                          const ros::Time &stamp,
                          const std::string &frame_id) {
    const Eigen::Vector3f translation = Twc.translation();
    Eigen::Quaternionf orientation(Twc.rotationMatrix());
    orientation.normalize();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    publisher.publish(pose_msg);
  }

  void PublishPose(const Sophus::SE3f &Twc, const ros::Time &stamp) {
    PublishPoseMessage(robot_pose_publisher_, Twc, stamp,
                       kContinuousMapFrame);
  }

  void PublishSlamPose(const Sophus::SE3f &Twc, const ros::Time &stamp) {
    PublishPoseMessage(slam_pose_publisher_, Twc, stamp, kSlamMapFrame);
  }

  void PublishCorrectedMapPose(const Sophus::SE3f &Twc,
                               const ros::Time &stamp) {
    PublishPoseMessage(corrected_map_pose_publisher_, Twc, stamp,
                       kPlanningMapFrame);
  }

  void PublishOdometry(const Sophus::SE3f &Twc, const ros::Time &stamp,
                       bool pose_fresh) {
    const Sophus::SE3f Twb = ConvertOpticalPoseToRosPose(Twc);
    const Sophus::SE3f T0b = GetOdometryPose(Twb);
    const Eigen::Vector3f translation = T0b.translation();
    Eigen::Quaternionf orientation(T0b.rotationMatrix());
    orientation.normalize();

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = kOdomFrame;
    odom_msg.child_frame_id = kRosCameraFrame;
    odom_msg.pose.pose.position.x = translation.x();
    odom_msg.pose.pose.position.y = translation.y();
    odom_msg.pose.pose.position.z = translation.z();
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();
    SetDiagonalCovariance(odom_msg.pose.covariance, kUnknownCovariance);

    TwistEstimate twist_estimate;
    if (pose_fresh) {
      twist_estimate = EstimateTwist(Twb, stamp);
    }
    odom_msg.twist.twist.linear.x = twist_estimate.linear.x();
    odom_msg.twist.twist.linear.y = twist_estimate.linear.y();
    odom_msg.twist.twist.linear.z = twist_estimate.linear.z();
    odom_msg.twist.twist.angular.x = twist_estimate.angular.x();
    odom_msg.twist.twist.angular.y = twist_estimate.angular.y();
    odom_msg.twist.twist.angular.z = twist_estimate.angular.z();
    SetDiagonalCovariance(
        odom_msg.twist.covariance,
        twist_estimate.valid ? kLowConfidenceTwistCovariance
                             : kUnknownCovariance);

    PublishTransforms(T0b, stamp);
    odometry_publisher_.publish(odom_msg);
  }

  void RefreshRelocalizationStatus() {
    if (!publish_relocalization_status_ || !mpSLAM ||
        !mpSLAM->IsOnlyTrackingEnabled()) {
      return;
    }

    int status_code = 0;
    if (!mpSLAM->TryGetLatestRelocalizationStatus(status_code)) {
      return;
    }

    const std::string status_string = RelocalizationStatusToString(
        static_cast<ORB_SLAM3::Tracking::eRelocalizationStatus>(status_code));

    std::lock_guard<std::mutex> lock(relocalization_status_mutex_);
    latest_relocalization_status_ = status_string;
    has_latest_relocalization_status_ = true;
  }

  void PublishRelocalizationStatusTimer(const ros::TimerEvent &) {
    if (!publish_relocalization_status_ || !mpSLAM ||
        !mpSLAM->IsOnlyTrackingEnabled()) {
      return;
    }

    RefreshRelocalizationStatus();

    std::string latest_status;
    {
      std::lock_guard<std::mutex> lock(relocalization_status_mutex_);
      if (!has_latest_relocalization_status_) {
        return;
      }
      latest_status = latest_relocalization_status_;
    }

    ORB_SLAM3::RelocalizationStatus status_msg;
    status_msg.timestamp_ns = ros::Time::now().toNSec();
    status_msg.status = latest_status;
    relocalization_status_publisher_.publish(status_msg);
  }

  bool SetPoseCorrectionEnabled(std_srvs::SetBool::Request &req,
                                std_srvs::SetBool::Response &res) {
    pose_correction_config_.enable_corrected_map_pose = req.data;
    res.success = true;
    res.message = std::string("Pose correction ") + (req.data ? "enabled" : "disabled");
    ROS_INFO_STREAM("Pose correction set to " << (req.data ? "ENABLED" : "DISABLED") );
    return true;
  }

  ORB_SLAM3::System *mpSLAM;
  PoseCorrectionConfig pose_correction_config_;
  ros::ServiceServer pose_correction_service_;
  ros::Publisher robot_pose_publisher_;
  ros::Publisher slam_pose_publisher_;
  ros::Publisher corrected_map_pose_publisher_;
  ros::Publisher odometry_publisher_;
  ros::Publisher tracking_status_publisher_;
  const bool publish_relocalization_status_;
  ros::Publisher relocalization_status_publisher_;
  ros::Timer relocalization_status_timer_;
  std::mutex relocalization_status_mutex_;
  bool has_latest_relocalization_status_ = false;
  std::string latest_relocalization_status_;
  bool do_rectify;
  cv::Mat M1l, M2l, M1r, M2r;

private:
  struct TwistEstimate {
    bool valid = false;
    Eigen::Vector3f linear = Eigen::Vector3f::Zero();
    Eigen::Vector3f angular = Eigen::Vector3f::Zero();
  };

  tf::Transform SophusToTfTransform(const Sophus::SE3f &pose) const {
    Eigen::Quaternionf orientation(pose.rotationMatrix());
    orientation.normalize();

    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(pose.translation().x(), pose.translation().y(),
                    pose.translation().z()));
    transform.setRotation(tf::Quaternion(
        orientation.x(), orientation.y(), orientation.z(), orientation.w()));
    return transform;
  }

  Sophus::SE3f ConvertOpticalPoseToRosPose(const Sophus::SE3f &Twc) const {
    return Twc * CameraLinkToOpticalTransform();
  }

  bool PoseIsFinite(const Sophus::SE3f &pose) const {
    return pose.translation().allFinite() && pose.rotationMatrix().allFinite();
  }

  void EnterUnstableMappingWindow() {
    if (!pose_correction_blocked_by_tracking_loss_) {
      ROS_WARN_STREAM(
          "Suppressing /robot_pose_map convergence until the next ORB-SLAM big "
          "map event because tracking became unstable");
    }
    pose_correction_blocked_by_tracking_loss_ = true;
    corrected_map_correction_active_ = false;
  }

  void MaybeLeaveUnstableMappingWindow(bool map_id_changed,
                                       bool big_change_idx_changed) {
    if (pose_correction_blocked_by_tracking_loss_ && !map_id_changed &&
        big_change_idx_changed) {
      pose_correction_blocked_by_tracking_loss_ = false;
      ROS_WARN_STREAM(
          "Resuming /robot_pose_map convergence after ORB-SLAM big map event "
          "closed the unstable mapping window");
    }
  }

  bool ShouldStartPoseCorrectionEvent(bool map_id_changed,
                                      bool big_change_idx_changed) const {
    return !pose_correction_blocked_by_tracking_loss_ && !map_id_changed &&
           big_change_idx_changed &&
           (pose_correction_config_.enable_corrected_map_pose ||
            pose_correction_config_.auto_enable_on_map_event);
  }

  void PublishPlanningMapTransform(const Sophus::SE3f &Tplanning_map,
                                   const ros::Time &stamp) {
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        SophusToTfTransform(Tplanning_map), stamp, kPlanningMapFrame,
        kSlamMapFrame));
  }

  Sophus::SE3f ApplyGradualCorrection(const Sophus::SE3f &base_pose,
                                      const Sophus::SE3f &correction_offset,
                                      int &correction_step,
                                      bool &correction_active) const {
    if (!correction_active) {
      return base_pose;
    }

    const int horizon_frames =
        std::max(1, pose_correction_config_.correction_horizon_frames);
    const float alpha =
        horizon_frames == 1
            ? 1.0f
            : static_cast<float>(correction_step) /
                  static_cast<float>(horizon_frames - 1);
    const Sophus::SE3f identity_pose;
    const Sophus::SE3f blended_offset =
        BlendPose(correction_offset, identity_pose, alpha);
    const Sophus::SE3f output_pose = blended_offset * base_pose;

    correction_step++;
    if (correction_step >= horizon_frames) {
      correction_active = false;
    }

    return output_pose;
  }

  Sophus::SE3f GetContinuousPose(const Sophus::SE3f &raw_Twc) {
    const long unsigned int map_id = mpSLAM ? mpSLAM->GetCurrentMapId() : 0;
    const int big_change_idx = mpSLAM ? mpSLAM->GetLastBigChangeIdx() : 0;
    const int map_change_idx = mpSLAM ? mpSLAM->GetCurrentMapChangeIndex() : 0;
    const bool map_id_changed =
        has_current_map_id_ && map_id != current_map_id_;
    if (!has_current_map_id_) {
      current_map_id_ = map_id;
      has_current_map_id_ = true;
    }
    if (!has_last_big_change_idx_) {
      last_big_change_idx_ = big_change_idx;
      has_last_big_change_idx_ = true;
    }
    if (!has_last_map_change_idx_) {
      last_map_change_idx_ = map_change_idx;
      has_last_map_change_idx_ = true;
    }

    if (!has_continuous_map_transform_) {
      continuous_T_orb_map_ = Sophus::SE3f();
      has_continuous_map_transform_ = true;
    } else if (map_id_changed && has_last_continuous_pose_) {
      continuous_T_orb_map_ = last_continuous_Twc_ * raw_Twc.inverse();
      ResetVelocityState();
      ROS_WARN_STREAM(
          "Rebased /robot_pose continuous_map after ORB-SLAM map switch"
          << " map_id " << current_map_id_ << " -> " << map_id
          << ", big_change " << last_big_change_idx_ << " -> "
          << big_change_idx << ", map_change " << last_map_change_idx_
          << " -> " << map_change_idx);
    }

    current_map_id_ = map_id;
    last_big_change_idx_ = big_change_idx;
    last_map_change_idx_ = map_change_idx;
    const Sophus::SE3f continuous_Twc = continuous_T_orb_map_ * raw_Twc;
    last_continuous_Twc_ = continuous_Twc;
    has_last_continuous_pose_ = true;
    return continuous_Twc;
  }

  void PublishTransforms(const Sophus::SE3f &T0b, const ros::Time &stamp) {
    if (!has_odom_origin_) {
      return;
    }

    tf_broadcaster_.sendTransform(tf::StampedTransform(
        SophusToTfTransform(odom_origin_pose_), stamp, kContinuousMapFrame,
        kOdomFrame));
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        SophusToTfTransform(T0b), stamp, kOdomFrame, kRosCameraFrame));
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(SophusToTfTransform(CameraLinkToOpticalTransform()),
                             stamp, kRosCameraFrame, kOpticalCameraFrame));
  }

  Sophus::SE3f GetOdometryPose(const Sophus::SE3f &Twb) {
    // Keep a session-stable local origin once the first valid body-frame pose
    // arrives.
    if (!has_odom_origin_) {
      odom_origin_pose_ = Twb;
      has_odom_origin_ = true;
    }

    return odom_origin_pose_.inverse() * Twb;
  }

  void ResetVelocityState() {
    has_previous_valid_pose_ = false;
    suppress_next_twist_ = false;
  }

  void StoreVelocityReference(const Sophus::SE3f &Twb,
                              const ros::Time &stamp) {
    previous_valid_pose_ = Twb;
    previous_valid_stamp_ = stamp;
    has_previous_valid_pose_ = true;
  }

  TwistEstimate EstimateTwist(const Sophus::SE3f &Twb,
                              const ros::Time &stamp) {
    TwistEstimate estimate;

    if (!has_previous_valid_pose_) {
      StoreVelocityReference(Twb, stamp);
      return estimate;
    }

    if (suppress_next_twist_) {
      suppress_next_twist_ = false;
      StoreVelocityReference(Twb, stamp);
      return estimate;
    }

    const double dt = (stamp - previous_valid_stamp_).toSec();
    if (!std::isfinite(dt) || dt <= 0.0 || dt < kMinVelocityDtSec ||
        dt > kMaxVelocityDtSec) {
      StoreVelocityReference(Twb, stamp);
      return estimate;
    }

    const Sophus::SE3f Tprev_current = previous_valid_pose_.inverse() * Twb;
    const Sophus::SO3f Rprev_current = Tprev_current.so3();
    const Sophus::SO3f Rcurrent_prev = Rprev_current.inverse();
    const Eigen::Vector3f linear_velocity =
        (Rcurrent_prev.matrix() * Tprev_current.translation()) /
        static_cast<float>(dt);
    const Eigen::Vector3f angular_velocity =
        Rcurrent_prev.log() / static_cast<float>(dt);

    const bool twist_is_finite =
        linear_velocity.allFinite() && angular_velocity.allFinite();
    const bool twist_is_reasonable =
        linear_velocity.norm() <= kMaxLinearVelocityMps &&
        angular_velocity.norm() <= kMaxAngularVelocityRadPerSec;

    StoreVelocityReference(Twb, stamp);

    if (!twist_is_finite || !twist_is_reasonable) {
      suppress_next_twist_ = true;
      return estimate;
    }

    estimate.valid = true;
    estimate.linear = linear_velocity;
    estimate.angular = angular_velocity;
    return estimate;
  }

  bool has_previous_valid_pose_ = false;
  bool suppress_next_twist_ = false;
  bool has_odom_origin_ = false;
  bool has_current_map_id_ = false;
  bool has_last_big_change_idx_ = false;
  bool has_last_map_change_idx_ = false;
  bool has_continuous_map_transform_ = false;
  bool has_last_continuous_pose_ = false;
  bool has_last_slam_pose_ = false;
  long unsigned int current_map_id_ = 0;
  int last_big_change_idx_ = 0;
  int last_map_change_idx_ = 0;
  tf::TransformBroadcaster tf_broadcaster_;
  Sophus::SE3f continuous_T_orb_map_;
  Sophus::SE3f last_continuous_Twc_;
  Sophus::SE3f last_slam_Twc_;
  Sophus::SE3f odom_origin_pose_;
  Sophus::SE3f previous_valid_pose_;
  ros::Time previous_valid_stamp_;

  bool has_last_corrected_map_pose_ = false;
  bool has_corrected_map_transform_ = false;
  bool has_last_corrected_map_transform_ = false;
  bool pose_correction_blocked_by_tracking_loss_ = false;
  bool corrected_map_correction_active_ = false;
  bool corrected_map_has_current_map_id_ = false;
  bool corrected_map_has_last_big_change_idx_ = false;
  bool corrected_map_has_last_map_change_idx_ = false;
  long unsigned int corrected_map_current_map_id_ = 0;
  int corrected_map_last_big_change_idx_ = 0;
  int corrected_map_last_map_change_idx_ = 0;
  int corrected_map_correction_step_ = 0;
  Sophus::SE3f corrected_map_T_orb_map_;
  Sophus::SE3f last_corrected_map_pose_;
  Sophus::SE3f last_corrected_map_transform_;
  Sophus::SE3f corrected_map_correction_offset_;

  Sophus::SE3f BlendPose(const Sophus::SE3f &from, const Sophus::SE3f &to,
                         float alpha) const {
    alpha = std::max(0.0f, std::min(1.0f, alpha));

    const Eigen::Vector3f translation =
        (1.0f - alpha) * from.translation() + alpha * to.translation();
    Eigen::Quaternionf q_from(from.rotationMatrix());
    Eigen::Quaternionf q_to(to.rotationMatrix());
    q_from.normalize();
    q_to.normalize();

    Eigen::Quaternionf q_blend = q_from.slerp(alpha, q_to);
    q_blend.normalize();
    return Sophus::SE3f(q_blend.toRotationMatrix(), translation);
  }

  Sophus::SE3f GetCorrectedMapPose(const Sophus::SE3f &raw_Twc) {
    const long unsigned int map_id = mpSLAM ? mpSLAM->GetCurrentMapId() : 0;
    const int big_change_idx = mpSLAM ? mpSLAM->GetLastBigChangeIdx() : 0;
    const int map_change_idx = mpSLAM ? mpSLAM->GetCurrentMapChangeIndex() : 0;
    const bool map_id_changed = corrected_map_has_current_map_id_ &&
                                map_id != corrected_map_current_map_id_;
    const bool big_change_idx_changed =
        corrected_map_has_last_big_change_idx_ &&
        big_change_idx != corrected_map_last_big_change_idx_;

    if (!corrected_map_has_current_map_id_) {
      corrected_map_current_map_id_ = map_id;
      corrected_map_has_current_map_id_ = true;
    }
    if (!corrected_map_has_last_big_change_idx_) {
      corrected_map_last_big_change_idx_ = big_change_idx;
      corrected_map_has_last_big_change_idx_ = true;
    }
    if (!corrected_map_has_last_map_change_idx_) {
      corrected_map_last_map_change_idx_ = map_change_idx;
      corrected_map_has_last_map_change_idx_ = true;
    }

    MaybeLeaveUnstableMappingWindow(map_id_changed, big_change_idx_changed);

    if (!has_corrected_map_transform_) {
      corrected_map_T_orb_map_ = Sophus::SE3f();
      has_corrected_map_transform_ = true;
    } else if (map_id_changed && has_last_corrected_map_pose_) {
      const Sophus::SE3f rebase_reference =
          has_last_continuous_pose_ ? last_continuous_Twc_
                                    : last_corrected_map_pose_;
      corrected_map_T_orb_map_ = rebase_reference * raw_Twc.inverse();
      corrected_map_correction_active_ = false;
      ROS_WARN_STREAM(
          "Rebased /robot_pose_map planning_map after ORB-SLAM map switch"
          << " map_id " << corrected_map_current_map_id_ << " -> " << map_id
          << ", big_change " << corrected_map_last_big_change_idx_ << " -> "
          << big_change_idx << ", map_change "
          << corrected_map_last_map_change_idx_ << " -> " << map_change_idx);
    }

    if (ShouldStartPoseCorrectionEvent(map_id_changed, big_change_idx_changed) &&
        has_last_corrected_map_pose_) {
      corrected_map_correction_active_ = true;
      corrected_map_correction_step_ = 0;
      corrected_map_correction_offset_ =
          last_corrected_map_pose_ * raw_Twc.inverse();
      ROS_WARN_STREAM(
          "Starting gradual /robot_pose_map planning_map correction after "
          "ORB-SLAM big map event"
          << " map_id " << corrected_map_current_map_id_ << " -> " << map_id
          << ", big_change " << corrected_map_last_big_change_idx_ << " -> "
          << big_change_idx << ", map_change "
          << corrected_map_last_map_change_idx_ << " -> " << map_change_idx);
    }

    corrected_map_current_map_id_ = map_id;
    corrected_map_last_big_change_idx_ = big_change_idx;
    corrected_map_last_map_change_idx_ = map_change_idx;

    const Sophus::SE3f base_corrected_map_Twc = corrected_map_T_orb_map_ * raw_Twc;
    Sophus::SE3f corrected_map_transform = corrected_map_T_orb_map_;
    Sophus::SE3f output_Twc = base_corrected_map_Twc;
    if (corrected_map_correction_active_) {
      const bool was_active = corrected_map_correction_active_;
      output_Twc = ApplyGradualCorrection(
          raw_Twc, corrected_map_correction_offset_,
          corrected_map_correction_step_, corrected_map_correction_active_);
      corrected_map_transform = output_Twc * raw_Twc.inverse();
      if (was_active && !corrected_map_correction_active_) {
        corrected_map_T_orb_map_ = Sophus::SE3f();
        corrected_map_transform = corrected_map_T_orb_map_;
      }
    }

    last_corrected_map_pose_ = output_Twc;
    has_last_corrected_map_pose_ = true;
    last_corrected_map_transform_ = corrected_map_transform;
    has_last_corrected_map_transform_ = true;
    return output_Twc;
  }
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

  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    ros::shutdown();
    return -1;
  }

  PoseCorrectionConfig pose_correction_config;
  if (!fsSettings["PoseCorrection.EnableCorrectedMapPose"].empty()) {
    pose_correction_config.enable_corrected_map_pose =
        static_cast<int>(fsSettings["PoseCorrection.EnableCorrectedMapPose"]) !=
        0;
  }
  if (!fsSettings["PoseCorrection.CorrectionHorizonFrames"].empty()) {
    pose_correction_config.correction_horizon_frames =
        static_cast<int>(fsSettings["PoseCorrection.CorrectionHorizonFrames"]);
  }
  if (!fsSettings["PoseCorrection.AutoEnableOnMapEvent"].empty()) {
    pose_correction_config.auto_enable_on_map_event =
        static_cast<int>(fsSettings["PoseCorrection.AutoEnableOnMapEvent"]) != 0;
  }

  bool bUseViewer = true;
  if (argc >= 5) {
    stringstream ss(argv[4]);
    ss >> boolalpha >> bUseViewer;
  }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO,
                         bUseViewer);

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ImageGrabber igb(&SLAM, nh, pose_correction_config);

  std::string left_image_topic = "/camera/infra1/image_rect_raw";
  std::string right_image_topic = "/camera/infra2/image_rect_raw";
  pnh.param<std::string>("left_image_topic", left_image_topic,
                         left_image_topic);
  pnh.param<std::string>("right_image_topic", right_image_topic,
                         right_image_topic);
  ROS_INFO_STREAM("Stereo subscribing left image topic: " << left_image_topic);
  ROS_INFO_STREAM("Stereo subscribing right image topic: " << right_image_topic);

  stringstream ss(argv[3]);
  ss >> boolalpha >> igb.do_rectify;

  if (igb.do_rectify) {
    // Load settings related to stereo calibration
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

  message_filters::Subscriber<sensor_msgs::Image> left_sub(nh,
                                                           left_image_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> right_sub(
      nh, right_image_topic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,
                                               right_sub);
  sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

  ros::spin();

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft,
                              const sensor_msgs::ImageConstPtr &msgRight) {
  // Copy the ros image message to cv::Mat.
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

  RefreshRelocalizationStatus();

  const int tracking_state = mpSLAM->GetTrackingState();
  const bool tracking_ok = tracking_state == ORB_SLAM3::Tracking::OK ||
                           tracking_state == ORB_SLAM3::Tracking::OK_KLT;
  const bool pose_ok = tracking_ok && PoseIsFinite(Tcw);
  PublishTrackingStatus(pose_ok);

  if (!pose_ok) {
    EnterUnstableMappingWindow();
    ResetVelocityState();
    if (has_last_continuous_pose_) {
      PublishPose(last_continuous_Twc_, cv_ptrLeft->header.stamp);
      PublishOdometry(last_continuous_Twc_, cv_ptrLeft->header.stamp, false);
    }
    if (has_last_slam_pose_) {
      PublishSlamPose(last_slam_Twc_, cv_ptrLeft->header.stamp);
    }
    if (has_last_corrected_map_pose_) {
      PublishCorrectedMapPose(last_corrected_map_pose_,
                              cv_ptrLeft->header.stamp);
      if (has_last_corrected_map_transform_) {
        PublishPlanningMapTransform(last_corrected_map_transform_,
                                    cv_ptrLeft->header.stamp);
      }
    }
    return;
  }

  const Sophus::SE3f raw_Twc = Tcw.inverse();
  last_slam_Twc_ = raw_Twc;
  has_last_slam_pose_ = true;
  PublishSlamPose(raw_Twc, cv_ptrLeft->header.stamp);

  const Sophus::SE3f corrected_map_Twc = GetCorrectedMapPose(raw_Twc);
  PublishCorrectedMapPose(corrected_map_Twc, cv_ptrLeft->header.stamp);
  if (has_last_corrected_map_transform_) {
    PublishPlanningMapTransform(last_corrected_map_transform_,
                                cv_ptrLeft->header.stamp);
  }

  const Sophus::SE3f continuous_Twc = GetContinuousPose(raw_Twc);
  PublishPose(continuous_Twc, cv_ptrLeft->header.stamp);
  PublishOdometry(continuous_Twc, cv_ptrLeft->header.stamp, true);
}
