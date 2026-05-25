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

class ImageGrabber {
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh)
      : mpSLAM(pSLAM),
        publish_relocalization_status_(pSLAM &&
                                       pSLAM->IsOnlyTrackingEnabled()) {
    robot_pose_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
    odometry_publisher_ = nh.advertise<nav_msgs::Odometry>("/odometry", 1);

    if (publish_relocalization_status_) {
      relocalization_status_publisher_ =
          nh.advertise<ORB_SLAM3::RelocalizationStatus>(
              "/relocalization_status", 1);
      relocalization_status_timer_ = nh.createTimer(
          ros::Duration(0.1),
          &ImageGrabber::PublishRelocalizationStatusTimer, this);
    }
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
    robot_pose_publisher_.publish(pose_msg);
  }

  void PublishOdometry(const Sophus::SE3f &Tcw, const ros::Time &stamp) {
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f translation = Twc.translation();
    Eigen::Quaternionf orientation(Twc.rotationMatrix());
    orientation.normalize();

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "left_camera";
    odom_msg.pose.pose.position.x = translation.x();
    odom_msg.pose.pose.position.y = translation.y();
    odom_msg.pose.pose.position.z = translation.z();
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();
    SetDiagonalCovariance(odom_msg.pose.covariance, kUnknownCovariance);

    TwistEstimate twist_estimate = EstimateTwist(Twc, stamp);
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

  ORB_SLAM3::System *mpSLAM;
  ros::Publisher robot_pose_publisher_;
  ros::Publisher odometry_publisher_;
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

  void ResetVelocityState() {
    has_previous_valid_pose_ = false;
    suppress_next_twist_ = false;
  }

  void StoreVelocityReference(const Sophus::SE3f &Twc,
                              const ros::Time &stamp) {
    previous_valid_pose_ = Twc;
    previous_valid_stamp_ = stamp;
    has_previous_valid_pose_ = true;
  }

  TwistEstimate EstimateTwist(const Sophus::SE3f &Twc,
                              const ros::Time &stamp) {
    TwistEstimate estimate;

    if (!has_previous_valid_pose_) {
      StoreVelocityReference(Twc, stamp);
      return estimate;
    }

    if (suppress_next_twist_) {
      suppress_next_twist_ = false;
      StoreVelocityReference(Twc, stamp);
      return estimate;
    }

    const double dt = (stamp - previous_valid_stamp_).toSec();
    if (!std::isfinite(dt) || dt <= 0.0 || dt < kMinVelocityDtSec ||
        dt > kMaxVelocityDtSec) {
      StoreVelocityReference(Twc, stamp);
      return estimate;
    }

    const Sophus::SE3f Tprev_current = previous_valid_pose_.inverse() * Twc;
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

    StoreVelocityReference(Twc, stamp);

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
  Sophus::SE3f previous_valid_pose_;
  ros::Time previous_valid_stamp_;
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

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO,
                         bUseViewer);

  ros::NodeHandle nh;
  ImageGrabber igb(&SLAM, nh);

  stringstream ss(argv[3]);
  ss >> boolalpha >> igb.do_rectify;

  if (igb.do_rectify) {
    // Load settings related to stereo calibration
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

  if (!tracking_ok) {
    ResetVelocityState();
    return;
  }

  PublishPose(Tcw, cv_ptrLeft->header.stamp);
  PublishOdometry(Tcw, cv_ptrLeft->header.stamp);
}
