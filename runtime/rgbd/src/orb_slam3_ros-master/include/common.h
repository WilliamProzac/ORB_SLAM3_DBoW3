#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cstdint>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <robot_interfaces/SetTypeService.h> 

// ORB-SLAM3-specific libraries
#include "System.h"
#include "ImuTypes.h"

extern ORB_SLAM3::System* pSLAM;
extern ORB_SLAM3::System::eSensor sensor_type;
extern bool rgbd_map_initialized;

void setup_services(ros::NodeHandle&, std::string);
void setup_publishers(ros::NodeHandle&, std::string);
struct PosePublishOutcome {
    unsigned state_publications = 0;
    bool state_values[2] = {false, false};
    std::uint64_t state_steady_ns[2] = {0, 0};
    bool raw_slam_emitted = false;
    bool raw_slam_fresh = false;
    bool raw_slam_valid = false;
    std::uint64_t raw_slam_steady_ns = 0;
};
void publish_pose_and_odometry(const Sophus::SE3f&, ros::Time, bool,
                               PosePublishOutcome* outcome = nullptr);

bool handle_mapping_service(robot_interfaces::SetTypeService::Request&, 
                            robot_interfaces::SetTypeService::Response&);
