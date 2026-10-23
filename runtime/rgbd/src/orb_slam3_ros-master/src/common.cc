/**
* 
* Common functions and variables for position publishing and mapping services
*
*/

#include "common.h"

ORB_SLAM3::System* pSLAM = nullptr;
bool rgbd_map_initialized = false;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::STEREO;  
ros::Publisher pose_pub, odom_pub, slam_pose_pub, tracking_ok_pub;
ros::Publisher legacy_camera_pose_pub, legacy_body_odom_pub;

namespace {

constexpr char kContinuousFrame[] = "map";
constexpr char kSlamFrame[] = "slam_map";
constexpr char kOdomChildFrame[] = "camera_infra1_frame";

bool has_current_map_id = false;
bool has_last_continuous_pose = false;
bool has_last_slam_pose = false;
long unsigned int current_map_id = 0;
Sophus::SE3f continuous_T_slam_map;
Sophus::SE3f last_continuous_pose;
Sophus::SE3f last_slam_pose;

bool poseIsFinite(const Sophus::SE3f &pose) {
    return pose.translation().allFinite() && pose.rotationMatrix().allFinite();
}

Sophus::SE3f convertTrackingPoseToRosPose(const Sophus::SE3f &Tcw) {
    const Sophus::SE3f Twc_vision = Tcw.inverse();

    Eigen::Matrix3f R_v2r;
    R_v2r << 0, 0, 1,
            -1, 0, 0,
             0, -1, 0;

    const Eigen::Vector3f t_ros = R_v2r * Twc_vision.translation();
    const Eigen::Matrix3f R_ros =
        R_v2r * Twc_vision.rotationMatrix() * R_v2r.transpose();
    return Sophus::SE3f(R_ros, t_ros);
}

std::uint64_t PublicationSteadyNanoseconds() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void publishTrackingStatus(bool tracking_ok, PosePublishOutcome* outcome) {
    std_msgs::Bool msg;
    msg.data = tracking_ok;
    tracking_ok_pub.publish(msg);
    if (outcome) {
        const unsigned index = outcome->state_publications++;
        if (index < 2) {
            outcome->state_values[index] = tracking_ok;
            outcome->state_steady_ns[index] = PublicationSteadyNanoseconds();
        }
    }
}

void RecordRawPublication(PosePublishOutcome* outcome,
                          const Sophus::SE3f& pose, bool fresh) {
    if (!outcome) return;
    outcome->raw_slam_emitted = true;
    outcome->raw_slam_fresh = fresh;
    outcome->raw_slam_valid = poseIsFinite(pose);
    outcome->raw_slam_steady_ns = PublicationSteadyNanoseconds();
}

void publishPoseMessage(const Sophus::SE3f &pose, const ros::Time &stamp,
                        const std::string &frame_id, ros::Publisher &publisher) {
    const Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf orientation(pose.rotationMatrix());
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

void publishOdometryMessage(const Sophus::SE3f &pose, const ros::Time &stamp) {
    const Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf orientation(pose.rotationMatrix());
    orientation.normalize();

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = kContinuousFrame;
    odom_msg.child_frame_id = kOdomChildFrame;
    odom_msg.pose.pose.position.x = translation.x();
    odom_msg.pose.pose.position.y = translation.y();
    odom_msg.pose.pose.position.z = translation.z();
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom_msg);
    legacy_body_odom_pub.publish(odom_msg);
}

}  // namespace

//////////////////////////////////////////////////
// Main functions
//////////////////////////////////////////////////

bool handle_mapping_service(robot_interfaces::SetTypeService::Request& req, 
                            robot_interfaces::SetTypeService::Response& res) {
    if (!pSLAM) {
        res.success = false;
        res.message = "SLAM system pointer is null";
        ROS_ERROR("SLAM system pointer is null in service callback");
        return true;
    }
    
    int cmd_type = req.type;
    
    switch(cmd_type) {
        case 0: { // 切换到定位模式（停止建图）
            ROS_INFO("Switching to localization mode (stopping mapping)...");
            try {
                pSLAM->ActivateLocalizationMode();
                res.success = true;
                res.message = "Successfully switched to localization mode. Mapping stopped.";
                ROS_INFO("%s", res.message.c_str());
            } catch (const std::exception& e) {
                res.success = false;
                res.message = std::string("Error switching to localization mode: ") + e.what();
                ROS_ERROR("%s", res.message.c_str());
            }
            break;
        }
                
        case 1: { // 切换到建图模式（恢复建图）
            ROS_INFO("Switching to mapping mode (resuming mapping)...");
            try {
                pSLAM->DeactivateLocalizationMode();
                res.success = true;
                res.message = "Successfully switched to mapping mode. Mapping resumed.";
                ROS_INFO("%s", res.message.c_str());
            } catch (const std::exception& e) {
                res.success = false;
                res.message = std::string("Error switching to mapping mode: ") + e.what();
                ROS_ERROR("%s", res.message.c_str());
            }
            break;
        }
                
        case 2: { // 保存地图并切换到定位模式
            // No trajectory origin exists before the first initialized RGB-D frame.
            if (sensor_type == ORB_SLAM3::System::RGBD && !rgbd_map_initialized) {
                res.success = true;
                res.message = "No initialized RGB-D map; skipped map and trajectory export.";
                break;
            }
            ROS_INFO("Saving map and switching to localization mode...");
            try {
                // 保存轨迹文件
                std::string timestamp = std::to_string(ros::Time::now().toSec());
                std::string kf_traj_file = "KeyFrameTrajectory_TUM_Format_" + timestamp + ".txt";
                std::string frame_traj_file = "FrameTrajectory_TUM_Format_" + timestamp + ".txt";
                
                pSLAM->SaveKeyFrameTrajectoryTUM(kf_traj_file);
                pSLAM->SaveTrajectoryTUM(frame_traj_file);
                
                // 使用您添加的新方法保存完整地图
                std::string map_filename = "ORB_SLAM3_Map_" + timestamp;
                bool map_saved = pSLAM->SaveMap(map_filename);
                
                // 切换到定位模式
                pSLAM->ActivateLocalizationMode();
                
                res.success = true;
                std::stringstream ss;
                if (map_saved) {
                    ss << "Successfully saved full map to " << map_filename 
                       << ".osa, trajectory files to " << kf_traj_file << " and " 
                       << frame_traj_file << ", then switched to localization mode.";
                } else {
                    ss << "Successfully saved trajectory files to " << kf_traj_file 
                       << " and " << frame_traj_file 
                       << ", but failed to save full map. Switched to localization mode.";
                }
                res.message = ss.str();
                ROS_INFO("%s", res.message.c_str());
            } catch (const std::exception& e) {
                res.success = false;
                res.message = std::string("Error saving map and switching to localization mode: ") + e.what();
                ROS_ERROR("%s", res.message.c_str());
            }
            break;
        }
                
        default:
            res.success = false;
            res.message = "Invalid command type. Valid types: 0(localize), 1(mapping), 2(save+localize)";
            ROS_WARN("Received invalid command type: %d", cmd_type);
            break;
    }
    
    return true;
}

void setup_services(ros::NodeHandle &node_handler, std::string node_name)
{
    static ros::ServiceServer mapping_service = node_handler.advertiseService("/slam/mapping", handle_mapping_service);
    ROS_INFO("ORB-SLAM3 Mapping Service initialized at /slam/mapping");
    ROS_INFO("Available commands:");
    ROS_INFO("  data = 0: Switch to localization mode (stop mapping)");
    ROS_INFO("  data = 1: Switch to mapping mode (resume mapping)");
    ROS_INFO("  data = 2: Save map and switch to localization mode");
}

void setup_publishers(ros::NodeHandle &node_handler, std::string node_name)
{
    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
    odom_pub = node_handler.advertise<nav_msgs::Odometry>("odometry", 1);
    slam_pose_pub = node_handler.advertise<geometry_msgs::PoseStamped>("robot_pose_slam", 1);
    tracking_ok_pub = node_handler.advertise<std_msgs::Bool>("robot_pose_tracking_ok", 1);
    legacy_camera_pose_pub =
        node_handler.advertise<geometry_msgs::PoseStamped>("orb_slam3/camera_pose", 1);
    legacy_body_odom_pub =
        node_handler.advertise<nav_msgs::Odometry>("orb_slam3/body_odom", 1);
}

void publish_pose_and_odometry(const Sophus::SE3f &Tcw, ros::Time stamp,
                               bool pose_ok, PosePublishOutcome* outcome) {
    if (outcome) *outcome = PosePublishOutcome();
    publishTrackingStatus(pose_ok, outcome);

    if (!pose_ok) {
        if (has_last_continuous_pose) {
            publishPoseMessage(last_continuous_pose, stamp, kContinuousFrame, pose_pub);
            publishPoseMessage(last_continuous_pose, stamp, kContinuousFrame,
                               legacy_camera_pose_pub);
            publishOdometryMessage(last_continuous_pose, stamp);
        }
        if (has_last_slam_pose) {
            publishPoseMessage(last_slam_pose, stamp, kSlamFrame, slam_pose_pub);
            RecordRawPublication(outcome, last_slam_pose, false);
        }
        return;
    }

    const Sophus::SE3f raw_slam_pose = convertTrackingPoseToRosPose(Tcw);
    if (!poseIsFinite(raw_slam_pose)) {
        publishTrackingStatus(false, outcome);
        if (has_last_continuous_pose) {
            publishPoseMessage(last_continuous_pose, stamp, kContinuousFrame, pose_pub);
            publishPoseMessage(last_continuous_pose, stamp, kContinuousFrame,
                               legacy_camera_pose_pub);
            publishOdometryMessage(last_continuous_pose, stamp);
        }
        if (has_last_slam_pose) {
            publishPoseMessage(last_slam_pose, stamp, kSlamFrame, slam_pose_pub);
            RecordRawPublication(outcome, last_slam_pose, false);
        }
        return;
    }

    const long unsigned int map_id = pSLAM ? pSLAM->GetCurrentMapId() : 0;
    const bool map_id_changed = has_current_map_id && map_id != current_map_id;

    if (!has_current_map_id) {
        current_map_id = map_id;
        has_current_map_id = true;
    }

    if (!has_last_continuous_pose) {
        continuous_T_slam_map = Sophus::SE3f();
    } else if (map_id_changed) {
        continuous_T_slam_map = last_continuous_pose * raw_slam_pose.inverse();
        ROS_WARN_STREAM("Rebased /robot_pose continuity after ORB-SLAM map switch map_id "
                        << current_map_id << " -> " << map_id);
    }

    const Sophus::SE3f continuous_pose = continuous_T_slam_map * raw_slam_pose;

    publishPoseMessage(raw_slam_pose, stamp, kSlamFrame, slam_pose_pub);
    RecordRawPublication(outcome, raw_slam_pose, true);
    publishPoseMessage(continuous_pose, stamp, kContinuousFrame, pose_pub);
    publishPoseMessage(continuous_pose, stamp, kContinuousFrame,
                       legacy_camera_pose_pub);
    publishOdometryMessage(continuous_pose, stamp);

    last_slam_pose = raw_slam_pose;
    has_last_slam_pose = true;
    last_continuous_pose = continuous_pose;
    has_last_continuous_pose = true;
    current_map_id = map_id;
}
