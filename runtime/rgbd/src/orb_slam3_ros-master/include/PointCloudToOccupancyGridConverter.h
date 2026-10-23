#ifndef POINT_CLOUD_TO_OCCUPANCY_GRID_CONVERTER_H
#define POINT_CLOUD_TO_OCCUPANCY_GRID_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>

class PointCloudToOccupancyGridConverter {
public:

    using Ptr = std::shared_ptr<PointCloudToOccupancyGridConverter>;  
    PointCloudToOccupancyGridConverter(ros::NodeHandle& nh);

    // 设置栅格地图的frame_id
    void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
    // 将点云转换为占据栅格
    nav_msgs::OccupancyGrid::Ptr convertToGrid(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg,
                                              const nav_msgs::Odometry::ConstPtr& odom_msg);
    // 发布占据栅格
    void publishGrid(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg,
                    const nav_msgs::Odometry::ConstPtr& odom_msg);

private:
    // ROS节点句柄
    ros::NodeHandle nh_;

    // 发布器
    ros::Publisher grid_pub_;

    // 参数
    double origin_x_;
    double origin_y_;
    double resolution_;
    int width_;
    int height_;
    double radius_;
    double z_min_;
    double z_max_;
    std::string frame_id_; 
    ros::Time last_publish_time_; // 上次发布时间
    double publish_interval_; // 发布时间间隔（秒）
    double height_min_;
    double height_max_;
};

#endif // POINT_CLOUD_TO_OCCUPANCY_GRID_CONVERTER_H 