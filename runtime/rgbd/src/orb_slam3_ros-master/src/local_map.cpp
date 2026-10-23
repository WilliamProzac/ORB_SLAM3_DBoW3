#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <string>
#include <memory>
#include "PointCloudToOccupancyGridConverter.h"

PointCloudToOccupancyGridConverter::PointCloudToOccupancyGridConverter(ros::NodeHandle& nh) : nh_(nh)
{
    // 配置参数
    resolution_ = nh_.param<double>("grid_resolution", 0.05); // 米/像素
    radius_ = nh_.param<double>("grid_radius", 3.0);         // 地图半径（米）
    height_min_ = nh_.param<double>("height_offset_min", 0.0); // 高度过滤下限
    height_max_ = nh_.param<double>("height_offset_max", 0.5); // 高度过滤上限

    // 从参数服务器获取 frame_id，默认为 "map"
    frame_id_ = nh_.param<std::string>("local_map_frame_id", "map");

    // 从参数服务器获取发布频率，默认为 10 Hz
    double publish_freq = nh_.param<double>("local_map_publish_freq", 10.0);
    publish_interval_ = publish_freq > 0.0 ? 1.0 / publish_freq : 0.1; // 确保非零间隔

    // 创建发布器
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/ppseg_gridmap", 1);

    // 初始化发布频率控制
    last_publish_time_ = ros::Time::now();

    ROS_INFO("PointCloud to OccupancyGrid converter initialized. Resolution: %.2f m, Radius: %.2f m, Freq: %.1f Hz", 
             resolution_, radius_, publish_freq);
}


// 将点云转换为占据栅格
nav_msgs::OccupancyGrid::Ptr PointCloudToOccupancyGridConverter::convertToGrid(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg,
                                                                             const nav_msgs::Odometry::ConstPtr& odom_msg) {
    try {
        // 转换ROS点云到PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // 获取相机Z坐标
        double cam_z = odom_msg->pose.pose.position.z;

        // 动态计算栅格参数
        double center_x = odom_msg->pose.pose.position.x;
        double center_y = odom_msg->pose.pose.position.y;

        // 计算栅格原点（左下角）
        double origin_x = center_x - radius_;
        double origin_y = center_y - radius_;

        // 计算栅格尺寸
        double grid_diameter = 2 * radius_;
        int grid_size = static_cast<int>(grid_diameter / resolution_);

        // 创建占据栅格消息
        nav_msgs::OccupancyGrid::Ptr grid_msg(new nav_msgs::OccupancyGrid());
        grid_msg->header.stamp = ros::Time::now();
        grid_msg->header.frame_id = frame_id_;

        // 设置栅格元数据
        grid_msg->info.resolution = resolution_;
        grid_msg->info.width = grid_size;
        grid_msg->info.height = grid_size;
        grid_msg->info.origin.position.x = origin_x;
        grid_msg->info.origin.position.y = origin_y;
        grid_msg->info.origin.position.z = 0.0;
        grid_msg->info.origin.orientation.w = 1.0;

        // 初始化栅格数据 (默认未知 -1)
        grid_msg->data.resize(grid_size * grid_size, -1); // -1 表示未知

        // 遍历点云，标记占据栅格
        for (const auto& point : cloud->points) {
            // 高度过滤
            if (point.z < (cam_z + height_min_) || point.z > (cam_z + height_max_)) {
                continue; // 跳过不符合高度要求的点
            }

            // 计算点在栅格地图中的索引
            int x_idx = static_cast<int>((point.x - origin_x) / resolution_);
            int y_idx = static_cast<int>((point.y - origin_y) / resolution_);

            // 检查索引是否在栅格范围内
            if (x_idx >= 0 && x_idx < grid_size && y_idx >= 0 && y_idx < grid_size) {
                // 将对应栅格标记为占据 (100)
                grid_msg->data[y_idx * grid_size + x_idx] = 100;
            }
        }
        return grid_msg;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Conversion error: " << e.what());
        return nullptr;
    }
}

// 发布占据栅格
void PointCloudToOccupancyGridConverter::publishGrid(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg,
                                                   const nav_msgs::Odometry::ConstPtr& odom_msg) {

    ros::Time current_time = ros::Time::now();
    if ((current_time - last_publish_time_).toSec() < publish_interval_) {
        return;
    }

    nav_msgs::OccupancyGrid::Ptr grid_msg = convertToGrid(point_cloud_msg, odom_msg);
    if (grid_msg != nullptr) {
        grid_pub_.publish(*grid_msg);
        last_publish_time_ = current_time;
        // ROS_DEBUG("Published local occupancy grid.");
    }
}