// occupancy_grid_publisher_node.cpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h> 
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <memory>
#include "PointCloudToOccupancyGridConverter.h" 


std::unique_ptr<PointCloudToOccupancyGridConverter> converter;


void pointCloudAndOdomCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg,
                              const nav_msgs::OdometryConstPtr& odom_msg) {
    
    converter->publishGrid(pc_msg, odom_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "occupancy_grid_publisher");
    ros::NodeHandle nh;

    converter = std::make_unique<PointCloudToOccupancyGridConverter>(nh);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/camera/depth/cloud_colored", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odometry", 1); 

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), pc_sub, odom_sub);
    sync.registerCallback(boost::bind(&pointCloudAndOdomCallback, _1, _2));

    ROS_INFO("Occupancy Grid Publisher Node initialized. Subscribing to /camera/depth/cloud_colored and /odometry.");

    ros::spin();

    return 0;
}