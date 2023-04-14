#ifndef SCAN_TO_POINTCLOUD2_CONVERTER_H
#define SCAN_TO_POINTCLOUD2_CONVERTER_H
#include<iostream>
#include<ros/ros.h>

#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>

#include<pcl/visualization/cloud_viewer.h>
#include<pcl/filters/filter.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/passthrough.h>




class ScanToPointcloud2Converter
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Subscriber laserscan_sub;
    ros::Subscriber odom_sub;
    ros::Publisher pointcloud2_pub;
    ros::Publisher odom_pub;

    PointT invalid_point;
    PointT copy_invalid_point;

public:
    ScanToPointcloud2Converter();
    ~ScanToPointcloud2Converter();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msgs);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs);

};
#endif