
#include<ros/ros.h>
#include<iostream>

#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/Odometry.h>

#include<tf/transform_broadcaster.h>
#include<pcl/visualization/cloud_viewer.h>

#include<pcl/filters/filter.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/passthrough.h>
#include<sensor_msgs/PointCloud2.h>



// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>



pcl::PointCloud<pcl::PointXYZI> cloud;
pcl::PointCloud<pcl::PointXYZI> copy_cloud;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher* pointcloud_pub_1 = NULL;
ros::Publisher* odom_pub_1 = NULL;

ros::Time odomTime;

void scancallback(const sensor_msgs::LaserScan::ConstPtr &input)
{
    //   pcl::PointCloud<pcl::PointXYZI> cloud;
    //   pcl::PointCloud<pcl::PointXYZI> copy_cloud;
      cloud.clear();
      copy_cloud.clear();
      //****************************给cloud点云赋值    laserscan->pointcloud2**********************************
      std::vector<float> ranges = input->ranges;
      //容量
      cloud.points.resize(ranges.size());
      //width 赋值
      cloud.height = 1;
      cloud.width = ranges.size();
      cloud.header.stamp = input->header.stamp.toSec();
      cloud.header.frame_id = input->header.frame_id;

      //****************************角度->坐标的转换*****************************
      for (int  i = 0; i < ranges.size(); i++)
      {
        double angle = input->angle_min + i * input->angle_increment;
        double lx = ranges[i] * cos(angle);
        double ly = ranges[i] * sin(angle);
        float intensity = input->intensities[i];
        cloud.points[i].x = lx;
        cloud.points[i].y = ly;
        cloud.points[i].z = 0;
        cloud.points[i].intensity = input->intensities[i];
        // cloud.points[i].intensity = 1;
        //  cloud.points[i].intensity = 1; 实际应该=input->intensities[i];
      }
      //****************************角度->坐标的转换*****************************


      //****************************垂直方向上复制坐标*****************************
      std::vector<int> mapping;
      //去除nan点
      pcl::removeNaNFromPointCloud(cloud, cloud, mapping);
      pcl::copyPointCloud(cloud, copy_cloud);

      for(int h=0;h<=20;h+=2)
      {
          for(int i=0;i<copy_cloud.points.size();i++)
          {
              copy_cloud.points[i].z=(float)h/10;
          }
          // std::cout<<"cloud_add.points[i].z: "<<(float)h/10<<std::endl;
          cloud=cloud+copy_cloud;
      }
      //****************************垂直方向上复制坐标*****************************
      
      //****************************发布pointCloud2*****************************

      double odomRecTime = odomTime.toSec();
      sensor_msgs::PointCloud2 ScanData;
      cloud.is_dense = false; // contains nans

      pcl::toROSMsg(cloud,ScanData);
    //   ScanData.header.stamp = ros::Time().fromSec(odomRecTime);
      ScanData.header.stamp = input->header.stamp;
      ScanData.header.frame_id = input->header.frame_id;
      pointcloud_pub_1->publish(ScanData);
}
void odomcallback(const nav_msgs::Odometry::ConstPtr &odominput)
{
    // odomData.header.stamp = odomTime;
    // odomData.pose.pose.orientation = geoQuat;
    // odomData.pose.pose.position.x = vehicleX;
    // odomData.pose.pose.position.y = vehicleY;
    // odomData.pose.pose.position.z = vehicleZ;
    // odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    // odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    // odomData.twist.twist.angular.z = vehicleYawRate;
    // odomData.twist.twist.linear.x = vehicleSpeed;
    // odomData.twist.twist.linear.z = 200.0 * (vehicleZ - vehicleRecZ);
    // pubVehicleOdom.publish(odomData);
    double odomRecTime = odomTime.toSec();
    nav_msgs::Odometry odomData;
    odomData.header.stamp = odominput->header.stamp;
    odomData.pose.pose.orientation = odominput->pose.pose.orientation;
    odomData.pose.pose.position.x = odominput->pose.pose.position.x;
    odomData.pose.pose.position.y = odominput->pose.pose.position.x;
    odomData.pose.pose.position.z = odominput->pose.pose.position.x;
    odomData.twist.twist.linear.x = odominput->twist.twist.linear.x;
    odomData.twist.twist.linear.y = odominput->twist.twist.linear.y;
    odomData.twist.twist.angular.x = odominput->twist.twist.angular.x;
    odomData.twist.twist.angular.y = odominput->twist.twist.angular.y;
    odomData.twist.twist.angular.z = odominput->twist.twist.angular.z;
    odomData.header.frame_id = odominput->header.frame_id;
    // odomData.child_frame_id = odominput->child_frame_id;
    odom_pub_1->publish(odomData);
}
int main(int argc, char **argv)
{
    
    //registered_scan   state_estimation  
    ros::init(argc, argv, "rs_se_pub");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::Subscriber scan_sub=nh.subscribe<sensor_msgs::LaserScan>("scan",10,scancallback);
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("odom",10,odomcallback);
    ros::Publisher pointcloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/registered_scan",2);
    pointcloud_pub_1 = &pointcloud_pub;
    ros::Publisher odom_pub=nh.advertise<nav_msgs::Odometry>("/state_estimation_at_scan",5);
    odom_pub_1 = &odom_pub;
    
    ros::spin();
    return 0;
}
