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


class ScanToPointCloud2
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Publisher  pointclould2_pub;
    ros::Publisher  odom_pub;
    ros::Subscriber laserscan_sub;
    ros::Subscriber odom_sub;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointCloud<pcl::PointXYZI> copy_cloud;

public:
    ScanToPointCloud2();
    ~ScanToPointCloud2();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &input);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs);

};

ScanToPointCloud2::ScanToPointCloud2()
{
    ROS_INFO_STREAM("\033[1;32m----> Scan To PointCloud2  Stated.\033[0m");
    pointclould2_pub = nh.advertise<sensor_msgs::LaserScan>("registered_scan", 10,this);
    odom_pub=nh.advertise<nav_msgs::Odometry>("state_estimation_at_scan",10,this);
    laserscan_sub=nh.subscribe("scan",10,&ScanToPointCloud2::ScanCallback,this);
    odom_sub=nh.subscribe("odom",10,&ScanToPointCloud2::OdomCallback,this);

}

ScanToPointCloud2::~ScanToPointCloud2()
{
}
void ScanToPointCloud2::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &input)
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
        // cloud.points[i].intensity = 1; 实际应该=input->intensities[i];
      }
      //****************************角度->坐标的转换*****************************


      //****************************垂直方向上复制坐标*****************************
      std::vector<int> mapping;
      //去除nan点
      pcl::copyPointCloud(cloud, copy_cloud);
      pcl::removeNaNFromPointCloud(cloud, cloud, mapping);

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

      sensor_msgs::PointCloud2 ScanData;
      pcl::toROSMsg(cloud,ScanData);
      ScanData.header.stamp = input->header.stamp;
      ScanData.header.frame_id = input->header.frame_id;
      pointclould2_pub.publish(ScanData);
}


void ScanToPointCloud2::OdomCallback(const nav_msgs::Odometry::ConstPtr &odominput){
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
    odomData.child_frame_id = odominput->child_frame_id;
    odom_pub.publish(odomData);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_point");
    ScanToPointCloud2 scantopoint;
    ros::spin();

    return 0;
}
