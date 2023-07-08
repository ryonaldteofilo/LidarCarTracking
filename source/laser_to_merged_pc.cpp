#include "common.h"

// ROS
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

laser_geometry::LaserProjection projector1;
ros::Publisher pc_pub1;
sensor_msgs::PointCloud2::Ptr cloud1 (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr cloud3 (new sensor_msgs::PointCloud2);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud3 (new pcl::PointCloud<pcl::PointXYZ>);

void scanCallback1 (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  projector1.projectLaser(*scan_in, *cloud1); // convert laserscan into point cloud form
}

void scanCallback2 (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud2 cloud2;
  projector1.projectLaser(*scan_in, cloud2); // convert laserscan into point cloud form
  pcl::fromROSMsg(*cloud1, *pcl_cloud1); // convert ROS message into PCL data type
  pcl::fromROSMsg(cloud2, *pcl_cloud2);
  for (std::size_t i = 0; i < (*pcl_cloud2).points.size (); ++i)
    {
      (*pcl_cloud2).points[i].y += 1.28;  //add 1.28m to the y-coordinate of points from lidar2
    }
  *pcl_cloud3  = *pcl_cloud1;
  *pcl_cloud3  += *pcl_cloud2; // concatenate point clouds
  pcl::toROSMsg (*pcl_cloud3 , *cloud3); //convert back to ROS message type
  pc_pub1.publish(*cloud3);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_to_pc");
  ros::NodeHandle n1;
  ros::Subscriber laser_sub1 = n1.subscribe<sensor_msgs::LaserScan>("/lidar1_filtered", 1, scanCallback1);
  ros::Subscriber laser_sub2 = n1.subscribe<sensor_msgs::LaserScan>("/lidar2_filtered", 1, scanCallback2);
  pc_pub1 = n1.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);

  ros::spin();

  return 0;
}
