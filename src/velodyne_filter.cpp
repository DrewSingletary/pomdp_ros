#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include "pcl_ros/point_cloud.h"
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/crop_box.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <stdlib.h>
#include <math.h> 

ros::Publisher cloud_pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  float depthThreshold = 0.5;
  float threshold2 = depthThreshold*depthThreshold;

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  pcl_conversions::toPCL(*cloud_msg, *cloud);

  float minX = -4, minY = -1.25, minZ = -2.5;
  float maxX = +2, maxY = +1.25, maxZ = +2.5;
  pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
  boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
  boxFilter.setInputCloud(cloudPtr);
  boxFilter.setNegative(true);
  boxFilter.filter(cloud_filtered);


  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (1, 1, 1);
  // sor.setFilterFieldName ("x");
  // sor.setFilterLimits (0, 100.0);
  // sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  cloud_pub.publish (output);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "integrator");

  ros::NodeHandle n_;

  ros::Subscriber cloud_sub = n_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, cloud_cb);
  cloud_pub = n_.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
  ROS_INFO("filter is ready");

  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}