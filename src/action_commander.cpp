#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Joy.h"

#include "nav_msgs/OccupancyGrid.h"

#include "pcl_ros/point_cloud.h"
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/crop_box.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "std_msgs/Bool.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <stdlib.h>
#include <math.h>

typedef Eigen::VectorXd state_type;

ros::Publisher uav_action_pub;
ros::Publisher segway_action_pub;
ros::Publisher flipper_action_pub;

int uav_action = 0;
int flipper_action = 0;
int segway_action = 0;

bool uav_selected = false;
bool segway_selected = false;
bool flipper_selected = false;

nav_msgs::OccupancyGrid uav_belief;
nav_msgs::OccupancyGrid segway_belief;
nav_msgs::OccupancyGrid flipper_belief;
nav_msgs::OccupancyGrid grid_hab;
nav_msgs::OccupancyGrid grid_sam;

void joy_cb(const sensor_msgs::Joy & msg) {
  int rate = 1;
  ros::Rate r(rate);
  if (msg.buttons[3] == 1) {
    uav_selected = true;
    segway_selected = false;
    flipper_selected = false;
  }
  if (msg.buttons[2] == 1) {
    uav_selected = false;
    segway_selected = true;
    flipper_selected = false;
  }
  if (msg.buttons[1] == 1) {
    uav_selected = false;
    segway_selected = false;
    flipper_selected = true;
  }
  if (uav_selected && msg.axes[6] == 1) {
    uav_action = 1;
  }
  if (uav_selected && msg.axes[7] == -1) {
    uav_action = 2;
  }
  if (uav_selected && msg.axes[6] == -1) {
    uav_action = 3;
  }
  if (uav_selected && msg.axes[7] == 1) {
    uav_action = 4;
  }

  if (segway_selected && msg.axes[6] == 1) {
    segway_action = 1;
  }
  if (segway_selected && msg.axes[7] == -1) {
    segway_action = 2;
  }
  if (segway_selected && msg.axes[6] == -1) {
    segway_action = 3;
  }
  if (segway_selected && msg.axes[7] == 1) {
    segway_action = 4;
  }

  if (flipper_selected && msg.axes[6] == 1) {
    flipper_action = 1;
  }
  if (flipper_selected && msg.axes[7] == -1) {
    flipper_action = 2;
  }
  if (flipper_selected && msg.axes[6] == -1) {
    flipper_action = 3;
  }
  if (flipper_selected && msg.axes[7] == 1) {
    flipper_action = 4;
  }

  if (msg.buttons[0] == 1) {
    uav_selected = false;
    segway_selected = false;
    flipper_selected = false;

    std_msgs::Int32 uav_action_message;
    std_msgs::Int32 segway_action_message;
    std_msgs::Int32 flipper_action_message;

    uav_action_message.data = uav_action;
    segway_action_message.data = segway_action;
    flipper_action_message.data = flipper_action;

    uav_action_pub.publish(uav_action_message);
    segway_action_pub.publish(segway_action_message);
    flipper_action_pub.publish(flipper_action_message);

    int uav_action = 0;
    int flipper_action = 0;
    int segway_action = 0;
    r.sleep();
  }
}

void uav_belief_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  uav_belief = *msg;
}

void segway_belief_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  segway_belief = *msg;
}

void flipper_belief_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  flipper_belief = *msg;
}

void grid_hab_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  grid_hab = *msg;
}

void grid_sam_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  grid_sam = *msg;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "action_commander");

  ros::NodeHandle n_;

  ros::Subscriber uav_belief_sub = n_.subscribe<nav_msgs::OccupancyGrid>("uav/belief", 1, uav_belief_cb);
  ros::Subscriber segway_belief_sub = n_.subscribe<nav_msgs::OccupancyGrid>("segway/belief", 1, segway_belief_cb);
  ros::Subscriber flipper_belief_sub = n_.subscribe<nav_msgs::OccupancyGrid>("flipper/belief", 1,flipper_belief_cb);
  ros::Subscriber grid_hab_sub = n_.subscribe<nav_msgs::OccupancyGrid>("grid/habitable", 1,grid_hab_cb);
  ros::Subscriber grid_sam_sub = n_.subscribe<nav_msgs::OccupancyGrid>("grid/sample", 1,grid_sam_cb);

  ros::Subscriber joy_sub = n_.subscribe("/joy", 1, joy_cb);

  uav_action_pub = n_.advertise<std_msgs::Int32>("uav/action", 1000);
  segway_action_pub = n_.advertise<std_msgs::Int32>("segway/action_desired", 1000);
  flipper_action_pub = n_.advertise<std_msgs::Int32>("flipper/action", 1000);

  ROS_INFO("action_commander is ready");

  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}
