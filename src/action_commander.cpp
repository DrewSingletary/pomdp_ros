#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Joy.h"

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

const int uav_nx = 12;
const int uav_nu = 3;
const int segway_nx = 7;
const int segway_nu = 2;
const int flipper_nx = 5;
const int flipper_nu = 2;

state_type uav_states = Eigen::VectorXd::Zero(uav_nx);
state_type segway_states = Eigen::VectorXd::Zero(segway_nx);
state_type flipper_states = Eigen::VectorXd::Zero(flipper_nx);

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

void uav_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < uav_nx; ++i)
    uav_states[i] = msg->data[i];
}

void segway_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < segway_nx; ++i)
    segway_states[i] = msg->data[i];
}

void flipper_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < flipper_nx; ++i)
    flipper_states[i] = msg->data[i];
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "action_commander");

  ros::NodeHandle n_;

  ros::Subscriber uav_states_sub = n_.subscribe<std_msgs::Float32MultiArray>("uav/states", 1, uav_states_cb);
  ros::Subscriber segway_states_sub = n_.subscribe<std_msgs::Float32MultiArray>("segway/states", 1, segway_states_cb);
  ros::Subscriber flipper_states_sub = n_.subscribe<std_msgs::Float32MultiArray>("flipper/states", 1,flipper_states_cb);

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