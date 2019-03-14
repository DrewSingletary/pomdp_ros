#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

#include <tf/transform_broadcaster.h>
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Joy.h"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <stdlib.h>
#include <math.h> 

typedef Eigen::VectorXd state_type;

double timeScale = 1;
double resolution = 1;

const int uav_nx = 12;
const int uav_nu = 3;
const int segway_nx = 7;
const int segway_nu = 2;
const int flipper_nx = 5;
const int flipper_nu = 2;

bool uav_action_finished = true;
bool segway_action_finished = true;
bool flipper_action_finished = true;

state_type uav_states = Eigen::VectorXd::Zero(uav_nx);
state_type segway_states = Eigen::VectorXd::Zero(segway_nx);
state_type flipper_states = Eigen::VectorXd::Zero(flipper_nx);

nav_msgs::OccupancyGrid uav_loc;
nav_msgs::OccupancyGrid segway_loc;
nav_msgs::OccupancyGrid flipper_loc;

nav_msgs::OccupancyGrid grid_sam;
nav_msgs::OccupancyGrid grid_hab;

ros::Publisher uav_grid_pub;
ros::Publisher segway_grid_pub;
ros::Publisher flipper_grid_pub;
ros::Publisher grid_hab_pub;
ros::Publisher grid_sam_pub;


void joy_cb(const sensor_msgs::Joy & msg) {

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

void uav_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (uav_action_finished == true && msg->data == false) {
    uav_action_finished = false;
  }
  if (uav_action_finished == false && msg->data == true) {
    ros::Time begin = ros::Time::now();
    uav_loc.header.stamp = begin;
    uav_loc.header.frame_id = "world";

    uav_loc.info.resolution = resolution;
    uav_loc.info.width = (uint32_t) 9/resolution;
    uav_loc.info.height = (uint32_t) 15/resolution;
    uint32_t width = uav_loc.info.width;
    uint32_t height = uav_loc.info.height;
    uav_loc.info.origin.position.x = -5;
    uav_loc.info.origin.position.y = -10;
    uav_loc.info.origin.position.z = .1;

    uav_loc.data.resize(uav_loc.info.width*uav_loc.info.height,-1);
    int8_t data[uav_loc.info.width*uav_loc.info.height] = {0};
    uint32_t gridsX = (uav_states[0]-uav_loc.info.origin.position.x)/resolution;
    uint32_t gridsY = (uav_states[1]-uav_loc.info.origin.position.y)/resolution;
    for (int i = 0; i < uav_loc.info.width*uav_loc.info.height; i++) {
      uav_loc.data[i] = 0;
    }
    uint32_t count = 0;
    ROS_INFO("%i %i %i %i", gridsX, gridsY, width, height);
    if ((int) gridsX-1 > -1) {
       uav_loc.data[(gridsX-1)+gridsY*uav_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsX-1 > -1 && (int) gridsY-1 > -1) {
       uav_loc.data[(gridsX-1)+(gridsY-1)*uav_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsY-1 > -1) {
       uav_loc.data[(gridsX)+(gridsY-1)*uav_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height) {
       uav_loc.data[(gridsX)+(gridsY+1)*uav_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height && gridsX+1 < width) {
       uav_loc.data[(gridsX+1)+(gridsY+1)*uav_loc.info.width] = 2;
       count++;
    }
    if (gridsX+1 < width) {
       uav_loc.data[(gridsX+1)+(gridsY)*uav_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height && (int) gridsX-1 > -1) {
       uav_loc.data[(gridsX-1)+(gridsY+1)*uav_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsY-1 > -1 && gridsX+1 < width) {
       uav_loc.data[(gridsX+1)+(gridsY-1)*uav_loc.info.width] = 2;
       count++;
    }

    uav_loc.data[gridsX+gridsY*uav_loc.info.width] = 100-2*count;
    uav_grid_pub.publish(uav_loc);
    uav_action_finished = true;
  }
}

void segway_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (segway_action_finished == true && msg->data == false) {
    segway_action_finished = false;
  }
  if (segway_action_finished == false && msg->data == true) {
    ros::Time begin = ros::Time::now();
    segway_loc.header.stamp = begin;
    segway_loc.header.frame_id = "world";

    segway_loc.info.resolution = resolution;
    segway_loc.info.width = (uint32_t) 9/resolution;
    segway_loc.info.height = (uint32_t) 15/resolution;
    uint32_t width = segway_loc.info.width;
    uint32_t height = segway_loc.info.height;
    segway_loc.info.origin.position.x = -5;
    segway_loc.info.origin.position.y = -10;
    segway_loc.info.origin.position.z = .1;

    segway_loc.data.resize(segway_loc.info.width*segway_loc.info.height,-1);
    int8_t data[segway_loc.info.width*segway_loc.info.height] = {0};
    uint32_t gridsX = (segway_states[0]-segway_loc.info.origin.position.x)/resolution;
    uint32_t gridsY = (segway_states[1]-segway_loc.info.origin.position.y)/resolution;
    for (int i = 0; i < segway_loc.info.width*segway_loc.info.height; i++) {
      segway_loc.data[i] = 0;
    }
    uint32_t count = 0;
    ROS_INFO("%i %i %i %i", gridsX, gridsY, width, height);
    if ((int) gridsX-1 > -1) {
       segway_loc.data[(gridsX-1)+gridsY*segway_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsX-1 > -1 && (int) gridsY-1 > -1) {
       segway_loc.data[(gridsX-1)+(gridsY-1)*segway_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsY-1 > -1) {
       segway_loc.data[(gridsX)+(gridsY-1)*segway_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height) {
       segway_loc.data[(gridsX)+(gridsY+1)*segway_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height && gridsX+1 < width) {
       segway_loc.data[(gridsX+1)+(gridsY+1)*segway_loc.info.width] = 2;
       count++;
    }
    if (gridsX+1 < width) {
       segway_loc.data[(gridsX+1)+(gridsY)*segway_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height && (int) gridsX-1 > -1) {
       segway_loc.data[(gridsX-1)+(gridsY+1)*segway_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsY-1 > -1 && gridsX+1 < width) {
       segway_loc.data[(gridsX+1)+(gridsY-1)*segway_loc.info.width] = 2;
       count++;
    }

    segway_loc.data[gridsX+gridsY*segway_loc.info.width] = 100-2*count;
    segway_grid_pub.publish(segway_loc);
    segway_action_finished = true;
  }
}

void flipper_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (flipper_action_finished == true && msg->data == false) {
    flipper_action_finished = false;
  }
  if (flipper_action_finished == false && msg->data == true) {
    ros::Time begin = ros::Time::now();
    flipper_loc.header.stamp = begin;
    flipper_loc.header.frame_id = "world";

    flipper_loc.info.resolution = resolution;
    flipper_loc.info.width = (uint32_t) 9/resolution;
    flipper_loc.info.height = (uint32_t) 15/resolution;
    uint32_t width = flipper_loc.info.width;
    uint32_t height = flipper_loc.info.height;
    flipper_loc.info.origin.position.x = -5;
    flipper_loc.info.origin.position.y = -10;
    flipper_loc.info.origin.position.z = .1;

    flipper_loc.data.resize(flipper_loc.info.width*flipper_loc.info.height,-1);
    int8_t data[flipper_loc.info.width*flipper_loc.info.height] = {0};
    uint32_t gridsX = (flipper_states[0]-flipper_loc.info.origin.position.x)/resolution;
    uint32_t gridsY = (flipper_states[1]-flipper_loc.info.origin.position.y)/resolution;
    for (int i = 0; i < flipper_loc.info.width*flipper_loc.info.height; i++) {
      flipper_loc.data[i] = 0;
    }
    uint32_t count = 0;
    ROS_INFO("%i %i %i %i", gridsX, gridsY, width, height);
    if ((int) gridsX-1 > -1) {
       flipper_loc.data[(gridsX-1)+gridsY*flipper_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsX-1 > -1 && (int) gridsY-1 > -1) {
       flipper_loc.data[(gridsX-1)+(gridsY-1)*flipper_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsY-1 > -1) {
       flipper_loc.data[(gridsX)+(gridsY-1)*flipper_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height) {
       flipper_loc.data[(gridsX)+(gridsY+1)*flipper_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height && gridsX+1 < width) {
       flipper_loc.data[(gridsX+1)+(gridsY+1)*flipper_loc.info.width] = 2;
       count++;
    }
    if (gridsX+1 < width) {
       flipper_loc.data[(gridsX+1)+(gridsY)*flipper_loc.info.width] = 2;
       count++;
    }
    if (gridsY+1 < height && (int) gridsX-1 > -1) {
       flipper_loc.data[(gridsX-1)+(gridsY+1)*flipper_loc.info.width] = 2;
       count++;
    }
    if ((int) gridsY-1 > -1 && gridsX+1 < width) {
       flipper_loc.data[(gridsX+1)+(gridsY-1)*flipper_loc.info.width] = 2;
       count++;
    }

    flipper_loc.data[gridsX+gridsY*flipper_loc.info.width] = 100-2*count;
    flipper_grid_pub.publish(flipper_loc);
    flipper_action_finished = true;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "grid_states");

  ros::NodeHandle n;

  ros::param::get("~_timeScale", timeScale);

  ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_cb);

  ros::Subscriber uav_states_sub = n.subscribe<std_msgs::Float32MultiArray>("uav/states", 1, uav_states_cb);
  ros::Subscriber segway_states_sub = n.subscribe<std_msgs::Float32MultiArray>("segway/states", 1, segway_states_cb);
  ros::Subscriber flipper_states_sub = n.subscribe<std_msgs::Float32MultiArray>("flipper/states", 1,flipper_states_cb);

  ros::Subscriber uav_complete_sub = n.subscribe<std_msgs::Bool>("uav/complete", 1, uav_complete_cb);
  ros::Subscriber segway_complete_sub = n.subscribe<std_msgs::Bool>("segway/complete", 1, segway_complete_cb);
  ros::Subscriber flipper_complete_sub = n.subscribe<std_msgs::Bool>("flipper/complete", 1,flipper_complete_cb);

  uav_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("uav/belief", 1);
  segway_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("segway/belief", 1);
  flipper_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("flipper/belief", 1);
  grid_hab_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/habitable", 1);
  grid_sam_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/sample", 1);

  ros::Rate loop_rate(200*timeScale);

  while (ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}