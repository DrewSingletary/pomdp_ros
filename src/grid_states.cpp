#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

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

const int uav_nx = 12;
const int uav_nu = 3;
const int segway_nx = 7;
const int segway_nu = 2;
const int flipper_nx = 5;
const int flipper_nu = 2;

state_type uav_states = Eigen::VectorXd::Zero(uav_nx);
state_type segway_states = Eigen::VectorXd::Zero(segway_nx);
state_type flipper_states = Eigen::VectorXd::Zero(flipper_nx);

state_type uav_inputs = Eigen::VectorXd::Zero(uav_nu);
state_type segway_inputs = Eigen::VectorXd::Zero(segway_nu);
state_type flipper_inputs = Eigen::VectorXd::Zero(flipper_nu);

nav_msgs::OccupancyGrid uav_loc;
nav_msgs::OccupancyGrid segway_loc;
nav_msgs::OccupancyGrid flipper_loc;

nav_msgs::OccupancyGrid grid_sam;
nav_msgs::OccupancyGrid grid_hab;

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

void uav_des_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  
}

void segway_des_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  
}

void flipper_des_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {

}

void uav_controller(void) {
  uav_inputs[2] = .1;
}

void segway_controller(void) {
  segway_inputs[0] = .2;
  segway_inputs[1] = .5;
}

void flipper_controller(void) {

}


int main(int argc, char **argv) {

  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::param::get("~_timeScale", timeScale);

  ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_cb);

  ros::Subscriber uav_states_sub = n.subscribe<std_msgs::Float32MultiArray>("uav/states", 1, uav_states_cb);
  ros::Subscriber segway_states_sub = n.subscribe<std_msgs::Float32MultiArray>("segway/states", 1, segway_states_cb);
  ros::Subscriber flipper_states_sub = n.subscribe<std_msgs::Float32MultiArray>("flipper/states", 1,flipper_states_cb);

  ros::Publisher uav_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("uav/belief", 1);
  ros::Publisher segway_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("segway/belief", 1);
  ros::Publisher flipper_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("flipper/belief", 1);

  ros::Publisher grid_hab_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/habitable", 1);
  ros::Publisher grid_sam_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/sample", 1);

  std_msgs::Float32MultiArray uav_msg;
  uav_msg.data.clear();
  uav_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  uav_msg.layout.dim[0].size = uav_nu;
  for (int i = 0; i < uav_nu; i++) {
    uav_msg.data.push_back(0.0);
  }

  std_msgs::Float32MultiArray segway_msg;
  segway_msg.data.clear();
  segway_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  segway_msg.layout.dim[0].size = segway_nu;
  for (int i = 0; i < segway_nu; i++) {
    segway_msg.data.push_back(0.0);
  }

  std_msgs::Float32MultiArray flipper_msg;
  flipper_msg.data.clear();
  flipper_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  flipper_msg.layout.dim[0].size = flipper_nu;
  for (int i = 0; i < flipper_nu; i++) {
    flipper_msg.data.push_back(0.0);
  }

  ros::Rate loop_rate(200*timeScale);

  while (ros::ok()) {
    
    // uav_controller();
    // for (int i = 0;i<uav_nu;i++) {
    //   uav_msg.data[i] = uav_inputs[i];
    // }
    // uav_grid_pub.publish(uav_msg);

    // segway_controller();
    // for (int i = 0;i<segway_nu;i++) {
    //   segway_msg.data[i] = segway_inputs[i];
    // }
    // segway_grid_pub.publish(segway_msg);

    // flipper_controller();
    // for (int i = 0;i<flipper_nu;i++) {
    //   flipper_msg.data[i] = flipper_inputs[i];
    // }
    // flipper_grid_pub.publish(flipper_msg);



    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}