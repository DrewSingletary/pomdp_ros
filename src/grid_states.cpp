#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

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

const int castWidth = 9;
const int castHeight = 15;

int width = (int) castWidth/resolution; 
int height = (int) castHeight/resolution;
int totalGrids = width*height;

state_type uav_belief = Eigen::VectorXd::Zero(width*height);
state_type segway_belief = Eigen::VectorXd::Zero(width*height);
state_type flipper_belief = Eigen::VectorXd::Zero(width*height);

state_type uav_belief_new = Eigen::VectorXd::Zero(width*height);
state_type segway_belief_new = Eigen::VectorXd::Zero(width*height);
state_type flipper_belief_new = Eigen::VectorXd::Zero(width*height);

bool uav_action_finished = true;
bool segway_action_finished = true;
bool flipper_action_finished = true;

int uav_action = 0;
int segway_action = 0;
int flipper_action = 0;

bool uav_first_action = true;
bool segway_first_action = true;
bool flipper_first_action = true;

state_type uav_states_old = Eigen::VectorXd::Zero(uav_nx);
state_type segway_states_old = Eigen::VectorXd::Zero(segway_nx);
state_type flipper_states_old = Eigen::VectorXd::Zero(flipper_nx);

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
  for (int i = 0; i < uav_nx; ++i) {
    uav_states_old[i] = uav_states[i];
    uav_states[i] = msg->data[i];
  }
}

void segway_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < segway_nx; ++i) {
    segway_states_old[i] = segway_states[i];
    segway_states[i] = msg->data[i];
  }
}

void flipper_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < flipper_nx; ++i) {
    flipper_states_old[i] = flipper_states[i];
    flipper_states[i] = msg->data[i];
  }
}

void uav_actions_cb(const std_msgs::Int32::ConstPtr& msg) {
    uav_action = msg->data;
}

void segway_actions_cb(const std_msgs::Int32::ConstPtr& msg) {
    segway_action = msg->data;
}

void flipper_actions_cb(const std_msgs::Int32::ConstPtr& msg) {
    flipper_action = msg->data;
}

void uav_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (uav_first_action) {
    uint32_t gridsX = (uav_states[0]-uav_loc.info.origin.position.x)/resolution;
    uint32_t gridsY = (uav_states[1]-uav_loc.info.origin.position.y)/resolution;
    uav_belief_new[gridsX+gridsY*uav_loc.info.width]= 1;
    uav_first_action = false;
    for (int i = 0; i < uav_loc.info.width*uav_loc.info.height; i++) {
      uav_loc.data[i] = (int) uav_belief_new[i]*100;
      uav_belief[i] = (int) uav_belief_new[i];
    }
    uav_grid_pub.publish(uav_loc);
  } else {
    if (uav_action_finished == true && msg->data == false) {
      uav_action_finished = false;
    }
    if (uav_action_finished == false && msg->data == true) {
      uav_belief_new.setZero();
      ros::Time begin = ros::Time::now();
      uav_loc.header.stamp = begin;
      // prior states
      uint32_t gridsX_old = (uav_states_old[0]-uav_loc.info.origin.position.x)/resolution;
      uint32_t gridsY_old = (uav_states_old[1]-uav_loc.info.origin.position.y)/resolution;
      //observations
      uint32_t gridsX = (uav_states[0]-uav_loc.info.origin.position.x)/resolution;
      uint32_t gridsY = (uav_states[1]-uav_loc.info.origin.position.y)/resolution;
      int gridDesX = 0;
      int gridDesY = 0;
      for (int i = 0; i < totalGrids; i ++) {
        int iGridX = i % width;
        int iGridY = (int) i/width;
        uint32_t count = 0;

        if (uav_action == 0 ) {
          if (iGridX == gridsX_old && iGridY == gridsY_old) {
            uav_belief_new[i] = uav_belief[i];
            continue;
          }
        }
        if (uav_action == 1 ) {
          gridDesX = iGridX+1;
          gridDesY = iGridY;
        }
        if (uav_action == 2 ) {
          gridDesX = iGridX;
          gridDesY = iGridY+1;
        }
        if (uav_action == 3 ) {
          gridDesX = iGridX-1;
          gridDesY = iGridY;
        }
        if (uav_action == 4 ) {
          gridDesX = iGridX;
          gridDesY = iGridY-1;
        }
        if ((int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX-1)+gridDesY*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.02;
          count++;
        }
        if ((int) gridDesX-1 > -1 && (int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX-1)+(gridDesY-1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.02;
          count++;
        }
        if ((int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX)+(gridDesY-1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.02;
          count++;
        }
        if (gridDesY+1 < height && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX)+(gridDesY+1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.02;
          count++;
        }
        if (gridDesY+1 < height && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX+1)+(gridDesY+1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.02;
          count++;
        }
        if (gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX+1)+(gridDesY)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.02;
          count++;
        }
        if (gridDesY+1 < height && (int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX-1)+(gridDesY+1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.02;
          count++;
        }
        if ((int) gridDesY-1 > -1 && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX+1)+(gridDesY-1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.02;
          count++;
        }
        double test = (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*(1-.02*count);
        double test2 = (double) uav_belief[i];
        if (gridDesX > -1 && gridDesY > -1) {
        uav_belief_new[gridDesX+gridDesY*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*(1-.02*count);
        } else {
          uav_belief_new[iGridX+iGridY*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*(1-.02*count);
        }
      }

      for (int i = 0; i < uav_loc.info.width*uav_loc.info.height; i++) {
        int data = 100*uav_belief_new[i];
        uav_loc.data[i] = data;
        uav_belief[i] =  uav_belief_new[i];
      }
      uav_grid_pub.publish(uav_loc);
      uav_action_finished = true;
      ROS_INFO("sum: %f",uav_belief.sum());
    }
  }
}

void segway_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (segway_action_finished == true && msg->data == false) {
    segway_action_finished = false;
  }
  if (segway_action_finished == false && msg->data == true) {
    ros::Time begin = ros::Time::now();
    segway_loc.header.stamp = begin;
    
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

  ros::Subscriber uav_actions_sub = n.subscribe<std_msgs::Int32>("uav/action", 1, uav_actions_cb);
  ros::Subscriber segway_actions_sub = n.subscribe<std_msgs::Int32>("segway/action", 1, segway_actions_cb);
  ros::Subscriber flipper_actions_sub = n.subscribe<std_msgs::Int32>("flipper/action", 1,flipper_actions_cb);

  ros::Subscriber uav_complete_sub = n.subscribe<std_msgs::Bool>("uav/complete", 1, uav_complete_cb);
  ros::Subscriber segway_complete_sub = n.subscribe<std_msgs::Bool>("segway/complete", 1, segway_complete_cb);
  ros::Subscriber flipper_complete_sub = n.subscribe<std_msgs::Bool>("flipper/complete", 1,flipper_complete_cb);

  uav_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("uav/belief", 1);
  segway_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("segway/belief", 1);
  flipper_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("flipper/belief", 1);
  grid_hab_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/habitable", 1);
  grid_sam_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/sample", 1);

  ros::Rate loop_rate(200*timeScale);

  uav_loc.header.frame_id = "world";
  uav_loc.info.resolution = resolution;
  uav_loc.info.width = (uint32_t) castWidth/resolution;
  uav_loc.info.height = (uint32_t) castHeight/resolution;
  uav_loc.info.origin.position.x = -5;
  uav_loc.info.origin.position.y = -10;
  uav_loc.info.origin.position.z = .1;
  uav_loc.data.resize(uav_loc.info.width*uav_loc.info.height,-1);

  flipper_loc.header.frame_id = "world";
  flipper_loc.info.resolution = resolution;
  flipper_loc.info.width = (uint32_t) castWidth/resolution;
  flipper_loc.info.height = (uint32_t) castHeight/resolution;
  flipper_loc.info.origin.position.x = -5;
  flipper_loc.info.origin.position.y = -10;
  flipper_loc.info.origin.position.z = .1;
  flipper_loc.data.resize(flipper_loc.info.width*flipper_loc.info.height,-1);

  segway_loc.header.frame_id = "world";
  segway_loc.info.resolution = resolution;
  segway_loc.info.width = (uint32_t) castWidth/resolution;
  segway_loc.info.height = (uint32_t) castHeight/resolution;
  segway_loc.info.origin.position.x = -5;
  segway_loc.info.origin.position.y = -10;
  segway_loc.info.origin.position.z = .1;
  segway_loc.data.resize(segway_loc.info.width*segway_loc.info.height,-1);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}