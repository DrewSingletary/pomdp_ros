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

// double obstacle_resolution = 3;

int width = (int) castWidth/resolution; 
int height = (int) castHeight/resolution;

// int obsWidth = (int) castWidth/obstacle_resolution; 
// int obsHeight = (int) castHeight/obstacle_resolution;
int totalGrids = width*height;

state_type obs_belief = Eigen::VectorXd::Ones(width*height);
state_type obs_belief_new = Eigen::VectorXd::Ones(width*height);
state_type true_obs_belief = Eigen::VectorXd::Zero(width*height);

state_type sam_belief = Eigen::VectorXd::Ones(width*height);
state_type sam_belief_new = Eigen::VectorXd::Ones(width*height);
state_type true_sam_belief = Eigen::VectorXd::Zero(width*height);

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
int segway_action_real = 0;
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
ros::Publisher segway_action_pub;

double segway_filter(void) {
  uint32_t gridsX_old = (segway_states_old[0]-segway_loc.info.origin.position.x)/resolution;
  uint32_t gridsY_old = (segway_states_old[1]-segway_loc.info.origin.position.y)/resolution;
  //observations
  uint32_t gridsX = (segway_states[0]-segway_loc.info.origin.position.x)/resolution;
  uint32_t gridsY = (segway_states[1]-segway_loc.info.origin.position.y)/resolution;
  int gridDesX = 0;
  int gridDesY = 0;
  for (int i = 0; i < totalGrids; i ++) {
    int iGridX = i % width;
    int iGridY = (int) i/width;
    uint32_t count = 0;

    if (segway_action == 0 ) {
      segway_action_real = 0;
      return 0.0F;
    }
    if (segway_action == 1 ) {
      gridDesX = iGridX+1;
      gridDesY = iGridY;
    }
    if (segway_action == 2 ) {
      gridDesX = iGridX;
      gridDesY = iGridY+1;
    }
    if (segway_action == 3 ) {
      gridDesX = iGridX-1;
      gridDesY = iGridY;
    }
    if (segway_action == 4 ) {
      gridDesX = iGridX;
      gridDesY = iGridY-1;
    }
    if ((int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
      segway_belief_new[(gridDesX-1)+gridDesY*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
      count++;
    }
    if ((int) gridDesX-1 > -1 && (int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
      segway_belief_new[(gridDesX-1)+(gridDesY-1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
      count++;
    }
    if ((int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
      segway_belief_new[(gridDesX)+(gridDesY-1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
      count++;
    }
    if (gridDesY+1 < height && gridDesX > -1 && gridDesY > -1) {
      segway_belief_new[(gridDesX)+(gridDesY+1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
      count++;
    }
    if (gridDesY+1 < height && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
      segway_belief_new[(gridDesX+1)+(gridDesY+1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
      count++;
    }
    if (gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
      segway_belief_new[(gridDesX+1)+(gridDesY)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
      count++;
    }
    if (gridDesY+1 < height && (int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
      segway_belief_new[(gridDesX-1)+(gridDesY+1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
      count++;
    }
    if ((int) gridDesY-1 > -1 && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
      segway_belief_new[(gridDesX+1)+(gridDesY-1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
      count++;
    }
    double test = (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*(1-.01*count);
    double test2 = (double) segway_belief[i];
    if (gridDesX > -1 && gridDesY > -1) {
    segway_belief_new[gridDesX+gridDesY*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*(1-.01*count);
    } else {
      segway_belief_new[iGridX+iGridY*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*(1-.01*count);
    }
  }

  for (int i = 0; i < segway_loc.info.width*segway_loc.info.height; i++) {
    int data = 100*segway_belief_new[i];
    segway_loc.data[i] = data;
    segway_belief[i] =  segway_belief_new[i];
  }
  segway_grid_pub.publish(segway_loc);
  segway_action_finished = true;
  double maxVal = 0;
  double maxAllowed = 0.05;
  for (int i = 0; i < totalGrids; i++) {
    double val = segway_belief[i] * obs_belief[i];
    maxVal += val;
  }
  return maxVal
}


void uav_reward(void) {
  double uav_r[totalGrids];
  double gridposx[totalGrids];
  double gridposy[totalGrids];
  for (int i = 0; i < totalGrids; i++) {
    uav_r[i] = -(sam_belief[i]-.5)*(sam_belief[i]-.5)+.25;
    int iGridX = i % width;
    int iGridY = (int) i/width;
    gridposx[i] = uav_loc.info.origin.position.x+iGridX*resolution;
    gridposy[i] = uav_loc.info.origin.position.y+iGridY*resolution;
  }
}

void flipper_reward(void) {
  double flipper_r[totalGrids];
  for (int i = 0; i < totalGrids; i++) {
    flipper_r[i] = -(sam_belief[i]-.5)*(sam_belief[i]-.5)+.25;
  }
}

void segway_reward(void) {

}

void grid_hab_update(void) {
  int gridsX = (flipper_states[0]-flipper_loc.info.origin.position.x)/resolution;
  int gridsY = (flipper_states[1]-flipper_loc.info.origin.position.y)/resolution;
  for (int i = 0; i < totalGrids; i++) {
    int iGridX = i % width;
    int iGridY = (int) i/width;
    if (abs(iGridX-gridsX) < 2 && abs(iGridY-gridsY) < 2) {
       obs_belief_new[i] = flipper_belief[gridsX+gridsY*width]*true_obs_belief[i];
       obs_belief_new[i] += (1-flipper_belief[gridsX+gridsY*width])*obs_belief[i];
    } else {
      obs_belief_new[i] = flipper_belief[i]*true_obs_belief[i];
      obs_belief_new[i] += (1-flipper_belief[i])*obs_belief[i];
    }
  }
  for (int i = 0; i < grid_hab.info.width*grid_hab.info.height; i++) {
    int data = 100*obs_belief_new[i];
    grid_hab.data[i] = data;
    obs_belief[i] =  obs_belief_new[i];
  }
  ros::Time begin = ros::Time::now();
  grid_hab.header.stamp = begin;
  grid_hab_pub.publish(grid_hab);
}

void grid_sam_update(void) {
  int gridsX = (uav_states[0]-uav_loc.info.origin.position.x)/resolution;
  int gridsY = (uav_states[1]-uav_loc.info.origin.position.y)/resolution;
  for (int i = 0; i < totalGrids; i++) {
    int iGridX = i % width;
    int iGridY = (int) i/width;
    if (abs(iGridX-gridsX) < 2 && abs(iGridY-gridsY) < 2) {
       sam_belief_new[i] = uav_belief[gridsX+gridsY*width]*true_sam_belief[i];
       sam_belief_new[i] += (1-uav_belief[gridsX+gridsY*width])*sam_belief[i];
    } else {
      sam_belief_new[i] = uav_belief[i]*true_sam_belief[i];
      sam_belief_new[i] += (1-uav_belief[i])*sam_belief[i];
    }
  }
  for (int i = 0; i < grid_sam.info.width*grid_sam.info.height; i++) {
    int data = 100*sam_belief_new[i];
    grid_sam.data[i] = data;
    sam_belief[i] =  sam_belief_new[i];
  }
  ros::Time begin = ros::Time::now();
  grid_sam.header.stamp = begin;
  grid_sam_pub.publish(grid_sam);
}

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
  segway_action_real = 0;
  segway_action = msg->data;
  double prob_failure = segway_filter();
  if (prob_failure < 0.05) {
    segway_action_real = segway_action;
  }
  if (prob_failure > 0.05 && segway_action == 2) {
    segway_action = 3;
    prob_failure = segway_filter();
    if (prob_failure > 0.05) {
      segway_action_real = 0;
    }
  }
  if (prob_failure > 0.05 && segway_action == 3) {
    segway_action = 2;
    prob_failure = segway_filter();
    if (prob_failure > 0.05) {
      segway_action_real = 0;
    }
  }
  std_msgs::Int32 segway_action_message;
  segway_action_message.data = segway_action_real;
  segway_action_pub.publish(segway_action_message);
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
          uav_belief_new[(gridDesX-1)+gridDesY*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesX-1 > -1 && (int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX-1)+(gridDesY-1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX)+(gridDesY-1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX)+(gridDesY+1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX+1)+(gridDesY+1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.01;
          count++;
        }
        if (gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX+1)+(gridDesY)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && (int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX-1)+(gridDesY+1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesY-1 > -1 && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          uav_belief_new[(gridDesX+1)+(gridDesY-1)*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*.01;
          count++;
        }
        double test = (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*(1-.01*count);
        double test2 = (double) uav_belief[i];
        if (gridDesX > -1 && gridDesY > -1) {
        uav_belief_new[gridDesX+gridDesY*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*(1-.01*count);
        } else {
          uav_belief_new[iGridX+iGridY*uav_loc.info.width] += (double) uav_belief[iGridX+iGridY*uav_loc.info.width]*(1-.01*count);
        }
      }

      for (int i = 0; i < uav_loc.info.width*uav_loc.info.height; i++) {
        int data = 100*uav_belief_new[i];
        uav_loc.data[i] = data;
        uav_belief[i] =  uav_belief_new[i];
      }
      uav_grid_pub.publish(uav_loc);
      uav_action_finished = true;
      grid_hab_update();
      grid_sam_update();
      ROS_INFO("sum: %f",uav_belief.sum());
    }
  }
}

void segway_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (segway_first_action) {
    uint32_t gridsX = (segway_states[0]-segway_loc.info.origin.position.x)/resolution;
    uint32_t gridsY = (segway_states[1]-segway_loc.info.origin.position.y)/resolution;
    segway_belief_new[gridsX+gridsY*segway_loc.info.width]= 1;
    segway_first_action = false;
    for (int i = 0; i < segway_loc.info.width*segway_loc.info.height; i++) {
      segway_loc.data[i] = (int) segway_belief_new[i]*100;
      segway_belief[i] = (int) segway_belief_new[i];
    }
    segway_grid_pub.publish(segway_loc);
  } else {
    if (segway_action_finished == true && msg->data == false) {
      segway_action_finished = false;
    }
    if (segway_action_finished == false && msg->data == true) {
      segway_belief_new.setZero();
      ros::Time begin = ros::Time::now();
      segway_loc.header.stamp = begin;
      // prior states
      uint32_t gridsX_old = (segway_states_old[0]-segway_loc.info.origin.position.x)/resolution;
      uint32_t gridsY_old = (segway_states_old[1]-segway_loc.info.origin.position.y)/resolution;
      //observations
      uint32_t gridsX = (segway_states[0]-segway_loc.info.origin.position.x)/resolution;
      uint32_t gridsY = (segway_states[1]-segway_loc.info.origin.position.y)/resolution;
      int gridDesX = 0;
      int gridDesY = 0;
      for (int i = 0; i < totalGrids; i ++) {
        int iGridX = i % width;
        int iGridY = (int) i/width;
        uint32_t count = 0;

        if (segway_action == 0 ) {
          if (iGridX == gridsX_old && iGridY == gridsY_old) {
            segway_belief_new[i] = segway_belief[i];
            continue;
          }
        }
        if (segway_action == 1 ) {
          gridDesX = iGridX+1;
          gridDesY = iGridY;
        }
        if (segway_action == 2 ) {
          gridDesX = iGridX;
          gridDesY = iGridY+1;
        }
        if (segway_action == 3 ) {
          gridDesX = iGridX-1;
          gridDesY = iGridY;
        }
        if (segway_action == 4 ) {
          gridDesX = iGridX;
          gridDesY = iGridY-1;
        }
        if ((int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          segway_belief_new[(gridDesX-1)+gridDesY*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesX-1 > -1 && (int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          segway_belief_new[(gridDesX-1)+(gridDesY-1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          segway_belief_new[(gridDesX)+(gridDesY-1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && gridDesX > -1 && gridDesY > -1) {
          segway_belief_new[(gridDesX)+(gridDesY+1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          segway_belief_new[(gridDesX+1)+(gridDesY+1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
          count++;
        }
        if (gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          segway_belief_new[(gridDesX+1)+(gridDesY)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && (int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          segway_belief_new[(gridDesX-1)+(gridDesY+1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesY-1 > -1 && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          segway_belief_new[(gridDesX+1)+(gridDesY-1)*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*.01;
          count++;
        }
        double test = (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*(1-.01*count);
        double test2 = (double) segway_belief[i];
        if (gridDesX > -1 && gridDesY > -1) {
        segway_belief_new[gridDesX+gridDesY*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*(1-.01*count);
        } else {
          segway_belief_new[iGridX+iGridY*segway_loc.info.width] += (double) segway_belief[iGridX+iGridY*segway_loc.info.width]*(1-.01*count);
        }
      }

      for (int i = 0; i < segway_loc.info.width*segway_loc.info.height; i++) {
        int data = 100*segway_belief_new[i];
        segway_loc.data[i] = data;
        segway_belief[i] =  segway_belief_new[i];
      }
      segway_grid_pub.publish(segway_loc);
      segway_action_finished = true;
      double maxVal = 0;
      double maxAllowed = 0.05;
      for (int i = 0; i < totalGrids; i++) {
        double val = segway_belief[i] * obs_belief[i];
        // if (val > maxVal) {
          maxVal += val;
        // }
      }
      if (maxVal > maxAllowed) {
        ROS_INFO("SAFETY VIOLATION: %f",maxVal);

      }
    }
  }
}

void flipper_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (flipper_first_action) {
    uint32_t gridsX = (flipper_states[0]-flipper_loc.info.origin.position.x)/resolution;
    uint32_t gridsY = (flipper_states[1]-flipper_loc.info.origin.position.y)/resolution;
    flipper_belief_new[gridsX+gridsY*flipper_loc.info.width]= 1;
    flipper_first_action = false;
    for (int i = 0; i < flipper_loc.info.width*flipper_loc.info.height; i++) {
      flipper_loc.data[i] = (int) flipper_belief_new[i]*100;
      flipper_belief[i] = (int) flipper_belief_new[i];
    }
    flipper_grid_pub.publish(flipper_loc);
  } else {
    if (flipper_action_finished == true && msg->data == false) {
      flipper_action_finished = false;
    }
    if (flipper_action_finished == false && msg->data == true) {
      flipper_belief_new.setZero();
      ros::Time begin = ros::Time::now();
      flipper_loc.header.stamp = begin;
      // prior states
      uint32_t gridsX_old = (flipper_states_old[0]-flipper_loc.info.origin.position.x)/resolution;
      uint32_t gridsY_old = (flipper_states_old[1]-flipper_loc.info.origin.position.y)/resolution;
      //observations
      uint32_t gridsX = (flipper_states[0]-flipper_loc.info.origin.position.x)/resolution;
      uint32_t gridsY = (flipper_states[1]-flipper_loc.info.origin.position.y)/resolution;
      int gridDesX = 0;
      int gridDesY = 0;
      for (int i = 0; i < totalGrids; i ++) {
        int iGridX = i % width;
        int iGridY = (int) i/width;
        uint32_t count = 0;

        if (flipper_action == 0 ) {
          if (iGridX == gridsX_old && iGridY == gridsY_old) {
            flipper_belief_new[i] = flipper_belief[i];
            continue;
          }
        }
        if (flipper_action == 1 ) {
          gridDesX = iGridX+1;
          gridDesY = iGridY;
        }
        if (flipper_action == 2 ) {
          gridDesX = iGridX;
          gridDesY = iGridY+1;
        }
        if (flipper_action == 3 ) {
          gridDesX = iGridX-1;
          gridDesY = iGridY;
        }
        if (flipper_action == 4 ) {
          gridDesX = iGridX;
          gridDesY = iGridY-1;
        }
        if ((int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          flipper_belief_new[(gridDesX-1)+gridDesY*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesX-1 > -1 && (int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          flipper_belief_new[(gridDesX-1)+(gridDesY-1)*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesY-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          flipper_belief_new[(gridDesX)+(gridDesY-1)*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && gridDesX > -1 && gridDesY > -1) {
          flipper_belief_new[(gridDesX)+(gridDesY+1)*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          flipper_belief_new[(gridDesX+1)+(gridDesY+1)*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*.01;
          count++;
        }
        if (gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          flipper_belief_new[(gridDesX+1)+(gridDesY)*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*.01;
          count++;
        }
        if (gridDesY+1 < height && (int) gridDesX-1 > -1 && gridDesX > -1 && gridDesY > -1) {
          flipper_belief_new[(gridDesX-1)+(gridDesY+1)*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*.01;
          count++;
        }
        if ((int) gridDesY-1 > -1 && gridDesX+1 < width && gridDesX > -1 && gridDesY > -1) {
          flipper_belief_new[(gridDesX+1)+(gridDesY-1)*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*.01;
          count++;
        }
        double test = (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*(1-.01*count);
        double test2 = (double) flipper_belief[i];
        if (gridDesX > -1 && gridDesY > -1) {
        flipper_belief_new[gridDesX+gridDesY*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*(1-.01*count);
        } else {
          flipper_belief_new[iGridX+iGridY*flipper_loc.info.width] += (double) flipper_belief[iGridX+iGridY*flipper_loc.info.width]*(1-.01*count);
        }
      }

      for (int i = 0; i < flipper_loc.info.width*flipper_loc.info.height; i++) {
        int data = 100*flipper_belief_new[i];
        flipper_loc.data[i] = data;
        flipper_belief[i] =  flipper_belief_new[i];
      }
      flipper_grid_pub.publish(flipper_loc);
      flipper_action_finished = true;
      grid_hab_update();
      grid_sam_update();
      ROS_INFO("sum: %f",flipper_belief.sum());
    }
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
  ros::Subscriber segway_actions_sub = n.subscribe<std_msgs::Int32>("segway/action_desired", 1, segway_actions_cb);
  ros::Subscriber flipper_actions_sub = n.subscribe<std_msgs::Int32>("flipper/action", 1,flipper_actions_cb);

  ros::Subscriber uav_complete_sub = n.subscribe<std_msgs::Bool>("uav/complete", 1, uav_complete_cb);
  ros::Subscriber segway_complete_sub = n.subscribe<std_msgs::Bool>("segway/complete", 1, segway_complete_cb);
  ros::Subscriber flipper_complete_sub = n.subscribe<std_msgs::Bool>("flipper/complete", 1,flipper_complete_cb);

  uav_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("uav/belief", 1);
  segway_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("segway/belief", 1);
  flipper_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("flipper/belief", 1);
  grid_hab_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/habitable", 1);
  grid_sam_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/sample", 1);
  
  segway_action_pub = n.advertise<std_msgs::Int32>("segway/action_actual", 1000);

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

  grid_hab.header.frame_id = "world";
  grid_hab.info.resolution = resolution;
  grid_hab.info.width = (uint32_t) castWidth/resolution;
  grid_hab.info.height = (uint32_t) castHeight/resolution;
  grid_hab.info.origin.position.x = -5;
  grid_hab.info.origin.position.y = -10;
  grid_hab.info.origin.position.z = .1;
  grid_hab.data.resize(grid_hab.info.width*grid_hab.info.height,-1);

  grid_sam.header.frame_id = "world";
  grid_sam.info.resolution = resolution;
  grid_sam.info.width = (uint32_t) castWidth/resolution;
  grid_sam.info.height = (uint32_t) castHeight/resolution;
  grid_sam.info.origin.position.x = -5;
  grid_sam.info.origin.position.y = -10;
  grid_sam.info.origin.position.z = .1;
  grid_sam.data.resize(grid_sam.info.width*grid_sam.info.height,-1);

  // Define grids with obstacles
  true_obs_belief(9*4+5) = 1;
  true_obs_belief(9*5+2) = 1;
  true_obs_belief(9*6+4) = 1;
  true_sam_belief(9*7+0) = 1;
  obs_belief = obs_belief*.5;
  sam_belief = sam_belief*.5;

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}