#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include <tf/transform_broadcaster.h>
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Joy.h"
#include <Eigen/Eigen>
#include <stdlib.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

typedef Eigen::VectorXd state_type;

visualization_msgs::Marker marker;
visualization_msgs::Marker marker2;

double timeScale = 1;
double resolution = 1;

std_msgs::Float32MultiArray barrier_msg;

double max_failure_rate = .1;

bool readyToMove1 = false;
bool readyToMove2 = false;

const int uav_nx = 12;
const int uav_nu = 3;
const int segway_nx = 7;
const int segway_nu = 2;
const int flipper_nx = 5;
const int flipper_nu = 2;

const int castWidth = 9;
const int castHeight = 15;
const int xmin = 1;
const int xmax = 6;
const int ymin = 6;
const int ymax = 10;

double segway_delta = 0.005;
double flipper_delta = 0.002;
double uav_delta = 0.002;

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
state_type segway_belief_old = Eigen::VectorXd::Zero(width*height);
state_type flipper_belief = Eigen::VectorXd::Zero(width*height);

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
ros::Publisher barrier_pub;
ros::Publisher segway_grid_pub;
ros::Publisher flipper_grid_pub;
ros::Publisher grid_hab_pub;
ros::Publisher grid_sam_pub;
// ros::Publisher segway_action_pub;

ros::Publisher marker_pub;
ros::Publisher marker_pub2;
ros::Publisher uav_action_pub;
ros::Publisher segway_action_pub;
ros::Publisher flipper_action_pub;

double origin[2] = {-5,-10};

void update_agent_belief(int agent)
{
  double states[2];
  int action;
  state_type belief_new = Eigen::VectorXd::Zero((width)*(height));
  belief_new.setZero();
  state_type belief;
  double delta = 0;
  if (agent == 0) { // segway
    for (int i = 0; i < 2; i++) {
      states[i] = segway_states[i];
    }
    action = segway_action;
    belief = segway_belief;
    delta = segway_delta;
  }
  if (agent == 1) { // flipper
    for (int i = 0; i < 2; i++) {
      states[i] = flipper_states[i];
    }
    action = flipper_action;
    belief = flipper_belief;
    delta = flipper_delta;
  }
  if (agent == 2) { // uav
    for (int i = 0; i < 2; i++) {
      states[i] = uav_states[i];
    }
    action = uav_action;
    belief = uav_belief;
    delta = uav_delta;
  }
  if (action == 0) {
    return;
  }
  uint32_t gridsX = (states[0]-origin[0])/resolution;
  uint32_t gridsY = (states[1]-origin[1])/resolution;
  int gridDesX = 0;
  int gridDesY = 0;
  for (int i = 0; i < totalGrids; i ++) {
    int iGridX = i % width;
    int iGridY = (int) i/width;
    uint32_t count = 0;

    if (action == 1 ) {
      gridDesX = iGridX+1;
      gridDesY = iGridY;
    }
    if (action == 2 ) {
      gridDesX = iGridX;
      gridDesY = iGridY+1;
    }
    if (action == 3 ) {
      gridDesX = iGridX-1;
      gridDesY = iGridY;
    }
    if (action == 4 ) {
      gridDesX = iGridX;
      gridDesY = iGridY-1;
    }
    if ((gridDesX-1) > -1 && gridDesY > -1 && gridDesY < height && gridDesX < width) {
      belief_new[(gridDesX-1)+gridDesY*width] += (double) belief[iGridX+iGridY*width]*delta;
      count++;
    }
    if ((gridDesX-1) > -1 && (gridDesY-1) > -1 && (gridDesY-1) < height && (gridDesX-1)< width) {
      belief_new[(gridDesX-1)+(gridDesY-1)*width] += (double) belief[iGridX+iGridY*width]*delta;
      count++;
    }
    if ((gridDesY-1) > -1 && gridDesX > -1  && (gridDesY-1) < height && gridDesX < width) {
      belief_new[(gridDesX)+(gridDesY-1)*width] += (double) belief[iGridX+iGridY*width]*delta;
      count++;
    }
    if (gridDesX > -1 && (gridDesY+1) > -1 && (gridDesY+1) < height && gridDesX < width) {
      belief_new[(gridDesX)+(gridDesY+1)*width] += (double) belief[iGridX+iGridY*width]*delta;
      count++;
    }
    if ((gridDesX+1) > -1 && (gridDesY+1) > -1 && (gridDesY+1) < height && gridDesX+1 < width) {
      belief_new[(gridDesX+1)+(gridDesY+1)*width] += (double) belief[iGridX+iGridY*width]*delta;
      count++;
    }
    if ((gridDesX+1) > -1 && gridDesY > -1 && gridDesY < height && (gridDesX+1) < width) {
      belief_new[(gridDesX+1)+(gridDesY)*width] += (double) belief[iGridX+iGridY*width]*delta;
      count++;
    }
    if ((gridDesX-1) > -1 && (gridDesY+1) > -1 && (gridDesY+1) < height && (gridDesX-1) < width) {
      belief_new[(gridDesX-1)+(gridDesY+1)*width] += (double) belief[iGridX+iGridY*width]*delta;
      count++;
    }
    if ((gridDesX+1) > -1 && (gridDesY-1) > -1 && (gridDesY-1) < height && (gridDesX+1) < width) {
      belief_new[(gridDesX+1)+(gridDesY-1)*width] += (double) belief[iGridX+iGridY*width]*delta;
      count++;
    }
    if (gridDesX > -1 && gridDesY > -1 && gridDesY < height && gridDesX < width) {
    belief_new[gridDesX+gridDesY*width] += (double) belief[iGridX+iGridY*width]*(1-delta*count);
    } else {
      belief_new[iGridX+iGridY*width] += (double) belief[iGridX+iGridY*width]*(1-delta*count);
    }
  }

  if (agent == 0) { // segway
    for (int i = 0; i < width*height; i++) {
      segway_belief[i] =  belief_new[i];
    }
  }

  if (agent == 1) { // flipper
    for (int i = 0; i < width*height; i++) {
      flipper_belief[i] =  belief_new[i];
    }
  }

  if (agent == 2) { // uav
    for (int i = 0; i < width*height; i++) {
      uav_belief[i] =  belief_new[i];
    }
  }
}

void uav_reward(void) {
  double uav_r[totalGrids];
  double gridposx[totalGrids];
  double gridposy[totalGrids];
  double max_r = 0;
  double reward;
  uint32_t gridDesX;
  uint32_t gridDesY;
  for (int i = 0; i < totalGrids; i++) {
    uav_r[i] = 0.25-(sam_belief[i]-.5)*(sam_belief[i]-.5);
    int iGridX = i % width;
    int iGridY = (int) i/width;
    gridposx[i] = origin[0]+iGridX*resolution;
    gridposy[i] = origin[1]+iGridY*resolution;
    double distance_x = (uav_states[0] - gridposx[i]);
    double distance_y = (uav_states[1] - gridposy[i]);
    double distance = sqrt(distance_x*distance_x+distance_y*distance_y);
    reward = uav_r[i];
    if (reward > max_r && distance < 2 && iGridX >= xmin && iGridX <= xmax && iGridY <= ymax && iGridY >=ymin) {
      gridDesX = iGridX;
      gridDesY = iGridY;
      max_r = reward;
    }
  }

  int gridsX = (int)((uav_states[0]-origin[0])/resolution)-gridDesX;
  int gridsY = gridDesY - (int)((uav_states[1]-origin[1])/resolution);
  int direction; int sign;
  if (abs(gridsX) > abs(gridsY)) {
    direction = gridsX;
    if (gridsX > 0){
      sign = 1;
    } else {
      sign = -1;
    }
  } else {
    direction = gridsY;
    if (gridsY > 0){
      sign = 1;
    } else {
      sign = -1;
    }
  }
  int action = 0;
  if (direction == gridsX && sign == 1)
    action = 3;
  if (direction == gridsX && sign == -1)
    action = 1;
  if (direction == gridsY && sign == -1)
    action = 4;
  if (direction == gridsY && sign == 1)
    action = 2;
  uav_action = action;
  std_msgs::Int32 uav_action_message;
  uav_action_message.data = uav_action;
  uav_action_pub.publish(uav_action);
}

void flipper_reward(void) {
  double flipper_r[totalGrids];
  double gridposx[totalGrids];
  double gridposy[totalGrids];
  double max_r = 0.0;
  double reward;
  uint32_t gridDesX;
  uint32_t gridDesY;
  int obsFound = 0;
  for (int i = 0; i < totalGrids; i++) {
    flipper_r[i] = .25-(obs_belief[i]-.5)*(obs_belief[i]-.5);
    if (obs_belief[i] > 0.85) {
      obsFound++;
    }
    if (obsFound >= 3) {
      // flipper_action = 0;
      // std_msgs::Int32 flipper_action_message;
      // flipper_action_message.data = flipper_action;
      // flipper_action_pub.publish(flipper_action);
      // return;
    }
    int iGridX = i % width;
    int iGridY = (int) i/width;
    gridposx[i] = origin[0]+iGridX*resolution;
    gridposy[i] = origin[1]+iGridY*resolution;
    double distance_x = (flipper_states[0] - gridposx[i]);
    double distance_y = (flipper_states[1] - gridposy[i]);
    double distance = sqrt(distance_x*distance_x+distance_y*distance_y);
    reward = flipper_r[i];
    if (distance_x < 0){
      reward = reward * 0.25;
    }
    if (distance < 4 && iGridX >= xmin && iGridX <= xmax && iGridY <= ymax && iGridY >=ymin) {
      // ROS_INFO("%i,%i: %f",iGridX,iGridY,reward);
    }
    if (reward > max_r && distance < 3 && iGridX >= xmin && iGridX <= xmax && iGridY <= ymax && iGridY >=ymin) {
      gridDesX = iGridX;
      gridDesY = iGridY;
      max_r = reward;
    }

  }
  // if (max_r == 0.1) {
  //   flipper_action = 0;
  //   std_msgs::Int32 flipper_action_message;
  //   flipper_action_message.data = flipper_action;
  //   flipper_action_pub.publish(flipper_action);
  //   return;
  // }
  // ROS_INFO("__________");
  int gridsX = (int)((flipper_states[0]-origin[0])/resolution)-gridDesX;
  int gridsY = gridDesY - (int)((flipper_states[1]-origin[1])/resolution);
  int direction; int sign;
  if (abs(gridsX) > abs(gridsY)) {
    direction = gridsX;
    if (gridsX > 0){
      sign = 1;
    } else {
      sign = -1;
    }
  } else {
    direction = gridsY;
    if (gridsY > 0){
      sign = 1;
    } else {
      sign = -1;
    }
  }
  int action = 0;
  if (direction == gridsX && sign == 1)
    action = 3;
  if (direction == gridsX && sign == -1)
    action = 1;
  if (direction == gridsY && sign == -1)
    action = 4;
  if (direction == gridsY && sign == 1)
    action = 2;
  flipper_action = action;
  std_msgs::Int32 flipper_action_message;
  flipper_action_message.data = flipper_action;
  flipper_action_pub.publish(flipper_action);
}

void segway_filter(double *obs, double *flip) {
  state_type segway_belief_old = segway_belief;
  update_agent_belief(0);
  double barrier_obs = 0;
  for (int i = 0; i < totalGrids; i++) {
    double val = segway_belief[i] * obs_belief[i];
    barrier_obs += val;
  }
  barrier_obs = max_failure_rate - barrier_obs;
  double barrier_flipper = 0;
  for (int i = 0; i < totalGrids; i++) {
    double val = segway_belief[i] * flipper_belief[i];
    barrier_flipper += val;
  }
  barrier_flipper = max_failure_rate - barrier_flipper;
  if (barrier_obs < 0 || barrier_flipper < 0) {
  }
  ROS_INFO("FILTER values, h(x) = (%f,%f)",barrier_obs,barrier_flipper);
  *obs = barrier_obs;
  *flip = barrier_flipper;
  segway_belief = segway_belief_old;
}

double segway_filter_ft(void) {
  // segway_action = 3;
  segway_belief_old = segway_belief;
  double hvalold = 0;
  double hval = 0;
  for (int i = 0; i < totalGrids; i++) {
    double val = segway_belief[i] * sam_belief[i];
    hvalold += val;
  }
  hvalold = hvalold - 0.5;
  update_agent_belief(0);
  for (int i = 0; i < totalGrids; i++) {
    double val = segway_belief[i] * sam_belief[i];
    hval += val;
  }
  hval = hval - 0.5;
  segway_belief = segway_belief_old;
  double rho = 0.99;
  double epsilon = 0.1;
  double barrier = hval-rho*hvalold-epsilon*(1-rho);
  barrier_msg.data[0] = hval;
  if (barrier <= 0.0001) {
    ROS_INFO("FT barrier failed!: (%f)",barrier);
    hval = 0;
    segway_action = 4;
    update_agent_belief(0);
    for (int i = 0; i < totalGrids; i++) {
      double val = segway_belief[i] * sam_belief[i];
      hval += val;
    }
    segway_belief = segway_belief_old;
    barrier = hval-rho*hvalold-epsilon*(1-rho);
    barrier_msg.data[1] = hval;
  } else {
    barrier_msg.data[1] = -1;
  }

  ROS_INFO("FT barrier: (%f)",barrier);
  return barrier;
}

void grid_hab_update(void) {
  int gridsX = (flipper_states[0]-origin[0])/resolution;
  int gridsY = (flipper_states[1]-origin[1])/resolution;
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

void uav_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < uav_nx; ++i) {
    uav_states_old[i] = uav_states[i];
    uav_states[i] = msg->data[i];
  }
}

void segway_actions_cb(const std_msgs::Int32::ConstPtr& msg) {
  segway_action = msg->data;
  marker.header.stamp = ros::Time();
  marker.pose.position.x = segway_states[0];
  marker.pose.position.y = segway_states[1];
  marker.pose.position.z = 0.4;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  marker2.header.stamp = ros::Time();
  marker2.pose.position.x = segway_states[0];
  marker2.pose.position.y = segway_states[1];
  marker2.pose.position.z = 0.4;
  marker2.pose.orientation.w = 1.0;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  if (segway_action == 0) {
    marker.points[0].x = 0;
    marker.points[1].x = 0;
    marker.points[0].y = 0;
    marker.points[1].y = 0;
    marker.points[0].z = 0;
    marker.points[1].z = .25;
  }
  if (segway_action == 1) {
    marker.points[0].x = 0;
    marker.points[1].x = 0.5;
    marker.points[0].y = 0;
    marker.points[1].y = 0;
    marker.points[0].z = 0;
    marker.points[1].z = 0;
  }
  if (segway_action == 2) {
    marker.points[0].x = 0;
    marker.points[1].x = 0;
    marker.points[0].y = 0;
    marker.points[1].y = 0.5;
    marker.points[0].z = 0;
    marker.points[1].z = 0;
  }
  if (segway_action == 3) {
    marker.points[0].x = 0;
    marker.points[1].x = -0.5;
    marker.points[0].y =0;
    marker.points[1].y =0;
    marker.points[0].z = 0;
    marker.points[1].z = 0;
  }
  if (segway_action == 4) {
    marker.points[0].x = 0;
    marker.points[1].x = 0;
    marker.points[0].y = 0;
    marker.points[1].y = -0.5;
    marker.points[0].z = 0;
    marker.points[1].z = 0;
  }
  marker_pub.publish( marker );
  double FTbarrier = segway_filter_ft();
  double obs[1]; double flip[1];
  segway_filter(obs,flip);
  barrier_msg.data[2] = obs[0];
  barrier_msg.data[3] = flip[0];
  if (fmin(obs[0],flip[0]) > 0) {
    segway_action_real = segway_action;
  }
  else if (fmin(obs[0],flip[0]) < 0 && segway_action == 2) {
    segway_action = 3;
    segway_filter(obs,flip);
    if (fmin(obs[0],flip[0]) < 0) {
      segway_action_real = 0;
    } else {
      segway_action_real = segway_action;
    }
  }
  else if (fmin(obs[0],flip[0]) < 0 && segway_action == 3) {
    segway_action = 2;
    segway_filter(obs,flip);
    if (fmin(obs[0],flip[0]) < 0) {
      segway_action_real = 0;
    } else {
      segway_action_real = segway_action;
    }
  } else {
    segway_action_real = 0;
  }
  if (segway_action_real == 0) {
    segway_action = 0;
    segway_filter(obs,flip);
    barrier_msg.data[4] = obs[0];
    barrier_msg.data[5] = flip[0];
  }
  segway_action = segway_action_real;
  std_msgs::Int32 segway_action_message;
  segway_action_message.data = segway_action_real;
  segway_action_pub.publish(segway_action_message);
  readyToMove1 = true;
  readyToMove2 = true;
  barrier_pub.publish(barrier_msg);
  if (segway_action == 1) {
    marker2.points[0].x = 0;
    marker2.points[1].x = 0.5;
    marker2.points[0].y = 0;
    marker2.points[1].y = 0;
    marker2.points[0].z = 0;
    marker2.points[1].z = 0;
  }
  if (segway_action == 2) {
    marker2.points[0].x = 0;
    marker2.points[1].x = 0;
    marker2.points[0].y = 0;
    marker2.points[1].y = 0.5;
    marker2.points[0].z = 0;
    marker2.points[1].z = 0;
  }
  if (segway_action == 3) {
    marker2.points[0].x = 0;
    marker2.points[1].x = -0.5;
    marker2.points[0].y =0;
    marker2.points[1].y =0;
    marker2.points[0].z = 0;
    marker2.points[1].z = 0;
  }
  if (segway_action == 4) {
    marker2.points[0].x = 0;
    marker2.points[1].x = 0;
    marker2.points[0].y = 0;
    marker2.points[1].y = -0.5;
    marker2.points[0].z = 0;
    marker2.points[1].z = 0;
  }
  if (segway_action == 0) {
    marker2.points[0].x = 0;
    marker2.points[1].x = 0;
    marker2.points[0].y = 0;
    marker2.points[1].y = 0;
    marker2.points[0].z = 0;
    marker2.points[1].z = .25;
  }
  marker_pub2.publish( marker2 );
}

void flipper_actions_cb(const std_msgs::Int32::ConstPtr& msg) {
    flipper_action = msg->data;
}

void uav_actions_cb(const std_msgs::Int32::ConstPtr& msg) {
    uav_action = msg->data;
}

void segway_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (segway_action_finished == true && msg->data == false) {
    segway_action_finished = false;
  }
  if (segway_action_finished == false && msg->data == true) {
    update_agent_belief(0);
    for (int i = 0; i < segway_loc.info.width*segway_loc.info.height; i++) {
      int data = 100*segway_belief[i];
      segway_loc.data[i] = data;
    }
    segway_loc.header.stamp = ros::Time::now();
    segway_grid_pub.publish(segway_loc);
    segway_action_finished = true;
  }
}

void flipper_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (flipper_action_finished == true && msg->data == false) {
    flipper_action_finished = false;
  }
  if (flipper_action_finished == false && msg->data == true) {
    update_agent_belief(1);
    flipper_action_finished = true;
    ROS_INFO("updated flipper belief");
  }
  if (readyToMove1){
    for (int i = 0; i < flipper_loc.info.width*flipper_loc.info.height; i++) {
      int data = 100*flipper_belief[i];
      flipper_loc.data[i] = data;
    }
    flipper_loc.header.stamp = ros::Time::now();
    flipper_grid_pub.publish(flipper_loc);
    grid_hab_update();
    // if (readyToMove1){
      flipper_reward();
      readyToMove1 = false;
    // }
  }
}

void uav_complete_cb(const std_msgs::Bool::ConstPtr& msg) {
  if (readyToMove2){
  // if (uav_action_finished == true && msg->data == false) {
  //   uav_action_finished = false;
  // }
  // if (uav_action_finished == false && msg->data == true) {
    update_agent_belief(2);
    for (int i = 0; i < uav_loc.info.width*uav_loc.info.height; i++) {
      int data = 100*uav_belief[i];
      uav_loc.data[i] = data;
    }
    uav_loc.header.stamp = ros::Time::now();
    uav_grid_pub.publish(uav_loc);
    uav_action_finished = true;
    grid_sam_update();
    ros::Duration(0.25).sleep();
    // if (readyToMove2){
      uav_reward();
      readyToMove2 = false;
    // }
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "grid_states");

  ros::NodeHandle n;

  ros::param::get("~_timeScale", timeScale);

  ros::Subscriber uav_states_sub = n.subscribe<std_msgs::Float32MultiArray>("uav/states", 1, uav_states_cb);
  ros::Subscriber segway_states_sub = n.subscribe<std_msgs::Float32MultiArray>("segway/states", 1, segway_states_cb);
  ros::Subscriber flipper_states_sub = n.subscribe<std_msgs::Float32MultiArray>("flipper/states", 1,flipper_states_cb);

  // ros::Subscriber uav_actions_sub = n.subscribe<std_msgs::Int32>("uav/action", 1, uav_actions_cb);
  // ros::Subscriber segway_actions_sub = n.subscribe<std_msgs::Int32>("segway/action_desired", 1, segway_actions_cb);
  // ros::Subscriber flipper_actions_sub = n.subscribe<std_msgs::Int32>("flipper/action", 1,flipper_actions_cb);

  ros::Subscriber uav_complete_sub = n.subscribe<std_msgs::Bool>("uav/complete", 1, uav_complete_cb);
  ros::Subscriber segway_complete_sub = n.subscribe<std_msgs::Bool>("segway/complete", 1, segway_complete_cb);
  ros::Subscriber flipper_complete_sub = n.subscribe<std_msgs::Bool>("flipper/complete", 1, flipper_complete_cb);

  ros::Subscriber segway_actions_sub = n.subscribe<std_msgs::Int32>("segway/action_desired", 1,segway_actions_cb);


  uav_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("uav/belief", 1);
  segway_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("segway/belief", 1);
  flipper_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("flipper/belief", 1);
  grid_hab_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/habitable", 1);
  grid_sam_pub = n.advertise<nav_msgs::OccupancyGrid>("grid/sample", 1);

  uav_action_pub = n.advertise<std_msgs::Int32>("uav/action", 1);
  flipper_action_pub = n.advertise<std_msgs::Int32>("flipper/action", 1);
  segway_action_pub = n.advertise<std_msgs::Int32>("segway/action_actual", 1);

  barrier_pub = n.advertise<std_msgs::Float32MultiArray>("barrier", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("action_desired", 1);
  marker_pub2 = n.advertise<visualization_msgs::Marker>("action_actual", 1);

  // segway_action_pub = n.advertise<std_msgs::Int32>("segway/action_actual", 1000);

  ros::Rate loop_rate(200*timeScale);

  uav_loc.header.frame_id = "world";
  uav_loc.info.resolution = resolution;
  uav_loc.info.width = (uint32_t) castWidth/resolution;
  uav_loc.info.height = (uint32_t) castHeight/resolution;
  uav_loc.info.origin.position.x = origin[0];
  uav_loc.info.origin.position.y = origin[1];
  uav_loc.info.origin.position.z = .1;
  uav_loc.data.resize(uav_loc.info.width*uav_loc.info.height,-1);

  flipper_loc.header.frame_id = "world";
  flipper_loc.info.resolution = resolution;
  flipper_loc.info.width = (uint32_t) castWidth/resolution;
  flipper_loc.info.height = (uint32_t) castHeight/resolution;
  flipper_loc.info.origin.position.x = origin[0];
  flipper_loc.info.origin.position.y = origin[1];
  flipper_loc.info.origin.position.z = .1;
  flipper_loc.data.resize(flipper_loc.info.width*flipper_loc.info.height,-1);

  segway_loc.header.frame_id = "world";
  segway_loc.info.resolution = resolution;
  segway_loc.info.width = (uint32_t) castWidth/resolution;
  segway_loc.info.height = (uint32_t) castHeight/resolution;
  segway_loc.info.origin.position.x = origin[0];
  segway_loc.info.origin.position.y = origin[1];
  segway_loc.info.origin.position.z = .1;
  segway_loc.data.resize(segway_loc.info.width*segway_loc.info.height,-1);

  grid_hab.header.frame_id = "world";
  grid_hab.info.resolution = resolution;
  grid_hab.info.width = (uint32_t) castWidth/resolution;
  grid_hab.info.height = (uint32_t) castHeight/resolution;
  grid_hab.info.origin.position.x = origin[0];
  grid_hab.info.origin.position.y = origin[1];
  grid_hab.info.origin.position.z = .1;
  grid_hab.data.resize(grid_hab.info.width*grid_hab.info.height,-1);

  grid_sam.header.frame_id = "world";
  grid_sam.info.resolution = resolution;
  grid_sam.info.width = (uint32_t) castWidth/resolution;
  grid_sam.info.height = (uint32_t) castHeight/resolution;
  grid_sam.info.origin.position.x = origin[0];
  grid_sam.info.origin.position.y = origin[1];
  grid_sam.info.origin.position.z = .1;
  grid_sam.data.resize(grid_sam.info.width*grid_sam.info.height,-1);

  ros::Duration(3).sleep();
  ros::spinOnce();

  uint32_t gridsX = (segway_states[0]-segway_loc.info.origin.position.x)/resolution;
  uint32_t gridsY = (segway_states[1]-segway_loc.info.origin.position.y)/resolution;
  segway_belief[gridsX+gridsY*segway_loc.info.width]= 1;
  segway_first_action = false;
  double totalSum = 0;
  for (int i = 0; i < segway_loc.info.width*segway_loc.info.height; i++) {
    segway_loc.data[i] = (int) segway_belief[i]*100;
    segway_belief[i] = (int) segway_belief[i];
  }
  segway_grid_pub.publish(segway_loc);

  gridsX = (flipper_states[0]-flipper_loc.info.origin.position.x)/resolution;
  gridsY = (flipper_states[1]-flipper_loc.info.origin.position.y)/resolution;
  flipper_belief[gridsX+gridsY*flipper_loc.info.width]= 1;
  flipper_first_action = false;
  for (int i = 0; i < flipper_loc.info.width*flipper_loc.info.height; i++) {
    flipper_loc.data[i] = (int) flipper_belief[i]*100;
    flipper_belief[i] = (int) flipper_belief[i];
  }
  flipper_grid_pub.publish(flipper_loc);

  gridsX = (uav_states[0]-uav_loc.info.origin.position.x)/resolution;
  gridsY = (uav_states[1]-uav_loc.info.origin.position.y)/resolution;
  uav_belief[gridsX+gridsY*uav_loc.info.width]= 1;
  uav_first_action = false;
  for (int i = 0; i < uav_loc.info.width*uav_loc.info.height; i++) {
    uav_loc.data[i] = (int) uav_belief[i]*100;
    uav_belief[i] = (int) uav_belief[i];
  }
  uav_grid_pub.publish(uav_loc);

  // Define grids with obstacles
  true_obs_belief(9*9+2) = 1; // true_obs_belief(9*4+5) = 1;
  true_obs_belief(9*7+2) = 1; // true_obs_belief(9*5+2) = 1;
  true_obs_belief(9*7+4) = 1; // true_obs_belief(9*6+4) = 1;
  true_sam_belief(9*9+1) = 1; // true_sam_belief(9*7+0) = 1;
  obs_belief = obs_belief*.5;
  sam_belief = sam_belief*.5;

  grid_hab_update();
  grid_sam_update();

  grid_hab_update();
  grid_sam_update();

  grid_hab_update();
  grid_sam_update();

  // std_msgs::Int32 uav_action_message;
  // uav_action_message.data = 0;
  // uav_action_pub.publish(uav_action_message);
  //
  // std_msgs::Int32 flipper_action_message;
  // flipper_action_message.data = 0;
  // flipper_action_pub.publish(flipper_action_message);

  //
  // std_msgs::Int32 segway_action_message;
  // segway_action_message.data = 0;
  // segway_action_pub.publish(segway_action_message);


  barrier_msg.data.clear();
  barrier_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  barrier_msg.layout.dim[0].size = 3;
  for (int i = 0; i < 6; i++) {
    barrier_msg.data.push_back(0.0);
  }

  marker.header.frame_id = "world";
  marker.ns = "";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = .1;
  marker.scale.y = 0.2;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.points.resize(2);

  marker2.header.frame_id = "world";
  marker2.ns = "";
  marker2.id = 0;
  marker2.type = visualization_msgs::Marker::ARROW;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.scale.x = .1;
  marker2.scale.y = 0.2;
  marker2.color.a = 1.0; // Don't forget to set the alpha!
  marker2.color.r = 0.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;
  marker2.points.resize(2);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
