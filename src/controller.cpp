#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Joy.h"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <stdlib.h>
#include <math.h> 

typedef Eigen::VectorXd state_type;

double timeScale = 1;
double resolution = .5;

double vmax_flipper = 1;
double vmax_segway = 2;
double vmax_uav = 2;

double amax_flipper = 1;
double amax_segway = 2;
double amax_uav = 2;

bool uav_state_received = false;
bool segway_state_received = false;
bool flipper_state_received = false;

const int uav_nx = 12;
const int uav_nu = 3;
const int segway_nx = 7;
const int segway_nu = 2;
const int flipper_nx = 5;
const int flipper_nu = 2;

state_type uav_states = Eigen::VectorXd::Zero(uav_nx);
state_type segway_states = Eigen::VectorXd::Zero(segway_nx);
state_type flipper_states = Eigen::VectorXd::Zero(flipper_nx);

state_type uav_des_pos = Eigen::VectorXd::Zero(2);
state_type segway_des_pos = Eigen::VectorXd::Zero(2);
state_type flipper_des_pos = Eigen::VectorXd::Zero(2);

double uav_des_ang = 0.0;
double segway_des_ang = 0.0;
double flipper_des_ang = 0.0;

state_type uav_diff = Eigen::VectorXd::Zero(2);
state_type segway_diff = Eigen::VectorXd::Zero(2);
state_type flipper_diff = Eigen::VectorXd::Zero(2);

state_type uav_inputs = Eigen::VectorXd::Zero(uav_nu);
state_type segway_inputs = Eigen::VectorXd::Zero(segway_nu);
state_type flipper_inputs = Eigen::VectorXd::Zero(flipper_nu);

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <class T> const T& min (const T& a, const T& b) {
  return !(b<a)?a:b;     // or: return !comp(b,a)?a:b; for version (2)
}

template <class T> const T& max (const T& a, const T& b) {
  return (a<b)?b:a;     // or: return comp(a,b)?b:a; for version (2)
}

void joy_cb(const sensor_msgs::Joy & msg) {

}

void uav_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < uav_nx; ++i)
    uav_states[i] = msg->data[i];
  if (!uav_state_received) {
    uav_des_pos = uav_states;
    uav_state_received = true;
  }
}

void segway_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < segway_nx; ++i)
    segway_states[i] = msg->data[i];
  if (!segway_state_received) {
    segway_des_pos = segway_states;
    segway_des_ang = segway_states[3];
    segway_state_received = true;
  }

}

void flipper_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < flipper_nx; ++i)
    flipper_states[i] = msg->data[i];
  if (!flipper_state_received) {
    flipper_des_pos = flipper_states;
    segway_des_ang = flipper_states[3];
    flipper_state_received = true;
  }
}

void uav_act_cb(const std_msgs::Int32::ConstPtr& msg) {
  if (msg->data == 0) {
    uav_des_pos[0] = uav_states[0];
    uav_des_pos[1] = uav_states[1];
  }
  if (msg->data == 1) {
    uav_des_pos[0] = uav_states[0]+resolution;
    uav_des_pos[1] = uav_states[1];
  }
  if (msg->data == 2) {
    uav_des_pos[0] = uav_states[0];
    uav_des_pos[1] = uav_states[1]+resolution;
  }
  if (msg->data == 3) {
    uav_des_pos[0] = uav_states[0]-resolution;
    uav_des_pos[1] = uav_states[1];
  }
  if (msg->data == 4) {
    uav_des_pos[0] = uav_states[0];
    uav_des_pos[1] = uav_states[1]-resolution;
  }
  
}

void segway_act_cb(const std_msgs::Int32::ConstPtr& msg) {
  if (msg->data == 0) {
    segway_des_pos[0] = segway_states[0];
    segway_des_pos[1] = segway_states[1];
    int offset = (int)segway_states[3]/(2*M_PI);
    segway_des_ang = segway_states[3];
  }
  if (msg->data == 1) {
    segway_des_pos[0] = segway_states[0]+resolution;
    segway_des_pos[1] = segway_states[1];
    int offset = (int)segway_states[3]/(2*M_PI);
    segway_des_ang = 2*M_PI*offset;
    if (segway_des_ang-segway_states[3] > M_PI) {
      segway_des_ang -= 2*M_PI;
    } else if (segway_des_ang-segway_states[3] < -M_PI) {
      segway_des_ang += 2*M_PI;
    }
  }
  if (msg->data == 2) {
    segway_des_pos[0] = segway_states[0];
    segway_des_pos[1] = segway_states[1]+resolution;
    int offset = (int)segway_states[3]/(2*M_PI);
    segway_des_ang = 2*M_PI*offset+M_PI/2;
    if (segway_des_ang-segway_states[3] > M_PI) {
      segway_des_ang -= 2*M_PI;
    } else if (segway_des_ang-segway_states[3] < -M_PI) {
      segway_des_ang += 2*M_PI;
    }
  }
  if (msg->data == 3) {
    segway_des_pos[0] = segway_states[0]-resolution;
    segway_des_pos[1] = segway_states[1];
    int offset = (int)segway_states[3]/(2*M_PI);
    segway_des_ang = 2*M_PI*offset+M_PI;
    if (segway_des_ang-segway_states[3] > M_PI) {
      segway_des_ang -= 2*M_PI;
    } else if (segway_des_ang-segway_states[3] < -M_PI) {
      segway_des_ang += 2*M_PI;
    }
  }
  if (msg->data == 4) {
    segway_des_pos[0] = segway_states[0];
    segway_des_pos[1] = segway_states[1]-resolution;
    int offset = (int)segway_states[3]/(2*M_PI);
    segway_des_ang = 2*M_PI*offset-M_PI/2;
    if (segway_des_ang-segway_states[3] > M_PI) {
      segway_des_ang -= 2*M_PI;
    } else if (segway_des_ang-segway_states[3] < -M_PI) {
      segway_des_ang += 2*M_PI;
    }
  }
}

void flipper_act_cb(const std_msgs::Int32::ConstPtr& msg) {
  if (msg->data == 0) {
    flipper_des_pos[0] = flipper_states[0];
    flipper_des_pos[1] = flipper_states[1];
    int offset = (int)flipper_states[3]/(2*M_PI);
    flipper_des_ang = flipper_states[3];
  }
  if (msg->data == 1) {
    flipper_des_pos[0] = flipper_states[0]+resolution;
    flipper_des_pos[1] = flipper_states[1];
    int offset = (int)flipper_states[3]/(2*M_PI);
    flipper_des_ang = 2*M_PI*offset;
    if (flipper_des_ang-flipper_states[3] > M_PI) {
      flipper_des_ang -= 2*M_PI;
    } else if (flipper_des_ang-flipper_states[3] < -M_PI) {
      flipper_des_ang += 2*M_PI;
    }
  }
  if (msg->data == 2) {
    flipper_des_pos[0] = flipper_states[0];
    flipper_des_pos[1] = flipper_states[1]+resolution;
    int offset = (int)flipper_states[3]/(2*M_PI);
    flipper_des_ang = 2*M_PI*offset+M_PI/2;
    if (flipper_des_ang-flipper_states[3] > M_PI) {
      flipper_des_ang -= 2*M_PI;
    } else if (flipper_des_ang-flipper_states[3] < -M_PI) {
      flipper_des_ang += 2*M_PI;
    }
  }
  if (msg->data == 3) {
    flipper_des_pos[0] = flipper_states[0]-resolution;
    flipper_des_pos[1] = flipper_states[1];
    int offset = (int)flipper_states[3]/(2*M_PI);
    flipper_des_ang = 2*M_PI*offset+M_PI;
    if (flipper_des_ang-flipper_states[3] > M_PI) {
      flipper_des_ang -= 2*M_PI;
    } else if (flipper_des_ang-flipper_states[3] < -M_PI) {
      flipper_des_ang += 2*M_PI;
    }
  }
  if (msg->data == 4) {
    flipper_des_pos[0] = flipper_states[0];
    flipper_des_pos[1] = flipper_states[1]-resolution;
    int offset = (int)flipper_states[3]/(2*M_PI);
    flipper_des_ang = 2*M_PI*offset-M_PI/2;
    if (flipper_des_ang-flipper_states[3] > M_PI) {
      flipper_des_ang -= 2*M_PI;
    } else if (flipper_des_ang-flipper_states[3] < -M_PI) {
      flipper_des_ang += 2*M_PI;
    }
  }
}

void uav_controller(void) {
  uav_diff = uav_des_pos-uav_states;
  uav_inputs[0] = max(min(5*uav_diff[0],vmax_uav),-vmax_uav);
  uav_inputs[1] = max(min(5*uav_diff[1],vmax_uav),-vmax_uav);
  uav_inputs[2] = 0;
}

void segway_controller(void) {
  segway_diff = segway_des_pos-segway_states;
  double d = sqrt(segway_diff[0]*segway_diff[0]+segway_diff[1]*segway_diff[1]);
  ROS_INFO("%f %f %f",segway_diff[0], segway_diff[1], d);
  double dtheta = segway_des_ang-segway_states[3];
  if (abs(dtheta) > M_PI/100) {
    segway_inputs[0] = 0;
    segway_inputs[1] = max(min(10*(dtheta),amax_segway),-amax_segway);
  }
  else {
    int ind;
    if (d < .01) {
      segway_inputs[0] = 0;
    } else {
      segway_inputs[0] = min(10*d,vmax_segway);
    }
    segway_inputs[1] = max(min(10*(dtheta),amax_segway),-amax_segway);
  }
}

void flipper_controller(void) {

  flipper_diff = flipper_des_pos-flipper_states;
  double d = sqrt(flipper_diff[0]*flipper_diff[0]+flipper_diff[1]*flipper_diff[1]);
  ROS_INFO("%f %f %f",flipper_diff[0], flipper_diff[1], d);
  double dtheta = flipper_des_ang-flipper_states[3];
  if (abs(dtheta) > M_PI/100) {
    flipper_inputs[0] = 0;
    flipper_inputs[1] = max(min(5*(dtheta),amax_flipper),-amax_flipper);
  }
  else {
    int ind;
    if (d < .01) {
      flipper_inputs[0] = 0;
    } else {
      flipper_inputs[0] = min(5*d,vmax_flipper);
    }
    flipper_inputs[1] = max(min(5*(dtheta),amax_flipper),-amax_flipper);
  }
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::param::get("~_timeScale", timeScale);

  ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_cb);

  ros::Subscriber uav_states_sub = n.subscribe<std_msgs::Float32MultiArray>("uav/states", 1, uav_states_cb);
  ros::Subscriber segway_states_sub = n.subscribe<std_msgs::Float32MultiArray>("segway/states", 1, segway_states_cb);
  ros::Subscriber flipper_states_sub = n.subscribe<std_msgs::Float32MultiArray>("flipper/states", 1,flipper_states_cb);

  ros::Subscriber uav_act_sub = n.subscribe<std_msgs::Int32>("uav/action", 1, uav_act_cb);
  ros::Subscriber segway_act_sub = n.subscribe<std_msgs::Int32>("segway/action", 1, segway_act_cb);
  ros::Subscriber flipper_act_sub = n.subscribe<std_msgs::Int32>("flipper/action", 1,flipper_act_cb);

  ros::Publisher uav_controller_pub = n.advertise<std_msgs::Float32MultiArray>("uav/inputs", 1000);
  ros::Publisher segway_controller_pub = n.advertise<std_msgs::Float32MultiArray>("segway/inputs", 1000);
  ros::Publisher flipper_controller_pub = n.advertise<std_msgs::Float32MultiArray>("flipper/inputs", 1000);


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

    if (uav_state_received && segway_state_received && flipper_state_received) {
    
    uav_controller();
    for (int i = 0;i<uav_nu;i++) {
      uav_msg.data[i] = uav_inputs[i];
    }
    uav_controller_pub.publish(uav_msg);

    segway_controller();
    for (int i = 0;i<segway_nu;i++) {
      segway_msg.data[i] = segway_inputs[i];
    }
    segway_controller_pub.publish(segway_msg);

    flipper_controller();
    for (int i = 0;i<flipper_nu;i++) {
      flipper_msg.data[i] = flipper_inputs[i];
    }
    flipper_controller_pub.publish(flipper_msg);

    } else {
      ROS_INFO_THROTTLE(1,"waiting for state info");
      ROS_INFO_THROTTLE(1,"%i %i %i",uav_state_received,segway_state_received,flipper_state_received);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}