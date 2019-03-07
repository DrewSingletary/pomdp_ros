#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32MultiArray.h"
#include "gazebo_msgs/ModelState.h"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "asif.h"


const int nx = 7;
typedef Eigen::VectorXd state_type;
state_type states = Eigen::VectorXd::Zero(nx);

ros::Publisher gazebo_pub;
geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
gazebo_msgs::ModelState modelstate;

void states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < nx; ++i)
    states[i] = msg->data[i];

  double cy = cos(states[2] * 0.5);
  double sy = sin(states[2] * 0.5);
  double cp = 0;
  double sp = 0;
  double cr = 0;
  double sr = 0;

  pose.position.x = states[0];
  pose.position.y = states[1];
  pose.position.z = 0;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(states[2]);
  pose.orientation = odom_quat;
  // pose.orientation.x = cy * cp * sr - sy * sp * cr;
  // pose.orientation.y = sy * cp * sr + cy * sp * cr;
  // pose.orientation.z = sy * cp * cr - cy * sp * sr;
  // pose.orientation.w = cy * cp * cr + sy * sp * sr;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  modelstate.model_name = (std::string) "boat";
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = pose;
  modelstate.twist = twist;
  gazebo_pub.publish(modelstate);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "gazebo_interface");

  ros::NodeHandle n;

  ros::Subscriber states_sub = n.subscribe<std_msgs::Float32MultiArray>("states", 1, states_cb);

  gazebo_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}