#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32MultiArray.h"
#include "gazebo_msgs/ModelState.h"
#include <Eigen/Eigen>
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

void uav_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < nx; ++i)
    states[i] = msg->data[i];

  pose.position.x = states[0];
  pose.position.y = states[1];
  pose.position.z = states[2];

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  pose.orientation = odom_quat;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  modelstate.model_name = (std::string) "uav";
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = pose;
  modelstate.twist = twist;
  gazebo_pub.publish(modelstate);
}

void segway_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < nx; ++i)
    states[i] = msg->data[i];

  pose.position.x = states[0];
  pose.position.y = states[1];
  pose.position.z = .2;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(states[3]);
  pose.orientation = odom_quat;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  modelstate.model_name = (std::string) "segway";
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = pose;
  modelstate.twist = twist;
  gazebo_pub.publish(modelstate);
}

void flipper_states_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < nx; ++i)
    states[i] = msg->data[i];

  pose.position.x = states[0];
  pose.position.y = states[1];
  pose.position.z = .15;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(1.5708,0,states[3]);
  pose.orientation = odom_quat;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  modelstate.model_name = (std::string) "flipper";
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = pose;
  modelstate.twist = twist;
  gazebo_pub.publish(modelstate);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "gazebo_interface");

  ros::NodeHandle n;

  ros::Subscriber uav_states_sub = n.subscribe<std_msgs::Float32MultiArray>("uav/states", 1, uav_states_cb);
  ros::Subscriber segway_states_sub = n.subscribe<std_msgs::Float32MultiArray>("segway/states", 1, segway_states_cb);
  ros::Subscriber flipper_states_sub = n.subscribe<std_msgs::Float32MultiArray>("flipper/states", 1, flipper_states_cb);

  gazebo_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
