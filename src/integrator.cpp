#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/numeric/odeint.hpp>
#include <stdlib.h>
#include <math.h>

typedef std::vector<double> state_type;


double t;
double timeScale = 1;

const int uav_nx = 12;
const int uav_nu = 3;
const int segway_nx = 7;
const int segway_nu = 2;
const int flipper_nx = 5;
const int flipper_nu = 2;

class UAVIntegrator {
public:
  UAVIntegrator() {
    for (int i = 0; i < uav_nx; i++)
      x.push_back(0.);
    for (int i = 0; i < uav_nu; i++)
      u.push_back(0.);
    x[0] = 1.5;
    x[1] = -3.5;
    x[2] = 2;
  }

  void callback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    for (int i = 0; i < uav_nu; i++) {
      u[i] = input->data[i];
    }
  }

  void rhs(const state_type &x, state_type &dxdt) {
    dxdt[0] = u[0];
    dxdt[1] = u[1];
    dxdt[2] = u[2];
    dxdt[3] = 0;
    dxdt[4] = 0;
    dxdt[5] = 0;
    dxdt[6] = 0;
    dxdt[7] = 0;
    dxdt[8] = 0;
    dxdt[9] = 0;
    dxdt[10] = 0;
    dxdt[11] = 0;
  }

  void integrate(double dt) {
    t = 0.0;
    boost::numeric::odeint::integrate(boost::bind(&UAVIntegrator::rhs, this, _1, _2),
                                      x, t, t+dt, 0.001);
    t += dt;
  }
  state_type x;
  state_type u;
};

class SegwayIntegrator {
public:
  SegwayIntegrator() {
    for (int i = 0; i < segway_nx; i++)
      x.push_back(0.);
    for (int i = 0; i < segway_nu; i++)
      u.push_back(0.);
    x[0] = 1.5;
    x[1] = -3.5;
    x[3] = M_PI;
  }

  void callback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    for (int i = 0; i < segway_nu; i++) {
      u[i] = input->data[i];
    }
  }

  void rhs(const state_type &x, state_type &dxdt) {
    /* x,y,v,theta,thetadot,psi,psidot  */
    dxdt[0] = u[0]*cos(x[3]);
    dxdt[1] = u[0]*sin(x[3]);
    dxdt[2] = 0;//u[0];
    dxdt[3] = u[1];
    dxdt[4] = 0;//u[1];
    dxdt[5] = x[6];
    dxdt[6] = 0;
  }

  void integrate(double dt) {
    t = 0.0;
    boost::numeric::odeint::integrate(boost::bind(&SegwayIntegrator::rhs, this, _1, _2),
                                      x, t, t+dt, 0.001);
    t += dt;
  }

  state_type x;
  state_type u;
};

class FlipperIntegrator {
public:
  FlipperIntegrator() {
    for (int i = 0; i < flipper_nx; i++)
      x.push_back(0.);
    for (int i = 0; i < flipper_nu; i++)
      u.push_back(0.);
    x[0] = 1.5;
    x[1] = -2.5;
    x[3] = M_PI;
  }

  void callback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    for (int i = 0; i < flipper_nu; i++) {
      u[i] = input->data[i];
    }
  }

  void rhs(const state_type &x, state_type &dxdt) {
        /* x,y,v,theta,thetadot */
    dxdt[0] = u[0]*cos(x[3]);
    dxdt[1] = u[0]*sin(x[3]);
    dxdt[2] = 0;
    dxdt[3] = u[1];
    dxdt[4] = 0;
  }

  void integrate(double dt) {
    t = 0.0;
    boost::numeric::odeint::integrate(boost::bind(&FlipperIntegrator::rhs, this, _1, _2),
                                      x, t, t+dt, 0.001);
    t += dt;
  }

  state_type x;
  state_type u;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "integrator");

  ros::NodeHandle n_;

  UAVIntegrator uavintegrator;
  SegwayIntegrator segwayintegrator;
  FlipperIntegrator flipperintegrator;

  ros::param::get("~_timeScale", timeScale);

  ros::Subscriber uav_sub_ = n_.subscribe("uav/inputs", 1, &UAVIntegrator::callback, &uavintegrator);
  ros::Subscriber segway_sub_ = n_.subscribe("segway/inputs", 1, &SegwayIntegrator::callback, &segwayintegrator);
  ros::Subscriber flipper_sub_ = n_.subscribe("flipper/inputs", 1, &FlipperIntegrator::callback, &flipperintegrator);
  ros::Publisher uav_pub_ = n_.advertise<std_msgs::Float32MultiArray>("uav/states", 1);
  ros::Publisher segway_pub_ = n_.advertise<std_msgs::Float32MultiArray>("segway/states", 1);
  ros::Publisher flipper_pub_ = n_.advertise<std_msgs::Float32MultiArray>("flipper/states", 1);

  tf::TransformBroadcaster uav_odom_broadcaster;
  tf::TransformBroadcaster segway_odom_broadcaster;
  tf::TransformBroadcaster flipper_odom_broadcaster;

  std_msgs::Float32MultiArray uav_states_msg;

  uav_states_msg.data.clear();
  uav_states_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  uav_states_msg.layout.dim[0].size = uav_nx;
  for (int i = 0; i < uav_nx; i++) {
    uav_states_msg.data.push_back(0.0);
  }

  std_msgs::Float32MultiArray segway_states_msg;

  segway_states_msg.data.clear();
  segway_states_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  segway_states_msg.layout.dim[0].size = segway_nx;
  for (int i = 0; i < segway_nx; i++) {
    segway_states_msg.data.push_back(0.0);
  }

  std_msgs::Float32MultiArray flipper_states_msg;

  flipper_states_msg.data.clear();
  flipper_states_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  flipper_states_msg.layout.dim[0].size = flipper_nx;
  for (int i = 0; i < flipper_nx; i++) {
    flipper_states_msg.data.push_back(0.0);
  }

  int rate = 200*timeScale;
  ros::Rate r(rate); // 10 hz

  ROS_INFO("integrator is ready");

  while (ros::ok()) {

    uavintegrator.integrate(1./(rate/timeScale));
    for (int i = 0; i < uav_nx; ++i)
    {
      uav_states_msg.data[i] = uavintegrator.x[i];
    }
    uav_pub_.publish(uav_states_msg);
    ros::Time current_time = ros::Time::now();
    geometry_msgs::Quaternion uav_odom_quat = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::TransformStamped uav_odom_trans;
    uav_odom_trans.header.stamp = current_time;
    uav_odom_trans.header.frame_id = "world";
    uav_odom_trans.child_frame_id = "uav/base_link";
    uav_odom_trans.transform.translation.x = uavintegrator.x[0];
    uav_odom_trans.transform.translation.y = uavintegrator.x[1];
    uav_odom_trans.transform.translation.z = uavintegrator.x[2];
    uav_odom_trans.transform.rotation = uav_odom_quat;
    uav_odom_broadcaster.sendTransform(uav_odom_trans);

    segwayintegrator.integrate(1./(rate/timeScale));
    for (int i = 0; i < uav_nx; ++i)
    {
      segway_states_msg.data[i] = segwayintegrator.x[i];
    }
    segway_pub_.publish(segway_states_msg);
    geometry_msgs::Quaternion segway_odom_quat = tf::createQuaternionMsgFromYaw(segwayintegrator.x[3]);
    geometry_msgs::TransformStamped segway_odom_trans;
    segway_odom_trans.header.stamp = current_time;
    segway_odom_trans.header.frame_id = "world";
    segway_odom_trans.child_frame_id = "segway/base_link";
    segway_odom_trans.transform.translation.x = segwayintegrator.x[0];
    segway_odom_trans.transform.translation.y = segwayintegrator.x[1];
    segway_odom_trans.transform.translation.z = 0;
    segway_odom_trans.transform.rotation = segway_odom_quat;
    segway_odom_broadcaster.sendTransform(segway_odom_trans);

    flipperintegrator.integrate(1./(rate/timeScale));
    for (int i = 0; i < uav_nx; ++i)
    {
      flipper_states_msg.data[i] = flipperintegrator.x[i];
    }
    flipper_pub_.publish(flipper_states_msg);
    geometry_msgs::Quaternion flipper_odom_quat = tf::createQuaternionMsgFromYaw(flipperintegrator.x[3]);
    geometry_msgs::TransformStamped flipper_odom_trans;
    flipper_odom_trans.header.stamp = current_time;
    flipper_odom_trans.header.frame_id = "world";
    flipper_odom_trans.child_frame_id = "flipper/base_link";
    flipper_odom_trans.transform.translation.x = flipperintegrator.x[0];
    flipper_odom_trans.transform.translation.y = flipperintegrator.x[1];
    flipper_odom_trans.transform.translation.z = 0;
    flipper_odom_trans.transform.rotation = flipper_odom_quat;
    flipper_odom_broadcaster.sendTransform(flipper_odom_trans);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
