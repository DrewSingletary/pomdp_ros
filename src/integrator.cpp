#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <stdlib.h>
#include <math.h> 

typedef Eigen::VectorXd state_type;


double t;
double timeScale = 1;

class UAVIntegrator {
public:
  UAVIntegrator() {
    x = Eigen::VectorXd::Zero(nx);
    u = Eigen::VectorXd::Zero(nu);
  }

  void callback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    for (int i = 0; i < nu; i++) {
      u[i] = input->data[i];
    } 
  }

  void rhs(const state_type &x, state_type &dxdt) {
    dxdt[0] = x[6];
    dxdt[1] = x[7];
    dxdt[2] = x[8];
    dxdt[3] = x[9];
    dxdt[4] = x[10];
    dxdt[5] = x[11];
    dxdt[6] = 0;
    dxdt[7] = 0;
    dxdt[8] = 0;
    dxdt[9] = 0;
    dxdt[10] = 0;
    dxdt[11] = 0;
  }

  void integrate(double dt) {
    t = 0.0;
    boost::numeric::odeint::integrate(boost::bind(&Integrator::rhs, this, _1, _2), 
                                      x, t, t+dt, 0.001);
    t += dt;
  }
  state_type x;
  state_type u;
  const int nx = 12;
  const int nu = 4;
};

class SegwayIntegrator {
public:
  SegwayIntegrator() {
    x = Eigen::VectorXd::Zero(nx);
    u = Eigen::VectorXd::Zero(nu);
  }

  void callback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    for (int i = 0; i < nu; i++) {
      u[i] = input->data[i];
    } 
  }

  void rhs(const state_type &x, state_type &dxdt) {
    /* x,y,v,theta,thetadot,psi,psidot  */
    dxdt[0] = x[2]*cos(x[3]);
    dxdt[1] = x[2]*sin(x[3]);
    dxdt[2] = 0;
    dxdt[3] = x[4];
    dxdt[4] = 0;
    dxdt[5] = x[6];
    dxdt[6] = 0;
  }

  void integrate(double dt) {
    t = 0.0;
    boost::numeric::odeint::integrate(boost::bind(&Integrator::rhs, this, _1, _2), 
                                      x, t, t+dt, 0.001);
    t += dt;
  }

  state_type x;
  state_type u;
  const int nx = 7;
  const int nu = 2;
};

class FlipperIntegrator {
public:
  UAVIntegrator() {
    x = Eigen::VectorXd::Zero(nx);
    u = Eigen::VectorXd::Zero(nu);
  }

  void callback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    for (int i = 0; i < nu; i++) {
      u[i] = input->data[i];
    } 
  }

  void rhs(const state_type &x, state_type &dxdt) {
        /* x,y,v,theta,thetadot */
    dxdt[0] = x[2]*cos(x[3]);
    dxdt[1] = x[2]*sin(x[3]);
    dxdt[2] = 0;
    dxdt[3] = x[4];
    dxdt[4] = 0;
  }

  void integrate(double dt) {
    t = 0.0;
    boost::numeric::odeint::integrate(boost::bind(&Integrator::rhs, this, _1, _2), 
                                      x, t, t+dt, 0.001);
    t += dt;
  }

  state_type x;
  state_type u;
  const int nx = 5;
  const int nu = 2;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "integrator");

  ros::NodeHandle n_;

  Integrator integrator;

  ros::param::get("~_timeScale", timeScale);

  ros::Subscriber sub_ = n_.subscribe("inputs_actual", 1, &Integrator::callback, &integrator);
  ros::Publisher pub_ = n_.advertise<std_msgs::Float32MultiArray>("states", 1);
  tf::TransformBroadcaster odom_broadcaster;

  std_msgs::Float32MultiArray states_msg;

  states_msg.data.clear();
  states_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  states_msg.layout.dim[0].size = nx;
  for (int i = 0; i < nx; i++) {
    states_msg.data.push_back(0.0);
  }

  int rate = 200*timeScale;
  ros::Rate r(rate); // 10 hz

  ROS_INFO("integrator is ready");

  while (ros::ok()) {

    // integrate
    integrator.integrate(1./(rate/timeScale));

    // publish
    for (int i = 0; i < nx; ++i)
    {
      states_msg.data[i] = integrator.x[i];
    }

    pub_.publish(states_msg);
    
    ros::Time current_time = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(integrator.x[2]);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = integrator.x[0];
    odom_trans.transform.translation.y = integrator.x[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}