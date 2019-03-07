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

const uint32_t nx = 9;
const uint32_t nu = 4;

double m_boat = 1000;
double I_boat = 1000;
double Rx = 3;
double Ry = 1;

double a1 = -1.33767;
double a2 = 0.00633;
double a3 = 0.02094;
double a4 = (-0.00501);
double u1 = 0.00161; ////

double b1 = -2;
double b2 = -0.15;
double u2 = -0.00047282;

double c2 = -.4;
double u3 = 10*(0.000014592);

class Integrator {
public:
  Integrator() {
    x = Eigen::VectorXd::Zero(nx);
    u = Eigen::VectorXd::Zero(nu);
  }

  void callback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    for (int i = 0; i < nu; i++) {
      u[i] = input->data[i];
    } 
  }

  void rhs(const state_type &x, state_type &dxdt) {
    dxdt[0] = cos(x[2])*x[3]-sin(x[2])*x[4];
    dxdt[1] = sin(x[2])*x[3]+cos(x[2])*x[4];
    dxdt[2] = x[5];

    // Eigen::Matrix<double, nx, nu> gx;
    // gx.setZero();
    // gx(3,0) = cos(x[2])/m_boat; gx(3,1) = -sin(x[2])/m_boat; gx(3,2) = cos(x[2])/m_boat; gx(3,3) = -sin(x[2])/m_boat;
    // gx(4,0) = sin(x[2])/m_boat; gx(4,1) = cos(x[2])/m_boat; gx(4,2) = sin(x[2])/m_boat; gx(4,3) = cos(x[2])/m_boat;
    // gx(5,0) = 1/I_boat; gx(5,1) = -3/I_boat; gx(5,2) = -1/I_boat; gx(5,3) = -3/I_boat;
    // Eigen::VectorXd gxu = gx*u;
    // dxdt[3] = gxu[3];
    // dxdt[4] = gxu[4];
    // dxdt[5] = gxu[5];
    
    // double w1x = u[0];
    // double w1y = u[1];
    // double w2x = u[2];
    // double w2y = u[3];
    // dxdt[3] =  -1.33767*x[3] + 0.00633*x[3]*abs(x[3]) + 0.02094*x[4]*x[5] + (-0.00501)*x[5]*x[5] + (0.00161)*(w1x + w2x);
    // dxdt[4] =   (-2.000000000)*x[4] + (-0.150000000)*x[5] + (-0.000472822)*(w1y + w2y);
    // double Nbar = -Rx*(w1y+w2y)+Ry*(w1x-w2x);
    // dxdt[5] =  (-0.400000000)*x[5] + 10*(0.000014592)*Nbar;


    dxdt[3] = a1*x[3] + a2*x[3]*abs(x[3]) + a3*x[4]*x[5] + a4*x[5]*x[5] + u1*(u[0]*cos(x[6])+u[2]*cos(x[7]));
    dxdt[4] = b1*x[4] + b2*x[5] + u2*(u[0]*sin(x[6])+u[2]*sin(x[7]));
    double Nbar = -Rx*(u[0]*sin(x[6])+u[2]*sin(x[7]))+Ry*(u[0]*cos(x[6])-u[2]*cos(x[7]));
    dxdt[5] = c2*x[5] + u3*Nbar;
    dxdt[6] = u[1]-x[6];
    dxdt[7] = u[3]-x[7];
    dxdt[8] = 1;

    double fx[nx];
    fx[0] = cos(x[2])*x[3]-sin(x[2])*x[4];
    fx[1] = sin(x[2])*x[3]+cos(x[2])*x[4];
    fx[2] = x[5];
    fx[3] = a1*x[3] + a2*x[3]*abs(x[3]) + a3*x[4]*x[5] + a4*x[5]*x[5];
    fx[4] = b1*x[4] + b2*x[5];
    fx[5] = c2*x[5];
    fx[6] = -x[6];
    fx[7] = -x[7];
    fx[8] = 1;

    Eigen::Matrix<double, nx, nu> gx;
    gx.setZero();
    gx(3,0) = u1*cos(x[6]); gx(3,1) = 0; gx(3,2) = u1*cos(x[7]); gx(3,3) = 0;
    gx(4,0) = u2*sin(x[6]); gx(4,1) = 0; gx(4,2) = u2*sin(x[7]); gx(4,3) = 0;
    gx(5,0) = -u3*Rx*sin(x[6])+u3*Ry*cos(x[6]); gx(5,1) = 0; gx(5,2) = -u3*Rx*sin(x[7])-u3*Ry*cos(x[7]); gx(5,3) = 0;
    gx(6,0) = 0; gx(6,1) = 1; gx(6,2) = 0; gx(6,3) = 0;
    gx(7,0) = 0; gx(7,1) = 0; gx(7,2) = 0; gx(7,3) = 1;
    Eigen::VectorXd gxu = gx*u;
    // ROS_INFO("1: %f %f %f %f %f %f %f %f %f",dxdt[0],dxdt[1],dxdt[2],dxdt[3],dxdt[4],dxdt[5],dxdt[6],dxdt[7],dxdt[8]);
    // ROS_INFO("2: %f %f %f %f %f %f %f %f %f",fx[0],fx[1],fx[2],fx[3]+gxu[3],fx[4]+gxu[4],fx[5]+gxu[5],fx[6]+gxu[6],fx[7]+gxu[7],fx[8]+gxu[8]);

  }

  void integrate(double dt) {
    t = 0.0;
    boost::numeric::odeint::integrate(boost::bind(&Integrator::rhs, this, _1, _2), 
                                      x, t, t+dt, 0.001);
    t += dt;
  }

  state_type x;
  state_type u;
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