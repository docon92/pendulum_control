#include "pendulum.h"

 pendulum::pendulum(  ros::NodeHandle& In_nh,
                      float theta_0,
                      float x_0,
                      float length,
                      float m_pendulum,
                      float M_cart)
{

  float g = 9.807;
  nh = In_nh;

  x1 = theta_0;
  x2 = 0.0;
  x3 = x_0;
  x4 = 0.0;
  delta_x1 = 0.0;
  delta_x2 = 0.0;
  delta_x3 = 0.0;
  delta_x4 = 0.0;
  
  l=length;
  m = m_pendulum;
  M = M_cart;
  
  LastTimestamp = ros::Time::now();
  dt = 0.0; 

  c1 = (M+m)*g/(M*l);
  c2 = -m*g/M;
  c3 = -1.0/(M*l);
  c4 = 1.0/M;

  F = 0.0;
  disturbance = 0.0;

  ROS_INFO("Initialized a pendulum!");
}

 pendulum::~pendulum(){}

void pendulum::run (void)
{
  pendulum::step();
}

void pendulum::step( void )
{
  ros::Duration time_temp = ros::Time::now() - LastTimestamp;
  dt = time_temp.toSec();
  //ROS_INFO("dt is: %f seconds",dt);

  u = F + disturbance;

  delta_x1 = x2*dt;
  delta_x2 = (c1*x1+c3*u)*dt;
  delta_x3 = (x4)*dt;
  delta_x4 = (c2*x1+c4*u)*dt;

  x1 += delta_x1;
  x2 += delta_x2;
  x3 += delta_x3;
  x4 += delta_x4;


  LastTimestamp = ros::Time::now();


}