#include "pendulum.h"

 pendulum::pendulum(ros::NodeHandle& In_nh)
{
    nh = In_nh;

    force_input_sub =
    nh.subscribe<std_msgs::Float64>("controller/output_force", 5, &pendulum::handle_force_input,this);
    position_pub = nh.advertise<geometry_msgs::Pose2D>("pendulum/position", 5);
    velocity_pub = nh.advertise<geometry_msgs::Pose2D>("pendulum/velocity", 5);

    init(nh);


}

 pendulum::~pendulum(){}

void pendulum::init (ros::NodeHandle& In_nh)
{

    float theta_default = 0.0;
    float theta_0, x_0;
    float x_default = 0.0;
    float l_default = 1.0;
    float m_default = 1.0;
    float M_default = 5.0;
    float max_disturbance_default = 10.0;

    In_nh.param("theta_0", theta_0,theta_default);
	  In_nh.param("x_0", x_0, x_default);
	  In_nh.param("l", l, l_default);
	  In_nh.param("m", m, m_default);
    In_nh.param("M", M, M_default);
    In_nh.param("max_disturbance", max_disturbance, max_disturbance_default);


  float g = 9.807;

  x1 = theta_0;
  x2 = 0.0;
  x3 = x_0;
  x4 = 0.0;
  delta_x1 = 0.0;
  delta_x2 = 0.0;
  delta_x3 = 0.0;
  delta_x4 = 0.0;
  
  // l=length;
  // m = m_pendulum;
  // M = M_cart;
  
  LastTimestamp = ros::Time::now();
  dt = 0.0; 

  c1 = (M+m)*g/(M*l);
  c2 = -m*g/M;
  c3 = -1.0/(M*l);
  c4 = 1.0/M;

  F = 0.0;
  // max_disturbance = max_dist;
  disturbance = 0.0;
  srand (static_cast <unsigned> (ros::Time::now().toSec()));
  //srand (static_cast <unsigned> (time(0)));
  ROS_INFO("Initialized a pendulum!");
}

void pendulum::run (void)
{
  pendulum::calculate_disturbance();
  pendulum::step();
}

void pendulum::handle_force_input (const std_msgs::Float64::ConstPtr& new_force_input)
{
  F = new_force_input->data;
}



/// Calclulate a random disturbance. Outputs a random float between -max_disturbance and + max_disturbance
/// rand() is initialized in the constructor
void pendulum::calculate_disturbance (void)
{
  
  disturbance = -max_disturbance + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX)/(2*max_disturbance));
  //ROS_INFO("disturbance is: %f",disturbance);
}

void pendulum::step( void )
{
  ros::Duration time_temp = ros::Time::now() - LastTimestamp;
  dt = time_temp.toSec();
  //ROS_INFO("dt is: %f seconds",dt);

  u = F;
  

  delta_x1 = x2*dt;
  delta_x2 = (c1*x1+c3*u + disturbance)*dt;
  delta_x3 = (x4)*dt;
  delta_x4 = (c2*x1+c4*u)*dt;

  x1 += delta_x1;
  x2 += delta_x2;
  x3 += delta_x3;
  x4 += delta_x4;
  
  position_msg.theta = x1;
  position_msg.x = x3;
  velocity_msg.theta = x2;
  velocity_msg.x =x4;
  

	position_pub.publish(position_msg);
	velocity_pub.publish(velocity_msg);

  LastTimestamp = ros::Time::now();
  //ROS_INFO("Angle is: %f", x1);

}

void pendulum::wait (void)
{

}
