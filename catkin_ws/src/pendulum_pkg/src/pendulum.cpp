#include "pendulum.h"

 pendulum::pendulum(ros::NodeHandle& In_nh,float rate)  : LoopRate(rate)
{
    
    ROS_INFO("rate is: %f", rate);
    nh = In_nh;



    run_enable_sub = nh.subscribe<std_msgs::Int32>("manager/sim_enable", 1, &pendulum::handle_run_enable,this,ros::TransportHints()                                                                             
                     .tcp()
                     .reliable()                                                                                       
                     .tcpNoDelay(true)); 
    pendulum_stop_sub = nh.subscribe<std_msgs::Int32>("manager/stop_pendulum", 1, &pendulum::handle_pendulum_stop,this,ros::TransportHints()                                                                             
                     .tcp()
                     .reliable()                                                                                       
                     .tcpNoDelay(true)); 
    force_input_sub = nh.subscribe<std_msgs::Float64>("controller/output_force", 5, &pendulum::handle_force_input,this);
    position_pub = nh.advertise<geometry_msgs::Pose2D>("pendulum/position", 5);
    velocity_pub = nh.advertise<geometry_msgs::Pose2D>("pendulum/velocity", 5);

        
    RUN_ENABLE = 1;
    PENDULUM_STOP = 0;
    init();


}

 pendulum::~pendulum(){}

void pendulum::init (void)
{

    float theta_default = 0.0;
    float theta_0, x_0;
    float x_default = 0.0;
    float l_default = 1.0;
    float m_default = 1.0;
    float M_default = 5.0;
    float max_disturbance_default = 10.0;
    int rand_seed_default = 2;

    nh.param("theta_0", theta_0,theta_default);
	  nh.param("x_0", x_0, x_default);
	  nh.param("l", l, l_default);
	  nh.param("m", m, m_default);
    nh.param("M", M, M_default);
    nh.param("max_disturbance", max_disturbance, max_disturbance_default);
    nh.param("rand_seed", rand_seed, rand_seed_default);


  float g = 9.807;

  x1 = theta_0;
  x2 = 0.0;
  x3 = x_0;
  x4 = 0.0;
  delta_x1 = 0.0;
  delta_x2 = 0.0;
  delta_x3 = 0.0;
  delta_x4 = 0.0;

  position_msg.theta = x1;
  position_msg.x = x3;
  velocity_msg.theta = x2;
  velocity_msg.x =x4;
  
  
  LastTimestamp = ros::Time::now();
  dt = 0.0; 

  c1 = (M+m)*g/(M*l);
  c2 = -m*g/M;
  c3 = -1.0/(M*l);
  c4 = 1.0/M;
  F = 0.0;
  disturbance = 0.0;
  time_t t;
  int seed = static_cast<unsigned> (time(&t))+rand_seed;
  srand(seed);
  //ROS_INFO("using random seed: %d",seed);
  //ROS_INFO("Initialized a pendulum!");
}

void pendulum::run (void)
{
      while(ros::ok())
	{

        if(RUN_ENABLE && !PENDULUM_STOP)
        {
          pendulum::calculate_disturbance();
          pendulum::step();
        }
        else if (!RUN_ENABLE && PENDULUM_STOP)
        {
            //ROS_WARN("INITIALIZING PENDULUM");
            pendulum::init();
        }
          
        pendulum::broadcast_state();
        LastTimestamp = ros::Time::now();
        ros::spinOnce();
        LoopRate.sleep();
  }

}

void pendulum::handle_force_input (const std_msgs::Float64::ConstPtr& new_force_input)
{
  F = new_force_input->data;
}

void pendulum::handle_run_enable (const std_msgs::Int32::ConstPtr& new_flag)
{
  RUN_ENABLE = new_flag->data;
  if (RUN_ENABLE==0)
    {ROS_INFO("Pendulum  disabled! run_enable: %d,pendulum_stop: %d",RUN_ENABLE,PENDULUM_STOP);}
}
void pendulum::handle_pendulum_stop (const std_msgs::Int32::ConstPtr& new_flag)
{
  PENDULUM_STOP = new_flag->data;
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
  
  //ROS_INFO("Angle is: %f", x1);
}

void pendulum::broadcast_state( void )
{
  position_pub.publish(position_msg);
	velocity_pub.publish(velocity_msg);
}

