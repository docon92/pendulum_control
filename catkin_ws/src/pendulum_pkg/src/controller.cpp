#include "controller.h"

controller::controller( ros::NodeHandle& In_nh,float rate) : LoopRate(rate)
{   
    CONTROLLER_INIT = false;
    RUN_ENABLE = 1;
    PENDULUM_STOP = 0;

    nh = In_nh;

   run_enable_sub = nh.subscribe<std_msgs::Int32>("/sim_enable", 5, &controller::handle_run_enable,this); 
   pendulum_stop_sub = nh.subscribe<std_msgs::Int32>("manager/stop_pendulum", 5, &controller::handle_pendulum_stop,this); 
   position_sub = nh.subscribe<geometry_msgs::Pose2D>("pendulum/position", 5, &controller::handle_position,this);
   velocity_sub = nh.subscribe<geometry_msgs::Pose2D>("pendulum/velocity", 5, &controller::handle_velocity,this);
   force_output_pub = nh.advertise<std_msgs::Float64>("controller/output_force", 5);
 
    float k1_default = -237.00;
    float k2_default = -72.4115;
    float k3_default = -22.8654;
    float k4_default = -26.1364;
    float k5_default = 10.0;
    float max_F_default = 10.0;

    float x3d_default = 0.0;

    nh.param("k1", k1,k1_default);
	  nh.param("k2", k2, k2_default);
	  nh.param("k3", k3, k3_default);
	  nh.param("k4", k4, k4_default);
    nh.param("k5", k5, k5_default);
    nh.param("x3d", x3d, x3d_default);
    nh.param("max_F", max_F, max_F_default);

    x1=0.0f;
    x2=0.0f;
    x3=0.0f;
    x4=0.0f;
    x3i=0.0f;
    x3_0=0.0f;


    F = 0.0f;
    Fd = 0.0f;

    LastTimestamp = ros::Time::now();
}
controller::~controller(void){}


void controller::run (void)
{

   while(ros::ok())
	{

    if(RUN_ENABLE && !PENDULUM_STOP)
        {
          controller::step();
        }
        else if (!RUN_ENABLE && PENDULUM_STOP)
        {
            x3i = 0.0;
            F = 0.0;
            x1=0.0f;
            x2=0.0f;
            x3=0.0f;
            x4=0.0f;
            Fd = 0.0f;
        }
          
        force_output_pub.publish(Force_msg);  
        LastTimestamp = ros::Time::now();
        ros::spinOnce();
        LoopRate.sleep();
    }



}

void controller::step( void )
{

    ros::Duration time_temp = ros::Time::now() - LastTimestamp;
    dt = time_temp.toSec();
    //ROS_INFO("dt is: %f, x3i is: %f",dt,x3i);

    Fd = -k1*x1-k2*x2-k3*(x3-x3_0)-k4*x4-k5*x3i;
    F = saturate_output(Fd);
    x3i += (x3d-x3)*dt;
    //ROS_INFO("x3i is: %f",x3i);
    Force_msg.data = F;

}


void controller::handle_position (const geometry_msgs::Pose2D::ConstPtr& new_position)
{

  x1 = new_position->theta;
  x3 = new_position->x;

 if (!CONTROLLER_INIT)
    {
       x3_0 = x3;
       CONTROLLER_INIT=true; 
    }
}
void controller::handle_velocity (const geometry_msgs::Pose2D::ConstPtr& new_velocity)
{
  x2 = new_velocity->theta;
  x4 = new_velocity->x;
}

void controller::handle_run_enable (const std_msgs::Int32::ConstPtr& new_flag)
{
  RUN_ENABLE = new_flag->data;
}
void controller::handle_pendulum_stop (const std_msgs::Int32::ConstPtr& new_flag)
{
  PENDULUM_STOP = new_flag->data;
}


float controller::sign_of_num(float num)
{ 
  if (num>=0.0) return 1.0;
  else return -1.0;
}

float controller::saturate_output (float cmd)
{ 
  if (cmd<max_F && cmd>-max_F) return cmd;
  else return max_F*sign_of_num(cmd);
}
