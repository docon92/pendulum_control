#include "controller.h"

controller::controller( ros::NodeHandle& In_nh,
                        float in_k1,
                        float in_k2,
                        float in_k3,
                        float in_k4,
                        float in_k5,
                        float in_x3d,
                        float in_max_output)
{   
    CONTROLLER_INIT = false;

    nh = In_nh;
   position_sub = nh.subscribe<geometry_msgs::Pose2D>("pendulum/position", 5, &controller::handle_position,this);
   velocity_sub = nh.subscribe<geometry_msgs::Pose2D>("pendulum/velocity", 5, &controller::handle_velocity,this);
   force_output_pub = nh.advertise<std_msgs::Float64>("controller/output_force", 5);
 
    k1 = in_k1;
    k2 = in_k2;
    k3 = in_k3;
    k4 = in_k4;
    k5 = in_k5;

    max_F = in_max_output;

    x1=0.0f;
    x2=0.0f;
    x3=0.0f;
    x4=0.0f;
    x3i=0.0f;
    x3_0=0.0f;
    x3d = in_x3d;

    F = 0.0f;
    Fd = 0.0f;

    LastTimestamp = ros::Time::now();
}
controller::~controller(void){}


void controller::run (void)
{
    controller::step();

}

void controller::step( void )
{

    ros::Duration time_temp = ros::Time::now() - LastTimestamp;
    dt = time_temp.toSec();
    //ROS_INFO("dt is: %f seconds",dt);

    Fd = -k1*x1-k2*x2-k3*(x3-x3_0)-k4*x4-k5*x3i;
    F = saturate_output(Fd);
    x3i = x3i + (x3d-x3)*dt;
    //ROS_INFO("x3i is: %f",x3i);
    Force_msg.data = F;
    force_output_pub.publish(Force_msg);

    //ROS_INFO("Desired force is: %f",Fd);
    LastTimestamp = ros::Time::now();

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
