#include "controller.h"

controller::controller( ros::NodeHandle& In_nh,
                        float in_k1,
                        float in_k2,
                        float in_k3,
                        float in_k4,
                        float in_k5,
                        float in_max_output)
{
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

    F = 0.0f;

    LastTimestamp = ros::Time::now();
}
controller::~controller(void){}

void controller::run (void)
{
    ROS_INFO("Running!");
}

void controller::step( void )
{

}

float controller::saturate_output (float commanded_force)
{
    return 0.0;
}

void controller::handle_position (const geometry_msgs::Pose2D::ConstPtr& new_position)
{

}
void controller::handle_velocity (const geometry_msgs::Pose2D::ConstPtr& new_velocity)
{

}
