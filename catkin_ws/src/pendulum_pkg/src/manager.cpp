#include "manager.h"

 manager::manager(ros::NodeHandle& In_nh)
{
    nh = In_nh;
    connect_to_neighbours();
    init();
}

 manager::~manager(){}

void manager::init ()
{

    float theta_default = 0.0;
    float theta_0, x_0;
    float x_default = 0.0;
    float l_default = 1.0;
    float m_default = 1.0;
    float M_default = 5.0;
    float max_disturbance_default = 10.0;

    nh.param("theta_0", theta_0,theta_default);
	  nh.param("x_0", x_0, x_default);
	  nh.param("l", l, l_default);
	  nh.param("m", m, m_default);
    nh.param("M", M, M_default);
    nh.param("max_disturbance", max_disturbance, max_disturbance_default);
  
  LastTimestamp = ros::Time::now();
  dt = 0.0; 

  ROS_INFO("Initialized a manager!");
}

void manager::run (void)
{
  manager::step();
}

void manager::handle_neighbour_1_pos (const geometry_msgs::Pose2D::ConstPtr& new_neighbour_pos)
{
  
}

void manager::handle_neighbour_2_pos (const geometry_msgs::Pose2D::ConstPtr& new_neighbour_pos)
{
  
}

void manager::step( void )
{
  ros::Duration time_temp = ros::Time::now() - LastTimestamp;
  dt = time_temp.toSec();
 
	position_pub.publish(position_msg);
	velocity_pub.publish(velocity_msg);

  LastTimestamp = ros::Time::now();
  //ROS_INFO("Angle is: %f", x1);

}

void manager::wait (void)
{

}

void manager::connect_to_neighbours(void)
{
  nh.param("neighbour_1", neighbour_1);
  nh.param("neighbour_2", neighbour_2);
	
  if(neighbour_1 == "VOID")
  {
    ROS_INFO("This pendulum has no neighbour_1!");
  }
  else
  {
    std::string topic_name =  "/" + neighbour_1 + "/pendulum/position";
    neighbour_1_sub = nh.subscribe<geometry_msgs::Pose2D>(
      "controller/output_force", 5, &manager::handle_neighbour_1_pos,this);
  }


  position_sub = nh.subscribe<geometry_msgs::Pose2D>("pendulum/position", 5, &controller::handle_position,this);

}

 if(neighbour_2 == "VOID")
  {
    ROS_INFO("This pendulum has no neighbour_1!");
  }
  else
  {
    std::string topic_name = "/" + neighbour_2 + "/pendulum/position";
    neighbour_1_sub = nh.subscribe<geometry_msgs::Pose2D>(
      "controller/output_force", 5, &manager::handle_neighbour_1_pos,this);
  }


  position_sub = nh.subscribe<geometry_msgs::Pose2D>("pendulum/position", 5, &controller::handle_position,this);

}