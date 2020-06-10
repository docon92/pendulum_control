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
  RUN_ENABLE = 1;
  PENDULUM_STOP = 0;
  pendulum_stop_msg.data = PENDULUM_STOP;
  x_neighbour_1 = 1.0E9;
  x_neighbour_2 = 1.0E9;
  x = 0.0;
  
  dist_n1 = 1.0E9;
  dist_n2 = 1.0E9;
  LastTimestamp = ros::Time::now();
  dt = 0.0; 

  //ROS_INFO("Initialized a manager!");
}

void manager::run (void)
{

  manager::step();

}

void manager::handle_sim_enable (const std_msgs::Int32::ConstPtr& new_sim_enable)
{
  RUN_ENABLE = new_sim_enable->data;
}

void manager::handle_neighbour_1_pos (const geometry_msgs::Pose2D::ConstPtr& new_neighbour_pos)
{
  //ROS_INFO("got neighbour 1 position!");
  x_neighbour_1 = new_neighbour_pos->x;
}

void manager::handle_neighbour_2_pos (const geometry_msgs::Pose2D::ConstPtr& new_neighbour_pos)
{
  //ROS_INFO("got neighbour 2 position!");
  x_neighbour_2 = new_neighbour_pos->x;
}

void manager::handle_position (const geometry_msgs::Pose2D::ConstPtr& new_position)
{
  x = new_position->x;
}

void manager::step( void )
{
    //ROS_INFO("running manager!");
  dist_n1 = x-x_neighbour_1;
  dist_n2 = x-x_neighbour_2;
  //ROS_INFO("distances: %f, %f", dist_n1,dist_n2);
  if(abs(dist_n1)<distance_threshold || abs(dist_n2)<distance_threshold )
  {
    //ROS_INFO("Pendulum is too close!");
    PENDULUM_STOP = 1;
    pendulum_stop_msg.data = PENDULUM_STOP;
  }
	status_pub.publish(pendulum_stop_msg);

}

void manager::wait (void)
{

}

void manager::connect_to_neighbours(void)
{
  

  std::string def = "VOID";
  float distance_threshold_default = 0.8;
  nh.param("neighbour_1", neighbour_1,def);
  nh.param("neighbour_2", neighbour_2, def);
  nh.param("distance_threshold", distance_threshold, distance_threshold_default);
	
  //ROS_INFO("neighbour_1 is: %s", neighbour_1.c_str());
  //ROS_INFO("neighbour_2 is: %s", neighbour_2.c_str());


  if(neighbour_1 == "VOID")
  {
    ROS_INFO("This pendulum has no neighbour_1!");
  }
  else
  {
    std::string topic_name =  "/" + neighbour_1 + "/pendulum/position";
    neighbour_1_sub = nh.subscribe<geometry_msgs::Pose2D>(
      topic_name, 5, &manager::handle_neighbour_1_pos,this);

      ROS_INFO("nb 1 sub topic is: %s", topic_name.c_str());
  }


 if(neighbour_2 == "VOID")
  {
    ROS_INFO("This pendulum has no neighbour_1!");
  }
  else
  {
    std::string topic_name = "/" + neighbour_2 + "/pendulum/position";
    neighbour_2_sub = nh.subscribe<geometry_msgs::Pose2D>(
      topic_name, 5, &manager::handle_neighbour_2_pos,this);
    ROS_INFO("nb 2 sub topic is: %s", topic_name.c_str());
  }


  position_sub = nh.subscribe<geometry_msgs::Pose2D>("pendulum/position", 5, &manager::handle_position,this);
  stop_sim_sub = nh.subscribe<std_msgs::Int32>("/sim_enable", 5, &manager::handle_sim_enable,this);
  status_pub = nh.advertise<std_msgs::Int32>("manager/stop_pendulum", 5);
  
}