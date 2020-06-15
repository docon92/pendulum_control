#include "manager.h"

 manager::manager(ros::NodeHandle& In_nh,float rate) : LoopRate(rate)
{
  nh = In_nh;

  pendulum_id = -1;
  if (!nh.getParam("pendulum_id", pendulum_id))
    {ROS_INFO("You must set the pendulum_id for this node!");}
  else ROS_INFO("Got pendulum_id: %d", pendulum_id);

  std_msgs::UInt16MultiArray init_msg;
  std::vector<short unsigned int> zeros;
  
  for(int i=0;i<5;i++)
    {zeros.push_back(0);}

  init_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
  init_msg.layout.dim[0].size = zeros.size();
  init_msg.layout.dim[0].stride = 1;
  init_msg.layout.dim[0].label = std::to_string(pendulum_id);

  std::vector<short unsigned int>::const_iterator itr, end(zeros.end());
    for(itr = zeros.begin(); itr!= end; ++itr) 
      {init_msg.data.push_back(*itr);}/**/

  status_array_msg = init_msg;

    SIM_ENABLE = 1;
    sim_enable_count = 0;
    stop_sim_count = 0;
    connect_to_neighbours();
    init();
}

 manager::~manager(){}

void manager::init ()
{
  PENDULUM_STOP = 0;
  pendulum_stop_msg.data = PENDULUM_STOP;
  status_array_msg.data[pendulum_id - 1] = PENDULUM_STOP; 
  x_neighbour_1 = 1.0E9;
  x_neighbour_2 = 1.0E9;
  x = 0.0;
  
  dist_n1 = 1.0E9;
  dist_n2 = 1.0E9;

  status_array_pub.publish(status_array_msg);
  pendulum_stop_pub.publish(pendulum_stop_msg);
  ros::Duration(1.0).sleep();
}

void manager::run (void)
{
  while(ros::ok())
  {
    manager::check_sim_status();
    
    if(SIM_ENABLE==1 && PENDULUM_STOP==0)
    {
      manager::step();
        //ROS_INFO("publishing status!");
        status_array_pub.publish(status_array_msg);
    }
    else if (SIM_ENABLE==0 && PENDULUM_STOP==1)
    {
      //ROS_WARN("PENDULUM %d DISABLED IMULATION,SIM_ENABLE: %d,PENDULUM_STOP: %d",pendulum_id,SIM_ENABLE,PENDULUM_STOP);
      pendulum_stop_msg.data = PENDULUM_STOP;
      sim_enable_msg.data = SIM_ENABLE;
      status_array_pub.publish(status_array_msg);
      sim_enable_pub.publish(sim_enable_msg);
      manager::wait();
      pendulum_stop_pub.publish(pendulum_stop_msg);
      manager::init();
      //ROS_WARN("PENDULUM %d DISABLED IMULATION,SIM_ENABLE: %d,PENDULUM_STOP: %d",pendulum_id,SIM_ENABLE,PENDULUM_STOP);
      pendulum_stop_msg.data = PENDULUM_STOP;
      pendulum_stop_pub.publish(pendulum_stop_msg);
      status_array_pub.publish(status_array_msg);
      while(!SIM_ENABLE && ros::ok())
      {
        status_array_pub.publish(status_array_msg);
        manager::manager::check_sim_status();
        ros::spinOnce();
        LoopRate.sleep();
      }

    }
    else
    {
       status_array_pub.publish(status_array_msg); 
    }
    
    sim_enable_msg.data = SIM_ENABLE;
    pendulum_stop_msg.data = PENDULUM_STOP; 
    sim_enable_pub.publish(sim_enable_msg);
    pendulum_stop_pub.publish(pendulum_stop_msg);
    //ROS_INFO("pendulum %d, SIM_ENABLE: %d, msg, %d,pstop: %d,pstop_msg: %d",pendulum_id,SIM_ENABLE,sim_enable_msg.data,PENDULUM_STOP,pendulum_stop_msg.data);
    ros::spinOnce();
    LoopRate.sleep();
  }
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

void manager::handle_stop_status(const std_msgs::UInt16MultiArray::ConstPtr& new_status)
{
  status_array_msg.layout.dim[0].size = new_status->layout.dim[0].size;
  if(SIM_ENABLE)
  {
    for(int i=0; i < status_array_msg.layout.dim[0].size; i++)
    {
      if(new_status->data[i])
        {status_array_msg.data[i] = 1;}
    }
  }
  else
  {
    for(int i=0; i < status_array_msg.layout.dim[0].size; i++)
    {
      if(!new_status->data[i])
        {status_array_msg.data[i] = 0;}
    }
  }
    status_array_msg.data[pendulum_id-1] = PENDULUM_STOP;
}

void manager::step( void )
{
  dist_n1 = x-x_neighbour_1;
  dist_n2 = x-x_neighbour_2;
  //ROS_INFO("distances: %f, %f", dist_n1,dist_n2);
  if(abs(dist_n1)<distance_threshold || abs(dist_n2)<distance_threshold )
  {
    //ROS_INFO("Pendulum is too close!");
    PENDULUM_STOP = 1;
    pendulum_stop_msg.data = PENDULUM_STOP;
    status_array_msg.data[pendulum_id-1] = PENDULUM_STOP; 
    status_array_pub.publish(status_array_msg);
  }
}

void manager::wait (void)
{
  time_t t;
  int seed = static_cast<unsigned> (time(&t))+rand_seed;
  srand(seed);
  float max_sleep =4.0;
  float sleep_time = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX)/(max_sleep));
  ROS_INFO("Waiting %f seconds to restart!",sleep_time);
  ros::Duration(sleep_time).sleep();
}

void manager::connect_to_neighbours(void)
{
  std::string def = "VOID";
  float distance_threshold_default = 0.8;
  int rand_seed_default = 22;

  nh.param("rand_seed", rand_seed, rand_seed_default);
  nh.param("neighbour_1", neighbour_1,def);
  nh.param("neighbour_2", neighbour_2, def);
  nh.param("distance_threshold", distance_threshold, distance_threshold_default);
	
  //ROS_INFO("neighbour_1 is: %s", neighbour_1.c_str());
  //ROS_INFO("neighbour_2 is: %s", neighbour_2.c_str());

  if(neighbour_1 == "VOID")
    {ROS_INFO("This pendulum has no neighbour_1!");}
  else
  {
    std::string pos_topic_name =  "/" + neighbour_1 + "/pendulum/position";
    std::string status_topic_name = "/" + neighbour_1 + "/manager/stop_status";

    neighbour_1_sub = nh.subscribe<geometry_msgs::Pose2D>(
                                                          pos_topic_name, 5,
                                                          &manager::handle_neighbour_1_pos,
                                                          this);
    stop_status_sub_n1 = nh.subscribe<std_msgs::UInt16MultiArray>(
                                                          status_topic_name, 1,
                                                          &manager::handle_stop_status,
                                                          this,
                                                          ros::TransportHints().reliable().tcpNoDelay(true));

     // ROS_INFO("nb 1 sub topic is: %s", topic_name.c_str());
  }

 if(neighbour_2 == "VOID")
  {ROS_INFO("This pendulum has no neighbour_1!");}
  else
  {
    std::string pos_topic_name = "/" + neighbour_2 + "/pendulum/position";
    std::string status_topic_name = "/" + neighbour_2 + "/manager/stop_status";
    
    neighbour_2_sub = nh.subscribe<geometry_msgs::Pose2D>(
                                                          pos_topic_name, 5,
                                                          &manager::handle_neighbour_2_pos,
                                                          this);
      
    stop_status_sub_n2 = nh.subscribe<std_msgs::UInt16MultiArray>(
                                                          status_topic_name, 1,
                                                          &manager::handle_stop_status,
                                                          this,
                                                          ros::TransportHints().reliable().tcpNoDelay(true));  
  }

  position_sub = nh.subscribe<geometry_msgs::Pose2D>("pendulum/position", 5, &manager::handle_position,this);
  pendulum_stop_pub = nh.advertise<std_msgs::Int32>("manager/stop_pendulum", 20);
  sim_enable_pub = nh.advertise<std_msgs::Int32>("manager/sim_enable", 20);
  status_array_pub = nh.advertise<std_msgs::UInt16MultiArray>("manager/stop_status", 1);
  ROS_INFO("initialized connections!");
}

  void manager::check_sim_status (void)
  {
    if(PENDULUM_STOP)
    { 
      int ALL_STOPPED = 1;
      for(int i=0; i < status_array_msg.layout.dim[0].size; i++)
      {
        if(status_array_msg.data[i]==0)
          ALL_STOPPED = 0;
      }
      if(ALL_STOPPED)
        {stop_sim_count += 1;}
      if(stop_sim_count==10)
      {
        SIM_ENABLE = 0;
        stop_sim_count = 0;
        ROS_INFO("Disabling simulation!");
      }  
    }

    if(!SIM_ENABLE)
    { 
      int ALL_READY = 1;
      for(int i=0; i < status_array_msg.layout.dim[0].size; i++)
      {
        if(status_array_msg.data[i]==1)
          {ALL_READY = 0;}
      }
      if(ALL_READY)
        {sim_enable_count += 1;}

      if(sim_enable_count==10)
      {
        SIM_ENABLE = 1;
        sim_enable_count = 0;
        ROS_INFO("Enabling simulation!");
      }
    }
  }