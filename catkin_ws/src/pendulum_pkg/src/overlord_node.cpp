#include <ros/ros.h>
#include "std_msgs/Int32.h"

/*****************************************************************************/
/*****************************************************************************/
ros::Subscriber status_1_sub;
ros::Subscriber status_2_sub;
ros::Subscriber status_3_sub;
ros::Subscriber status_4_sub;
ros::Subscriber status_5_sub;

ros::Publisher sim_enable_pub;

std_msgs::Int32 sim_enable_msg;

int RUN_ENABLE, stop_count, restart_count, pendulum_count;
int P1_STOP, P2_STOP, P3_STOP, P4_STOP, P5_STOP;
int P1_RESTART, P2_RESTART, P3_RESTART, P4_RESTART, P5_RESTART;

void handle_p1_stop (const std_msgs::Int32::ConstPtr& new_stop_msg)
{
    int stop_flag = new_stop_msg->data;
    if(stop_flag)
    {
        if(!P1_STOP)
        {
            P1_STOP = 1;
            stop_count+=1;
        }
    }
    else if (!RUN_ENABLE)
    {
        if(!P1_RESTART)
        {
            P1_RESTART = 1;
            restart_count+=1;
        }
    } 
    
}
void handle_p2_stop (const std_msgs::Int32::ConstPtr& new_stop_msg)
{
    int stop_flag = new_stop_msg->data;
    if(stop_flag)
    {
        if(!P2_STOP)
        {
            P2_STOP = 1;
            stop_count+=1;
        }
    }
    else if (!RUN_ENABLE)
    {
        if(!P2_RESTART)
        {
            P2_RESTART = 1;
            restart_count+=1;
        }
    } 
    
}
void handle_p3_stop (const std_msgs::Int32::ConstPtr& new_stop_msg)
{
    int stop_flag = new_stop_msg->data;
    if(stop_flag)
    {
        if(!P3_STOP)
        {
            P3_STOP = 1;
            stop_count+=1;
        }
    }
    else if (!RUN_ENABLE)
    {
        if(!P3_RESTART)
        {
            P3_RESTART = 1;
            restart_count+=1;
        }
    } 
    
}
void handle_p4_stop (const std_msgs::Int32::ConstPtr& new_stop_msg)
{
    int stop_flag = new_stop_msg->data;
    if(stop_flag)
    {
        if(!P4_STOP)
        {
            P4_STOP = 1;
            stop_count+=1;
        }
    }
    else if (!RUN_ENABLE)
    {
        if(!P4_RESTART)
        {
            P4_RESTART = 1;
            restart_count+=1;
        }
    } 
    
}
void handle_p5_stop (const std_msgs::Int32::ConstPtr& new_stop_msg)
{
    int stop_flag = new_stop_msg->data;
    if(stop_flag)
    {
        if(!P5_STOP)
        {
            P5_STOP = 1;
            stop_count+=1;
        }
    }
    else if (!RUN_ENABLE)
    {
        if(!P5_RESTART)
        {
            P5_RESTART = 1;
            restart_count+=1;
        }
    } 
    
}

int main(int argc, char* argv[])
{
    RUN_ENABLE = 1;
    stop_count = 0;
    restart_count = 0;
    P1_STOP = 0;
    P2_STOP = 0;
    P3_STOP = 0;    
    P4_STOP = 0;
    P5_STOP = 0;
    P1_RESTART = 0;
    P2_RESTART = 0;
    P3_RESTART = 0;    
    P4_RESTART = 0;
    P5_RESTART = 0;
    pendulum_count = 5;
    /* Init "ROS" */
	ros::init(argc, argv, "overlord_node");
	ros::NodeHandle NodeHandle;
    ros::Rate LoopRate(3.0);



    float overlord_rate;
    float overlord_rate_default = 5.0;
    NodeHandle.param("overlord", overlord_rate, overlord_rate_default);

    status_1_sub = NodeHandle.subscribe<std_msgs::Int32>("/p1/manager/stop_pendulum", 5, handle_p1_stop); 
    status_2_sub = NodeHandle.subscribe<std_msgs::Int32>("/p2/manager/stop_pendulum", 5, handle_p2_stop); 
    status_3_sub = NodeHandle.subscribe<std_msgs::Int32>("/p3/manager/stop_pendulum", 5, handle_p3_stop); 
    status_4_sub = NodeHandle.subscribe<std_msgs::Int32>("/p4/manager/stop_pendulum", 5, handle_p4_stop); 
    status_5_sub = NodeHandle.subscribe<std_msgs::Int32>("/p5/manager/stop_pendulum", 5, handle_p5_stop); 

    sim_enable_pub = NodeHandle.advertise<std_msgs::Int32>("/sim_enable", 5);

    while(ros::ok())
    {

        if(stop_count==5) RUN_ENABLE=0;
        if(restart_count==5)
        {
            RUN_ENABLE=1;
            stop_count = 0;
            restart_count = 0;
            ROS_INFO("Restarting sim! ");
        } 
        //ROS_INFO("stop_count is: %d ", stop_count);
        sim_enable_msg.data = RUN_ENABLE;
        sim_enable_pub.publish(sim_enable_msg);
        ros::spinOnce();
        LoopRate.sleep();

    }

    return EXIT_SUCCESS;
}