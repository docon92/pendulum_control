#include "pendulum.h"
#include <cstdlib>


/*****************************************************************************/
/*****************************************************************************/
int main(int argc, char* argv[])
{

 

    /* Init "ROS" */
	ros::init(argc, argv, "pendulum_node");
	ros::NodeHandle NodeHandle;
    
    float sim_rate;
    float sim_rate_default = 100.0;


    NodeHandle.param("sim_rate", sim_rate, sim_rate_default);
    
    ros::Rate LoopRate(sim_rate);


    pendulum pend = pendulum(NodeHandle);

    while(ros::ok())
	{
        //ROS_INFO("running!");
        pend.run();
        ros::spinOnce();
        LoopRate.sleep();
    }

    return EXIT_SUCCESS;
}