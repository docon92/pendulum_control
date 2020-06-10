#include "manager.h"
#include <cstdlib>


/*****************************************************************************/
/*****************************************************************************/
int main(int argc, char* argv[])
{

 

    /* Init "ROS" */
	ros::init(argc, argv, "manager_node");
	ros::NodeHandle NodeHandle;
    
    float manager_rate;
    float manager_rate_default = 100.0;


    NodeHandle.param("manager_rate", manager_rate, manager_rate_default);
    
    ros::Rate LoopRate(manager_rate);


    manager mngr = manager(NodeHandle);

    while(ros::ok())
	{
        //ROS_INFO("running!");
        mngr.run();
        ros::spinOnce();
        LoopRate.sleep();
    }

    return EXIT_SUCCESS;
}