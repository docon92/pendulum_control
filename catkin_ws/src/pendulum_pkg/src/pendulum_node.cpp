#include "pendulum.h"
#include <cstdlib>


/*****************************************************************************/
/*****************************************************************************/
int main(int argc, char* argv[])
{
    /* Init "ROS" */
	ros::init(argc, argv, "pendulum_node");
	ros::NodeHandle NodeHandle;

    float rate;
    float sim_rate_default = 100.0;
    NodeHandle.param("sim_rate", rate, sim_rate_default);


    pendulum pend = pendulum(NodeHandle,rate);

    //ROS_INFO("running!");
    pend.run();
    return EXIT_SUCCESS;
}