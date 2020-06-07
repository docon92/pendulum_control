#include "pendulum.h"
#include <cstdlib>


/*****************************************************************************/
/*****************************************************************************/
int main(int argc, char* argv[])
{

 

    /* Init "ROS" */
	ros::init(argc, argv, "pendulum_node");
	ros::NodeHandle NodeHandle;
    
    float theta_0 = 0.0;
    float x_0 = 0.0;
    float l = 1.0;
    float m = 2.0;
    float M = 3.0;
    float max_disturbance = 10.0;
    float sim_rate = 20.0;

//    NodeHandle.param("theta_0", theta_0);
//	NodeHandle.param("x_0", x_0, 0.0);
//	NodeHandle.param("l", l, 1.0);
//	NodeHandle.param("m", m, 1.0);
//    NodeHandle.param("m", m, 1.0);
//    NodeHandle.param("M", M, 2.0);
//    NodeHandle.param("sim_rate", sim_rate, 20.0);
    ros::Rate LoopRate(sim_rate);


    

    pendulum pend = pendulum(NodeHandle,theta_0,x_0,l,m,M,max_disturbance);

    while(ros::ok())
	{
        ROS_INFO("running!");
        pend.run();
        ros::spinOnce();
        LoopRate.sleep();
    }

    return EXIT_SUCCESS;
}