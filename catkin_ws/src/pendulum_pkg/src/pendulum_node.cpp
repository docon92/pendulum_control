#include "pendulum.h"
#include <cstdlib>


/*****************************************************************************/
/*****************************************************************************/
int main(int argc, char* argv[])
{

 

    /* Init "ROS" */
	ros::init(argc, argv, "pendulum_node");
	ros::NodeHandle NodeHandle;
    
    float theta_default = 0.0;
    float theta_0, x_0, m, M, l, max_disturbance, sim_rate;
    float x_default = 0.0;
    float l_default = 1.0;
    float m_default = 1.0;
    float M_default = 5.0;
    float max_disturbance_default = 10.0;
    float sim_rate_default = 100.0;

    NodeHandle.param("theta_0", theta_0,theta_default);
	NodeHandle.param("x_0", x_0, x_default);
	NodeHandle.param("l", l, l_default);
	NodeHandle.param("m", m, m_default);
    NodeHandle.param("M", M, M_default);
    NodeHandle.param("max_disturbance", max_disturbance, max_disturbance_default);
    NodeHandle.param("sim_rate", sim_rate, sim_rate_default);
    
    ros::Rate LoopRate(sim_rate);


    pendulum pend = pendulum(NodeHandle,theta_0,x_0,l,m,M,max_disturbance);

    while(ros::ok())
	{
        //ROS_INFO("running!");
        pend.run();
        ros::spinOnce();
        LoopRate.sleep();
    }

    return EXIT_SUCCESS;
}