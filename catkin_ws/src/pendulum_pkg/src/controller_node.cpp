#include "controller.h"
#include <cstdlib>


/*****************************************************************************/
/*****************************************************************************/
int main(int argc, char* argv[])
{

 

    /* Init "ROS" */
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle NodeHandle;
    

    float k1,k2,k3,k4,k5,max_F, ctrl_rate;
    float k1_default = -1.0;
    float k2_default = -1.0;
    float k3_default = -1.0;
    float k4_default = -1.0;
    float k5_default = -1.0;
    float max_F_default = 10.0;
    float ctrl_rate_default = 10.0;

    NodeHandle.param("k1", k1,k1_default);
	NodeHandle.param("k2", k2, k2_default);
	NodeHandle.param("k3", k3, k3_default);
	NodeHandle.param("k4", k4, k4_default);
    NodeHandle.param("k5", k5, k5_default);
    NodeHandle.param("max_F", max_F, max_F_default);
    NodeHandle.param("ctrl_rate", ctrl_rate, ctrl_rate_default);
    
    ros::Rate LoopRate(ctrl_rate);


    controller ctrl = controller(NodeHandle,k1,k2,k3,k4,k5,max_F);

    while(ros::ok())
	{
        //ROS_INFO("running!");
        ctrl.run();
        ros::spinOnce();
        LoopRate.sleep();
    }

    return EXIT_SUCCESS;
}