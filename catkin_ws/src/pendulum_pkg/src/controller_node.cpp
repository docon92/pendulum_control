#include "controller.h"
#include <cstdlib>


/*****************************************************************************/
/*****************************************************************************/
int main(int argc, char* argv[])
{

 

    /* Init "ROS" */
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle NodeHandle;
    

    float k1,k2,k3,k4,k5,max_F, ctrl_rate,x3d;
    float k1_default = -130.2;
    float k2_default = -38.18;
    float k3_default = -0.3162;
    float k4_default = -2.143;
    float k5_default = -0.0;
    float max_F_default = 10.0;
    float ctrl_rate_default = 40.0;
    float x3d_default = 0.0;

    NodeHandle.param("k1", k1,k1_default);
	NodeHandle.param("k2", k2, k2_default);
	NodeHandle.param("k3", k3, k3_default);
	NodeHandle.param("k4", k4, k4_default);
    NodeHandle.param("k5", k5, k5_default);
    NodeHandle.param("x3d", x3d, x3d_default);
    NodeHandle.param("max_F", max_F, max_F_default);
    NodeHandle.param("ctrl_rate", ctrl_rate, ctrl_rate_default);
    
    ros::Rate LoopRate(ctrl_rate);


    controller ctrl = controller(NodeHandle,k1,k2,k3,k4,k5,x3d,max_F);

    while(ros::ok())
	{
        //ROS_INFO("running!");
        ctrl.run();
        ros::spinOnce();
        LoopRate.sleep();
    }

    return EXIT_SUCCESS;
}