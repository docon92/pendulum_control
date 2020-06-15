#include "controller.h"
#include <cstdlib>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle NodeHandle;
    
    float ctrl_rate;
    float ctrl_rate_default = 40.0;
    NodeHandle.param("ctrl_rate", ctrl_rate, ctrl_rate_default);

    controller ctrl = controller(NodeHandle,ctrl_rate);
    ctrl.run();

    return EXIT_SUCCESS;
}