#include "manager.h"
#include <cstdlib>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "manager_node");
	ros::NodeHandle NodeHandle;
    
    float manager_rate;
    float manager_rate_default = 3.0;

    NodeHandle.param("manager_rate", manager_rate, manager_rate_default);
    manager mngr = manager(NodeHandle,manager_rate);
    mngr.run();
      
    return EXIT_SUCCESS;
}