# pendulum_control
## Overview

This repo contains Source code and doc for an inverted pendulum assignment. It contains a ROS package called pendulum_control that defines the following processes:

- **pendulum_node** 
	A configurable linear state-space model for a simple inverted pendulum as defined in `prasad2014.pdf`, equation 19.The model also incorporates a disturbance input, which is defined as a random gaussian white noise input to the derivative of the angular rate. Length, cart mass and pendulum mass can be defined in configuration files found in `catkin_ws/src/pendulum_pkg/config`. Initial position and simulation rate can also be configured. Solving of the system is based on the Euler method.
-  **controller_node**
	A configurable state-feedback controller with integral action, designed with LQR techniques. The gains of the controller can be calculated by configuring the parameters of the pendulum model and running inverted_pedulum_lqr.m in MATLAB. Default values for these gains are found in the config files for each pendulum. The controller assumes that the cart position, cart velocity, pendulum angle and pendulum angular velocity are measured. This assumption can be relaxed with a state estimator, such as a Kalman Filter. Note that if you wish to modify any parameters of the pendulum system, a recalculation of the LQR gains with the new values for l,m and M would provide better results.
- **manager_node** 
	A ros node that subscribes to relevant data only of its immediate neighbours and commands the controller and pendulum to stop or start depending on the state of the other pendulums. It applies a decentralized approach te determine if the simulation is allowed to continue.
- **ui_node**
	A node that collects data on a pendulum and converts it into visualization markers that can be displayed using ROS' visualization tool rviz

## Requirements

- A machine running Ubuntu 18
- ROS melodic installed, find instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
- rviz installed ``sudo apt update && sudo apt install ros-melodic-rviz``

## Running the Simulation

The binaries for the nodes defined in this package have already been produced and can be found in `catkin_ws/install/`. `catkin_ws/install/launch/multi_pendulum.launch` is a launch script that will launch five pendulums and the ui. To run this script, open a terminal and run:

```
cd pendulum_control/catkin_ws
source install/setup.bash
roslaunch pendulum_pkg multi_pendulum.launch
```
You should see some prints in the terminal as well as the ui with all five pendulums present.

## Discussion

All nodes are coded in C++ using the ROS framework. Information (state data, simulation flags, etc.) is passed between processes using  [ROS' publishing and subscribing infrastructure](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers). The communication method (e.g. UDP, TCP) can either be automatically assigned by ROS or specified by the user. Each stream of information is represented by a ros topic that can either be subscribed or published to by each node.

### Assumptions
It is assumed that each pendulum group (pendulum, controller, manager, ui_bridge) has access to its own state data. It also knows the identity of its immediate neighbours and their position. Using this information we can create the behaviour as described in the original assignment.

### Communication Architecture

There are two main conditions that must be satisfied for each controller and pendulum to run:

- **The individual pendulum must be allowed to run** 

	Each pendulum and controller periodically checks a flag named PENDULUM_STOP
- The overall simulation must be enabled

Each pendulum and controller can only operate if

