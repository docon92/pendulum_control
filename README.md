pendulum_control
================

This repo contains Source code and doc for an inverted pendulum assignment. It contains a ROS package called pendulum_control that defines the following processes:

	* ``pendulum_node`` A configurable linear state-space model for a simple inverted pendulum as defined in ``prasad2014.pdf``, equation 19.The model also incorporates a disturbance input, which is defined as a random gaussian white noise input to the derivative of the angular rate. Length, cart mass and pendulum mass can be defined in configuration files found in ``catkin_ws/src/pendulum_pkg/config``. Initial position and simulation rate can also be configured. Solving of the system is based on the Euler method.
	* ``controller_node`` A configurable state-feedback controller with integral action, designed with LQR techniques. The gains of the controller can be calculated by configuring the parameters of the pendulum model and running inverted_pedulum_lqr.m in MATLAB. The controller assumes that the cart position, cart velocity, pendulum angle and pendulum angular velocity are measured. This assumption can be relaxed with a state estimator, such as a Kalman Filter.
	* ``manager_node`` A ros node that subscribes to relevant data only of its immediate neighbours and commands the controller and pendulum to stop or start depending on the state of the other pendulums. It applies a decentralized approach te determine if the simulation is allowed to continue.
	* ``ui_node`` A node that collects data on a pendulum and converts it into visualization markers that can be displayed using ROS' visualization tool rviz
