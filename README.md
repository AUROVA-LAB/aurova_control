# aurova_control (DEPRECATED!!)
This is a metapackage that contains different packages that perform local control. This metapackage can contain controllers for different kind of vehicle. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "packagename_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**control_demo**
This package contains a node that, as input, reads the topics /amcl_pose of type geometry_msgs::PoseWithCovarianceStamped, and /move_base_simple/goal of type geometry_msgs::PoseStamped. This node calculates the orientation of the line it crosses /move_base_simple/goal, and calculates the error of orientation and distance of the robot (/amcl_pose) from it. With these error signals it performs a proportional control that generates a speed and a steering as an output. When the goal is reached, a request is made for the reception of a new goal. The node output is published in the topics /desired_ackermann_state of type ackermann_msgs::AckermannDriveStamped, and /request_goal of type std_msgs::Bool.
* ~control_demo/ku_d (default: 0.0): Steering proportional control adjustment constant for distance.
* ~control_demo/ku_a (default: 0.0): Steering proportional control adjustment constant for orientation.
* ~control_demo/v_base (default: 0.0): Minimum speed at which we want to circulate.
* ~control_demo/v_max (default: 0.0): Maximum speed at which we want to circulate.
* ~control_demo/max_steering (default: 0.0): Maximum steering, in absolute value, of our platform.
* ~time_out_wait_goal (default: 0.0): Maximum waiting time since the request for new goal. If this time is exceeded without receiving a new goal, the output values will be set to '0'.
* ~error_d_sat (default: 0.0): This parameter is for saturate the output signals.
