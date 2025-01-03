# MotionPlanner
This is a ROS2 motion planner implementation designed to receive target position(s) and navigate a robot through those positions. A path is created using spline interpolation through requested 3D target positions and velocity targets are generated to move the robot along the path. The motion planner exposes an action server for navigation of holonomic vehicles assuming no dependency between linear and angular velocities. During movement, the robot's orientation can either be 1) along the direction of travel (or an offset to it), 2) constant orientation, or 3) spherically interpolated between target position orientations. 

This code was developed apart of a larger project at the University of Alberta to design and compete an autonomous underwater vehicle.
