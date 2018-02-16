# RobotPathPlanningPSO

## Mobile Robot Path Planning Using Particle Swarm Optimization

Evan Krell & Arun Prassanth R.B.

This project explores using [particle swarm optimization (PSO)](https://en.wikipedia.org/wiki/Particle_swarm_optimization) for mobile robot path planning. 
In this system, a simulated [Turtlebot](http://www.turtlebot.com/) is able to generate a map of the environment which it
can then use to generate a path from its current position to a user-specified target position. A very high-level overview of the system flow is summarized in the 
System Flow diagram, below. 

![alt text][system_flow]

[system_flow]: doc/img/system_flow.png "System Flow"

The [Gazebo simulator](http://gazebosim.org/) is used for the Turtlebot and its environment. 
A virtual machine with the environment used is available [here](https://www.mathworks.com/supportfiles/robotics/ros/virtual_machines/v3/installation_instructions.htm).
The Turtlebot is controlled through a MATLAB script on a Windows computer. Communication is handled by [ROS](http://www.ros.org/), 
which is supported by MATLAB's [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html). 
Path planning is done on a separate Linux server. The MATLAB script coordinates the Turtlebot's activity, sends generated occupancy maps to the path planning server, 
and uses the solution paths to direct waypoint following. This is summarized in the System Overview diagram, below. 

![alt text][system_overview]

[system_overview]: doc/img/system_overview.png "System Overview"

