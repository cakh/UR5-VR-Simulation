# UR5-VR-Simulation
This repository contains instructions and codes to visualise the movement of a UR5 (Universal Robots) robot in a virtual reality headset. The robot can be either a real robot connected to a PC or the simulation of UR5 running in virtualbox (ursim). In this repository, the term robot refers to both real as well as simulated robot.
## Communication between UR5 and VR-Headset in a nutshell
### Connecting UR5 to ROS
The robot is connected to a PC running [ROS](https://wiki.ros.org/noetic) (referred to as ROS-PC in this repository). The [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) is used to establish the communication between the robot and ROS. RViz is then used to visualise the current position of the UR5. With MoveIt! running in the background, the interactive marker at the TCP of the robot can be moved to a new position to set the goal position of the robot. At this stage, by clicking on *plan & execute* on RViz, ROS sends the goal position to the robot controller, which then plans a path and moves the joints of the robot. During this movement, the controller streams the joint values (position, velocity and acceleration of each joint) to ROS, which are used to represent the movement of the robot in RViz.

This connection between UR5 and ROS has been fully implemented in the [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) repository which also includes detailed description on the connection between the robot and ROS.

After establishing the connection between UR5 and ROS, a communication between ROS and Unity has to be established.
### Connecting ROS to Unity
