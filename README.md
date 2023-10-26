# UR5-VR-Simulation
This repository contains instructions and codes to visualise the movement of a UR5 (Universal Robots) robot in a virtual reality headset. The robot can be either a real robot connected to a PC or the simulation of UR5 running in virtualbox (ursim). In this repository, the term robot refers to both real as well as simulated robot.
## Communication between UR5 and VR-Headset in a nutshell
### Connecting UR5 to ROS
The robot is connected to a PC running [ROS](https://wiki.ros.org/noetic) (referred to as ROS-PC in this repository). The [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) is used to establish the communication between the robot and ROS. RViz is then used to visualise the current position of the UR5. With MoveIt! running in the background, the interactive marker at the TCP of the robot can be moved to a new position to set the goal position of the robot. At this stage, by clicking on *plan & execute* on RViz, ROS sends the goal position to the robot controller, which then plans a path and moves the joints of the robot. During this movement, the controller streams the joint values (position, velocity and acceleration of each joint) to ROS, which are used to represent the movement of the robot in RViz.

This connection between UR5 and ROS has been fully implemented in the previously mentioned [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) repository which also includes detailed description on the connection between the robot and ROS.

After establishing the connection between UR5 and ROS, a communication between ROS and Unity has to be established.
### Connecting ROS to Unity within the ROS-PC
>This step describes how to connect ROS to Unity, and thus visualise the robot in Unity. But for viewing this robot in a VR headset, this step might not be sufficient for many users. The reason for this is two fold:
> - ROS usually runs on linux based systems. Many VR-headsets including the Meta Quest series do not have an official SDK for linux environment. This means, you can visualise the robot in Unity, but you might not be able to connect the VR-headset to unity.
> - The ROS-PC might not be VR ready: to view a Unity scene directly on a VR-headset, the PC running Unity should have high enough [specifications](https://help.irisvr.com/hc/en-us/articles/213711747-Recommended-VR-Ready-Computers).
>
> To solve these issues, the ROS-PC can be connected to a VR-ready PC (named Unity-PC in the repository) through an ethernet cable or router. The information regarding joint angles can then be sent from the ROS-PC to the Unity-PC to be processed in Unity. This processed information is used to manipulate the virtual robot in Unity and can thus be displayed on any VR headset.
> *In this section, Unity is installed inside the ROS-PC (linux) to visualise UR5. The next step describes installing Unity on the Unity-PC and the connection between the two PCs.

