# UR5-VR-Simulation
This repository contains instructions and codes to visualise the movement of a UR5 (Universal Robots) robot in a virtual reality headset. The robot can be either a real robot connected to a PC or the simulation of UR5 running in virtualbox (ursim). In this repository, the term robot refers to both real as well as simulated robot.
## Communication between UR5 and VR-Headset in a nutshell
> A detailed step by step instruction is provided in the tutorials folder.
### Connecting UR5 to ROS
The robot is connected to a PC running [ROS](https://wiki.ros.org/noetic) (referred to as ROS-PC in this repository). The [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) is used to establish the communication between the robot and ROS. RViz is then used to visualise the current position of the UR5. With MoveIt! running in the background, the interactive marker at the TCP of the robot can be moved to a new position to set the goal position of the robot. 

At this stage, by clicking on *plan & execute* on RViz, ROS sends the goal position to the robot controller, which then plans a path and moves the joints of the robot. During this movement, the controller streams the joint values (position, velocity and acceleration of each joint) to ROS, which are used to represent the movement of the robot in RViz.

This connection between UR5 and ROS has been fully implemented in the previously mentioned [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) repository which also includes detailed description on establishing this connection.

After establishing the connection between UR5 and ROS, a communication between ROS and Unity has to be established.
### Connecting ROS to Unity within the ROS-PC
>This step describes how to connect ROS to Unity, and thus visualise the robot in Unity. This Unity scene can then be displayed on a VR-headset. But installing Unity on the ROS-PC might not provide all users with the capability of viewing the robot in VR. The reason for this is two fold:
> - ROS usually runs on linux based systems. Many VR-headsets including the Meta Quest series do not have an official SDK for linux environment. This means, you can visualise the robot in Unity, but you might not be able to connect the VR-headset to unity.
> - The ROS-PC might not be VR ready: to view a Unity scene directly on a VR-headset, the PC running Unity should have high enough [specifications](https://help.irisvr.com/hc/en-us/articles/213711747-Recommended-VR-Ready-Computers).
>
> To solve these issues, the ROS-PC can be connected to a second VR-ready PC (named Unity-PC in the repository) through an ethernet cable or a router. The information regarding robot joints can then be sent from the ROS-PC to the Unity-PC to be processed in Unity. This processed information is used to manipulate the virtual robot in Unity. Since the Unity-PC is running on windows and is VR-Ready, it can display this scene with the robot on any VR-headset.
>
> *Inspite of this, in this section, Unity is installed inside the ROS-PC (linux) to visualise UR5. The next sections describe installing Unity on the Unity-PC and the connection between the two PCs. This makes it easier to debug the system, since the communication between two PCs are not easy to establish*.

The [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) package provides the codes and instructions to establish a communication between ROS and Unity. After going through the [*Quick Installation Instructions*](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md) and the Part 0, Part 1 and Part 2 of the [*Pick-and-Place Tutorial*](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/README.md), a Niryo robot can be visualised in Unity and controlled using ROS.

In order to visualise the UR5 in Unity
- Open a new unity project and do the [Quick Installation Instructions](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md) from Unity-Robotics-Hub package.
- Follow the [Part 0: ROS Setup](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/0_ros_setup.md) from Unity-Robotics-Hub package.
- Follow the [Part 1: Create Unity scene with imported URDF tuorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/1_urdf.md) from Unity-Robotics-Hub package, but make the following changes:
   - Copy the folder UR5_URDF from this package to Assets/URDF/ in your Unity workspace.
   - Under the subsection *Setting up the Robot* in the above mentioned tutorial, instead of choosing *niryo_one.urdf*, choose the newly copied *ur5.urdf* in the UR5_URDF folder. This will import UR5 to the unity scene.
   - Select the robot and change the Y position to 0.63 to place it on the table
   - Select and and check the option *immovable* to fix the robot.
   - Follow all the remaining steps from the above mentioned tutorial to control each axis of the robot using the arrow keys.
- Follow the [Part 2: ROS–Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/0_ros_setup.md) until step 5 of *The Unity Side*. Afterwards:
  - Copy the *ROSController* and *ROSJoint* scripts from this repository to the Scripts folder in the Unity Package.
  - Attach the *ROSController* script to the robot and delete the *Controller* script from the robot.
- For setting up your ROS side, copy the package in this repository to your catkin_ws/src and run catkin_make.
- After connecting the UR5 to ROS as mentioned in the previous section (robot connected to ROS-PC, *ur_robot_driver* and *MoveIt!* running in background and robot visualised in RViz), run the following command in a new terminal
    ```bash
   roslaunch niryo_moveit part_2.launch tcp_ip:=127.0.0.1 tcp_port:=10005
   ```
  Once you click *play* on Unity, you should see the UR5 in Unity move to the position of the robot on the ROS-side.

Move the robot using RViz and then clicking on *plan & execute*. This results in the robot moving to the target position. The virtual robot in Unity copies the same path
### Connecting ROS to Unity installed on another PC
> This section does EXACTLY what was done in the previous section, except on a different PC. At the end of the section, you can visualise the UR5 in Unity running on the Unity-PC that is connected to ROS-PC via ethernet cable.

As mentioned in the previous section, Unity has to be installed on a seperate VR-Ready PC running windows. In order to visualise the UR5 in ROS
- Install Unity on the Unity-PC and repeat all steps done on Unity from the previous sections.
- On the drop down menu, type the IP address of the ROS-PC and click play.
- On the ROS-PC. connect UR5 to ROS and run the follwing command on a new terminal
     ```bash
   roslaunch niryo_moveit part_2.launch tcp_ip:=127.0.0.1 tcp_port:=10005
   ```
You should now see the UR5 in Unity move to the position of the robot on the ROS-side.

### Connecting Unity to VR-headset
Now you have a Unity scene containing the model of UR5 which follows the motion of the real robot. By connecting this Unity scene to a VR headset, you can visualise the robot movement in VR. For that, follow the [tutorial](https://youtu.be/HhtTtvBF5bI) for the first 10 minutes, but inside the scene involving the robot. You should now be able to see the robot moving in the VR headset.

