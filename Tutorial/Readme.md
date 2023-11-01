## Step 1: Setup ROS workspace
> This step is done in ROS-PC and not Unity-PC

The ROS-PC should have Ubuntu as the operating system and should have ROS installed in it. [Here](http://wiki.ros.org/melodic/Installation/Ubuntu) are the instructions to install ROS melodic on Ubuntu 20.04.
### Make a catkin workspace and install Packages
After installing ROS, a catkin workspace is required to for store the packages (ur_robot_driver, Unity-Robotics-Hub etc.).

> It is assumed that you have installed ROS melodic on an Ubuntu 20.04 OS. If you have installed ROS noetic on Ubuntu 22.04, please change *melodic* from the follwing codes to *noetic*

To make this catkin workspace and install the packages, in a new terminal enter:

```bash
# source global ros
 source /opt/ros/melodic/setup.bash

# create a catkin workspace
 mkdir -p catkin_ws/src && cd catkin_ws

# clone the Universal Robots ROS driver: The driver lets you communicate with the robot controller
sudo apt install ros-${ROS_DISTRO}-ur-robot-driver

# clone ROS-TCP-Endpoint to connect ROS with Unity
 git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint src/ROS-TCP-Endpoint

# install dependencies
 sudo apt update -qq
 rosdep update
 rosdep install --from-paths src --ignore-src -y

# build the workspace
 catkin_make

# source the workspace
 source devel/setup.bash
```
copy the file [*unityconnect.py*](https://github.com/cakh/UR5-VR-Simulation/blob/main/ROS/Scripts/unityconnect.py) from this repository to */catkin_ws/src/ROS-TCP-Endpoint* in your ROS-PC. Inside the folder *ROS-TCP-Endpoint*, right click and select *Open in Terminal*. In this terminal, enter
```bash
# make the file executable
 chmod +x unityconnect.py
```
From the [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) package, copy the folder [ROS](https://github.com/cakh/Unity-Robotics-Hub/tree/main/tutorials/pick_and_place/ROS) under `/PATH/TO/Unity-Robotics-Hub/tutorials/pick_and_place/ROS` to `/home/<user>/catkin_ws/src` in your ROS-PC. Open a new terminal and type:

```bash
# source global ros
 source /opt/ros/melodic/setup.bash

# naviage to catkin workspace
 cd catkin_ws

# Update
 sudo apt-get update && sudo apt-get upgrade

# Install some packages
 sudo apt-get install python-pip ros-melodic-robot-state-publisher ros-melodic-moveit ros-melodic-rosbridge-suite ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-tf2-web-republisher

 sudo -H pip install rospkg jsonpickle

# build the workspace and source it
 catkin_make && source devel/setup.bash
```

Now you have the catkin package with all the required packages on your ROS side.

## Step 2: Control the robot in Unity
> This step is done in ROS-PC and not Unity-PC

> As mentioned in [Readme.md](https://github.com/cakh/UR5-VR-Simulation/blob/main/README.md), Unity is set up in the ROS-PC to test if it is working. Afterwards, another Unity-PC is set up with Unity installed in it and connected to ROS-PC via ethernet or router.

1. Download and install [Unity Hub](https://docs.unity3d.com/2020.1/Documentation/Manual/GettingStartedInstallingHub.html) from the official website and add a new project. Select the options as seen in the following image:
<div align="center">
    <img src="https://github.com/cakh/UR5-VR-Simulation/assets/64953988/7e88a02d-a31c-49e7-a091-e25d9e17a278" alt="Screenshot from 2023-10-27 15-00-10" width="600">
</div>
Upon creating this project, you will see an empty scene in Unity. 

2. Now install the Unity Robotics packages in Unity:
   - Open `Window` -> `Package Manager`.
   - In the Package Manager window, find and click the `+` button in the upper lefthand corner of the window. Select `Add package from git URL...`.

     ![image](https://github.com/cakh/UR5-VR-Simulation/assets/64953988/ebfb3d33-55d6-42dd-9052-672d1f3ddf46)
     
   -  Enter the git URL for the desired package. Note: you can append a version tag to the end of the git url, like `#v0.4.0` or `#v0.5.0`, to declare a specific package version, or exclude the tag to get the latest from the package's `main` branch.
      - For the [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector), enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`.
      - For the [URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer), enter `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`.
   - Click `Add`.

3. Copy the folder [UR5_URDF](https://github.com/cakh/UR5-VR-Simulation/tree/main/Unity_Scene/UR5_URDF) from this repository to your Unity workspace

   ![Peek 2023-10-27 15-35](https://github.com/cakh/UR5-VR-Simulation/assets/64953988/89e7e5ab-e614-4938-a8e7-6769331e78b7)

4. Open the folder *UR5_URDF* in Unity workspace, right click on *ur5.urdf* and click on *Import Robot from selected URDF File*. This will import the robot to your Unity scene.

   ![importrobo](https://github.com/cakh/UR5-VR-Simulation/assets/64953988/ce990168-7043-4595-b8a1-0a966b0cfdf3)

5. Select the *Main Camera* and change its position to 0, 1, -0.7 and rotation to 45, 0, 0
   
   <img src="https://github.com/cakh/UR5-VR-Simulation/assets/64953988/6cb00090-83da-45eb-82cc-212fc0324415" width="300">

7. From the *Hierarchy* window, choose *ur5_robot*. At the inspection window under *Urdf Robot (script)*, next to the option *Use Gravity*, select the option *Disable*
8. Expand ur5_robot and Click on *base*. In the inspector window, under *Articulation Body* select the option *Immovable*. Repeat this step for *base_link_inertia*.
   ![immovable](https://github.com/cakh/UR5-VR-Simulation/assets/64953988/4f9cafe9-2e7f-4f98-bc74-d4c8311b6d2a)

9. Select ur5_robot again from the *Hierrachy* window and under *inspection* in *Controller (script)* give the following values
   ![image](https://github.com/cakh/UR5-VR-Simulation/assets/64953988/950858a7-e1a7-4b6f-9865-42ef198b6008)

   If you click on play, you will be able to now move the robot with your arrow keys!
![robomove](https://github.com/cakh/UR5-VR-Simulation/assets/64953988/6b6f474a-41d6-4565-aa88-ec82ac1f564e)



## Step 3: Connect Unity with ROS
> This step is done in ROS-PC and not Unity-PC

Now that UR5 is imported in Unity and can be controlled with arrow keys, we need to establish a connection between Unity and ROS so that this Unity scene can subscribe to a ros topic that streams joint state values (position, velocity acceleration etc.). This is achieved by replacing the *Controller (script)* attached to *ur5_robot* with a custom controller. Since this custom controller (included in this package) uses several ROS messages, these ROS messages should be installed in Unity.

### Install ROS messages in Unity
1. After downloading this repository, locate the file *RobotTrajectory.msg* (UR5-VR-Simulation/Unity_Scene/ROS_Messages/RobotTrajectory.msg)
2. In Unity, under *Robotics* menu, choose *Generate ROS Messages..*.
3. Click on *browse* under *ROS Message Path* and go to the folder where the *RobotTrajectory.msg* is stored and click *open*.
4. Click on *Build 1 msg* button to build the message
   
![rosmsg](https://github.com/cakh/UR5-VR-Simulation/assets/64953988/d54b3cb9-841d-4795-92c0-44be080494e0)

### Attach custom controller to the robot
1. Copy the [*RosController.cs*](https://github.com/cakh/UR5-VR-Simulation/blob/main/Unity_Scene/Scripts/RosController.cs) script from this repository to your Unity workspace and attach it to ur5_robot by drag and drop.
2. Click on ur5_robot from *Hierarchy* window and delete the *Controller (script)* (the default controller) from the robot.
3. From the *Project* window, navigate to *Packages -> URDF Importer -> Runtime -> Controller* and open *JointControl.cs* script.
4. Replace the contents with the [*JointControl.cs* script in this repository](https://github.com/cakh/UR5-VR-Simulation/blob/main/Unity_Scene/Scripts/JointControl.cs).

Now the robot is ready to connect to ROS and receive messages. After setting up ROS (next step), you can press play on Unity and the robot in Unity will follow the real robot.

## Step 4: Connect Robot to ROS
> This step is done in ROS-PC and not Unity-PC
All the steps to connect the robot to ROS are mentioned in the [Universal_Robots_ROS_Driver package](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). Assuming all those steps are complete, enter the follwing code, each in a new terminal:

> Do not forget to source your setup.bash!


```bash
# connect ROS with UR5 (replace robot_ip with the ip address of your robot)
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.56.101 kinematics_config:=${HOME}/my_robot_calibration.yaml

# start MoveIt!
 roslaunch ur5_moveit_config moveit_planning_execution.launch limited:=true

# view the robot in RViz
 roslaunch ur5_moveit_config moveit_rviz.launch config:=true

```

Now you should be able to move the interactive marker on the Robot in RViz and click on *plan & execute* to see the robot moving.

After establishing connection between ROS and UR5, the joint states of the robot should be sent to unity. For that, open a new terminal and type

```bash
# program to establish connection with Unity
 roslaunch ros_tcp_endpoint endpoint.launch

# program that reads joint state values from /tf topics, packs it into *RobotTrajectory.msg* message and sends it to Unity
 rosrun ur_robot_driver unityconnect.py

```

If you press play on Unity, you should now see the UR5 in Unity follwing the path of the robot.
