# Setup ROS-PC
The ROS-PC should have Ubuntu as the operating system and should have ROS installed in it. [Here](http://wiki.ros.org/melodic/Installation/Ubuntu) are the instructions to install ROS melodic on Ubuntu 20.04.
## Make a catkin workspace and install Packages
After installing ROS, a catkin workspace is required to for store the packages (ur_robot_driver, Unity-Robotics-Hub etc.).

> It is assumed that you have installed ROS melodic on an Ubuntu 20.04 OS. If you have installed ROS noetic on Ubuntu 22.04, please change *melodic* from the follwing codes to *noetic*

On a new terminal type

```bash
# source global ros
 source /opt/ros/melodic/setup.bash

# create a catkin workspace
 mkdir -p catkin_ws/src && cd catkin_ws

# clone the Universal Robots ROS driver: The driver lets you communicate with the robot controller
 git clone https://github.com/cakh/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone the robot description. It contains the URDF and visual info about the UR robots. Currently, it is necessary to use the melodic-devel-staging branch.
 git clone -b melodic-devel-staging https://github.com/cakh/universal_robot.git src/universal_robot

# clone ROS-TCP-Endpoint to connect ROS with Unity
 git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint

# install dependencies
 sudo apt update -qq
 rosdep update
 rosdep install --from-paths src --ignore-src -y

# build the workspace
 catkin_make
```
