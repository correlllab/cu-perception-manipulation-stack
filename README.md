# cu-perception-manipulation-stack
Contains ros packages for perception, calibration, and arm manipulation of the Jaco arm. 

## Installation

### Dependencies:

+ ros-indigo installation
+ ubuntu 14.04
+ catkin_tools: http://catkin-tools.readthedocs.io/en/latest/installing.html

Using sudo apt-get install:
+ ros-indigo-ar-track-alvar
+ ros-indigo-moveit-core
+ ros-indigo-graph-msgs
+ ros-indigo-moveit-ros-robot-interaction
+ ros-indigo-openni2-launch
+ ros-indigo-keyboard
+ ros-indigo-moveit-ros

### Setting up a workspace and adding the packages

In your home directory:
```
mkdir -p ~/ros/jaco_ws/src
cd ros/jaco_ws/src
catkin_init_workspace
cd ..
catkin build
```
Add the following github repos in the src directory in your workspace. 
```
cd src
git clone <package>
```
packages:
+ -b indigo-devel https://github.com/davetcoleman/rviz_visual_tools.git
+ https://github.com/ros-planning/moveit_robots.git
+ -b indigo-devel https://github.com/davetcoleman/moveit_visual_tools.git
+ https://github.com/correlllab/cu-perception-manipulation-stack.git
+ https://github.com/Kinovarobotics/kinova-ros.git

Note: follow kinova-ros instructions for adding udev rules. kinova-ros may fail to build. check correct include file paths in kinova_comm.cpp: #include "kinova/KinovaTypes.h". Some other packages may fail in moveit, just add .CATKIN_IGNORE file or delete the directory.

restart udev so it reads the new rules:

```
sudo /etc/init.d/udev restart
```

### Compile
Use "catkin build" in your workspace one more time
```
catkin build
```

### Sourcing and Configuring .bashrc
Assuming you were able to compile, you will need to source your packages to be able to run them:
```
gedit ~/.bashrc
```
.bashrc lines to add. modify ip and username in <>:
```
# Sourcing ROS and your workspace(s)
source /opt/ros/indigo/setup.bash 
source ~/ros/jaco_ws/devel/setup.bash

# roscore on this machine
export ROS_IP=`hostname -I | tr -d '[[:space:]]'`

# roscore on the gigabyte
export ROS_MASTER_URI=http://128.138.244.28:11311

# local host
export ROS_HOSTNAME=<your ip>

# ROS Workspaces
function rosPackagePath()
{
    arr=$(echo $CMAKE_PREFIX_PATH | tr ":" "\n")
    for x in $arr
    do
	rootpath1="/home/<username>/ros/"
	rootpath2="/opt/ros/"
	x=${x#${rootpath1}}
	echo " " ${x#${rootpath2}}
    done
};
rosPackagePath
echo "ROS_HOSTNAME = "$ROS_HOSTNAME
echo "ROS_IP = "$ROS_IP
echo "ROS_MASTER_URI = "$ROS_MASTER_URI
```

Either open a new terminal or source that file:
```
source ~/.bashrc
```

## Running the everything on your machine
Uncomment and comment out pertaining sections in .bashrc shown above and run the following in a terminal(s)
```
roscore
```

### launch files:
Single one that launches all of the nodes:
```
   roslaunch pick_and_place pap_full.launch
```
Launching the nodes individually:
```
    roslaunch openni2_launch openni2.launch depth_registration:=true publish_tf:=true
    roslaunch camera_calibration_tool calibration.launch
    roslaunch perception interface.launch
    roslaunch kinova_bringup kinova_robot.launch
    rosrun image_view image_view image:=/camera/rgb/image_raw
    rosrun keyboard keyboard
```

### for finger sensors, one of the following: 
```
    rosrun finger_sensor sensor.py
    rosrun finger_sensor sensor_visual.py
```
### Main manipulation script for jaco:
```
    rosrun pick_and_place pap_with_perception.py
```

## Connecting to roscore in the lab
.bashrc script above configures your computer to connect with ROS. Make sure this is sourced properly
Test connection:
```
rostopic list
```
Possible solutions if connection fails, but pinging is successful; installing openssh-client and opentssh-server. Using netcat and testing ports. Restarting. Ones of these magically fixed our issues. 

### launch files:
Launching the nodes:
```
    roslaunch perception interface.launch
    rosrun image_view image_view image:=/camera/rgb/image_raw
    rosrun keyboard keyboard
```
### for finger sensors, one of the following: 
```
    rosrun finger_sensor sensor.py
    rosrun finger_sensor sensor_visual.py
```
### Main manipulation script for jaco:
```
    rosrun pick_and_place pap_with_perception.py
```

## More instructions coming soon!

