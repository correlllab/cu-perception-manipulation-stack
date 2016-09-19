# cu-perception-manipulation-stack
Contains ros packages for perception, calibration, and arm manipulation of the Jaco arm. 

## Installation

### Dependencies:

+ ros-indigo
+ ubuntu 14.04
+ ros-indigo-ar-track-alvar
+ ...
+ the rest to be added later

### Setting up a workspace and adding the packages

In your home directory:
```
mkdir ros/jaco_ws/src
cd ros/aco_ws/src
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
+ b indigo-devel https://github.com/davetcoleman/rviz_visual_tools.git
+ https://github.com/ros-planning/moveit_robots.git
+ -b indigo-devel https://github.com/davetcoleman/moveit_visual_tools.git
+ https://github.com/correlllab/cu-perception-manipulation-stack.git
+ https://github.com/Kinovarobotics/kinova-ros.git

- follow kinova-ros instructions for udev rules. kinova-ros may fail to build. check correct include file paths in kinova_comm.cpp: #include "kinova/KinovaTypes.h" 

### Compile
Use "catkin build" in your workspace
```
catkin build
```

### Sourcing
Assuming you were able to compile, you will need to source your packages to be able to run them:
```
gedit ~/.bashrc
```
.bashrc lines to add:
```
source /opt/ros/indigo/setup.bash 
source ~/ros/jaco_ws/devel/setup.bash

#export HOSTNAME=011305P0009.local  # To connect to Baxter
export HOSTNAME=localhost  # Jaco, etc
export ROS_MASTER_URI=http://$HOSTNAME:11311
export ROS_IP=`hostname -I | tr -d '[[:space:]]'`
```
Either open a new terminal or source that file:
```
source ~/.bashrc
```

## Running the packages
If connecting to Jaco, run:
```
roscore
```

### launch files:
```
    roslaunch openni2_launch openni2.launch 
    roslaunch camera_calibration_tool calibration.launch
    roslaunch perception interface.launch
```
### rosrun scripts:
```
    rosrun image_view image_view image:=/camera/rgb/image_raw
    rosrun keyboard keyboard
```

### for finger sensors, one of the following: 
```
    rosrun finger_sensor sensor.py
    rosrun finger_sensor sensor_visual.py
```
### Main manipulation script (will be restructured):
```
    rosrun finger_sensor stacking_blocks.py
```

## More instructions coming soon!

