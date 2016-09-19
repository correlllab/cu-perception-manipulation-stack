# cu-perception-manipulation-stack

Dependencies:
ros-indigo
ubuntu 14.04
ros-indigo-ar-track-alvar
...
the rest to be added later

Setting up a workspace
In your home directory:
mkdir ros
cd ros
mkdir jaco_ws/src

follow instructions on ros for setting up a workspace before proceeding. Add the following github repos in the src directory in your workspace(s). Use "catkin build" instead of "catkin_make" when compiling. 

use git clone inside jaco_ws/src:
b indigo-devel https://github.com/davetcoleman/rviz_visual_tools.git
https://github.com/ros-planning/moveit_robots.git
-b indigo-devel https://github.com/davetcoleman/moveit_visual_tools.git
https://github.com/Kinovarobotics/kinova-ros.git 
    -kinova-ros may fail to build. check correct include file paths in kinova_comm.cpp: #include "kinova/KinovaTypes.h"
    -follow their instructions for udev rules
https://github.com/correlllab/cu-perception-manipulation-stack.git

.bashrc lines to add:

source /opt/ros/indigo/setup.bash 
source ~/ros/jaco_ws/devel/setup.bash

#export HOSTNAME=011305P0009.local  # Baxter
export HOSTNAME=localhost  # Jaco, etc
export ROS_MASTER_URI=http://$HOSTNAME:11311
export ROS_IP=`hostname -I | tr -d '[[:space:]]'`

launch files:
    roslaunch openni2_launch openni2.launch 
    roslaunch camera_calibration_tool calibration.launch
    roslaunch perception interface.launch

rosrun scripts:
To enable or disable baxter:
    rosrun baxter_tools enable_robot.py -e

    rosrun image_view image_view image:=/camera/rgb/image_raw
    rosrun keyboard keyboard

for finger sensors, one of the following: 
    rosrun finger_sensor sensor.py
    rosrun finger_sensor sensor_visual.py

Main manipulation script (will be restructured):
    rosrun finger_sensor stacking_blocks.py

More instructions coming soon!

