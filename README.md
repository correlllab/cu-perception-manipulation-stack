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
+ libboost-all-dev
+ ros-indigo-pcl


+ ros-indigo-baxter-sdk
+ ros-indigo-moveit-full
+ ros-indigo-moveit-visual-tools

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

If package are still missing, try adding these into your workspace src folder:
+ -b indigo-devel https://github.com/davetcoleman/rviz_visual_tools.git
+ https://github.com/ros-planning/moveit_robots.git
+ -b indigo-devel https://github.com/davetcoleman/moveit_visual_tools.git
+ https://github.com/Kinovarobotics/kinova-ros.git


Note: follow kinova-ros instructions for adding udev rules. kinova-ros may fail to build. check correct include file paths in kinova_comm.cpp: #include "kinova/KinovaTypes.h". Some other packages may fail in moveit, just add .CATKIN_IGNORE file or delete the directory.

restart udev so it reads the new rules:

```
sudo /etc/init.d/udev restart
```

### Compiling errors

There is an issue currently with PCL from ROS. If you have errors, such as "mets.h", use the following copy command to move missing files directly into PCL's directory on your machine. compile code using catkin build:
```
sudo cp <perception>/include/hv/* /usr/include/pcl-1.7/pcl/recognition/hv/
catkin build

```

### For Jaco

```
gedit ~/.bashrc
```

Add the following at the bottom:
```
source /opt/ros/indigo/setup.bash 
source ~/<your_workspace>/devel/setup.bash

export HOSTNAME=localhost
export ROS_MASTER_URI=http://$HOSTNAME:11311
export ROS_IP=`hostname -I | tr -d '[[:space:]]'`
```

Save and source the file for each terminal:
```
source ~/.bashrc
```

### For Baxter

Follow instructions for simulator installation if packages are missing still:
http://sdk.rethinkrobotics.com/wiki/Simulator_Installation

Baxter runs roscore when powered on. This requires you to setup remote ROS on your machine.
Use your favorite text editor for .bashrc:
```
gedit ~/.bashrc
```

Add the following at the bottom:
```
source /opt/ros/indigo/setup.bash 
source ~/<your_workspace>/devel/setup.bash

export HOSTNAME=011305P0009.local
export ROS_MASTER_URI=http://$HOSTNAME:11311
export ROS_IP=`hostname -I | tr -d '[[:space:]]'`
```

Save and source the file for each terminal:
```
source ~/.bashrc
```

## Running the code



### For Jaco
Each of these commands should be ran in a separate window. Some files may need to be modified to work with Jaco:
```
    roscore
    roslaunch kinova_bringup kinova_robot.launch
    roslaunch perception interface.launch
```

#### Main manipulation script for jaco:
```
    rosrun pick_and_place pap_with_perception.py
    rosrun keyboard keyboard
```

### For Baxter
Each of these commands should be ran in a separate window. Some files may need to be modified to work with Baxter:
```
    roslaunch perception interface.launch
```

### Camera
Modify roslaunch for Baxter or Jaco:

type="camera_alignment_baxter"

type="camera_alignment"


```
    roslaunch openni2_launch openni2.launch
    roslaunch camera_calibration_tool calibration.launch
    rosrun image_view image_view image:=/camera/rgb/image_raw
```

Scene calibration. Make sure AR tag is visible by looking in RVIZ for transform:
```
    rostopic pub /alignment/doit/ std_msgs/Bool True
```

If you don't plan to move the camera anytime soon, record transform (position and orientation) of camera_link in RVIZ under TF frames. Paste these values into transform_camera.py to avoid previous steps. 
When transform is known for the camera, skip above steps and run:
```
    rosrun camera_calibration_tool transform_camera.py
```

### Finger Sensors

Ignore finger_sensor_msgs package


You may need to flash an arduino with baxter.ino located in finger_sensor/baxter/arduino. Jaco will also need an arduino with the proper file loaded. Modify sensor.py for the right serial port, baud rate, and number of sensors of the code running on the arduino:
```
    rosrun finger_sensor sensor.py
    rosrun finger_sensor sensor_visual.py
```

sensor.py can also be modified to find an average base value for each sensor. For finger_sensor_perception, you will need to find the correct average base value. All sensor values should be less than 100 in the terminal

### Perception

Starting perception. Run "False" in place of True to stop
```
    rostopic pub /perception/enabled/ std_msgs/Bool True
```

Special way to modify variables such as gripper width in the code during runtime:
```
    rosrun rqt_reconfigure rqt_reconfigure
```

## ROS special commands

Test connection and view topics:
```
    rostopic list
```

View rate of publishing from a topic:
```
    rostopic hz /topic_name
```

jump into a package's source directory:
```
    roscd <package>
```

