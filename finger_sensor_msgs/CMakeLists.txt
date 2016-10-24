cmake_minimum_required(VERSION 2.8.3)
project(finger_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
  geometry_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  FingerDetect.msg
  FingerFAI.msg
  FingerSAI.msg
  FingerTouch.msg
)

add_service_files(
  FILES

)

add_action_files(DIRECTORY action
  FILES
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
)
