# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "perception: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iperception:/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(perception_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg/identified_object.msg" NAME_WE)
add_custom_target(_perception_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "perception" "/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg/identified_object.msg" "geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Pose"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(perception
  "/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg/identified_object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception
)

### Generating Services

### Generating Module File
_generate_module_cpp(perception
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(perception_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(perception_generate_messages perception_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg/identified_object.msg" NAME_WE)
add_dependencies(perception_generate_messages_cpp _perception_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perception_gencpp)
add_dependencies(perception_gencpp perception_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perception_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(perception
  "/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg/identified_object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception
)

### Generating Services

### Generating Module File
_generate_module_lisp(perception
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(perception_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(perception_generate_messages perception_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg/identified_object.msg" NAME_WE)
add_dependencies(perception_generate_messages_lisp _perception_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perception_genlisp)
add_dependencies(perception_genlisp perception_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perception_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(perception
  "/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg/identified_object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception
)

### Generating Services

### Generating Module File
_generate_module_py(perception
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(perception_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(perception_generate_messages perception_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/correlllab/ros/jaco_ws/src/cu-perception-manipulation-stack/perception/msg/identified_object.msg" NAME_WE)
add_dependencies(perception_generate_messages_py _perception_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perception_genpy)
add_dependencies(perception_genpy perception_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perception_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(perception_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(perception_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(perception_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(perception_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(perception_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(perception_generate_messages_py geometry_msgs_generate_messages_py)
