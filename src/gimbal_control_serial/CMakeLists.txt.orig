cmake_minimum_required(VERSION 3.0.2)
project(gimbal_control_serial)

## Compile as C++11
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  serial
  std_msgs
  message_generation
)

################################################
## Declare ROS messages                       ##
################################################
add_message_files(
  FILES
  GimbalCmd.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## catkin specific configuration               ##
################################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs serial message_runtime
)

###################################
## Include directories            ##
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###################################
## Declare executables             ##
###################################
add_executable(gimbal_control_node
  src/gimbal_control_node.cpp
  src/gimbal_protocol.cpp
  src/gimbal_serial.cpp
)

## Add dependencies for message generation
add_dependencies(gimbal_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Link libraries
target_link_libraries(gimbal_control_node
  ${catkin_LIBRARIES}
)
