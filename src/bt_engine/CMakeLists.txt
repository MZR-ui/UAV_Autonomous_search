cmake_minimum_required(VERSION 3.0.2)
project(bt_engine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  msp_interface
  remote_info
  sensor_msgs
  geometry_msgs
  std_msgs
  behaviortree_cpp
)

find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bt_server_node
  src/bt_server_node.cpp
  src/ros_backend.cpp
  src/skill_loader.cpp
)

add_dependencies(bt_server_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(bt_server_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(TARGETS bt_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES launch/bt_engine.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.xml"
)

install(DIRECTORY skills/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/skills
)
