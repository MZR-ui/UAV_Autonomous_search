cmake_minimum_required(VERSION 3.0.2)
project(msp_interface)

## 编译选项：C++14
set(CMAKE_CXX_STANDARD 14)

## 查找 catkin 包及其组件
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rosbag
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS filesystem system)

## 生成自定义消息
add_message_files(
  FILES
  MspSensor.msg
  MspChannel.msg
  EscTelem.msg
  Attitude.msg
)

## 生成消息（依赖 std_msgs）
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## catkin 包配置
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

## 头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## 声明可执行文件（节点）
add_executable(${PROJECT_NAME}_node
  src/msp_interface_node.cpp
  src/serial_driver.cpp
  src/msp_protocol.cpp
  src/msp_parser.cpp
)

## 添加消息生成依赖
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


# 新节点：mode_manage_node
add_executable(mode_manage_node
  src/mode_manage_node.cpp
)

# 添加依赖（确保消息头文件生成）
add_dependencies(mode_manage_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mode_manage_node
  ${catkin_LIBRARIES}
)

# rosbag 录制节点（从 mode_manage 拆出）
add_executable(rosbag_recorder_node
  src/rosbag_recorder_node.cpp
)
add_dependencies(rosbag_recorder_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rosbag_recorder_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

## 安装节点
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装头文件（供其他包使用）
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## 安装 launch 文件
install(FILES launch/msp_interface.launch launch/rosbag_recorder.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


# 安装三个节点
install(TARGETS msp_interface_node mode_manage_node rosbag_recorder_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)