cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  remote_info
)

add_message_files(
  FILES
  OrderTask.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(task_manager_node src/task_manager_node.cpp)
add_dependencies(task_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_manager_node ${catkin_LIBRARIES})