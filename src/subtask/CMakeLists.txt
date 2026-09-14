cmake_minimum_required(VERSION 3.0.2)
project(subtask)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  task_manager
)

add_message_files(
  FILES
  ControlData.msg
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

add_executable(task_1_base_act src/task_1_base_act.cpp)
add_dependencies(task_1_base_act
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(task_1_base_act ${catkin_LIBRARIES})