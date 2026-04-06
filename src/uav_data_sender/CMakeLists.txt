cmake_minimum_required(VERSION 2.8.3)
project(uav_data_sender)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  prometheus_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav_state_sender src/uav_state_sender.cpp)
target_link_libraries(uav_state_sender ${catkin_LIBRARIES})

add_dependencies(uav_state_sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

