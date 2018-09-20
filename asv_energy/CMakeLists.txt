cmake_minimum_required(VERSION 2.8.3)
project(asv_energy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  asv_messages
)

catkin_package(
  CATKIN_DEPENDS asv_messages
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)