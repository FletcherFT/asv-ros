cmake_minimum_required(VERSION 2.8.3)
project(asv_launch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  asv_messages
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asv_launch
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS asv_messages
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

