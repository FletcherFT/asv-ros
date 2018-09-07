cmake_minimum_required(VERSION 2.8.3)
project(asv_mission)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES Plan.msg Task.msg)

generate_messages(DEPENDENCIES std_msgs asv_mission)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})