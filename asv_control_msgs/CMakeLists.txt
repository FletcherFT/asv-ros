cmake_minimum_required(VERSION 2.8.3)
project(asv_control_msgs)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES ThrusterComponents.msg )

add_service_files(DIRECTORY srv FILES EnableControl.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
