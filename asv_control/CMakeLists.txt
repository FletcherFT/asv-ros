cmake_minimum_required(VERSION 2.8.3)
project(asv_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  asv_messages
)

#catkin_python_setup()

# Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/Autopilot.cfg
   cfg/PoseController.cfg
 )

catkin_package(
  CATKIN_DEPENDS asv_messages
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)