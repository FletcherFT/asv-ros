cmake_minimum_required(VERSION 2.8.3)
project(asv_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  asv_control_msgs
  std_msgs
  dynamic_reconfigure
  std_srvs
)

catkin_python_setup()

# Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/TwistController.cfg
   cfg/PoseController.cfg
 )

catkin_package(
  CATKIN_DEPENDS asv_control_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/my_python_script
  src/asv_control/pose_controller.py
  src/asv_control/nav_sim.py
  src/asv_control/qp_allocation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
