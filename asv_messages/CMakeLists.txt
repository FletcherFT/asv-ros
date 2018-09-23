cmake_minimum_required(VERSION 2.8.3)
project(asv_messages)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

# Generate messages in the 'msg' folder.
add_message_files(
  DIRECTORY msg 
  FILES
  ThrusterComponents.msg
  Thrusters.msg
  Float64Stamped.msg
  Readings.msg
  Task.msg
  Plan.msg
  )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   ConfigureSteppers.srv
   EnableControl.srv
   PlanService.srv
   UTMService.srv
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})