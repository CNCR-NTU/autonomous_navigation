cmake_minimum_required(VERSION 2.8.3)
project(autonomous_navigation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LaserPoint.msg
  Position.msg
  ObjectsAtPosition.msg
  LaserAtPosition.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)