cmake_minimum_required(VERSION 3.0.2)
project(tb3_move)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Tb3Move.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tb3_move
  CATKIN_DEPENDS actionlib actionlib_msgs rospy std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

