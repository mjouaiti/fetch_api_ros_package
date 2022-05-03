cmake_minimum_required(VERSION 3.0.2)
project(fetch_api)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  fetch_moveit_config
  fetch_navigation
  moveit_commander
  moveit_python
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES fetch_api
  #CATKIN_DEPENDS roscpp actionlib fetch_moveit_config fetch_navigation moveit_commander moveit_python rospy std_msgs
#  DEPENDS system_lib
)

#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)
