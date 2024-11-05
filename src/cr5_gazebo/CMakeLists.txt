cmake_minimum_required(VERSION 3.0.2)
project(cr5_gazebo)

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  rospy
  rosmsg
  roscpp
)
find_package(roslaunch)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cr5_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch controller config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

