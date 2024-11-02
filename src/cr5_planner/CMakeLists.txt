cmake_minimum_required(VERSION 3.0.2)
project(cr5_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
SET(Python3_INCLUDE_DIRS "/usr/include/python3.8")
SET( PYTHON_EXECUTABLE /usr/bin/python3)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  rosconsole
  moveit_msgs
  pluginlib
  roscpp
  std_msgs
  tf
  tf2
  trajectory_msgs
  sensor_msgs
  shape_msgs
  dynamic_reconfigure
  rospy
  actionlib
  sensor_msgs
  geometry_msgs
)

#boost
set(BOOST_INCLUDEDIR "/usr/include/boost")
set(BOOST_LIBRARYDIR "/usr/lib")
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
include_directories( 
include
${catkin_INCLUDE_DIRS}
SYSTEM
${Boost_INCLUDE_DIRS} )

include_directories( "/usr/include/eigen3" )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES srrt_cr5
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_library(${PROJECT_NAME}
		src/tree.cpp
		src/planner.cpp
		src/env.cpp
		src/kinematics.cpp
        include/interpolation.h
		src/interpolation.cpp)
add_executable(plan_test src/plan_test.cpp)
target_link_libraries(plan_test
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
        )
add_executable(dynamic_test src/dynamic_test.cpp)
target_link_libraries(dynamic_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
        )
