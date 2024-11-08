cmake_minimum_required(VERSION 3.0.2)
project(gp_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_ros
  trajectory_msgs
)

# Eigen
include_directories( "/usr/include/eigen3" )

# gtsam
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

#boost
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

#gpmp2
find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})

include_directories(
 include
 include/global_planner
 include/local_planner
 ${catkin_INCLUDE_DIRS}
 SYSTEM
 ${Boost_INCLUDE_DIRS}
)

add_library(Global_Planner
  src/global_planner/tree.cpp
  src/global_planner/planner.cpp
  src/global_planner/env.cpp
  src/global_planner/kinematics.cpp
  src/global_planner/interpolation.cpp
  include/global_planner/interpolation.h
  include/global_planner/env.h
  include/global_planner/kinematics.h
  include/global_planner/planner.h
  include/global_planner/tree.h
  include/global_planner/header.h
)

add_library(Local_Planner
  src/local_planner/ArmKinematics.cpp
  include/local_planner/headers.h
  include/local_planner/ArmKinematics.h
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gp_planner Global_Planner Local_Planner
#  CATKIN_DEPENDS actionlib control_msgs gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools pluginlib roscpp roslib rospy sensor_msgs std_msgs tf2_eigen tf2_ros trajectory_msgs
#  DEPENDS system_lib
)


add_executable(rrt_plan_test src/global_planner/plan_test.cpp)
target_link_libraries(rrt_plan_test
  Global_Planner
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
add_executable(rrt_dynamic_test src/global_planner/dynamic_test.cpp)
target_link_libraries(rrt_dynamic_test
  Global_Planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(kinematics_test src/local_planner/kinematics_test.cpp)
target_link_libraries(kinematics_test
  Local_Planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)
