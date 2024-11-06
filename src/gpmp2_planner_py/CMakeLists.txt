cmake_minimum_required(VERSION 3.0.2)
project(gpmp2_planner_py)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    control_msgs
    actionlib
    roslib
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    tf2_eigen
    tf2_ros
    gazebo_msgs
    gazebo_plugins
    gazebo_ros
    gazebo_ros_control
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gpmp2_planner
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 SYSTEM
 ${Boost_INCLUDE_DIRS}
)


# Eigen
include_directories( "/usr/include/eigen3" )


# gtsam
find_package(GTSAMCMakeTools)
find_package( GTSAM REQUIRED )
include_directories( ${GTSAM_INCLUDE_DIR} )

#boost
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

#gpmp2
find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})

