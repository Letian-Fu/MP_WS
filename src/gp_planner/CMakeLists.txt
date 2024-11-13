cmake_minimum_required(VERSION 3.0.2)
project(gp_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")



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
  nav_msgs
  cv_bridge
)

# Eigen
include_directories( "/usr/local/include/eigen3" )

# Find OpenCV
find_package(OpenCV REQUIRED)

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
  src/global_planner/kinematics.cpp
  src/global_planner/interpolation.cpp
  src/global_planner/rrtplanner.cpp
  include/global_planner/interpolation.h
  include/global_planner/kinematics.h
  include/global_planner/tree.h
  include/global_planner/header.h
  include/global_planner/rrtplanner.h
)

add_library(Local_Planner
  src/local_planner/ArmKinematics.cpp
  src/local_planner/ArmModel.cpp
  src/local_planner/OptimizerSetting.cpp
  src/local_planner/Optimizer.cpp
  include/local_planner/headers.h
  include/local_planner/ArmKinematics.h
  include/local_planner/ArmModel.h
  include/local_planner/SDF.h
  include/local_planner/CostFunction.h
  include/local_planner/OptimizerSetting.h
  include/local_planner/GPInterpolate.h
  include/local_planner/PriorFactor.h
  include/local_planner/ObsFactor.h
  include/local_planner/GaussianFactor.h
  include/local_planner/LimitFactor.h
  include/local_planner/Optimizer.h
  include/local_planner/SelfCollisionFactor.h
  include/local_planner/GoalFactor.h
)

add_library(My_Planner
  src/my_planner/MyPlanner.cpp
  include/my_planner/MyPlanner.h
)

# 链接 My_Planner 到 Global_Planner 和 Local_Planner
target_link_libraries(My_Planner
  Global_Planner
  Local_Planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gp_planner Global_Planner Local_Planner My_Planner
  CATKIN_DEPENDS actionlib control_msgs gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools pluginlib roscpp roslib rospy sensor_msgs std_msgs tf2_eigen tf2_ros trajectory_msgs cv_bridge
  DEPENDS Boost GTSAM
)

# add_executable(kinematics_test src/local_planner/test/kinematics_test.cpp)
# target_link_libraries(kinematics_test
#   Local_Planner
#   Global_Planner
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES} 
#   ${GTSAM_LIBRARIES}
# )

# add_executable(sdf_test src/local_planner/test/sdf_test.cpp)
# target_link_libraries(sdf_test
#   Local_Planner
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES} 
#   ${GTSAM_LIBRARIES}
# )

# add_executable(planner_test src/local_planner/test/planner_test.cpp)
# target_link_libraries(planner_test
#   Local_Planner
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES} 
#   ${GTSAM_LIBRARIES}
# )

# add_executable(gtsam_test src/local_planner/test/gtsam_test.cpp)
# target_link_libraries(gtsam_test
#   ${catkin_LIBRARIES}
#   ${GTSAM_LIBRARIES}
# )

add_executable(plan_test src/my_planner/plan_test.cpp)
target_link_libraries(plan_test
  My_Planner
  ${catkin_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
