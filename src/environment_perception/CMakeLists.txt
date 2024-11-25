cmake_minimum_required(VERSION 3.0.2)
project(environment_perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  rosconsole
  grid_map_core
  grid_map_ros
  grid_map_msgs
  octomap_msgs
  octomap_ros
)

# 查找 OctoMap
find_package(octomap REQUIRED)
message(STATUS "OctoMap libraries: ${OCTOMAP_LIBRARIES}")
message(STATUS "OctoMap include dirs: ${OCTOMAP_INCLUDE_DIRS}")

find_package(PCL REQUIRED COMPONENTS common io filters surface segmentation)

# 包含 PCL 的头文件
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES environment_perception
#  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(fillting src/fillting.cpp)
target_link_libraries(fillting 
${catkin_LIBRARIES}
${PCL_LIBRARIES}
pcl_filters
)

add_executable(cluster src/cluster.cpp)
target_link_libraries(cluster 
${catkin_LIBRARIES}
${PCL_LIBRARIES}
pcl_filters
)

add_executable(cluster_pcd src/cluster_pcd.cpp)
target_link_libraries(cluster_pcd 
${catkin_LIBRARIES}
${PCL_LIBRARIES}
pcl_filters
)

add_executable(occupacy_map src/occupacy_map.cpp)
target_link_libraries(occupacy_map
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${OCTOMAP_LIBRARIES}
)

add_executable(occupacy_map_pcd src/occupacy_map_pcd.cpp)
target_link_libraries(occupacy_map_pcd 
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${OCTOMAP_LIBRARIES}
)