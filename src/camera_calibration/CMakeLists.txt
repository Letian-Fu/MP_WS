cmake_minimum_required(VERSION 3.0.2)
project(camera_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  image_transport
)

find_package(OpenCV REQUIRED)
include_directories( "/usr/local/include/eigen3" )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_calibration
#  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(calibrate src/calibrate.cpp)

target_link_libraries(calibrate
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(hand_eye_calibration src/handeyecalib.cpp)

target_link_libraries(hand_eye_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
