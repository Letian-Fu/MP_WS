# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "actionlib;control_msgs;gazebo_msgs;gazebo_plugins;gazebo_ros;gazebo_ros_control;geometry_msgs;moveit_core;moveit_ros_planning;moveit_ros_planning_interface;moveit_visual_tools;pluginlib;roscpp;roslib;rospy;sensor_msgs;std_msgs;tf2_eigen;tf2_ros;trajectory_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgp_planner;-lGlobal_Planner;-lLocal_Planner;-lMy_Planner;/usr/local/lib/libgtsam.so.4.3a0".split(';') if "-lgp_planner;-lGlobal_Planner;-lLocal_Planner;-lMy_Planner;/usr/local/lib/libgtsam.so.4.3a0" != "" else []
PROJECT_NAME = "gp_planner"
PROJECT_SPACE_DIR = "/home/roboert/MP_WS/install"
PROJECT_VERSION = "0.0.0"
