//
// Created by robert on 9/4/22.
//

#ifndef RRT_PLANNER_HEADER_H
#define RRT_PLANNER_HEADER_H

#pragma once

#include <ros/ros.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <map>
#include <sstream>
#include <random>
#include <chrono>
// 包含msgs
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/String.h>
// 包含moveit的API
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
// 包含TCP/IP通信
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <arpa/inet.h>


using namespace Eigen;

#endif //RRT_PLANNER_HEADER_H
