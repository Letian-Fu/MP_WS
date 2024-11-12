#ifndef RRT_PLANNER_RRTPLANNER_H
#define RRT_PLANNER_RRTPLANNER_H

#pragma once

#include "header.h"
#include "tree.h"
#include "kinematics.h"
#include "interpolation.h"

namespace rrt_planner{

class RRTPlanner {
    typedef moveit::planning_interface::MoveGroupInterface Move_Group;
    typedef moveit::planning_interface::PlanningSceneInterface Scene;
    typedef robot_model_loader::RobotModelLoader Robot_Model_Loader;
public:
    int dof_;
    VectorXd dh_alpha_, dh_a_, dh_d_, dh_theta_;
    Eigen::VectorXd start_conf_, goal_conf_, arm_pos_;
    Tree tree_;
    int NUM_INITIATION_;
    double step_;
    double step_rate_;
    double fai_;
    double fai_rate_;
    int MAX_TREE_NUM_;
    int MAX_FAIL_NUM_;
    int is_adapt_step_;
    int is_adapt_fai_;
    int inter_num_;
    bool is_plan_success_;
    std::vector<std::string> joint_all_names_;
    robot_state::RobotStatePtr kinematic_state_;
    robot_model::RobotModelPtr kinematic_model_;
    const robot_state::JointModelGroup *joint_model_group_;

    // std::vector<std::string> arm_joint_names_;
    // Scene *planning_scene_;
    // Move_Group *move_group_;
    // Robot_Model_Loader *robot_model_loader_;
    // ros::Subscriber arm_state_sub_, plan_sub_;
    // std::vector<Eigen::VectorXd> global_results_;

public:
    RRTPlanner(){}
    RRTPlanner(ros::NodeHandle nh, const robot_model::RobotModelPtr kinematic_model);
    bool CheckCollision(VectorXd proper_point, VectorXd new_point, const planning_scene::PlanningScenePtr& col_scene, double inter_step) const;
    std::vector<VectorXd> Pruning_path(const planning_scene::PlanningScenePtr& col_scene);
    bool RRT_Plan(const planning_scene::PlanningScenePtr& col_scene, const VectorXd& start_conf, const VectorXd& end_conf, std::vector<VectorXd>& results);
    // void armStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    // void planCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg);
    ~RRTPlanner();
};
}   //namespace

#endif //RRT_PLANNER_RRTPLANNER_H