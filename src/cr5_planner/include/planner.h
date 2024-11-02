//
// Created by robert on 9/4/22.
//

#ifndef CR5_PLANNER_PLANNER_H
#define CR5_PLANNER_PLANNER_H

#include "header.h"
#include "tree.h"
#include "kinematics.h"
#include "interpolation.h"


class MyPlanner {
public:
    int dof;
    int plan_flag;
    bool is_plan_success;
    VectorXd start_conf, goal_conf, arm_pos;
    std::vector<std::string> arm_joint_names;
//    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
    typedef moveit::planning_interface::MoveGroupInterface Move_Group;
    typedef moveit::planning_interface::PlanningSceneInterface Scene;
    typedef robot_model_loader::RobotModelLoader Robot_Model_Loader;
    robot_state::RobotStatePtr kinematic_state;
    const robot_state::JointModelGroup *joint_model_group;
    robot_model::RobotModelPtr kinematic_model;
    Scene *planning_scene;
    Move_Group *move_group;
//    TrajClient *traj_client;
    Robot_Model_Loader *robot_model_loader;
//    control_msgs::FollowJointTrajectoryGoal traj;
    ros::Subscriber arm_state_sub, plan_sub;
    std::string arm_state_topic;
    VectorXd dh_alpha, dh_a, dh_d, dh_theta;
    Tree tree;
    int NUM_INITIATION;
    double step;
    double step_rate;
    double fai;
    double fai_rate;
    int MAX_TREE_NUM;
    int MAX_FAIL_NUM;
    int sockfd;
    struct sockaddr_in servaddr;
    int is_adapt_step;
    int is_adapt_fai;
public:
    explicit MyPlanner(ros::NodeHandle nh);
    void armStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void planCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg);
    bool CheckCollision(VectorXd proper_point, VectorXd new_point, const planning_scene::PlanningScenePtr& col_scene, double inter_step) const;
    void execute();
    void execute_real();
    std::vector<VectorXd> Pruning_path(const planning_scene::PlanningScenePtr& col_scene);
    VectorXd MyPlan(const planning_scene::PlanningScenePtr& col_scene);
    VectorXd MyPlan_RRTConnect(const planning_scene::PlanningScenePtr& col_scene);
    VectorXd MyPlan_BG(const planning_scene::PlanningScenePtr& col_scene);
    ~MyPlanner();

};
#endif //CR5_PLANNER_PLANNER_H
