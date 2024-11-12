#ifndef MYPLANNER_PLANNER_H
#define MYPLANNER_PLANNER_H

#pragma once

#include "global_planner/rrtplanner.h" 
#include "local_planner/Optimizer.h"
#include <gtsam/base/Matrix.h>

class MyPlanner{
private:
    typedef moveit::planning_interface::MoveGroupInterface Move_Group;
    typedef moveit::planning_interface::PlanningSceneInterface Scene;
    typedef robot_model_loader::RobotModelLoader Robot_Model_Loader;
public:
    // moveit相关参数
    std::vector<std::string> arm_joint_names_;
    robot_state::RobotStatePtr kinematic_state_;
    robot_model::RobotModelPtr kinematic_model_;
    Scene *planning_scene_;
    Move_Group *move_group_;
    Robot_Model_Loader *robot_model_loader_;
    std::vector<std::string> joint_all_names_;
    const robot_state::JointModelGroup *joint_model_group_;

    // 机器人相关参数
    int dof_;
    Eigen::VectorXd dh_alpha_, dh_a_, dh_d_, dh_theta_;
    gp_planner::Arm arm_;
    gp_planner::ArmModel robot_;

    // 全局规划器
    rrt_planner::RRTPlanner rrt_planner_;
    // 局部规划相关参数
    double delta_t_, inter_dt_, exec_step_;
    int control_inter_;
    gp_planner::SDF sdf_;
    gp_planner::OptimizerSetting opt_setting_;
    // 规划结果
    std::vector<Eigen::VectorXd> global_results_;
    std::vector<Eigen::VectorXd> local_results_;
    gtsam::Values exec_values_;

    // ros相关
    ros::Subscriber arm_state_sub_, plan_sub_,obs_sub_;
    std::string arm_state_topic_;
    bool is_plan_success_, is_global_success_, is_local_success_;
    Eigen::VectorXd start_conf_, end_conf_, arm_pos_, start_vel_, end_vel_;
    // 定时器相关
    double local_planner_frenquency_;
    ros::Timer timer;
    // 动态障碍物参数
    VectorXd obs_info_;

public:
    MyPlanner(ros::NodeHandle nh);
    ~MyPlanner();

    void armStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void planCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg);
    void obstacleCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void LocalPlanningCallback(const ros::TimerEvent&);
    void execute();
    // 找到全局路径中距离当前位置最近的点
    int findClosestPoint(const vector<VectorXd>& globalPath);
    // 从全局路径获取局部参考路径
    vector<VectorXd> getLocalRefPath();
    // 从全局路径生成初始values
    gtsam::Values InitWithRef(const std::vector<Eigen::VectorXd>& ref_path, int total_step);

    // GtsamVector转换成EigenVector
    inline Eigen::VectorXd ConvertToEigenVector(const gtsam::Vector &gtsamVector){
        Eigen::VectorXd eigenVector;
        eigenVector.resize(gtsamVector.size());
        for(size_t i=0;i<gtsamVector.size();i++){
            eigenVector[i] = gtsamVector[i];
        }
        return eigenVector;
    }
    // EigenVector转换成GtsamVector
    inline gtsam::Vector ConvertToGtsamVector(const Eigen::VectorXd &eigenVector){
        gtsam::Vector gtsamVector;
        gtsamVector.resize(eigenVector.size());
        for(size_t i=0;i<eigenVector.size();i++){
            gtsamVector[i] = eigenVector[i];
        }
        return gtsamVector;
    }

    // 计算两点之间的距离
    inline double calculateDistance(const VectorXd& point1, const VectorXd& point2) {
        return (point1 - point2).norm();
    }

    // 线性插值
    inline VectorXd interpolatePoint(const VectorXd& start, const VectorXd& end, double t) {
        return start + t * (end - start);
    }



};


#endif //MYPLANNER_PLANNER_H