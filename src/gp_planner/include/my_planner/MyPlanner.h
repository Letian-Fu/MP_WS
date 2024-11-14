#ifndef MYPLANNER_PLANNER_H
#define MYPLANNER_PLANNER_H

#pragma once

#include "global_planner/rrtplanner.h" 
#include "local_planner/Optimizer.h"
#include <gtsam/base/Matrix.h>
#include <opencv2/opencv.hpp>

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
    int control_inter_, ref_inter_num_;
    double goal_tolerance_;
    double decay_factor_;   //衰减因子
    // esdf相关
    gp_planner::SDF sdf_;
    std::vector<gtsam::Matrix> static_data_;
    std::vector<gtsam::Matrix> dynamic_data_;
    std::vector<gtsam::Matrix> map_;
    std::vector<gtsam::Matrix> prob_map_;
    // 优化器参数
    gp_planner::OptimizerSetting opt_setting_;
    // 规划结果
    std::vector<Eigen::VectorXd> global_results_;
    std::vector<Eigen::VectorXd> local_results_;
    gtsam::Values exec_values_;

    // ros相关
    ros::NodeHandle nh_;
    ros::Subscriber arm_state_sub_, plan_sub_,obs_sub_,map_sub_;
    ros::Publisher read_pub_;
    std::string arm_state_topic_;
    bool is_plan_success_, is_global_success_, is_local_success_;
    Eigen::VectorXd start_conf_, end_conf_, arm_pos_, start_vel_, end_vel_;
    // 定时器相关
    double local_planner_frenquency_;
    ros::Timer planner_timer_;
    // 动态障碍物参数
    VectorXd obs_info_;
    // 控制器相关
    ros::Publisher local_path_pub_, global_path_pub_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_;
    std::shared_ptr<std::mutex> mutex_;  // 互斥锁
    ros::Timer map_timer_;  // 定时器

public:
    MyPlanner(ros::NodeHandle& nh);
    ~MyPlanner();

    void armStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void GlobalPlanCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg);
    void DynamicCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void LocalPlanningCallback(const ros::TimerEvent&);
    void readSDFFile(const ros::TimerEvent&);
    void publishTrajectory();
    void publishLocalPath();
    void publishGlobalPath();

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
    // // 将gtsam::Matrix转换为cv::Mat
    inline cv::Mat gtsamMatrixToCvMat(const gtsam::Matrix& matrix) {
        cv::Mat cvMat(matrix.rows(), matrix.cols(), CV_8U);
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                cvMat.at<uchar>(i, j) = static_cast<uchar>(matrix(i, j) > 0.75) * 255;
            }
        }
        return cvMat;
    }

    // // 将cv::Mat转换为gtsam::Matrix
    // inline gtsam::Matrix cvMatToGtsamMatrix(const cv::Mat& cvMat) {
    //     gtsam::Matrix matrix(cvMat.rows, cvMat.cols);
    //     for (int i = 0; i < cvMat.rows; ++i) {
    //         for (int j = 0; j < cvMat.cols; ++j) {
    //             matrix(i, j) = static_cast<float>(cvMat.at<uchar>(i, j)) / 255.0;
    //         }
    //     }
    //     return matrix;
    // }
    // // Function to convert a 3D cv::Mat back to vector of gtsam::Matrix
    // inline std::vector<gtsam::Matrix> convertToGtsamMatrices(const cv::Mat& mat3D) {
    //     int depth = mat3D.size[0];
    //     int rows = mat3D.size[1];
    //     int cols = mat3D.size[2];
    //     std::vector<gtsam::Matrix> gtsam_matrices;
    //     gtsam_matrices.reserve(depth);
    //     for (int z = 0; z < depth; ++z) {
    //         gtsam::Matrix slice(rows, cols);
    //         for (int i = 0; i < rows; ++i) {
    //             for (int j = 0; j < cols; ++j) {
    //                 slice(i, j) = mat3D.at<double>(z, i, j);
    //             }
    //         }
    //         gtsam_matrices.push_back(slice);
    //     }
    //     return gtsam_matrices;
    // }
    // struct Voxel {
    //     int x, y, z;
    //     double dist;
    // };
    // inline cv::Mat compute3DDistanceTransform(const cv::Mat& binaryMap) {
    //     int depth = binaryMap.size[0];
    //     int rows = binaryMap.size[1];
    //     int cols = binaryMap.size[2];
    //     int sizes[] = {depth, rows, cols};
    //     cv::Mat distMap(3, sizes, CV_64F, cv::Scalar(std::numeric_limits<double>::max()));
    //     std::queue<Voxel> q;
    //     for (int z = 0; z < depth; ++z) {
    //         for (int y = 0; y < rows; ++y) {
    //             for (int x = 0; x < cols; ++x) {
    //                 if (binaryMap.at<uchar>(z, y, x) == 0) {
    //                     distMap.at<double>(z, y, x) = 0;
    //                     q.push({x, y, z, 0});
    //                 }
    //             }
    //         }
    //     }
    //     int dX[6] = {1, -1, 0, 0, 0, 0};
    //     int dY[6] = {0, 0, 1, -1, 0, 0};
    //     int dZ[6] = {0, 0, 0, 0, 1, -1};
    //     while (!q.empty()) {
    //         Voxel v = q.front();
    //         q.pop();
    //         for (int i = 0; i < 6; ++i) {
    //             int nx = v.x + dX[i];
    //             int ny = v.y + dY[i];
    //             int nz = v.z + dZ[i];
    //             if (nx >= 0 && ny >= 0 && nz >= 0 && nx < cols && ny < rows && nz < depth) {
    //                 double newDist = v.dist + 1;
    //                 if (newDist < distMap.at<double>(nz, ny, nx)) {
    //                     distMap.at<double>(nz, ny, nx) = newDist;
    //                     q.push({nx, ny, nz, newDist});
    //                 }
    //             }
    //         }
    //     }
    //     return distMap;
    // }
    // inline void mergeMaps(const std::vector<gtsam::Matrix>& static_data, const std::vector<gtsam::Matrix>& dynamic_data, std::vector<gtsam::Matrix>& map) {
    //     // 清空map_，以便重新填充
    //     map.clear();
    //     // 检查地图的维度是否相同
    //     if (static_data.size() != dynamic_data.size()) {
    //         throw std::runtime_error("Static and dynamic maps must have the same number of layers");
    //     }
    //     // 合并static_data和dynamic_data
    //     for (size_t i = 0; i < static_data.size(); ++i) {
    //         const gtsam::Matrix& static_matrix = static_data[i];
    //         const gtsam::Matrix& dynamic_matrix = dynamic_data[i];
    //         // 检查每一层的维度是否相同
    //         if (static_matrix.rows() != dynamic_matrix.rows() || static_matrix.cols() != dynamic_matrix.cols()) {
    //             throw std::runtime_error("Static and dynamic matrices in layer " + std::to_string(i) + " must have the same dimensions");
    //         }
    //         // 逐元素相加
    //         gtsam::Matrix merged_matrix = static_matrix.array() + dynamic_matrix.array();
    //         map.push_back(merged_matrix);
    //     }
    // }
    // // Function to compute the signed distance field for the entire 3D space
    // cv::Mat signedDistanceField3D(const std::vector<gtsam::Matrix>& ground_truth_map, double cell_size);
    // // 更新动态地图
    // void add_dynamic_obstacle(const std::vector<int>& position, const double& size, 
    //               const double& velocity, const std::vector<double>& direction, 
    //               double total_time, std::vector<gtsam::Matrix>& map, std::vector<gtsam::Matrix>& prob_map);

};


#endif //MYPLANNER_PLANNER_H