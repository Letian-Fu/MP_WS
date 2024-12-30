#ifndef MYPLANNER_PLANNER_H
#define MYPLANNER_PLANNER_H

#pragma once
#include <cstddef>
#include <cstring>
#include "global_planner/rrtplanner.h" 
#include "local_planner/Optimizer.h"
#include <gtsam/base/Matrix.h>
#include <opencv2/opencv.hpp>
#define HAVE_CSTDDEF
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
#undef HAVE_CSTDDEF
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Dashboard.h"
#include "DobotMove.h"
#include "Feedback.h"
#include "ErrorInfoBean.h"
#include "ErrorInfoHelper.h"

struct ADBodySphere {
    size_t link_id;        // 关联的关节索引
    double radius;         // 球体半径
    Eigen::Vector3d center; // 球心相对于关节基坐标的偏移

    // 构造函数
    ADBodySphere(size_t id, double r, const Eigen::Vector3d& c)
        : link_id(id), radius(r), center(c) {}
};

// 定义机械臂类
class ADArm {
public:
    size_t dof_;                                    // 机械臂的自由度
    Eigen::VectorXd a_, alpha_, d_, theta_bias_;    // DH 参数
    Eigen::Matrix4d base_pose_;                     // 基坐标系的位姿
    std::vector<ADBodySphere> body_spheres_;          // 碰撞球体

    ADArm(){}
    // 构造函数
    ADArm(size_t dof, const Eigen::VectorXd& a, const Eigen::VectorXd& alpha,
        const Eigen::VectorXd& d, const Eigen::Matrix4d& base_pose,
        const Eigen::VectorXd& theta_bias,
        const std::vector<ADBodySphere>& body_spheres)
        : dof_(dof), a_(a), alpha_(alpha), d_(d), base_pose_(base_pose),
        theta_bias_(theta_bias), body_spheres_(body_spheres) {}

    // 计算正运动学，返回每个关节的位姿
    template <typename T>
    void forwardKinematics(const std::vector<T>& joint_positions,
                        std::vector<Eigen::Matrix<T, 4, 4>>& link_poses) const {
        // 初始化
        link_poses.resize(dof_);

        // 基坐标系
        Eigen::Matrix<T, 4, 4> current_pose = base_pose_.template cast<T>();

        // 迭代计算每个关节的位姿
        for (size_t i = 0; i < dof_; i++) {
            // 将 DH 参数转换为模板类型 T
            T a = T(a_(i));
            T alpha = T(alpha_(i));
            T d = T(d_(i));
            T theta = joint_positions[i] + T(theta_bias_(i));

            // 计算当前关节的变换矩阵
            Eigen::Matrix<T, 4, 4> dh_transform = computeDHTransform(a, alpha, d, theta);

            // 更新当前位姿
            current_pose = current_pose * dh_transform;

            // 保存当前关节的位姿
            link_poses[i] = current_pose;
        }
    }

    // 计算碰撞球体的位置
    template <typename T>
    void sphereCenters(const std::vector<T>& joint_positions,
                    std::vector<Eigen::Matrix<T, 3, 1>>& sphere_centers) const {
        // 计算每个关节的位姿
        std::vector<Eigen::Matrix<T, 4, 4>> link_poses;
        forwardKinematics(joint_positions, link_poses);

        // 计算每个碰撞球体的中心位置
        sphere_centers.resize(body_spheres_.size());
        for (size_t i = 0; i < body_spheres_.size(); i++) {
            const ADBodySphere& sphere = body_spheres_[i];
            Eigen::Matrix<T, 4, 1> sphere_center_homogeneous;
            sphere_center_homogeneous << sphere.center.template cast<T>(), T(1.0);

            // 通过正运动学变换得到球心的世界坐标
            Eigen::Matrix<T, 4, 4> link_pose = link_poses[sphere.link_id];
            Eigen::Matrix<T, 4, 1> world_sphere_center = link_pose * sphere_center_homogeneous;

            // 提取球心坐标
            sphere_centers[i] = world_sphere_center.template head<3>();
        }
    }

private:
    // 计算 DH 参数的变换矩阵
    template <typename T>
    Eigen::Matrix<T, 4, 4> computeDHTransform(T a, T alpha, T d, T theta) const {
        Eigen::Matrix<T, 4, 4> transform;
        transform << CppAD::cos(theta), -CppAD::sin(theta) * CppAD::cos(alpha),
                    CppAD::sin(theta) * CppAD::sin(alpha), a * CppAD::cos(theta),
                    CppAD::sin(theta), CppAD::cos(theta) * CppAD::cos(alpha),
                    -CppAD::cos(theta) * CppAD::sin(alpha), a * CppAD::sin(theta),
                    T(0), CppAD::sin(alpha), CppAD::cos(alpha), d,
                    T(0), T(0), T(0), T(1);
        return transform;
    }
};


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
    string group_name_;

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
    double max_vel_,traj_total_time_;
    bool ref_flag_;
    int count_;
    double obs_thresh_;
    double traj_cut_;
    // esdf相关
    gp_planner::SDF sdf_;
    std::vector<gtsam::Matrix> static_data_;
    std::vector<gtsam::Matrix> dynamic_data_;
    std::vector<gtsam::Matrix> map_;
    std::vector<gtsam::Matrix> prob_map_;
    std::string static_file_,dynamic_file_;
    // 优化器参数
    gp_planner::OptimizerSetting opt_setting_;
    double goal_sigma_;
    double joint_pos_limits_up_,joint_pos_limits_down_;
    double Qc_;
    int max_iterations_;
    /// obstacle cost settings
    double epsilon_;          // eps of hinge loss function (see the paper)
    double cost_sigma_;       // sigma of obstacle cost (see the paper)
    int obs_check_inter_;  // number of point interpolated for obstacle cost,
    double fix_pose_sigma_, fix_vel_sigma_, pos_limit_sigma_, vel_limit_sigma_;
    // 规划结果
    std::vector<Eigen::VectorXd> global_results_;
    std::vector<Eigen::VectorXd> local_results_;
    gtsam::Values init_values_,exec_values_;
    bool use_random_perturbation_,use_obstacle_gradient_;
    double perturbation_scale_,gradient_step_size_;

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
    Vector3d obs_pos_,obs_direction_;
    double obs_vel_;
    // 控制器相关
    ros::Publisher local_path_pub_, global_path_pub_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gazebo_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> real_robot_client_;
    bool real_robot_;
    std::shared_ptr<std::mutex> mutex_;  // 互斥锁
    ros::Timer map_timer_;  // 定时器
    VectorXd cur_vel_;
    double max_acc_;
    double w_obs_,w_pos_,w_v_,w_goal_;
    bool obs_constrained_;
    
    // 测试相关
    double path_length_,end_path_length_;
    long double plan_time_cost_, success_time_cost_;
    long plan_times_,success_times_;
    std::string planner_type_;
    ros::Time last_update_time_;

    // 实物相关
    std::string robotIp_;
    unsigned int controlPort_;
    unsigned int movePort_;
    unsigned int feekPort_;

    Dobot::CDobotMove m_DobotMove_;
    Dobot::CDashboard m_Dashboard_;
    Dobot::CFeedback m_CFeedback_;
    Dobot::CFeedbackData feedbackData_;
    Dobot::CErrorInfoBeans m_ErrorInfoBeans_;
    Dobot::CErrorInfoHelper m_CErrorInfoHelper_;
    int robot_speed_ratio_; //速度比例
    int robot_acc_ratio_;   //加速度比例
    int robot_cp_ratio_;    //平滑过渡比例
    int control_frequency_;
    // 用于记录时间的变量
    std::chrono::time_point<std::chrono::high_resolution_clock> last_record_time_; // 上一次记录的时间点
    bool plan_real_robot_;
    ros::Publisher joint_pub;


public:
    MyPlanner(ros::NodeHandle& nh);
    ~MyPlanner();

    void armStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void GlobalPlanCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg);
    void DynamicCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void LocalPlanningCallback(const ros::TimerEvent&);
    void obstacleInfoCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void readSDFFile(const ros::TimerEvent&);
    void publishTrajectory(int exec_step,bool pub_vel);
    void publishLocalPath();
    void publishGlobalPath();
    void generateCombinations(std::vector<int>& current, int index, std::vector<std::vector<int>>& combinations);
    void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void activeCb();
    void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
    std::vector<VectorXd> mpcSolve(const vector<VectorXd>& ref_path, const VectorXd& x0, const VectorXd& obs_pos, const double& obs_vel, const VectorXd& obs_direction);

    std::vector<VectorXd> generateTraj(const VectorXd& cur_pos);

    // 找到全局路径中距离当前位置最近的点
    int findClosestPoint(const vector<VectorXd>& globalPath);
    // 从全局路径获取局部参考路径
    vector<VectorXd> getLocalRefPath();
    // 从全局路径生成初始values
    gtsam::Values InitWithRef(const std::vector<Eigen::VectorXd>& ref_path, int total_step);
    // 对初值添加随机扰动
    gtsam::Values addRandomPerturbation(const gtsam::Values& init_values, double perturbation_scale);
    // 根据障碍物梯度改变初值
    gtsam::Values adjustByObstacleGradient(const gtsam::Values& init_values, const gp_planner::SDF& sdf, double step_size);

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

    inline VectorXd limitJointVelocities(const Eigen::VectorXd& velocities, double max_velocity) {
        Eigen::VectorXd limited_velocities = velocities;

        // 遍历每个分量，限制其值
        for (int i = 0; i < limited_velocities.size(); ++i) {
            if (limited_velocities[i] > max_velocity) {
                limited_velocities[i] = max_velocity;
            } else if (limited_velocities[i] < -max_velocity) {
                limited_velocities[i] = -max_velocity;
            }
        }

        return limited_velocities;
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