#include "PriorFactor.h"
#include "GaussianFactor.h"
#include "ObsFactor.h"
#include "Optimizer.h"
#include "ArmModel.h"
#include "SDF.h"
// #include <gpmp2/kinematics/JointLimitFactorVector.h>
// #include <gpmp2/kinematics/VelocityLimitFactorVector.h>
// #include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
// #include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
// #include <gpmp2/gp/GaussianProcessPriorLinear.h>
// #include <gpmp2/planner/TrajUtils.h>
// #include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

// using namespace gp_planner;

void add_obstacle(const std::vector<int>& position, const std::vector<int>& size, std::vector<gtsam::Matrix>& map, std::vector<gtsam::Vector>& corners) {
    int half_size_row = std::floor((size[0] - 1) / 2);
    int half_size_col = std::floor((size[1] - 1) / 2);
    int half_size_z = std::floor((size[2] - 1) / 2);

    for (int i = position[0] - half_size_row; i <= position[0] + half_size_row; ++i) {
        for (int j = position[1] - half_size_col; j <= position[1] + half_size_col; ++j) {
            for (std::size_t k = position[2] - half_size_z; k <= position[2] + half_size_z; ++k) {
                if (i >= 0 && i < map[0].rows() && j >= 0 && j < map[0].cols() && k >= 0 && k < map.size()) {
                    map[k](i, j) = 1.0;
                }
            }
        }
    }

    gtsam::Vector corner(6);
    corner << position[0] - half_size_row, position[0] + half_size_row,
             position[1] - half_size_col, position[1] + half_size_col,
             position[2] - half_size_z, position[2] + half_size_z;
    corners.push_back(corner);
}

std::vector<gtsam::Matrix> signedDistanceField3D(const std::vector<gtsam::Matrix>& ground_truth_map, double cell_size) {
    std::vector<gtsam::Matrix> field(ground_truth_map.size());
    for (std::size_t k = 0; k < ground_truth_map.size(); ++k) {
        field[k] = gtsam::Matrix::Zero(ground_truth_map[k].rows(), ground_truth_map[k].cols());

        for (int i = 0; i < ground_truth_map[k].rows(); ++i) {
            for (int j = 0; j < ground_truth_map[k].cols(); ++j) {
                if (ground_truth_map[k](i, j) > 0.75) {
                    double distance = std::numeric_limits<double>::max();
                    for (int ni = -1; ni <= 1; ++ni) {
                        for (int nj = -1; nj <= 1; ++nj) {
                            if (i + ni >= 0 && i + ni < ground_truth_map[k].rows() && j + nj >= 0 && j + nj < ground_truth_map[k].cols()) {
                                double dist = std::sqrt(std::pow(i + ni - j, 2) + std::pow(j + nj - j, 2));
                                distance = std::min(distance, dist);
                            }
                        }
                    }
                    field[k](i, j) = distance * cell_size;
                } else {
                    field[k](i, j) = -1.0;
                }
            }
        }
    }
    return field;
}

int main(int argc, char **argv){
    int dof = 6;
    gtsam::Vector a = gtsam::Vector::Zero(6); 
    a << 0, -0.427, -0.357, 0, 0, 0;
    gtsam::Vector d = gtsam::Vector::Zero(6);
    d << 0.147, 0, 0, 0.141, 0.116, 0.105;
    gtsam::Vector theta_bias = gtsam::Vector::Zero(6);
    theta_bias << 0, -1.5708, 0, -1.5708, 0, 0;
    gtsam::Vector alpha = gtsam::Vector::Zero(6);
    alpha << 1.5708, 0.0, 0.0, 1.5708, -1.5708, 0.0;
    gtsam::Pose3 base_pose(gtsam::Rot3(), gtsam::Point3(0, 0, 0));
    gtsam::Vector xs(14),zs(14),ys(14),rs(14);
    vector<std::size_t> js={0,0,1,1,1,1,1,1,2,2,2,3,4,5};
    xs<<0,0,0,0.105,0.210,0.315,0.420,0,0.11,0.22,0,0,0,0;
    ys<<-0.1,0,0,0,0,0,0,0,0,0,0,0,0,0;
    zs<<0,0,0.1,0.1,0.1,0.1,0.1,0,0,0,0,0,0,0;
    rs<<0.08,0.08,0.06,0.06,0.06,0.06,0.06,0.08,0.06,0.06,0.08,0.07,0.06,0.06;
    // 初始化数据集
    gtsam::Point3 origin(-1.0, -1.0, -0.5);
    double cell_size = 0.05;
    std::size_t rows = 40, cols = 40, z = 40;
    std::vector<gtsam::Matrix> map(z, gtsam::Matrix::Zero(rows, cols));
    // 添加障碍物
    // std::vector<gtsam::Vector> corners;
    // add_obstacle({20, 9, 10}, {3, 3, 3}, map, corners);
    // add_obstacle({20, 20, 5}, {30, 30, 3}, map, corners);

    // 计算签名距离场
    auto field = signedDistanceField3D(map, cell_size);
    gtsam::Vector start_conf(6),end_conf(6),start_vel(6),end_vel(6);
    start_conf<<-0.8, -1.7, 1.64, 1.29, 1.1, -1.106;
    end_conf<<0.5, 0.94, 0, 1.6, 0, -0.919;
    start_vel<<0,0,0,0,0,0;
    end_vel<<0,0,0,0,0,0;

    // // vector<gpmp2::BodySphere> body_spheres;
    // // for(int i=0;i<1;i++){
    // //     body_spheres.push_back(gpmp2::BodySphere(js[i],rs[i],gtsam::Point3(xs[i],ys[i],zs[i])));
    // // }
    // // gpmp2::Arm cr5(dof,a,alpha,d,base_pose,theta_bias);
    // // gpmp2::ArmModel cr5_model(cr5,body_spheres);
    // // // 创建 SignedDistanceField 对象
    // // gpmp2::SignedDistanceField sdf(origin, cell_size, rows, cols, z);
    // // for (std::size_t z = 0; z < field.size(); ++z) {
    // //     sdf.initFieldData(z, field[z]);
    // // }
    // // gpmp2::TrajOptimizerSetting setting(dof);
    // // setting.setGaussNewton();
    // // // setting.opt_type = gpmp2::TrajOptimizerSetting::GaussNewton;
    // // gtsam::Values init_values = gpmp2::initArmTrajStraightLine(start_conf,end_conf,setting.total_step);
    // // const double delta_t = setting.total_time / static_cast<double>(setting.total_step);
    // // const double inter_dt = delta_t / static_cast<double>(setting.obs_check_inter + 1);
    // // gtsam::NonlinearFactorGraph graph;
    // // for(std::size_t i=0;i<=setting.total_step;i++){
    // //     gtsam::Key pose_key = gtsam::Symbol('x',i);
    // //     gtsam::Key vel_key = gtsam::Symbol('v',i);
    // //     // start and end
    // //     if(i == 0){
    // //         graph.add(gtsam::PriorFactor<gtsam::Vector>(pose_key,start_conf,setting.conf_prior_model));
    // //         graph.add(gtsam::PriorFactor<gtsam::Vector>(vel_key,start_vel,setting.vel_prior_model));
    // //     } else if(i == setting.total_step){
    // //         graph.add(gtsam::PriorFactor<gtsam::Vector>(pose_key,end_conf,setting.conf_prior_model));
    // //         graph.add(gtsam::PriorFactor<gtsam::Vector>(vel_key,end_vel,setting.vel_prior_model));
    // //     }
    // //     // Limits for joints and velocity
    // //     // if(setting.flag_pos_limit){
    // //     //     graph.add(gpmp2::JointLimitFactorVector(pose_key,setting.pos_limit_model,setting.joint_pos_limits_down,setting.joint_pos_limits_up,setting.pos_limit_thresh));
    // //     // }
    // //     // if(setting.flag_vel_limit){
    // //     //     graph.add(gpmp2::VelocityLimitFactorVector(vel_key,setting.vel_limit_model,setting.vel_limits,setting.vel_limit_thresh));
    // //     // }
    // //     // non-interpolated cost factor
    // //     graph.add(gpmp2::ObstacleSDFFactorArm(pose_key, cr5_model, sdf, setting.cost_sigma, setting.epsilon));
        
    // //     if(i>0){
    // //         gtsam::Key last_pose_key = gtsam::Symbol('x', i - 1);
    // //         gtsam::Key last_vel_key = gtsam::Symbol('v', i - 1);
    // //         // GP factor
    // //         graph.add(gpmp2::GaussianProcessPriorLinear(last_pose_key,last_vel_key,pose_key,vel_key,delta_t,setting.Qc_model));
    // //         // interpolated cost factor
    // //         if(setting.obs_check_inter > 0){
    // //             for(std::size_t j = 1; j<=setting.obs_check_inter; j++){
    // //                 const double tau = inter_dt * static_cast<double>(j);
    // //                 graph.add(gpmp2::ObstacleSDFFactorGPArm(last_pose_key, last_vel_key, pose_key, vel_key,
    // //                                     cr5_model, sdf, setting.cost_sigma, setting.epsilon, setting.Qc_model, delta_t, tau));
    // //             }
    // //         }
    // //     }
    // // }
    // // gtsam::Values result = gpmp2::optimize(graph, init_values, setting);

    vector<gp_planner::BodySphere> body_spheres;
    for(int i=0;i<1;i++){
        body_spheres.push_back(gp_planner::BodySphere(js[i],rs[i],gtsam::Point3(xs[i],ys[i],zs[i])));
    }
    gp_planner::Arm cr5(dof,a,alpha,d,base_pose,theta_bias);
    gp_planner::ArmModel cr5_model(cr5,body_spheres);
    // 创建 SignedDistanceField 对象
    gp_planner::SDF sdf(origin, cell_size, rows, cols, z);
    for (std::size_t z = 0; z < field.size(); ++z) {
        sdf.initFieldData(z, field[z]);
    }
    // OptimizerSetting setting(dof);
    gp_planner::OptimizerSetting setting(dof);
    setting.setGaussNewton();
    setting.total_step = 10;
    setting.obs_check_inter = 5;
    setting.cost_sigma = 0.2;
    setting.epsilon = 0.1;
    setting.set_conf_prior_model(0.0001);
    setting.set_vel_prior_model(0.0001);
    const double delta_t = setting.total_time / static_cast<double>(setting.total_step);
    const double inter_dt = delta_t / static_cast<double>(setting.obs_check_inter + 1);
    //build graph
    gtsam::NonlinearFactorGraph graph;
    for(std::size_t i=0;i<=setting.total_step;i++){
        gtsam::Key pose_key = gtsam::Symbol('x',i);
        gtsam::Key vel_key = gtsam::Symbol('v',i);
        // start and end
        if(i == 0){
            graph.add(gp_planner::PriorFactorConf(pose_key,start_conf,setting.conf_prior_model));
            graph.add(gp_planner::PriorFactorVel(vel_key,start_vel,setting.vel_prior_model));
        } else if(i == setting.total_step){
            graph.add(gp_planner::PriorFactorConf(pose_key,end_conf,setting.conf_prior_model));
            graph.add(gp_planner::PriorFactorVel(vel_key,end_vel,setting.vel_prior_model));
        }
        // if(i == 0){
        //     graph.add(gtsam::PriorFactor<gtsam::Vector>(pose_key,start_conf,setting.conf_prior_model));
        //     graph.add(gtsam::PriorFactor<gtsam::Vector>(vel_key,start_vel,setting.vel_prior_model));
        // } else if(i == setting.total_step){
        //     graph.add(gtsam::PriorFactor<gtsam::Vector>(pose_key,end_conf,setting.conf_prior_model));
        //     graph.add(gtsam::PriorFactor<gtsam::Vector>(vel_key,end_vel,setting.vel_prior_model));
        // }
        // Limits for joints and velocity
        if(setting.flag_pos_limit){
            graph.add(gp_planner::JointLimitFactor(pose_key,setting.pos_limit_model,setting.joint_pos_limits_down,setting.joint_pos_limits_up,setting.pos_limit_thresh));
        }
        if(setting.flag_vel_limit){
            graph.add(gp_planner::VelLimitFactor(vel_key,setting.vel_limit_model,setting.vel_limits,setting.vel_limit_thresh));
        }
        // non-interpolated cost factor
        graph.add(gp_planner::ObsFactor(pose_key, cr5_model, sdf, setting.cost_sigma, setting.epsilon));
        if(i>0){
            gtsam::Key last_pose_key = gtsam::Symbol('x', i - 1);
            gtsam::Key last_vel_key = gtsam::Symbol('v', i - 1);
            // GP factor
            graph.add(gp_planner::GPFactor(last_pose_key,last_vel_key,pose_key,vel_key,delta_t,setting.Qc_model));
            // interpolated cost factor
            if(setting.obs_check_inter > 0){
                for(std::size_t j = 1; j<=setting.obs_check_inter; j++){
                    const double tau = inter_dt * static_cast<double>(j);
                    graph.add(gp_planner::ObsGPFactor(last_pose_key, last_vel_key, pose_key, vel_key,
                                        cr5_model, sdf, setting.cost_sigma, setting.epsilon, setting.Qc_model, delta_t, tau));
                }
            }
        }
    }
    std::cout<<"graph initialized"<<std::endl;
    graph.print("Graph: ");
    gtsam::Values init_values = gp_planner::initArmTrajStraightLine(start_conf,end_conf,setting.total_step);
    for (const auto& kv : init_values) {
        const gtsam::Symbol& key = kv.key;
        const gtsam::Vector& value = init_values.at<gtsam::Vector>(key);

        // 根据键的类型（位置或速度）确定要打印的内容
        if (key.chr() == 'x') {
            std::cout << "Position at step " << key.index() << ": ";
        } else if (key.chr() == 'v') {
            std::cout << "Average Velocity: ";
        }

        // 打印向量值
        for (size_t j = 0; j < value.size(); ++j) {
            std::cout << std::setprecision(3) << value(j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "initial error=" << graph.error(init_values) << std::endl;
    gtsam::Values result = gp_planner::optimize(graph, init_values, setting);
    std::cout << "final error=" << graph.error(result) << std::endl;
    for (const auto& kv : result) {
        const gtsam::Symbol& key = kv.key;
        const gtsam::Vector& value = result.at<gtsam::Vector>(key);

        // 根据键的类型（位置或速度）确定要打印的内容
        if (key.chr() == 'x') {
            std::cout << "Position at step " << key.index() << ": ";
        } else if (key.chr() == 'v') {
            std::cout << "Average Velocity: ";
        }

        // 打印向量值
        for (size_t j = 0; j < value.size(); ++j) {
            std::cout << std::setprecision(3) << value(j) << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}