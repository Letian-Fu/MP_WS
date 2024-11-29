#include "my_planner/MyPlanner.h"
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <iomanip> // for std::put_time

MyPlanner::MyPlanner(ros::NodeHandle& nh):
nh_(nh),
real_robot_client_("/cr5_robot/joint_controller/follow_joint_trajectory", true),
gazebo_client_("/arm_controller/follow_joint_trajectory", true)
{
    move_group_ = new Move_Group("arm");
    planning_scene_ = new Scene();
    robot_model_loader_ = new Robot_Model_Loader("robot_description");
    kinematic_model_ = robot_model_loader_->getModel();
    kinematic_state_=std::make_shared<robot_state::RobotState>(kinematic_model_);
    kinematic_state_->setToDefaultValues();
    joint_model_group_ = kinematic_model_->getJointModelGroup("arm");
    if (nh.hasParam("robot/DOF"))
        nh.getParam("robot/DOF", dof_);
    else{
        dof_=6;
        cout<<"No DOF Data Loaded!"<<endl;
    }
    start_conf_.resize(dof_);
    end_conf_.resize(dof_);
    arm_pos_.resize(dof_);
    start_vel_.resize(dof_);
    start_vel_.setZero();
    end_vel_.resize(dof_);
    end_vel_.setZero();
    if (nh.hasParam("robot/arm_state_topic"))
        nh.getParam("robot/arm_state_topic", arm_state_topic_);
    else{
        cout<<"No arm_state_topic Data Loaded!"<<endl;    
    }
    if (nh.hasParam("robot/arm_joint_names"))
        nh.getParam("robot/arm_joint_names", arm_joint_names_);
    else{
        cout<<"No arm_joint_names Data Loaded!"<<endl;
    }
    dh_alpha_.resize(dof_);
    dh_a_.resize(dof_);
    dh_d_.resize(dof_);
    dh_theta_.resize(dof_);
    dh_alpha_<<M_PI_2,0.0,0.0,M_PI_2,-M_PI_2,0.0;
    dh_a_<<0.0,-0.427,-0.357,0.0,0.0,0.0;
    dh_d_<<0.147,0.0,0.0,0.141,0.116,0.105;
    dh_theta_<<0.0,-M_PI_2,0.0,-M_PI_2,0.0,0.0;
    
    vector<gp_planner::BodySphere> body_spheres;
    VectorXd xs(16),zs(16),ys(16),rs(16);
    vector<size_t> js={0,0,1,1,1,1,1,1,2,2,2,3,4,5,5,5};
    xs<<0,0,0,0.105,0.210,0.315,0.420,0,0.11,0.22,0,0,0,0,0,0;
    ys<<-0.1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    zs<<0,0,0.1,0.1,0.1,0.1,0.1,0,0,0,0,0,0,0,0.1,0.2;
    rs<<0.08,0.08,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.07,0.07;
    for(int i=0;i<xs.size();i++){
        body_spheres.push_back(gp_planner::BodySphere(js[i],rs[i],gtsam::Point3(xs[i],ys[i],zs[i])));
    }
    arm_ = gp_planner::Arm(dof_, dh_a_, dh_alpha_, dh_d_, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)), dh_theta_);
    robot_ = gp_planner::ArmModel(arm_,body_spheres);
    opt_setting_ = gp_planner::OptimizerSetting(static_cast<size_t>(dof_));
    int total_step;  // number of steps (states) optimized the whole trajectory
    /// joint position and velocity limit settings
    bool flag_pos_limit;  // whether enable joint position limits
    bool flag_vel_limit;  // whether enable velocity limits
    double joint_pos_limits_up, joint_pos_limits_down;  // joint position limits
    /// obstacle cost settings
    double epsilon;          // eps of hinge loss function (see the paper)
    double cost_sigma;       // sigma of obstacle cost (see the paper)
    int obs_check_inter;  // number of point interpolated for obstacle cost,
    double fix_pose_sigma, fix_vel_sigma, pos_limit_sigma, vel_limit_sigma;
    double Qc;
    string opt_type;
    nh.getParam("settings/total_time", traj_total_time_);
    nh.getParam("settings/total_step", total_step);
    nh.getParam("settings/flag_pos_limit", flag_pos_limit);
    nh.getParam("settings/flag_pos_limit", flag_vel_limit);
    nh.getParam("settings/joint_pos_limits_up", joint_pos_limits_up);
    nh.getParam("settings/joint_pos_limits_down", joint_pos_limits_down);
    nh.getParam("settings/max_vel", max_vel_);
    nh.getParam("settings/epsilon", epsilon);
    nh.getParam("settings/cost_sigma", cost_sigma);
    nh.getParam("settings/fix_pose_sigma", fix_pose_sigma);
    nh.getParam("settings/fix_vel_sigma", fix_vel_sigma);
    nh.getParam("settings/pos_limit_sigma", pos_limit_sigma);
    nh.getParam("settings/vel_limit_sigma", vel_limit_sigma);
    nh.getParam("settings/obs_check_inter", obs_check_inter);
    nh.getParam("settings/Qc", Qc);
    nh.getParam("settings/opt_type", opt_type);
    nh.getParam("settings/control_inter", control_inter_);
    nh.getParam("settings/ref_inter_num", ref_inter_num_);
    nh.getParam("settings/goal_tolerance", goal_tolerance_);

    gtsam::Vector joints_pos_limits_up(dof_), joints_pos_limits_down(dof_), vel_limits(dof_);
    gtsam::Vector pos_limit_sigmas(dof_), vel_limit_sigmas(dof_);
    for(size_t i = 0;i<dof_;i++){
        joints_pos_limits_up[i]=joint_pos_limits_up;
        joints_pos_limits_down[i]=joint_pos_limits_down;
        vel_limits[i]=max_vel_;
        pos_limit_sigmas[i]=pos_limit_sigma;
        vel_limit_sigmas[i]=vel_limit_sigma;
    }
    opt_setting_.set_total_time(traj_total_time_);
    // opt_setting_.set_total_time(6);
    opt_setting_.set_total_step(static_cast<size_t>(total_step));
    opt_setting_.set_conf_prior_model(fix_pose_sigma);
    opt_setting_.set_vel_prior_model(fix_vel_sigma);
    opt_setting_.set_epsilon(epsilon);
    opt_setting_.set_cost_sigma(cost_sigma);
    opt_setting_.set_cost_sigma(0.1);
    opt_setting_.set_obs_check_inter(static_cast<size_t>(obs_check_inter));
    opt_setting_.set_Qc_model(gtsam::Matrix::Identity(dof_,dof_)*Qc);

    opt_setting_.set_flag_pos_limit(flag_pos_limit);
    opt_setting_.set_flag_vel_limit(flag_vel_limit);
    opt_setting_.set_joint_pos_limits_up(joints_pos_limits_up);
    opt_setting_.set_joint_pos_limits_down(joints_pos_limits_down);
    opt_setting_.set_vel_limits(vel_limits);
    opt_setting_.set_pos_limit_model(pos_limit_sigmas);
    opt_setting_.set_vel_limit_model(vel_limit_sigmas);

    if(opt_type=="LM")  opt_setting_.setLM();
    else if(opt_type=="GaussNewton")    opt_setting_.setGaussNewton();
    else    opt_setting_.setDogleg();

    delta_t_ = opt_setting_.total_time / static_cast<double>(opt_setting_.total_step);
    inter_dt_ = delta_t_ / static_cast<double>(opt_setting_.obs_check_inter + 1);
    exec_step_ = opt_setting_.total_step + 10 * (opt_setting_.total_step - 1);

    // opt_setting_.setVerbosityError();
    // esdf相关
    nh.getParam("sdf/static_file", static_file_);
    nh.getParam("sdf/dynamic_file", dynamic_file_);
    double origin_x,origin_y,origin_z,cell_size;
    int rows,cols,z;
    nh.getParam("sdf/origin_x", origin_x);
    nh.getParam("sdf/origin_y", origin_y);
    nh.getParam("sdf/origin_z", origin_z);
    nh.getParam("sdf/cell_size", cell_size);
    nh.getParam("sdf/rows", rows);
    nh.getParam("sdf/cols", cols);
    nh.getParam("sdf/z", z);
    gtsam::Point3 origin(origin_x, origin_y, origin_z);
    static_data_ = std::vector<gtsam::Matrix>(z,gtsam::Matrix::Zero(rows,cols));
    dynamic_data_ = std::vector<gtsam::Matrix>(z,gtsam::Matrix::Zero(rows,cols));
    prob_map_ = std::vector<gtsam::Matrix>(z,gtsam::Matrix::Ones(rows,cols));
    map_ = std::vector<gtsam::Matrix>(z,gtsam::Matrix::Zero(rows,cols));
    // gp_planner::add_obstacle({20, 11, 17}, {5, 5, 5}, map_);
    // gp_planner::add_obstacle({20, 20, 4}, {20, 20, 2}, map_);
    // mergeMaps(static_data_, dynamic_data_, map_);
    // 创建 SignedDistanceField 对象
    sdf_ = gp_planner::SDF(origin, cell_size, rows, cols, z);
    // cv::Mat field_cv = signedDistanceField3D(map_,cell_size);
    // auto field = convertToGtsamMatrices(field_cv);
    // for (size_t z = 0; z < field.size(); ++z) {
    //     sdf_.initFieldData(z, field[z]);
    // }
    sdf_.loadSDF(static_file_);
    sdf_.saveSDF("/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_static.txt");

    rrt_planner_ = rrt_planner::RRTPlanner(nh, kinematic_model_);
    nh.getParam("settings/frequency", local_planner_frenquency_);
    planner_timer_ = nh.createTimer(ros::Duration(1.0 / local_planner_frenquency_), &MyPlanner::LocalPlanningCallback, this);
    arm_state_sub_ = nh.subscribe(arm_state_topic_, 1, &MyPlanner::armStateCallback,this);
    plan_sub_ = nh.subscribe("move_group/goal", 1, &MyPlanner::GlobalPlanCallback,this);
    obs_sub_ = nh.subscribe("/obstacle_info_mpc", 10, &MyPlanner::obstacleInfoCallback, this);
    map_sub_ = nh.subscribe("/map_updated", 1, &MyPlanner::DynamicCallback, this);
    local_path_pub_ = nh.advertise<nav_msgs::Path>("local_path",10);
    global_path_pub_ = nh.advertise<nav_msgs::Path>("global_path",10);
    read_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/is_reading",10);
    double map_update_frenquency;
    nh.getParam("settings/map_update_frenquency", map_update_frenquency);
    map_timer_ = nh_.createTimer(ros::Duration(1/map_update_frenquency), &MyPlanner::readSDFFile, this);  // 创建定时器

    // std::cout<<"Waiting for Gazebo action server..."<<std::endl;
    // gazebo_client_.waitForServer();
    // std::cout<<"Gazebo action server connected."<<std::endl;
    ROS_INFO("Waiting for Gazebo action server...");
    gazebo_client_.waitForServer();
    ROS_INFO("Gazebo action server connected.");

    real_robot_ = false;
    nh.getParam("real_robot", real_robot_);
    if(real_robot_){
        ROS_INFO("Waiting for Real Robot action server...");
        real_robot_client_.waitForServer();
        ROS_INFO("Real Robot action server connected.");
    }

    is_plan_success_ = false;
    is_global_success_ = false;
    is_local_success_ = false;
    ref_flag_ = false;

    path_length_ = 0;
    plan_time_cost_ = 0;
    plan_times_ = 0;
    end_path_length_ = 0;
    planner_type_ = "gp";
    nh.getParam("settings/planner_type", planner_type_);
}

gtsam::Values MyPlanner::InitWithRef(const std::vector<Eigen::VectorXd>& ref_path, int total_step){
    gtsam::Values init_values;
    gtsam::Vector init_conf = ConvertToGtsamVector(ref_path[0]);
    gtsam::Vector end_conf = ConvertToGtsamVector(ref_path[ref_path.size()-1]);
    // init pose
    for (size_t i = 0; i <= total_step; i++) {
        gtsam::Vector conf;
        conf = ConvertToGtsamVector(ref_path[i]);
        init_values.insert(gtsam::Symbol('x', i), conf);
    }
    // init vel as avg vel
    gtsam::Vector avg_vel = (end_conf - init_conf) / total_step;
    for (size_t i = 0; i <= total_step; i++)
        init_values.insert(gtsam::Symbol('v', i), avg_vel);
    return init_values;
}

void MyPlanner::generateCombinations(std::vector<int>& current, int index, std::vector<std::vector<int>>& combinations) {
    if (index == dof_) {
        combinations.push_back(current);
        return;
    }
    for (int i = 0; i < 3; ++i) {
        current[index] = i;
        generateCombinations(current, index + 1, combinations);
    }
}

vector<VectorXd> MyPlanner::generateTraj(const VectorXd& cur_pos){
    int numPoints = 5;
    vector<VectorXd> best_traj;
    double bestScore_obs = std::numeric_limits<double>::max();
    double bestScore_goal = std::numeric_limits<double>::max();
    double bestScore_gp = std::numeric_limits<double>::max();
    gp_planner::ObsFactor obs_factor = gp_planner::ObsFactor(gtsam::Symbol('x', 0), robot_, sdf_, opt_setting_.cost_sigma, 0);
    gp_planner::PriorFactorConf prior_factor = gp_planner::PriorFactorConf(gtsam::Symbol('x', 0),ConvertToGtsamVector(end_conf_),opt_setting_.conf_prior_model);
    gp_planner::GPFactor gp_factor = gp_planner::GPFactor(gtsam::Symbol('x', 0),gtsam::Symbol('v', 0),gtsam::Symbol('x', 1),gtsam::Symbol('v', 1),delta_t_,opt_setting_.Qc_model);
    int numCombinations = pow(3,dof_);
    std::vector<int> combination(dof_, 0);
    std::vector<std::vector<int>> all_combinations;
    generateCombinations(combination, 0, all_combinations);
    for(const auto& comb : all_combinations){
        vector<VectorXd> traj(numPoints);
        for(int j=0;j<numPoints;j++){
            traj[j].resize(2*dof_);
            for(int k=0;k<dof_;k++){
                if(comb[k]==0){
                    traj[j](k) = cur_pos[k] + 0;
                    traj[j](k+dof_) = 0;
                }
                else if(comb[k]==1){
                    traj[j](k) = cur_pos[k] + 0.01*(j+1);
                    traj[j](k+dof_) = 0.01/0.2;
                }
                else if(comb[k]==2){
                    traj[j](k) = cur_pos[k] - 0.01*(j+1);
                    traj[j](k+dof_) = -0.01/0.2;
                }
            }
        }
        gtsam::Values temp_values = InitWithRef(traj,4);
        double coll_cost = 0;
        double goal_cost = 0;
        double gp_cost = 0;
        for(size_t m=0; m<numPoints;m++){
            coll_cost += (obs_factor.evaluateError(temp_values.at<gtsam::Vector>(gtsam::Symbol('x',m)),nullptr)).sum();
            goal_cost += (prior_factor.evaluateError(temp_values.at<gtsam::Vector>(gtsam::Symbol('x',m)),nullptr)).sum();
        }
        for(size_t m=1;m<numPoints;m++){
            gp_cost += (gp_factor.evaluateError(temp_values.at<gtsam::Vector>(gtsam::Symbol('x',m)),gtsam::Vector::Zero(6),
                        temp_values.at<gtsam::Vector>(gtsam::Symbol('x',m)),gtsam::Vector::Zero(6),nullptr,nullptr,nullptr,nullptr)).sum();
        }
        coll_cost = coll_cost / 5;
        // 比较，优先coll_cost最低（最低为0），在coll_cost一样的基础上比较goal_cost,选goal_cost最小的那个为best_traj
        if (coll_cost <= 0.2 && ((coll_cost <= 0.2 && gp_cost < bestScore_gp)|| (coll_cost <= 0.2 && gp_cost == bestScore_gp && goal_cost<bestScore_goal))) {
            bestScore_obs = coll_cost;
            bestScore_goal = goal_cost;
            bestScore_gp = gp_cost;
            best_traj = traj;
        }
    }
    return best_traj;
}

int MyPlanner::findClosestPoint(const vector<VectorXd>& globalPath) {
    double minDistance = std::numeric_limits<double>::max();
    int closestIndex = -1;
    for (size_t i = 0; i < globalPath.size(); ++i) {
        VectorXd global_point(6);
        global_point<<globalPath[i](0),globalPath[i](1),globalPath[i](2),globalPath[i](3),globalPath[i](4),globalPath[i](5);
        double distance = calculateDistance(global_point, arm_pos_);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }
    return closestIndex;
}

vector<VectorXd> MyPlanner::getLocalRefPath() {
    int closestIndex = findClosestPoint(global_results_);
    if (closestIndex == -1) {
        throw std::runtime_error("No closest point found");
    }
    vector<VectorXd> localRefPath;
    localRefPath.push_back(arm_pos_);
    int n = global_results_.size();
    for(int i=1;i<=opt_setting_.total_step;i++){
        int index = closestIndex + i * ref_inter_num_;
        if(index>=n){
            //如果全局路径的点数不足，进行插值
            int remaining = n - closestIndex;
            double t = (index - n) / static_cast<double>(ref_inter_num_);
            index = n - 1;
            VectorXd global_point_1(6);
            VectorXd global_point_2(6);
            global_point_1<<global_results_[index](0),global_results_[index](1),global_results_[index](2),global_results_[index](3),global_results_[index](4),global_results_[index](5);
            global_point_2<<global_results_[index-1](0),global_results_[index-1](1),global_results_[index-1](2),global_results_[index-1](3),global_results_[index-1](4),global_results_[index-1](5);
            localRefPath.push_back(interpolatePoint(global_point_1,global_point_2,t));
        } else {
            VectorXd global_point(6);
            global_point<<global_results_[index](0),global_results_[index](1),global_results_[index](2),global_results_[index](3),global_results_[index](4),global_results_[index](5);
            localRefPath.push_back(global_point);
        }
    }
    return localRefPath;
}

// cv::Mat MyPlanner::signedDistanceField3D(const std::vector<gtsam::Matrix>& ground_truth_map, double cell_size) {
//     int depth = static_cast<int>(ground_truth_map.size());
//     int rows = ground_truth_map[0].rows();
//     int cols = ground_truth_map[0].cols();
//     int sizes[] = {depth, rows, cols};
//     cv::Mat volume(3, sizes, CV_8U);
//     for (int z = 0; z < depth; ++z) {
//         for (int i = 0; i < rows; ++i) {
//             for (int j = 0; j < cols; ++j) {
//                 volume.at<uchar>(z, i, j) = ground_truth_map[z](i, j) > 0.75 ? 1 : 0;
//             }
//         }
//     }
//     cv::Mat inv_volume = 1 - volume;
//     cv::Mat map_dist = compute3DDistanceTransform(inv_volume);
//     cv::Mat inv_map_dist = compute3DDistanceTransform(volume);
//     cv::Mat field = (map_dist - inv_map_dist) * cell_size;
//     return field;
// }
// void MyPlanner::add_dynamic_obstacle(const std::vector<int>& position, const double& size, 
//                   const double& velocity, const std::vector<double>& direction,
//                   double total_time, std::vector<gtsam::Matrix>& map, std::vector<gtsam::Matrix>& prob_map){
//     // 计算障碍物的半径
//     int half_size_row = std::floor((size - 1) / 2);
//     int half_size_col = std::floor((size - 1) / 2);
//     int half_size_z = std::floor((size - 1) / 2);
//     // 计算障碍物的未来位置
//     std::vector<double> future_position = {
//         position[0] + std::round(velocity * direction[0] * total_time),
//         position[1] + std::round(velocity * direction[1] * total_time),
//         position[2] + std::round(velocity * direction[2] * total_time)
//     };
//     // 计算障碍物的移动距离
//     double distance_moved = velocity * total_time;
//     // 遍历障碍物的每个单元格，并在地图上设置为障碍物或衰减值
//     for (int i = future_position[0] - half_size_row; i <= future_position[0] + half_size_row; ++i) {
//         for (int j = future_position[1] - half_size_col; j <= future_position[1] + half_size_col; ++j) {
//             for (int k = future_position[2] - half_size_z; k <= future_position[2] + half_size_z; ++k) {
//                 if (i >= 0 && i < map[0].rows() && j >= 0 && j < map[0].cols() && k >= 0 && k < map.size()) {
//                     double distance_to_future = std::sqrt(std::pow(i - future_position[0], 2) + 
//                                                           std::pow(j - future_position[1], 2) + 
//                                                           std::pow(k - future_position[2], 2));
//                     double decayed_distance = opt_setting_.epsilon * (1.0 - distance_to_future / distance_moved);
//                     double probability = 1.0 - std::min(1.0, distance_to_future / distance_moved);
//                     probability = std::max(0.0, std::min(1.0, probability));
//                     prob_map[k](i, j) = probability;
//                     if (decayed_distance > 0) {
//                         map[k](i, j) = 1.0;  // 障碍物位置
//                     } else {
//                         map[k](i, j) = std::max(0.0, map[k](i, j) - 0.1);  // 衰减值
//                     }
//                 }
//             }
//         }
//     }
//     // // 遍历障碍物的每个单元格，并在地图上设置为障碍物（不衰减）
//     // for (int i = future_position[0] - half_size_row; i <= future_position[0] + half_size_row; ++i) {
//     //     for (int j = future_position[1] - half_size_col; j <= future_position[1] + half_size_col; ++j) {
//     //         for (int k = future_position[2] - half_size_z; k <= future_position[2] + half_size_z; ++k) {
//     //             if (i >= 0 && i < map[0].rows() && j >= 0 && j < map[0].cols() && k >= 0 && k < map.size()) {
//     //                 map[k](i, j) = 1.0;
//     //             }
//     //         }
//     //     }
//     // }
// }

void MyPlanner::armStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    for (int i=0;i<dof_;i++)
        arm_pos_(i)=msg->position[i+2];
}

void MyPlanner::GlobalPlanCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg) {
    is_plan_success_ = false;
    is_global_success_ = false;
    is_local_success_ = false;
    count_ = 0;
    for(int i=0;i<dof_;i++){
        start_conf_(i) = arm_pos_(i);
        end_conf_(i) = msg->goal.request.goal_constraints[0].joint_constraints[i].position;
    }
    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
    ps->getCurrentStateNonConst().update();
    planning_scene::PlanningScenePtr col_scene = ps->diff();
    col_scene->decoupleParent();
    is_global_success_ = rrt_planner_.RRT_Plan(col_scene,start_conf_,end_conf_,global_results_);
    if(is_global_success_)  publishGlobalPath();
}

void MyPlanner::LocalPlanningCallback(const ros::TimerEvent&){
    is_plan_success_ = false;
    is_local_success_ = false;
    local_results_.clear();
    auto start = std::chrono::high_resolution_clock::now();
    if(is_global_success_ && calculateDistance(arm_pos_, end_conf_)> goal_tolerance_){
        plan_times_ ++;
        if(planner_type_ == "gp"){
            vector<VectorXd> local_ref_path = getLocalRefPath();
            gtsam::Vector start_conf = ConvertToGtsamVector(arm_pos_);
            gtsam::Vector end_conf = ConvertToGtsamVector(local_ref_path[local_ref_path.size()-1]);
            // end_conf = ConvertToGtsamVector(end_conf_);
            gtsam::Values init_values;
            if(calculateDistance(arm_pos_,local_ref_path[0])>0.2 || !ref_flag_){
                init_values = gp_planner::initArmTrajStraightLine(ConvertToGtsamVector(arm_pos_), end_conf, opt_setting_.total_step);
            }
            else{
                init_values = InitWithRef(local_ref_path, opt_setting_.total_step);
            }
            double total_time =  traj_total_time_*calculateDistance(start_conf,end_conf) / max_vel_ ;
            total_time = opt_setting_.total_time;
            start_vel_ = (end_conf - start_conf) / total_time;
            end_vel_ = (end_conf - start_conf) / total_time;
            opt_setting_.set_total_time(total_time);
            gtsam::Values opt_values = gp_planner::Optimizer(robot_,sdf_,start_conf,end_conf,
                                                            ConvertToGtsamVector(start_vel_),ConvertToGtsamVector(end_vel_),
                                                            init_values,opt_setting_,
                                                            ConvertToGtsamVector(end_conf_));
            // 停止计时
            auto stop = std::chrono::high_resolution_clock::now();
            // 计算持续时间
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            plan_time_cost_ += static_cast<double>(duration.count());
            // 输出花费的时间
            // std::cout << "Local planning took " << duration.count() << " ms" << std::endl;
            ROS_INFO("Local planning took %.2f ms", static_cast<double>(duration.count()));
            exec_values_ = gp_planner::interpolateTraj(opt_values, opt_setting_.Qc_model, delta_t_, control_inter_);
            // double init_coll_cost = gp_planner::CollisionCost(robot_, sdf_, init_values, opt_setting_);
            double opt_coll_cost = gp_planner::CollisionCost(robot_, sdf_, opt_values, opt_setting_)/(opt_setting_.total_step*opt_setting_.obs_check_inter-opt_setting_.total_step);
            if(opt_coll_cost< 0.2)    {
                is_local_success_ = true;
                // ref_flag_ = true;
            }
            else {
                // cout<<"plan error: "<<opt_coll_cost<<endl;
                // ROS_INFO("Plan error: %f", opt_coll_cost);
                auto start_2 = std::chrono::high_resolution_clock::now();
                local_results_ = generateTraj(arm_pos_);
                local_results_.clear();
                is_local_success_ = false;
                // 停止计时
                auto stop_2 = std::chrono::high_resolution_clock::now();
                // 计算持续时间
                auto duration_2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                plan_time_cost_ += static_cast<double>(duration_2.count());
                if(local_results_.size()==5){
                    nav_msgs::Path path;
                    path.header.stamp = ros::Time::now();
                    path.header.frame_id="world";
                    path.poses.clear();
                    for(size_t i=0;i<local_results_.size();i++){
                        VectorXd theta(6);
                        theta<<local_results_[i](0),local_results_[i](1),local_results_[i](2),local_results_[i](3),local_results_[i](4),local_results_[i](5);
                        MatrixXd endT = rrt_planner::transformMatrix_DH(dh_alpha_,dh_a_,dh_d_,dh_theta_,theta);
                        VectorXd endPose(3);
                        endPose<<endT(0,3),endT(1,3),endT(2,3);
                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header = path.header;
                        pose_stamped.pose.position.x = endPose(0);
                        pose_stamped.pose.position.y = endPose(1);
                        pose_stamped.pose.position.z = endPose(2);
                        pose_stamped.pose.orientation.w = 1.0; // 假设没有旋转
                        path.poses.push_back(pose_stamped);
                    }
                    local_path_pub_.publish(path);
                    // 获取当前ROS时间
                    ros::Time now = ros::Time::now();
                    // 将ROS时间转换为字符串
                    std::stringstream ss;
                    ss << std::fixed << now.toSec(); // 将时间转换为秒
                    control_msgs::FollowJointTrajectoryGoal goal;
                    goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
                    goal.trajectory.points.resize(6);
                    goal.trajectory.points[0].positions.resize(dof_);
                    for(int j=0;j<dof_;j++){
                        goal.trajectory.points[0].positions[j] = arm_pos_(j);
                    }
                    goal.trajectory.points[0].time_from_start = ros::Duration(0);
                    for (int i = 0; i < 5; i++) {
                        trajectory_msgs::JointTrajectoryPoint& traj_point = goal.trajectory.points[i+1];
                        traj_point.positions.resize(dof_);
                        traj_point.velocities.resize(dof_);
                        for(int j=0;j<dof_;j++){
                            traj_point.positions[j] = local_results_[i](j);
                            traj_point.velocities[j] = local_results_[i](j+dof_);
                        }
                        traj_point.time_from_start = ros::Duration((i+1) * delta_t_);
                    }
                    gazebo_client_.sendGoal(goal,
                        boost::bind(&MyPlanner::doneCb, this, _1, _2),       // 目标完成时的回调
                        boost::bind(&MyPlanner::activeCb, this),            // 目标激活时的回调
                        boost::bind(&MyPlanner::feedbackCb, this, _1));     // 接收到反馈的回调
                }
                else{
                    count_++;
                    if(count_ > 100){
                        planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
                        monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
                        planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
                        ps->getCurrentStateNonConst().update();
                        planning_scene::PlanningScenePtr col_scene = ps->diff();
                        col_scene->decoupleParent();
                        is_global_success_ = rrt_planner_.RRT_Plan(col_scene,arm_pos_,end_conf_,global_results_);
                        if(is_global_success_)  publishGlobalPath();
                        count_ = 0;
                    }
                }
                // ref_flag_ = false;
            }
        }
        else if(planner_type_ == "mpc"){
            auto start = std::chrono::high_resolution_clock::now();
            vector<VectorXd> local_ref_path = getLocalRefPath();
            vector<VectorXd> temp;
            delta_t_ = opt_setting_.total_time / opt_setting_.total_step;
            temp = mpcSolve(local_ref_path,arm_pos_,obs_pos_,obs_vel_,obs_direction_);
            VectorXd next_pos(6);
            next_pos<<temp[0](0),temp[0](1),temp[0](2),temp[0](3),temp[0](4),temp[0](5);
            if (temp.empty() || calculateDistance(arm_pos_,next_pos)>max_vel_*delta_t_ ){
                std::cerr << "Error: temp is empty after MPC solve!" << std::endl;
                is_local_success_ = false;
            }
            else{
                is_local_success_ = true;
                // local_results_ = temp;
                // exec_step_ = local_results_.size();
                
                exec_step_ = 2;
                local_results_.resize(exec_step_);
                local_results_[0]=temp[0];
                local_results_[1]=temp[1];

                // exec_step_ = opt_setting_.total_step + control_inter_ * (opt_setting_.total_step - 1);
                // local_results_.resize(exec_step_);
                // int control_inter = (exec_step_ - opt_setting_.total_step) / (opt_setting_.total_step - 1); // 每两个原始点之间插入点数
                // int index = 0; // 插值结果的写入索引
                // for (int i = 0; i < opt_setting_.total_step - 1; ++i) {
                //     const VectorXd& start = temp[i];       // 当前点
                //     const VectorXd& end = temp[i + 1];    // 下一个点
                //     // 原始点直接插入
                //     local_results_[index++] = start;
                //     // 在当前点和下一个点之间插值
                //     for (int j = 1; j <= control_inter; ++j) {
                //         double alpha = static_cast<double>(j) / (control_inter + 1); // 插值比例
                //         VectorXd inter = (1 - alpha) * start + alpha * end;         // 线性插值公式
                //         local_results_[index++] = inter;
                //     }
                // }
                // 最后一个点直接插入
                // local_results_[index] = temp.back();
            }
            // 停止计时
            auto stop = std::chrono::high_resolution_clock::now();
            // 计算持续时间
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            plan_time_cost_ += static_cast<double>(duration.count());
            // 输出花费的时间
            ROS_INFO("Local planning took %.2f ms", static_cast<double>(duration.count()));
        }
    }
    else if(is_global_success_ && calculateDistance(arm_pos_, end_conf_)<= goal_tolerance_){
        is_local_success_ = true;
        // is_global_success_ = false;
        ROS_INFO("Plan Finished...");
    }
    is_plan_success_ = is_global_success_ && is_local_success_;
    if(is_plan_success_){
        if(calculateDistance(arm_pos_, end_conf_)<= goal_tolerance_){
            ROS_INFO("Plan Finished...");
        }
        publishLocalPath();
        if(planner_type_=="gp"){
            exec_step_ = opt_setting_.total_step + control_inter_ * (opt_setting_.total_step - 1);
            local_results_.resize(exec_step_);
            for(size_t i = 0;i<exec_step_;i++){
                gtsam::Vector pos_temp = exec_values_.at<gtsam::Vector>(gtsam::Symbol('x',i));
                gtsam::Vector vel_temp = exec_values_.at<gtsam::Vector>(gtsam::Symbol('v',i));
                local_results_[i].resize(dof_* 2);
                for(size_t j=0;j<dof_;j++){
                    local_results_[i](j) = pos_temp(j);
                    local_results_[i](j+dof_) = vel_temp(j);
                }
            }
            ROS_ASSERT(arm_pos_.size() == dof_);
            ROS_ASSERT(local_results_[i].size() >= 2 * dof_);
            publishTrajectory(static_cast<int>(exec_step_),true);
        }
        else if(planner_type_ =="mpc"){
            ROS_ASSERT(arm_pos_.size() == dof_);
            ROS_ASSERT(local_results_[0].size() >= 2 * dof_);
            publishTrajectory(static_cast<int>(exec_step_),true);
        }
    }

}

// 回调函数，处理障碍物信息
void MyPlanner::obstacleInfoCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 检查数据长度是否符合预期（8 个元素）
    if (msg->data.size() != 8) {
        ROS_WARN("Received obstacle info with unexpected size: %ld", msg->data.size());
        return;
    }
    // 提取障碍物信息
    double x = msg->data[0];
    double y = msg->data[1];
    double z = msg->data[2];
    double obs_size = msg->data[3];
    double linear_speed = msg->data[4];
    double direction_x = msg->data[5];
    double direction_y = msg->data[6];
    double direction_z = msg->data[7];
    obs_pos_<<x,y,z;
    obs_vel_ = linear_speed;
    obs_direction_ << direction_x,direction_y,direction_z;
    // 打印收到的障碍物信息
    // ROS_INFO("Received Obstacle Info:");
    // ROS_INFO("Position: (x: %.2f, y: %.2f, z: %.2f)", x, y, z);
    // ROS_INFO("Size: %.2f", obs_size);
    // ROS_INFO("Linear Speed: %.2f", linear_speed);
    // ROS_INFO("Direction: (%.2f, %.2f, %.2f)", direction_x, direction_y, direction_z);
}

std::vector<VectorXd> MyPlanner::mpcSolve(const vector<VectorXd>& ref_path, const VectorXd& x0, const VectorXd& obs_pos, const double& obs_vel, const VectorXd& obs_direction) {
    size_t N = opt_setting_.total_step;       // 时间步数
    double T = opt_setting_.total_time / N;  // 每个时间步的时间间隔
    double safe_radius = opt_setting_.epsilon;
    size_t n_states = dof_;                  // 状态维度（仅关节位置）
    size_t n_controls = dof_;                // 控制输入维度（关节速度）
    size_t n_vars = N * n_states + (N-1) * n_controls;  // 总优化变量数量
    size_t n_constraints = 6 + (N-2) * n_controls + N*robot_.nr_body_spheres() + (N - 1) * n_states ;      // 总约束数量(初始状态约束，加速度约束,障碍约束，模型约束)
    // size_t n_constraints = (N - 1) * n_states + N;      // 总约束数量
    // size_t n_constraints = (N - 1) * n_states;      // 总约束数量

    // std::cout << "=== Debug Information ===" << std::endl;
    // std::cout << "N: " << N << ", T: " << T << ", safe_radius: " << safe_radius << std::endl;
    // std::cout << "n_states: " << n_states << ", n_controls: " << n_controls << std::endl;
    // std::cout << "n_vars: " << n_vars << ", n_constraints: " << n_constraints << std::endl;

    // IPOPT 变量初始化
    std::vector<double> vars(n_vars, 0.0); // 优化变量初始值
    for(size_t i=0;i<dof_;i++)  vars[i] = x0[i];

    // IPOPT 变量上下界
    std::vector<double> vars_lowerbound(n_vars, -1e19);
    std::vector<double> vars_upperbound(n_vars, 1e19);
    for (size_t t = 0; t < N; ++t) {
        for (size_t i = 0; i < n_states; ++i) {
            vars_lowerbound[t * n_states + i] = -3.14; // 关节位置下界
            vars_upperbound[t * n_states + i] = 3.14;  // 关节位置上界
        }
    }
    for (size_t t=0;t<N-1;t++){
        for(size_t i=0;i<n_controls;i++){
            size_t control_index = N * n_states + t * n_controls + i;
            vars_lowerbound[control_index] = -1.0 * max_vel_; // 控制输入（关节速度）下界
            vars_upperbound[control_index] = max_vel_;  // 控制输入（关节速度）上界
        }
    }

    // IPOPT 约束上下界
    std::vector<double> constraints_lowerbound(n_constraints, 0.0);
    std::vector<double> constraints_upperbound(n_constraints, 0.0);
    // 设置加速度的上下限
    for(size_t t=0;t<N-2;t++){
        for(size_t i=0;i<n_controls;i++){
            size_t idx = 6 + t * n_controls + i;
            constraints_lowerbound[idx] = -100 * T;
            constraints_upperbound[idx] = 100 * T;
        }
    }
    // 设置避障约束的上下限
    for (size_t t = 6 + (N-2) * n_controls; t < 6 + (N-2) * n_controls + N*robot_.nr_body_spheres(); ++t) {
        // 避障约束的上界：无穷大
        constraints_upperbound[t] = 1e19;
    }

    // 定义 FG_eval（目标函数和约束）
    class FG_eval {
    public:
        typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
        const std::vector<VectorXd>& ref_path;
        const VectorXd& obs_pos;
        const double& obs_vel;
        const VectorXd& obs_direction;
        const double safe_radius;
        const size_t n_states;
        const size_t n_controls;
        const size_t N;
        const double T;
        const VectorXd& cur_state;
        const VectorXd& goal_pose;
        ADArm Arm;

        FG_eval(const std::vector<VectorXd>& ref_path_, const VectorXd& obs_pos_, const double& obs_vel_, 
                const VectorXd& obs_direction_, const double safe_radius_, size_t n_states_, size_t n_controls_, size_t N_, double T_,const VectorXd& cur_,const VectorXd& goal_)
            : ref_path(ref_path_), obs_pos(obs_pos_), obs_vel(obs_vel_), obs_direction(obs_direction_),
                safe_radius(safe_radius_), n_states(n_states_), n_controls(n_controls_), N(N_), T(T_),cur_state(cur_),goal_pose(goal_){
                VectorXd dh_alpha,dh_a,dh_d,dh_theta;
                dh_alpha.resize(6);
                dh_a.resize(6);
                dh_d.resize(6);
                dh_theta.resize(6);
                dh_alpha<<M_PI_2,0.0,0.0,M_PI_2,-M_PI_2,0.0;
                dh_a<<0.0,-0.427,-0.357,0.0,0.0,0.0;
                dh_d<<0.147,0.0,0.0,0.141,0.116,0.105;
                dh_theta<<0.0,-M_PI_2,0.0,-M_PI_2,0.0,0.0;
                
                vector<ADBodySphere> body_spheres;
                VectorXd xs(16),zs(16),ys(16),rs(16);
                vector<size_t> js={0,0,1,1,1,1,1,1,2,2,2,3,4,5,5,5};
                xs<<0,0,0,0.105,0.210,0.315,0.420,0,0.11,0.22,0,0,0,0,0,0;
                ys<<-0.1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
                zs<<0,0,0.1,0.1,0.1,0.1,0.1,0,0,0,0,0,0,0,0.1,0.2;
                rs<<0.08,0.08,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.10,0.07,0.07;
                for(int i=0;i<xs.size();i++){
                    body_spheres.emplace_back(ADBodySphere(js[i],rs[i],Eigen::Vector3d(xs[i],ys[i],zs[i])));
                }
                Eigen::Matrix4d base_pose = Eigen::Matrix4d::Identity();
                Arm = ADArm(6, dh_a, dh_alpha, dh_d, base_pose, dh_theta,body_spheres);
            }

        CppAD::AD<double> ADcomputeSignedDistance(
            const Eigen::Matrix<CppAD::AD<double>, 3, 1>& sphere1_center,
            const Eigen::Matrix<CppAD::AD<double>, 3, 1>& sphere2_center,
            CppAD::AD<double> sphere1_radius,
            CppAD::AD<double> sphere2_radius) {
            CppAD::AD<double> distance = (sphere1_center - sphere2_center).norm();
            CppAD::AD<double> signed_distance = distance - (sphere1_radius + sphere2_radius);
            return signed_distance;
        }

        void operator()(ADvector& fg, const ADvector& vars) {
            fg[0] = 0; // 初始化目标函数

            // 目标函数
            for (size_t t = 0; t < N; ++t) {
                // 位置误差代价
                for (size_t i = 0; i < n_states; ++i) {
                    CppAD::AD<double> pos_error = vars[t * n_states + i] - ref_path[t][i];
                    fg[0] += CppAD::pow(pos_error, 2);
                }

                // 控制输入代价
                for (size_t i = 0; i < n_controls; ++i) {
                    size_t control_index = N * n_states + t * n_controls + i;
                    fg[0] += 0.1 * CppAD::pow(vars[control_index], 2);
                }
            }

            for(size_t t=0; t<N;t++){
                for(size_t i=0;i<n_states;i++){
                    CppAD::AD<double> goal_error = vars[t * n_states + i] - goal_pose[i];
                    fg[0] += 0.3 * CppAD::pow(goal_error,2);
                }
            }

            for(size_t i = 0;i<6;i++){
                fg[1+i] = vars[i] - cur_state[i];
            }

            for(size_t t = 1; t < N-1; t++){
                for(size_t i=0;i<n_controls;i++){
                    size_t idx = 6 + (t-1) * (N-1) + i;
                    CppAD::AD<double> cur_v = vars[N * n_states + t * n_controls + i];
                    CppAD::AD<double> last_v = vars[N * n_states + (t-1) * n_controls + i];
                    fg[1 + idx] = cur_v - last_v;
                }
            }
            // 动态障碍物位置初始化
            CppAD::AD<double> obs_x = obs_pos[0];
            CppAD::AD<double> obs_y = obs_pos[1];
            CppAD::AD<double> obs_z = obs_pos[2];
            for(size_t t=0;t<N;t++){
                // 动态障碍物更新
                if (t < N - 1) {
                    obs_x += obs_vel * T * obs_direction[0];
                    obs_y += obs_vel * T * obs_direction[1];
                    obs_z += obs_vel * T * obs_direction[2];
                }
                std::vector<CppAD::AD<double>> joint_positions(6);
                for (int i = 0; i < 6; i++) {
                    joint_positions[i] = vars[t * n_states + i]; // 显式转换为 double
                }
                std::vector<Eigen::Matrix<CppAD::AD<double>, 3, 1>> sphere_centers;
                Arm.sphereCenters(joint_positions, sphere_centers);
                Eigen::Matrix<CppAD::AD<double>, 3, 1> obstacle_center;
                obstacle_center << obs_x, obs_y, obs_z;
                CppAD::AD<double> obstacle_radius = CppAD::AD<double>(0.08);
                CppAD::AD<double> dist = 1000;
                // 计算每个碰撞球体到障碍物的距离
                for (size_t i = 0; i < sphere_centers.size(); i++) {
                    size_t obstacle_constraint_index = 6 + (N-2) * n_controls + t * Arm.body_spheres_.size() + i;
                    // std::cout<<obstacle_constraint_index<<std::endl;
                    CppAD::AD<double> signed_distance = ADcomputeSignedDistance(
                        sphere_centers[i], obstacle_center, CppAD::AD<double>(Arm.body_spheres_[i].radius), obstacle_radius);
                    fg[1 + obstacle_constraint_index] = signed_distance - safe_radius;
                    // CppAD::AD<double> dist = signed_distance - safe_radius;
                    // if(dist<signed_distance-safe_radius)    dist=signed_distance-safe_radius;
                    // fg[0] += 0.1 * CppAD::exp(-1.0 * dist);
                }
                // fg[0] += 0.1 * CppAD::exp(-1.0*dist);
                // fg[1+t] = dist;
            }

            // 动力学约束
            for (size_t t = 0; t < N - 1; ++t) {
                for (size_t i = 0; i < n_states; ++i) {
                    CppAD::AD<double> pos = vars[t * n_states + i];         // 当前关节位置
                    CppAD::AD<double> vel = vars[N * n_states + t * n_controls + i]; // 当前关节速度
                    CppAD::AD<double> next_pos = vars[(t + 1) * n_states + i]; // 下一时刻关节位置

                    // 动力学约束：x_{k+1} = x_k + v_k * T
                    // cout<<1+N * Arm.body_spheres_.size() + t * n_states + i<<std::endl;
                    size_t idx = 6 + (N-2) * n_controls + N * Arm.body_spheres_.size() + t * n_states + i;
                    // size_t idx = N + t * n_states + i;
                    // size_t idx = t * n_states + i;
                    fg[1 + idx] = next_pos - (pos + vel * T);
                }
            }
        }
    };

    FG_eval fg_eval(ref_path, obs_pos, obs_vel, obs_direction, safe_radius, n_states, n_controls, N, T, arm_pos_, end_conf_);

    // IPOPT 选项
    std::string options;
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    options += "Numeric max_cpu_time          0.5\n";
    options += "Numeric tol           1e-4\n";

    // 调用 IPOPT 求解
    CppAD::ipopt::solve_result<std::vector<double>> solution;
    try {
        CppAD::ipopt::solve<std::vector<double>, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound,
            constraints_lowerbound, constraints_upperbound, fg_eval, solution);
    } catch (const std::exception& e) {
        std::cerr << "IPOPT solve failed with exception: " << e.what() << std::endl;
        throw;
    }
    // 提取结果
    std::vector<VectorXd> trajectory(N-1, VectorXd::Zero(n_states*2));
    for (size_t t = 0; t < N-1; ++t) {
        // 提取关节角度（位置）
        for (size_t i = 0; i < n_states; ++i) {
            trajectory[t][i] = solution.x[(t+1) * n_states + i]; // 前部分是关节角度
        }
        // 提取关节速度（控制量）
        for (size_t i = 0; i < n_controls; ++i) {
            trajectory[t][n_states + i] = solution.x[N * n_states + t * n_controls + i]; // 后部分是关节速度
        }
    }
    // ROS_INFO("Optimization completed successfully!");
    // // 打印 arm_pos_（6维向量）
    // std::ostringstream oss;
    // oss << "arm_pos_: [";
    // for (size_t i = 0; i < arm_pos_.size(); ++i) {
    //     oss << std::fixed << std::setprecision(3) << arm_pos_[i];
    //     if (i != arm_pos_.size() - 1) {
    //         oss << ", ";
    //     }
    // }
    // oss << "]";
    // ROS_INFO_STREAM(oss.str());

    // // 打印 trajectory
    // for (size_t t = 0; t < trajectory.size(); ++t) {
    //     std::ostringstream oss;
    //     oss << "Time step " << t << ": [";
    //     for (size_t i = 0; i < trajectory[t].size(); ++i) {
    //         oss << std::fixed << std::setprecision(3) << trajectory[t][i];
    //         if (i != trajectory[t].size() - 1) {
    //             oss << ", ";
    //         }
    //     }
    //     oss << "]";
    //     ROS_INFO_STREAM(oss.str());
    // }
    return trajectory;
}

void MyPlanner::publishLocalPath(){
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id="world";
    path.poses.clear();
    if(planner_type_ == "gp"){
        exec_step_ = opt_setting_.total_step + control_inter_ * (opt_setting_.total_step - 1);
        local_results_.resize(exec_step_);
        for(size_t i = 0;i<exec_step_;i++){
            gtsam::Vector pos_temp = exec_values_.at<gtsam::Vector>(gtsam::Symbol('x',i));
            gtsam::Vector vel_temp = exec_values_.at<gtsam::Vector>(gtsam::Symbol('v',i));
            local_results_[i].resize(dof_* 2);
            for(size_t j=0;j<dof_;j++){
                local_results_[i](j) = pos_temp(j);
                local_results_[i](j+dof_) = vel_temp(j);
            }
        }
    }
    else if(planner_type_=="mpc"){
        exec_step_ = local_results_.size();
    }
    double end_length = 0;
    VectorXd lastPose(3);
    for(size_t i=0;i<exec_step_;i++){
        VectorXd theta(6);
        theta<<local_results_[i](0),local_results_[i](1),local_results_[i](2),local_results_[i](3),local_results_[i](4),local_results_[i](5);
        MatrixXd endT = rrt_planner::transformMatrix_DH(dh_alpha_,dh_a_,dh_d_,dh_theta_,theta);
        VectorXd endPose(3);
        endPose<<endT(0,3),endT(1,3),endT(2,3);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position.x = endPose(0);
        pose_stamped.pose.position.y = endPose(1);
        pose_stamped.pose.position.z = endPose(2);
        pose_stamped.pose.orientation.w = 1.0; // 假设没有旋转
        path.poses.push_back(pose_stamped);
        if(i>=1)    end_length+=(endPose-lastPose).norm();
        lastPose = endPose;
    }
    end_path_length_ += end_length;
    local_path_pub_.publish(path);
}

void MyPlanner::publishGlobalPath(){
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id="world";
    path.poses.clear();
    for(size_t i=0;i<global_results_.size();i++){
        VectorXd theta(6);
        theta<<global_results_[i](0),global_results_[i](1),global_results_[i](2),global_results_[i](3),global_results_[i](4),global_results_[i](5);
        MatrixXd endT = rrt_planner::transformMatrix_DH(dh_alpha_,dh_a_,dh_d_,dh_theta_,theta);
        VectorXd endPose(3);
        endPose<<endT(0,3),endT(1,3),endT(2,3);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position.x = endPose(0);
        pose_stamped.pose.position.y = endPose(1);
        pose_stamped.pose.position.z = endPose(2);
        pose_stamped.pose.orientation.w = 1.0; // 假设没有旋转
        path.poses.push_back(pose_stamped);
    }
    global_path_pub_.publish(path);
    // ROS_INFO("Global Plan Finished.");
}

void MyPlanner::publishTrajectory(int exec_step, bool pub_vel){
    // 获取当前ROS时间
    ros::Time now = ros::Time::now();
    // 将ROS时间转换为字符串
    std::stringstream ss;
    ss << std::fixed << now.toSec(); // 将时间转换为秒
    double length = 0;
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    if(planner_type_=="gp"){
        goal.trajectory.points.resize(exec_step+1);
        goal.trajectory.points[0].positions.resize(dof_);
        if(pub_vel) goal.trajectory.points[0].velocities.resize(dof_);
        for(int j=0;j<dof_;j++){
            goal.trajectory.points[0].positions[j] = arm_pos_(j);
            if(pub_vel) goal.trajectory.points[0].velocities[j] = local_results_[0](j+dof_);
        }
        goal.trajectory.points[0].time_from_start = ros::Duration(0);
        for (int i = 0; i < exec_step; i++) {
            trajectory_msgs::JointTrajectoryPoint& traj_point = goal.trajectory.points[i+1];
            traj_point.positions.resize(dof_);

            if(pub_vel) traj_point.velocities.resize(dof_);
            for(int j=0;j<dof_;j++){
                traj_point.positions[j] = local_results_[i](j);
                if(pub_vel) traj_point.velocities[j] = local_results_[i](j+dof_);
            }
            traj_point.time_from_start = ros::Duration((i+1) * delta_t_ / control_inter_);
            if(i>=1){
                VectorXd p1(6),p2(6);
                for(int j=0;j<dof_;j++){
                    p1(j) = local_results_[i](j);
                    p2(j) = local_results_[i-1](j);
                }
                length+=(p1-p2).norm();
            }
        }
    }
    else if(planner_type_ == "mpc"){
        goal.trajectory.points.resize(exec_step+1);
        goal.trajectory.points[0].positions.resize(dof_);
        if(pub_vel) goal.trajectory.points[0].velocities.resize(dof_);
        for(int j=0;j<dof_;j++){
            goal.trajectory.points[0].positions[j] = arm_pos_(j);
            // goal.trajectory.points[0].velocities[j] = local_results_[0](j+dof_);
        }
        goal.trajectory.points[0].time_from_start = ros::Duration(0);
        for (int i = 0; i < exec_step; i++) {
            trajectory_msgs::JointTrajectoryPoint& traj_point = goal.trajectory.points[i+1];
            traj_point.positions.resize(dof_);
            if(pub_vel) traj_point.velocities.resize(dof_);
            for(int j=0;j<dof_;j++){
                traj_point.positions[j] = local_results_[i](j);
                if(pub_vel) traj_point.velocities[j] = local_results_[i](j+dof_);
            }
            traj_point.time_from_start = ros::Duration((i+1) * delta_t_);
            if(i>=1){
                VectorXd p1(6),p2(6);
                for(int j=0;j<dof_;j++){
                    p1(j) = local_results_[i](j);
                    p2(j) = local_results_[i-1](j);
                }
                length+=(p1-p2).norm();
            }
        }
    }
    // ROS_INFO("Sending trajectory with %lu points", goal.trajectory.points.size());
    // ROS_INFO("Joint names:");
    // for (const auto& name : goal.trajectory.joint_names) {
    //     ROS_INFO(" - %s", name.c_str());
    // }
    // for (size_t i = 0; i < goal.trajectory.points.size(); ++i) {
    //     const auto& point = goal.trajectory.points[i];
    //     ROS_INFO("Point %lu positions size: %lu", i, point.positions.size());
    //     ROS_INFO("Point %lu velocities size: %lu", i, point.velocities.size());
    // }
    // for (size_t i = 0; i < goal.trajectory.points.size(); ++i) {
    //     const auto& point = goal.trajectory.points[i];
    //     ROS_INFO("Point %lu:", i);
    //     for (size_t j = 0; j < point.positions.size(); ++j) {
    //         ROS_INFO("  Joint %lu position: %f", j, point.positions[j]);
    //         if (pub_vel) ROS_INFO("  Joint %lu velocity: %f", j, point.velocities[j]);
    //     }
    //     ROS_INFO("  Time from start: %f", point.time_from_start.toSec());
    // }
    path_length_ += length;
    // 输出带有时间戳的信息
    // ROS_INFO_STREAM("Publish Traj at " << ss.str());
    // gazebo_client_.sendGoal(goal);
    // gazebo_client_.waitForResult(ros::Duration(1.0));
    gazebo_client_.sendGoal(goal,
                     boost::bind(&MyPlanner::doneCb, this, _1, _2),       // 目标完成时的回调
                     boost::bind(&MyPlanner::activeCb, this),            // 目标激活时的回调
                     boost::bind(&MyPlanner::feedbackCb, this, _1));     // 接收到反馈的回调

}

void MyPlanner::doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result){
    ROS_INFO("Trajectory execution finished with state: %s", state.toString().c_str());
    if (result) {
        ROS_INFO("Trajectory result error code: %d", result->error_code);
    }
}
void MyPlanner::activeCb(){
    // ROS_INFO("Trajectory goal is now active.");
}
void MyPlanner::feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback){
    // ROS_INFO("Received feedback:");
    // for (size_t i = 0; i < feedback->actual.positions.size(); ++i) {
    //     ROS_INFO("Joint %ld: %f", i, feedback->actual.positions[i]);
    // }
}

void MyPlanner::DynamicCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if(msg->data[0]==3){
        // sdf_.loadSDF("/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_dynamic_py.txt");
        std_msgs::Float64MultiArray read_msg;
        read_msg.data.push_back(1);
        read_pub_.publish(read_msg);
    }
}

void MyPlanner::readSDFFile(const ros::TimerEvent&) {
    std_msgs::Float64MultiArray read_msg;
    read_msg.data.push_back(1);
    read_pub_.publish(read_msg);
    gp_planner::SDF temp;
    // 读取SDF文件
    if(temp.loadSDF(dynamic_file_)) sdf_=temp;
    read_msg.data.clear();
    read_msg.data.push_back(0);
    read_pub_.publish(read_msg);
}

MyPlanner::~MyPlanner() = default;