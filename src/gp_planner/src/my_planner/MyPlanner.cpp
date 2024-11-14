#include "my_planner/MyPlanner.h"
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

MyPlanner::MyPlanner(ros::NodeHandle& nh):nh_(nh),client_("/arm_controller/follow_joint_trajectory", true){
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
    VectorXd xs(14),zs(14),ys(14),rs(14);
    vector<size_t> js={0,0,1,1,1,1,1,1,2,2,2,3,4,5};
    xs<<0,0,0,0.105,0.210,0.315,0.420,0,0.11,0.22,0,0,0,0;
    ys<<-0.1,0,0,0,0,0,0,0,0,0,0,0,0,0;
    zs<<0,0,0.1,0.1,0.1,0.1,0.1,0,0,0,0,0,0,0;
    rs<<0.08,0.08,0.06,0.06,0.06,0.06,0.06,0.08,0.06,0.06,0.08,0.07,0.06,0.06;
    for(int i=0;i<xs.size();i++){
        body_spheres.push_back(gp_planner::BodySphere(js[i],rs[i],gtsam::Point3(xs[i],ys[i],zs[i])));
    }
    arm_ = gp_planner::Arm(dof_, dh_a_, dh_alpha_, dh_d_, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)), dh_theta_);
    robot_ = gp_planner::ArmModel(arm_,body_spheres);
    opt_setting_ = gp_planner::OptimizerSetting(static_cast<size_t>(dof_));
    int total_step;  // number of steps (states) optimized the whole trajectory
    double total_time;  // time duration (second) of the whole trajectory
    /// joint position and velocity limit settings
    bool flag_pos_limit;  // whether enable joint position limits
    bool flag_vel_limit;  // whether enable velocity limits
    double joint_pos_limits_up, joint_pos_limits_down;  // joint position limits
    double max_vel;   // joint velocity limits, for all DOF
    /// obstacle cost settings
    double epsilon;          // eps of hinge loss function (see the paper)
    double cost_sigma;       // sigma of obstacle cost (see the paper)
    int obs_check_inter;  // number of point interpolated for obstacle cost,
    double fix_pose_sigma, fix_vel_sigma, pos_limit_sigma, vel_limit_sigma;
    double Qc;
    string opt_type;
    nh.getParam("settings/total_time", total_time);
    nh.getParam("settings/total_step", total_step);
    nh.getParam("settings/flag_pos_limit", flag_pos_limit);
    nh.getParam("settings/flag_pos_limit", flag_vel_limit);
    nh.getParam("settings/joint_pos_limits_up", joint_pos_limits_up);
    nh.getParam("settings/joint_pos_limits_down", joint_pos_limits_down);
    nh.getParam("settings/max_vel", max_vel);
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
        vel_limits[i]=max_vel;
        pos_limit_sigmas[i]=pos_limit_sigma;
        vel_limit_sigmas[i]=vel_limit_sigma;
    }
    opt_setting_.set_total_time(total_time);
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
    gtsam::Point3 origin(-1.0, -1.0, -0.35);
    double cell_size = 0.05;
    size_t rows = 40, cols = 40, z = 30;
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
    sdf_.loadSDF("/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_static_py.txt");
    sdf_.saveSDF("/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_static.txt");

    rrt_planner_ = rrt_planner::RRTPlanner(nh, kinematic_model_);
    nh.getParam("settings/frequency", local_planner_frenquency_);
    planner_timer_ = nh.createTimer(ros::Duration(1.0 / local_planner_frenquency_), &MyPlanner::LocalPlanningCallback, this);
    arm_state_sub_ = nh.subscribe(arm_state_topic_, 1, &MyPlanner::armStateCallback,this);
    plan_sub_ = nh.subscribe("move_group/goal", 1, &MyPlanner::GlobalPlanCallback,this);
    // obs_sub_ = nh.subscribe("/obstacle_info", 1, &MyPlanner::obstacleCallback, this);
    map_sub_ = nh.subscribe("/map_updated", 1, &MyPlanner::DynamicCallback, this);
    local_path_pub_ = nh.advertise<nav_msgs::Path>("local_path",10);
    global_path_pub_ = nh.advertise<nav_msgs::Path>("global_path",10);
    read_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/is_reading",10);

    mutex_ = std::make_shared<std::mutex>();  // 创建互斥锁
    map_timer_ = nh_.createTimer(ros::Duration(0.5), &MyPlanner::readSDFFile, this);  // 创建定时器

    is_plan_success_ = false;
    is_global_success_ = false;
    is_local_success_ = false;
}

gtsam::Values MyPlanner::InitWithRef(const std::vector<Eigen::VectorXd>& ref_path, int total_step){
    gtsam::Values init_values;
    gtsam::Vector init_conf = ConvertToGtsamVector(ref_path[0]);
    gtsam::Vector end_conf = ConvertToGtsamVector(ref_path[ref_path.size()-1]);
    int n = ref_path.size();
    double stepSize = (n-1) / total_step;
    // init pose
    for (size_t i = 0; i <= total_step; i++) {
        gtsam::Vector conf;
        // double t = i * stepSize;
        // int index = static_cast<int>(t);
        // conf = ConvertToGtsamVector(ref_path[index]);
        conf = ConvertToGtsamVector(ref_path[i]);
        init_values.insert(gtsam::Symbol('x', i), conf);
    }
    // init vel as avg vel
    gtsam::Vector avg_vel = (end_conf - init_conf) / total_step;
    for (size_t i = 0; i <= total_step; i++)
        init_values.insert(gtsam::Symbol('v', i), avg_vel);
    return init_values;
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

    if(is_global_success_ && calculateDistance(arm_pos_, end_conf_)> goal_tolerance_){
        vector<VectorXd> local_ref_path = getLocalRefPath();
        gtsam::Vector start_conf = ConvertToGtsamVector(arm_pos_);
        gtsam::Vector end_conf;
        gtsam::Values init_values;
        if(calculateDistance(arm_pos_,local_ref_path[0])>0.2){
            init_values = gp_planner::initArmTrajStraightLine(ConvertToGtsamVector(arm_pos_), ConvertToGtsamVector(end_conf_), opt_setting_.total_step);
            end_conf = ConvertToGtsamVector(end_conf_);
        }
        else{
            init_values = InitWithRef(local_ref_path, opt_setting_.total_step);
            end_conf = ConvertToGtsamVector(local_ref_path[local_ref_path.size()-1]);
        }
        init_values = InitWithRef(local_ref_path, opt_setting_.total_step);
        end_conf = ConvertToGtsamVector(local_ref_path[local_ref_path.size()-1]);
        start_vel_ = (end_conf - start_conf) / opt_setting_.total_time;
        end_vel_ = (end_conf - start_conf) / opt_setting_.total_time;

        gtsam::Values opt_values = gp_planner::Optimizer(robot_,sdf_,start_conf,end_conf,
                                                        ConvertToGtsamVector(start_vel_),ConvertToGtsamVector(end_vel_),init_values,opt_setting_);

        exec_values_ = gp_planner::interpolateTraj(opt_values, opt_setting_.Qc_model, delta_t_, control_inter_);
        // cout<<opt_setting_.epsilon<<endl;
        double init_coll_cost = gp_planner::CollisionCost(robot_, sdf_, init_values, opt_setting_);
        // std::cout<<"Init col_cost: "<<init_coll_cost<<std::endl;
        double opt_coll_cost = gp_planner::CollisionCost(robot_, sdf_, opt_values, opt_setting_);
        // cout<<opt_setting_.epsilon<<endl;
        // std::cout<<"Opt col_cost: "<<opt_coll_cost<<std::endl;
        if(opt_coll_cost< 0.5)    is_local_success_ = true;
        else    {
            // cout<<"plan error: "<<opt_coll_cost<<endl;
            planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
            monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
            planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
            ps->getCurrentStateNonConst().update();
            planning_scene::PlanningScenePtr col_scene = ps->diff();
            col_scene->decoupleParent();
            is_global_success_ = rrt_planner_.RRT_Plan(col_scene,start_conf_,end_conf_,global_results_);
            if(is_global_success_)  {
                local_ref_path = getLocalRefPath();
                // publishGlobalPath();
            }
        }
        // is_local_success_ = true;
    }
    is_plan_success_ = is_global_success_ && is_local_success_;
    if(is_plan_success_){
        if(calculateDistance(arm_pos_, end_conf_)<= goal_tolerance_){
            cout<<endl<<"************* Plan Finished ***************"<<endl<<endl;
        }
        publishLocalPath();
        publishTrajectory();
    }
}

void MyPlanner::publishLocalPath(){
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id="world";
    path.poses.clear();
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
    }
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
}

void MyPlanner::publishTrajectory(){
    exec_step_ = opt_setting_.total_step + control_inter_ * (opt_setting_.total_step - 1);
    exec_step_ = static_cast<int>(0.5*exec_step_ / opt_setting_.total_time); 
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
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    goal.trajectory.points.resize(exec_step_);
    for (int i = 0; i < exec_step_; i++) {
        trajectory_msgs::JointTrajectoryPoint& traj_point = goal.trajectory.points[i];
        traj_point.positions.resize(dof_);
        traj_point.velocities.resize(dof_);
        for(int j=0;j<dof_;j++){
            traj_point.positions[j] = local_results_[i](j);
            // traj_point.velocities[j] = local_results_[i](j+dof_);
        }
        traj_point.time_from_start = ros::Duration(i * delta_t_ / control_inter_);
    }
    client_.sendGoal(goal);
    client_.waitForResult();
}

void MyPlanner::DynamicCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if(msg->data[0]==3){
        sdf_.loadSDF("/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_dynamic_py.txt");
        std_msgs::Float64MultiArray read_msg;
        read_msg.data.push_back(1);
        read_pub_.publish(read_msg);
    }
}

void MyPlanner::readSDFFile(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(*mutex_);  // 使用互斥锁
    std_msgs::Float64MultiArray read_msg;
    read_msg.data.push_back(1);
    read_pub_.publish(read_msg);
    gp_planner::SDF temp;
    if(temp.loadSDF("/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_dynamic_py.txt")) sdf_=temp;
    // 读取SDF文件
    // sdf_.loadSDF("/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_dynamic_py.txt");

    read_msg.data.clear();
    read_msg.data.push_back(0);
    read_pub_.publish(read_msg);

    // // 获取当前时间
    // ros::Time now = ros::Time::now();
    // double timestamp = now.toSec();
    // std::cout << "load dynamic sdf at " << timestamp << std::endl;
}

MyPlanner::~MyPlanner() = default;