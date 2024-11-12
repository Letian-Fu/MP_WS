#include "my_planner/MyPlanner.h"

MyPlanner::MyPlanner(ros::NodeHandle nh){
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
    for(int i=0;i<1;i++){
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
    opt_setting_.set_total_step(static_cast<size_t>(total_step));
    opt_setting_.set_conf_prior_model(fix_pose_sigma);
    opt_setting_.set_vel_prior_model(fix_vel_sigma);
    opt_setting_.set_epsilon(epsilon);
    opt_setting_.set_cost_sigma(cost_sigma);
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

    gtsam::Point3 origin(-1.5, -1.5, -0.35);
    double cell_size = 0.1;
    size_t rows = 30, cols = 30, z = 20;
    std::vector<gtsam::Matrix> map(z, gtsam::Matrix::Zero(rows, cols));
    std::vector<gtsam::Vector> corners;
    // gp_planner::add_obstacle({20, 9, 10}, {3, 3, 3}, map, corners);
    auto field = gp_planner::signedDistanceField3D(map, cell_size);
    // 创建 SignedDistanceField 对象
    sdf_ = gp_planner::SDF(origin, cell_size, rows, cols, z);
    for (size_t z = 0; z < field.size(); ++z) {
        sdf_.initFieldData(z, field[z]);
    }
    sdf_.saveSDF("/home/roboert/MP_WS/src/gp_planner/src/my_planner/sdf_data.txt");
    sdf_.print();

    rrt_planner_ = rrt_planner::RRTPlanner(nh, kinematic_model_);
    local_planner_frenquency_ = 10;
    timer = nh.createTimer(ros::Duration(1.0 / local_planner_frenquency_), &MyPlanner::LocalPlanningCallback, this);
    arm_state_sub_ = nh.subscribe(arm_state_topic_, 1, &MyPlanner::armStateCallback,this);
    plan_sub_ = nh.subscribe("move_group/goal", 1, &MyPlanner::planCallback,this);
    obs_sub_ = nh.subscribe("/obstacle_info", 1, &MyPlanner::obstacleCallback, this);

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
        double t = i * stepSize;
        int index = static_cast<int>(t);
        conf = ConvertToGtsamVector(ref_path[index]);
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
        double distance = calculateDistance(globalPath[i], arm_pos_);
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
        int index = closestIndex + i * 3;
        if(index>=n){
            //如果全局路径的点数不足，进行插值
            int remaining = n - closestIndex;
            double t = (index - n) / 3.0;
            index = n - 1;
            localRefPath.push_back(interpolatePoint(global_results_[index],global_results_[index-1],t));
        } else {
            localRefPath.push_back(global_results_[index]);
        }
    }
    return localRefPath;
}


void MyPlanner::armStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    for (int i=0;i<dof_;i++)
        arm_pos_(i)=msg->position[i];
}

void MyPlanner::planCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg) {
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
}

void MyPlanner::LocalPlanningCallback(const ros::TimerEvent&){
    is_plan_success_ = false;
    is_local_success_ = false;
    if(is_global_success_){
        vector<VectorXd> local_ref_path = getLocalRefPath();
        gtsam::Values init_values = InitWithRef(local_ref_path, opt_setting_.total_step);
        // gtsam::Values init_values = gp_planner::initArmTrajStraightLine(ConvertToGtsamVector(arm_pos_), ConvertToGtsamVector(end_conf_), opt_setting_.total_step);
        gtsam::Values opt_values = gp_planner::Optimizer(robot_,sdf_,ConvertToGtsamVector(arm_pos_),ConvertToGtsamVector(end_conf_),
                                                        ConvertToGtsamVector(start_vel_),ConvertToGtsamVector(end_vel_),init_values,opt_setting_);

        exec_values_ = gp_planner::interpolateTraj(opt_values, opt_setting_.Qc_model, delta_t_, control_inter_);
        double init_coll_cost = gp_planner::CollisionCost(robot_, sdf_, init_values, opt_setting_);
        // std::cout<<"Init col_cost: "<<init_coll_cost<<std::endl;
        double opt_coll_cost = gp_planner::CollisionCost(robot_, sdf_, opt_values, opt_setting_);
        // std::cout<<"Opt col_cost: "<<opt_coll_cost<<std::endl;
        if(opt_coll_cost<= init_coll_cost)    is_local_success_ = true;
        if(is_local_success_){
            exec_step_ = opt_setting_.total_step + control_inter_ * (opt_setting_.total_step - 1);
            local_results_.resize(exec_step_);
            for(size_t i = 0;i<exec_step_;i++){
                gtsam::Vector temp = exec_values_.at<gtsam::Vector>(gtsam::Symbol('x',i));
                local_results_[i].resize(dof_);
                for(size_t j=0;j<dof_;j++)  local_results_[i](j) = temp(j);
            }
        }
    }
    is_plan_success_ = is_global_success_ && is_local_success_;
}

void MyPlanner::obstacleCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    double x = msg->data[0];
    double y = msg->data[1];
    double height = msg->data[2];
    double radius = msg->data[3];
    obs_info_.resize(4);
    obs_info_<<x,y,height,radius;
    // std::cout<<obs_info_.transpose()<<std::endl;
    // 局部更新sdf
}

void MyPlanner::execute() {
    if(is_plan_success_){
        double total_time = 10;
        int exec_step = local_results_.size();
        moveit_msgs::RobotTrajectory plan_traj = moveit_msgs::RobotTrajectory();
        plan_traj.joint_trajectory.header.frame_id = move_group_->getPlanningFrame();
        plan_traj.joint_trajectory.joint_names = move_group_->getActiveJoints();
        plan_traj.joint_trajectory.points.resize(exec_step);
        for (int i = 0; i < exec_step; i++) {
            plan_traj.joint_trajectory.points[i].positions.resize(6);
            ros::Duration time_from_start;
            time_from_start = ros::Duration(total_time * i / exec_step);
            for (int j = 0; j < dof_; j++)
                plan_traj.joint_trajectory.points[i].positions[j] = local_results_[i](j);
            plan_traj.joint_trajectory.points[i].time_from_start = time_from_start;
        }
        move_group_->execute(plan_traj);
        move_group_->clearPoseTargets();
        move_group_->setStartState(*move_group_->getCurrentState());
        move_group_->stop();
    }
}

MyPlanner::~MyPlanner() = default;