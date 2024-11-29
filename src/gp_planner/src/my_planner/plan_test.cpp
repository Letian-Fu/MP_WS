#include "my_planner/MyPlanner.h"
#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <gazebo_msgs/ContactsState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <cmath>
#include <thread>  // std::this_thread::sleep_for



bool is_collision = false;

void bumperCallback(const gazebo_msgs::ContactsState::ConstPtr& msg) {
    if (!msg->states.empty()) {
        // ROS_INFO("Collision detected!");
        is_collision = true;
        // for (const auto& contact : msg->states) {
        //     ROS_INFO("Contact between [%s] and [%s]",
        //              contact.collision1_name.c_str(),
        //              contact.collision2_name.c_str());
        // }
    } else {
        // ROS_INFO("No collision detected.");
    }
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "plan_test");
    ros::NodeHandle n;
    MyPlanner my_planner(n);
    // TF2 Buffer 和 Listener，用于记录末端执行器的位置信息
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    // 定义目标点（关节角度）
    VectorXd target_a(6),target_b(6);
    target_a << 0.656,1.433,-0.866,-0.566,-2.115,0;  // 点 A
    target_b << -0.851,1.309,-0.735,-0.574,-0.608,0;  // 点 B

    // 状态变量
    bool moving_to_a = true;  // 当前目标是点 A 或点 B

    // 实验控制变量
    int max_iterations = 2;  // 最大往复次数（比如 10 次往返）
    n.getParam("iterations", max_iterations);
    int iteration = 0;
    bool is_reached = true;
    std::vector<VectorXd> results(max_iterations, VectorXd::Zero(6));

    ros::Subscriber sub = n.subscribe("/bumper_states", 1000, bumperCallback);
    // 开始多线程处理回调
    ros::AsyncSpinner spinner(2); // 使用 2 个线程处理回调
    spinner.start();

    // my_planner.planner_type_ = "gp";
    auto start = std::chrono::high_resolution_clock::now();
    auto stop = std::chrono::high_resolution_clock::now();
    // 计算持续时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    VectorXd current_joints(6), last_joints(6);
    current_joints.setZero();
    last_joints.setZero();
    // 记录末端路径
    Vector3d current_end_effector_positions,last_end_effector_positions;
    current_end_effector_positions.setZero();
    last_end_effector_positions.setZero();
    double joints_change = 0;
    double path_length = 0;
    while(ros::ok() && iteration < max_iterations){
        current_joints = my_planner.arm_pos_;
        if(last_joints.norm()!=0){
            double diff_norm = (current_joints - last_joints).norm();
            if (std::isfinite(diff_norm)) { // 检查是否为有限值
                // ROS_WARN("Joints change calculation resulted in inf or NaN.");
                joints_change += diff_norm;
            }
        }
        last_joints = current_joints;

        try {
            geometry_msgs::TransformStamped transform_stamped =
                tf_buffer.lookupTransform("base_link", "Link6", ros::Time(0), ros::Duration(0.5));
            Eigen::Vector3d position(transform_stamped.transform.translation.x,
                                        transform_stamped.transform.translation.y,
                                        transform_stamped.transform.translation.z);
            current_end_effector_positions=position;
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF2 Exception: %s", ex.what());
        }
        if(last_end_effector_positions.norm()!=0){
            double diff_norm = (current_end_effector_positions - last_end_effector_positions).norm();
            if (std::isfinite(diff_norm)) { // 检查是否为有限值
                // ROS_WARN("Joints change calculation resulted in inf or NaN.");
                path_length += diff_norm;
            }
        }
        last_end_effector_positions = current_end_effector_positions;

        if(is_reached){
            VectorXd target = moving_to_a ? target_a : target_b;
            planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
            monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
            planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
            ps->getCurrentStateNonConst().update();
            planning_scene::PlanningScenePtr col_scene = ps->diff();
            col_scene->decoupleParent();
            my_planner.start_conf_ = my_planner.arm_pos_;
            my_planner.end_conf_ = target;
            my_planner.is_global_success_ = my_planner.rrt_planner_.RRT_Plan(col_scene,my_planner.start_conf_,my_planner.end_conf_,my_planner.global_results_);
            if(my_planner.is_global_success_)  {
                my_planner.publishGlobalPath();
                is_reached = false;
            }
            start = std::chrono::high_resolution_clock::now();
        }
        if(my_planner.is_global_success_ && my_planner.calculateDistance(my_planner.arm_pos_, my_planner.end_conf_)< my_planner.goal_tolerance_){
             // 切换目标点
            moving_to_a = !moving_to_a;
            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            results[iteration](0) = true;
            results[iteration](2) = static_cast<double>(duration.count());
            results[iteration](3) = my_planner.plan_time_cost_/static_cast<double>(my_planner.plan_times_);
            // results[iteration](4) = my_planner.path_length_;
            // results[iteration](5) = my_planner.end_path_length_;
            results[iteration](4) = joints_change;
            results[iteration](5) = path_length;
            my_planner.plan_time_cost_ = 0;
            my_planner.plan_times_=0;
            my_planner.path_length_=0;
            my_planner.end_path_length_ = 0;
            joints_change = 0;
            path_length=0;
            // 增加迭代次数
            // 增加迭代次数
            my_planner.is_global_success_ = false;
            my_planner.is_local_success_ = false;
            my_planner.is_plan_success_ = false;
            is_reached = true;
            std::cout << std::setw(12) << "Iteration"
              << std::setw(12) << "Success"
              << std::setw(12) << "Collision"
              << std::setw(12) << "Total Time"
              << std::setw(12) << "Opt Time"
              << std::setw(12) << "Joints Change(rad)"
              << std::setw(12) << "Path Length(m)" << std::endl;

            std::cout << std::string(12 * 7, '-') << std::endl;
            std::cout << std::setw(12) << iteration +1;
            for (int j = 0; j < results[iteration].size(); j++) {
                std::cout << std::setw(12) << results[iteration][j];
            }
            std::cout << std::endl;
            iteration++;

            // 延时 300 毫秒
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if(is_collision){
            results[iteration](1) = 1;
            is_collision = false;
        }
        // // 等待下一个循环
        // ros::spinOnce();
    }
    // 打印结果
    std::cout << std::setw(12) << "Iteration"
              << std::setw(12) << "Success"
              << std::setw(12) << "Collision"
              << std::setw(12) << "Total Time"
              << std::setw(12) << "Opt Time"
              << std::setw(12) << "Joints Change(rad)"
              << std::setw(12) << "Path Length(m)" << std::endl;

    std::cout << std::string(12 * 7, '-') << std::endl;

    for (int i = 0; i < results.size(); i++) {
        std::cout << std::setw(12) << i + 1;
        for (int j = 0; j < results[i].size(); j++) {
            std::cout << std::setw(12) << results[i][j];
        }
        std::cout << std::endl;
    }
    // 计算每列的平均值
    std::vector<double> averages(results[0].size(), 0.0); // 初始化为0
    for (int j = 0; j < results[0].size(); j++) {
        double sum = 0.0;
        for (int i = 0; i < results.size(); i++) {
            sum += results[i][j];
        }
        averages[j] = sum / results.size();
    }

    // 输出平均值
    std::cout << std::string(12 * 7, '-') << std::endl;
    std::cout << std::setw(12) << "Average";
    for (double avg : averages) {
        std::cout << std::setw(12) << avg;
    }
    std::cout << std::endl;
    return 0;
}