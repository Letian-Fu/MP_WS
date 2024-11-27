#include "my_planner/MyPlanner.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_msgs/ContactsState.h>


bool is_collision = false;

void contactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg) {
    for (const auto& state : msg->states) {
        std::string link_name = "ball";  // 球的 link 名称
        
        // 检查碰撞事件中是否涉及球和机械臂的 link
        if ((state.collision1_name.find(link_name) != std::string::npos) ||
            (state.collision2_name.find(link_name) != std::string::npos)) {
            ROS_INFO("Collision detected!");
            is_collision = true;

            // 打印碰撞的物体名称
            ROS_INFO("Collision between: %s and %s",
                     state.collision1_name.c_str(),
                     state.collision2_name.c_str());
        }
    }
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "plan_test");
    ros::NodeHandle n;
    MyPlanner my_planner(n);
    // ros::spin();
    // 定义目标点（关节角度）
    VectorXd target_a(6),target_b(6);
    target_a << 0.656,1.433,-0.866,-0.566,-2.115,0;  // 点 A
    target_b << -0.851,1.309,-0.735,-0.574,-0.608,0;  // 点 B

    // 状态变量
    bool moving_to_a = true;  // 当前目标是点 A 或点 B

    // 实验控制变量
    int max_iterations = 10;  // 最大往复次数（比如 10 次往返）
    int iteration = 0;
    bool is_reached = true;
    std::vector<VectorXd> results(max_iterations);
    for(int i=0;i<max_iterations;i++){
        results[i].resize(4);//success,collision,time,path_length,
    }

    ros::Subscriber sub = n.subscribe("/gazebo/default/physics/contacts", 10, contactCallback);

    while(ros::ok() && iteration < max_iterations){
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
        }
        if(my_planner.is_global_success_ && my_planner.calculateDistance(my_planner.arm_pos_, my_planner.end_conf_)< my_planner.goal_tolerance_){
             // 切换目标点
            moving_to_a = !moving_to_a;
            results[iteration](0) = true;
            results[iteration](2) = my_planner.plan_time_cost_/static_cast<double>(my_planner.plan_times_);
            results[iteration](3) = my_planner.path_length_;
            // 增加迭代次数
            iteration++;
            my_planner.is_global_success_ = false;
            my_planner.is_local_success_ = false;
            my_planner.is_plan_success_ = false;
            is_reached = true;
        }
        if(is_collision){
            results[iteration](1) = true;
            is_collision = false;
        }
        // 等待下一个循环
        ros::spinOnce();
    }
    // 打印标题行
    std::cout << std::setw(12) << "Iteration"
              << std::setw(12) << "Success"
              << std::setw(12) << "Collision"
              << std::setw(12) << "Time"
              << std::setw(12) << "Path_Length" << std::endl;

    // 打印分隔线
    std::cout << std::string(12 * 5, '-') << std::endl;

    // 打印结果数据
    for (int i = 0; i < results.size(); i++) {
        std::cout << std::setw(12) << i + 1; // Iteration number
        for (int j = 0; j < results[i].size(); j++) {
            std::cout << std::setw(12) << results[i][j]; // Values for each column
        }
        std::cout << std::endl;
    }
    return 0;
}