#include "my_planner/MyPlanner.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include "Dashboard.h"
#include "DobotMove.h"
#include "Feedback.h"
#include "ErrorInfoBean.h"
#include "ErrorInfoHelper.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>

// 定义点位结构体
struct JointAngles {
    double j1, j2, j3, j4, j5, j6;
};

// 读取点位文件，支持逗号分隔
std::vector<JointAngles> readPointsFromFile(const std::string &file_path) {
    std::vector<JointAngles> points;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return points;
    }

    std::string line;
    while (std::getline(file, line)) {
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // 使用逗号分割每一行的内容
        std::stringstream ss(line);
        std::string value;
        JointAngles point;
        int i = 0;

        while (std::getline(ss, value, ',')) {
            try {
                double joint_value = std::stod(value); // 将字符串转换为双精度浮点数
                switch (i) {
                    case 0: point.j1 = joint_value; break;
                    case 1: point.j2 = joint_value; break;
                    case 2: point.j3 = joint_value; break;
                    case 3: point.j4 = joint_value; break;
                    case 4: point.j5 = joint_value; break;
                    case 5: point.j6 = joint_value; break;
                    default: break;
                }
                i++;
            } catch (const std::exception &e) {
                std::cerr << "Error parsing value: " << value << " (" << e.what() << ")" << std::endl;
                break;
            }
        }

        // 如果解析到的值不足6个，忽略该行
        if (i == 6) {
            points.push_back(point);
        } else {
            std::cerr << "Invalid line format: " << line << std::endl;
        }
    }

    file.close();
    return points;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "control_test");
    ros::NodeHandle nh;
    std::string robotIp = "192.168.43.9";
    unsigned int controlPort = 29999;
    unsigned int movePort = 30003;
    unsigned int feekPort = 30004;

    Dobot::CDobotMove m_DobotMove;
    Dobot::CDashboard m_Dashboard;
    Dobot::CFeedback m_CFeedback;
    Dobot::CFeedbackData feedbackData;
    Dobot::CErrorInfoBeans m_ErrorInfoBeans;
    Dobot::CErrorInfoHelper m_CErrorInfoHelper;

    bool plan_real_robot = false;
    nh.getParam("plan_real_robot", plan_real_robot);
    int robot_speed_ratio_ = 20;
    int robot_acc_ratio_ = 50;
    int robot_cp_ratio_ = 50;
    nh.getParam("robot_speed_ratio", robot_speed_ratio_);
    nh.getParam("robot_acc_ratio", robot_acc_ratio_);
    nh.getParam("robot_cp_ratio", robot_cp_ratio_);

    if(plan_real_robot){
        m_Dashboard.Connect(robotIp, controlPort);
        m_DobotMove.Connect(robotIp, movePort);
        m_CFeedback.Connect(robotIp, feekPort);
        // 连接并启动机器人
        m_Dashboard.EnableRobot();
        m_Dashboard.ClearError();
        m_Dashboard.SpeedFactor(robot_speed_ratio_);
        m_Dashboard.AccJ(robot_acc_ratio_);
        m_Dashboard.CP(robot_cp_ratio_);
    }

    // 点位文件路径
    const std::string file_path = "/home/roboert/MP_WS/src/gp_planner/exp_data/joint_angles_1.txt"; // 假设点位文件名为 path_points.csv

    // 读取点位文件
    std::vector<JointAngles> points = readPointsFromFile(file_path);
    if (points.empty()) {
        std::cerr << "No points to execute." << std::endl;
        return -1;
    }
    for(const auto &point: points){
        // std::cout<<point.j1<<","<<point.j2<<","<<point.j3<<","<<point.j4<<","<<point.j5<<","<<point.j6<<std::endl;
        if(plan_real_robot){
            // auto start_time = std::chrono::high_resolution_clock::now();
            // 发送运动指令
            Dobot::CJointPoint Point;
            Point.j1=point.j1;
            Point.j2=point.j2;
            Point.j3=point.j3;
            Point.j4=point.j4;
            Point.j5=point.j5;
            Point.j6=point.j6;
            // m_Dashboard.ClearError();
            // m_Dashboard.Continue();
            // m_DobotMove.Sync();
            // m_DobotMove.ServoJ(Point,"t=0.1","lookahead_time=50","gain=500");
            m_DobotMove.JointMovJ(Point);
            m_DobotMove.Sync();
            // // 记录结束时间
            // auto end_time = std::chrono::high_resolution_clock::now();

            // // 计算指令执行时间
            // auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            // // 计算剩余时间并等待
            // int remaining_time = 30 - elapsed_time;  // 30ms 控制周期
            // if (remaining_time > 0) {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(remaining_time));
            // } else {
            //     std::cerr << "Warning: Execution time exceeded 30ms! (" << elapsed_time << "ms)" << std::endl;
            // }
        }
    }


    if(plan_real_robot){
        // 断开连接
        m_DobotMove.Disconnect();
        m_Dashboard.Disconnect();
        m_CFeedback.Disconnect();
    }

    return 0;
}