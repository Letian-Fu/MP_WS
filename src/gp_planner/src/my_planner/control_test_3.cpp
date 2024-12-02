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

class DobotController{

private:
    Dobot::CDobotMove m_DobotMove;
    Dobot::CDashboard m_Dashboard;
    Dobot::CFeedback m_CFeedback;
    Dobot::CFeedbackData feedbackData;
    Dobot::CErrorInfoBeans m_ErrorInfoBeans;
    Dobot::CErrorInfoHelper m_CErrorInfoHelper;
    bool plan_real_robot_;
    int robot_speed_ratio_ ;
    int robot_acc_ratio_ ;
    int robot_cp_ratio_ ;
    ros::Subscriber cur_joint_sub_;

public:
    DobotController(ros::NodeHandle &nh){
        nh.getParam("plan_real_robot", plan_real_robot_);
        nh.getParam("robot_speed_ratio", robot_speed_ratio_);
        nh.getParam("robot_acc_ratio", robot_acc_ratio_);
        nh.getParam("robot_cp_ratio", robot_cp_ratio_);
        cur_joint_sub_ = nh.subscribe("/current_joints_deg", 10, &DobotController::JointCallback,this);
    }

    void Connect(){
        if(plan_real_robot_){
            std::string robotIp = "192.168.43.9";
            unsigned int controlPort = 29999;
            unsigned int movePort = 30003;
            unsigned int feekPort = 30004;
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
    }

    void Disconnect(){
        if(plan_real_robot_){
            // 断开连接
            m_DobotMove.Disconnect();
            m_Dashboard.Disconnect();
            m_CFeedback.Disconnect();
        }
    }

    void JointCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        // 检查消息的 layout 和数据是否有效
        if (msg->layout.dim.empty() || msg->layout.dim[0].label != "joints") {
            ROS_ERROR("Invalid layout in received joint message. Expected label: 'joints'.");
            return;
        }

        if (msg->layout.dim[0].size != 6 || msg->data.size() != 6) {
            ROS_ERROR("Invalid joint data size. Expected 6 values, but got %lu.", msg->data.size());
            return;
        }
        if(plan_real_robot_){
            Dobot::CJointPoint Point;
            Point.j1=msg->data[0];
            Point.j2=msg->data[1];
            Point.j3=msg->data[2];
            Point.j4=msg->data[3];
            Point.j5=msg->data[4];
            Point.j6=msg->data[5];
            m_Dashboard.ClearError();
            // m_Dashboard.Continue();
            // m_DobotMove.Sync();
            m_DobotMove.ServoJ(Point,"t=0.5");
            // m_DobotMove.Sync();
            // m_DobotMove.JointMovJ(Point);
        }
    }

    ~DobotController() {
        // 确保在析构时断开连接
        Disconnect();
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "control_test");
    ros::NodeHandle nh;

    DobotController controller(nh);
    // 连接机器人
    controller.Connect();
    // 开始多线程处理回调
    ros::AsyncSpinner spinner(2); // 使用 2 个线程处理回调
    spinner.start();
    // 等待 ROS 运行
    ros::waitForShutdown();

    return 0;
}