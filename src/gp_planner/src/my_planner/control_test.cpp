#include "my_planner/MyPlanner.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include "Dashboard.h"
#include "DobotMove.h"
#include "Feedback.h"
#include "ErrorInfoBean.h"
#include "ErrorInfoHelper.h"

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
    m_Dashboard.Connect(robotIp, controlPort);
    m_DobotMove.Connect(robotIp, movePort);
    m_CFeedback.Connect(robotIp, feekPort);

    // 连接并启动机器人
    m_Dashboard.EnableRobot();
    m_Dashboard.ClearError();

    // 发送运动指令
    Dobot::CJointPoint PointA;
    Dobot::CJointPoint PointB;
    std::vector<double> arm_pos_ = {-0.3896469859665963, 1.4763991778866625, -0.6610719524804844, -0.5897822008469467, -1.0095268161938282, -0.025555435324681852}; 
    Eigen::VectorXd target_a(6),target_b(6);
    target_a << 0.656,1.433,-0.866,-0.566,-2.115,0;  // 点 A
    target_b << -0.851,1.309,-0.735,-0.574,-0.608,0;  // 点 B
    // PointA.j1=target_a[0] * 180.0 / M_PI;
    // PointA.j2=target_a[1] * 180.0 / M_PI;
    // PointA.j3=target_a[2] * 180.0 / M_PI;
    // PointA.j4=target_a[3] * 180.0 / M_PI;
    // PointA.j5=target_a[4] * 180.0 / M_PI;
    // PointA.j6=target_a[5] * 180.0 / M_PI;

    // PointA.j1=target_b[0] * 180.0 / M_PI;
    // PointA.j2=target_b[1] * 180.0 / M_PI;
    // PointA.j3=target_b[2] * 180.0 / M_PI;
    // PointA.j4=target_b[3] * 180.0 / M_PI;
    // PointA.j5=target_b[4] * 180.0 / M_PI;
    // PointA.j6=target_b[5] * 180.0 / M_PI;
    m_DobotMove.JointMovJ(PointA);
    // DobotTcpDemo::moveArriveFinish(PointA);
    // m_DobotMove.JointMovJ(PointB);
    // DobotTcpDemo::moveArriveFinish(PointB);

    // 断开连接
    m_DobotMove.Disconnect();
    m_Dashboard.Disconnect();
    m_CFeedback.Disconnect();

    return 0;
}