#include "ros/ros.h"
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
    double pointa[] = {37.9, 82.1, -49.6, -32.4, -121.27, 0};
    // double pointb[] = {-48.8, 74.9, -42.1, -32.9, -34.8, 0};
    std::vector<double> arm_pos_ = {-0.3896469859665963, 1.4763991778866625, -0.6610719524804844, -0.5897822008469467, -1.0095268161938282, -0.025555435324681852}; 
    // for(int i=0;i<6;i++){
    //     arm_pos[i] = arm_pos[i] * 180.0 / M_PI;
    // }
    // memcpy(&PointA, pointa, sizeof(PointA));
    // memcpy(&PointB, pointb, sizeof(PointB));
    PointA.j1=arm_pos_[0] * 180.0 / M_PI;
    PointA.j2=arm_pos_[1] * 180.0 / M_PI;
    PointA.j3=arm_pos_[2] * 180.0 / M_PI;
    PointA.j4=arm_pos_[3] * 180.0 / M_PI;
    PointA.j5=arm_pos_[4] * 180.0 / M_PI;
    PointA.j6=arm_pos_[5] * 180.0 / M_PI;
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