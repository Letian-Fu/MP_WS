//
// Created by robert on 9/4/22.
//
#include "kinematics.h"

namespace rrt_planner{

Matrix<double,4,4> transformMatrix_DH(const VectorXd& dh_alpha,const VectorXd& dh_a,const VectorXd& dh_d,const VectorXd& dh_theta,const VectorXd& theta)
{
    VectorXd theta_real;
    theta_real.resize(6);
    theta_real = theta + dh_theta;
    Matrix<double,4,4> trans_Mat,temp_Mat;
    trans_Mat.setIdentity();
    for(int i = 0; i<6; i++)
    {
        temp_Mat.setZero();
        temp_Mat(0,0)=cos(theta_real(i));
        temp_Mat(0,1)=-sin(theta_real(i))*cos(dh_alpha(i));
        temp_Mat(0,2)=sin(theta_real(i))*sin(dh_alpha(i));
        temp_Mat(0,3)=dh_a(i)*cos(theta_real(i));
        temp_Mat(1,0)= sin(theta_real(i));
        temp_Mat(1,1)=cos(theta_real(i))*cos(dh_alpha(i));
        temp_Mat(1,2)=-cos(theta_real(i))*sin(dh_alpha(i));
        temp_Mat(1,3)=dh_a(i)*sin(theta_real(i));
        temp_Mat(2,0)=0.0;
        temp_Mat(2,1)=sin(dh_alpha(i));
        temp_Mat(2,2)=cos(dh_alpha(i));
        temp_Mat(2,3)=dh_d(i);
        temp_Mat(3,0)=0.0;
        temp_Mat(3,1)=0.0;
        temp_Mat(3,2)=0.0;
        temp_Mat(3,3)=1.0;
        trans_Mat = trans_Mat * temp_Mat;
        // std::cout<<i<<std::endl<<trans_Mat<<std::endl;
    }
    return trans_Mat;
}

Matrix<double,4,4> orientationMatrix(double x,double y,double z,double yaw,double pitch,double roll)
{
    Matrix<double,4,4> OriMat;
    OriMat.setZero();
    OriMat(0,0)=cos(yaw)*cos(pitch);
    OriMat(0,1)=cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll);
    OriMat(0,2)=cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
    OriMat(0,3)=x;
    OriMat(1,0)=sin(yaw)*cos(pitch);
    OriMat(1,1)=sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll);
    OriMat(1,2)=sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
    OriMat(1,3)=y;
    OriMat(2,0)=-sin(pitch);
    OriMat(2,1)=cos(pitch)*sin(roll);
    OriMat(2,2)=cos(pitch)*cos(roll);
    OriMat(2,3)=z;
    OriMat(3,0)=0.0;
    OriMat(3,1)=0.0;
    OriMat(3,2)=0.0;
    OriMat(3,3)=1;
    return OriMat;
}

VectorXd ReverseKinematics_Collision(const VectorXd& dh_alpha,const VectorXd& dh_a,const VectorXd& dh_d,const VectorXd& dh_theta,
                                     Matrix<double,4,4> oriMat,
                                     const planning_scene::PlanningScenePtr& col_scene,
                                     const robot_state::JointModelGroup* joint_model_group)
{
    int success_num=0;
    VectorXd theta(6);
    double nx = oriMat(0,0);
    double ny = oriMat(1,0);
    double nz = oriMat(2,0);
    double ox = oriMat(0,1);
    double oy = oriMat(1,1);
    double oz = oriMat(2,1);
    double ax = oriMat(0,2);
    double ay = oriMat(1,2);
    double az = oriMat(2,2);
    double px = oriMat(0,3);
    double py = oriMat(1,3);
    double pz = oriMat(2,3);
    for(int i=0;i<8;i++)
    {
        int flag=0;
        if(i==0)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                    ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
//                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==1)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                    ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
//                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = -acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==2)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                    ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
//                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==3)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                    ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
//                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = -acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==4)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), -sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                    ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
//                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==5)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), -sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                    ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
//                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = -acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==6)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), -sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                    ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
//                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==7)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), -sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                    ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
//                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = -acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else
            theta.setZero();
//            std::cout<<theta.transpose()*180/M_PI<<std::endl;
        if(flag==1)
        {
            for(int j=0;j<6;j++)
            {
                while(theta(j)>2*M_PI || theta(j)<-2*M_PI)
                {
                    if(theta(j)>2*M_PI)
                        theta(j)-=2*M_PI;
                    if(theta(j)<-2*M_PI)
                        theta(j)+=2*M_PI;
                }
            }
                std::vector<double> joint_values;
                for (int j = 0; j < 6; j++)
                    joint_values.push_back(theta(j));
                robot_state::RobotState &current_state = col_scene->getCurrentStateNonConst();
                current_state.setJointGroupPositions(joint_model_group, joint_values);
                collision_detection::CollisionRequest collision_request;
                collision_detection::CollisionResult collision_result;
                col_scene->checkCollision(collision_request, collision_result, current_state);
                if (collision_result.collision == 0) {
                    success_num += 1;
                    break;
                }
                else
                    theta.setZero();
        }
    }
    if(success_num>0)
        return theta;
    else
    {
        theta.resize(1);
        return theta;
    }
}

VectorXd ReverseKinematics_Collision_Nearest(const VectorXd& dh_alpha,const VectorXd& dh_a,const VectorXd& dh_d,const VectorXd& dh_theta,
                                     Matrix<double,4,4> oriMat,
                                     const planning_scene::PlanningScenePtr& col_scene,
                                     const robot_state::JointModelGroup* joint_model_group,
                                     VectorXd now_joint)
                                     {
    std::vector<VectorXd> solutions;
    int success_num=0;
    VectorXd theta(6);
    double nx = oriMat(0,0);
    double ny = oriMat(1,0);
    double nz = oriMat(2,0);
    double ox = oriMat(0,1);
    double oy = oriMat(1,1);
    double oz = oriMat(2,1);
    double ax = oriMat(0,2);
    double ay = oriMat(1,2);
    double az = oriMat(2,2);
    double px = oriMat(0,3);
    double py = oriMat(1,3);
    double pz = oriMat(2,3);
    for(int i=0;i<8;i++)
    {
        int flag=0;
        if(i==0)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                            ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
            //                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==1)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                            ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
            //                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = -acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==2)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                            ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
            //                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==3)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                            ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
            //                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = -acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==4)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), -sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                            ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
            //                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==5)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), -sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                            ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
            //                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = -acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==6)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), -sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                            ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
            //                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else if(i==7)
        {
            double m = dh_d(5) * ay - py;
            double n = ax * dh_d(5) - px;
            double theta1 = atan2(m, n) - atan2(dh_d(3), -sqrt(m * m + n * n - dh_d(3) * dh_d(3)));
            double theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
            double mm = nx * sin(theta1) - ny * cos(theta1);
            double nn = ox * sin(theta1) - oy * cos(theta1);
            double theta6 = atan2(mm, nn) - atan2(sin(theta5), 0);
            double mmm = dh_d(4) * (sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) + cos(theta6) * (
                    ox * cos(theta1) + oy * sin(theta1))) + px * cos(theta1) - dh_d(5) * (
                            ax * cos(theta1) + ay * sin(theta1)) + py * sin(theta1);
            double nnn = dh_d(4) * (oz * cos(theta6) + nz * sin(theta6)) + pz - dh_d(0) - az * dh_d(5);
            double temp = (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2));
            //                std::cout<<"temp: "<<temp<<std::endl<<"acos: "<<acos(temp)<<std::endl;
            if (-1 <= temp && temp <= 1)
            {
                flag=1;
                double theta3 = -acos(
                        (mmm * mmm + nnn * nnn - dh_a(1) * dh_a(1) - dh_a(2) * dh_a(2)) / (2 * dh_a(1) * dh_a(2)));
                double s2 = ((dh_a(2) * cos(theta3) + dh_a(1)) * nnn - dh_a(2) * sin(theta3) * mmm) / (
                        dh_a(1) * dh_a(1) + dh_a(2) * dh_a(2) + 2 * dh_a(1) * dh_a(2) * cos(theta3));
                double c2 = (mmm + dh_a(2) * sin(theta3) * s2) / (dh_a(2) * cos(theta3) + dh_a(1));
                double theta2 = atan2(s2, c2);
                double theta4 = atan2((-sin(theta6) * (nx * cos(theta1) + ny * sin(theta1)) - cos(theta6) * (
                        ox * cos(theta1) + oy * sin(theta1))), (oz * cos(theta6) + nz * sin(theta6))) - theta2 -
                                theta3;
                theta << theta1, theta2, theta3, theta4, theta5, theta6;
                theta = theta - dh_theta;
            }
            else
                theta.setZero();
        }
        else
            theta.setZero();
        //            std::cout<<theta.transpose()*180/M_PI<<std::endl;
        if(flag==1)
        {
            for(int j=0;j<6;j++)
            {
                while(theta(j)>1*M_PI || theta(j)<-1*M_PI)
                {
                    if(theta(j)>1*M_PI)
                        theta(j)-=1*M_PI;
                    if(theta(j)<-1*M_PI)
                        theta(j)+=1*M_PI;
                }
            }
            std::vector<double> joint_values;
            for (int j = 0; j < 6; j++)
                joint_values.push_back(theta(j));
            robot_state::RobotState &current_state = col_scene->getCurrentStateNonConst();
            current_state.setJointGroupPositions(joint_model_group, joint_values);
            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            col_scene->checkCollision(collision_request, collision_result, current_state);
            if (collision_result.collision == 0) {
                success_num += 1;
                solutions.push_back(theta);
            }
            else
                theta.setZero();
        }
    }
//    std::cout<<solutions.size()<<std::endl;
    if(success_num>0)
    {
        double min_error=10000000;
        int final_idx=0;
        for(int i=0;i<solutions.size();i++)
        {
            VectorXd temp=solutions[i]-now_joint;
            for(int j=0;j<6;j++)
            {
                while(temp(j)>1*M_PI || temp(j)<-1*M_PI)
                {
                    if(temp(j)>1*M_PI)
                        temp(j)-=1*M_PI;
                    if(temp(j)<-1*M_PI)
                        temp(j)+=1*M_PI;
                }
            }
            double temp_error=temp.norm();
//            std::cout<<temp_error<<std::endl;
//            std::cout<<solutions[i].transpose()<<std::endl;
            if(temp_error<min_error)
            {
                final_idx=i;
                min_error=temp_error;
            }
        }
        theta=solutions[final_idx];
        return theta;
    }
    else
    {
        theta.resize(1);
        return theta;
    }
}

}       // namespace rrt_planner
