//
// Created by robert on 9/4/22.
//

#ifndef CR5_PLANNER_KINEMATICS_H
#define CR5_PLANNER_KINEMATICS_H

#include "header.h"

Matrix<double,4,4> transformMatrix_DH(const VectorXd& dh_alpha,const VectorXd& dh_a,const VectorXd& dh_d,const VectorXd& dh_theta,const VectorXd& theta);
Matrix<double, 4, 4> orientationMatrix(double x, double y, double z, double yaw, double pitch, double roll);
VectorXd ReverseKinematics_Collision(const VectorXd& dh_alpha,const VectorXd& dh_a,const VectorXd& dh_d,const VectorXd& dh_theta,
                                     Matrix<double,4,4> oriMat,
                                     const planning_scene::PlanningScenePtr& col_scene,
                                     const robot_state::JointModelGroup* joint_model_group);
VectorXd ReverseKinematics_Collision_Nearest(const VectorXd& dh_alpha,const VectorXd& dh_a,const VectorXd& dh_d,const VectorXd& dh_theta,
                                             Matrix<double,4,4> oriMat,
                                             const planning_scene::PlanningScenePtr& col_scene,
                                             const robot_state::JointModelGroup* joint_model_group,
                                             VectorXd now_joint);

#endif //CR5_PLANNER_KINEMATICS_H
