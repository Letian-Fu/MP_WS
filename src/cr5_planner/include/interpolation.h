//
// Created by robert on 23-4-21.
//

#ifndef CR5_PLANNER_INTERPOLATION_H
#define CR5_PLANNER_INTERPOLATION_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

double basis_function(const std::vector<double>& nodes, int i, int p, double u);

std::vector<double> create_nodes(const std::vector<VectorXd> &data_points);

MatrixXd find_control_points(const std::vector<VectorXd> &data_points, std::vector<double> &nodes) ;

std::vector<VectorXd> interplote(const std::vector<VectorXd> &data_points, int inter_num, int dof) ;

#endif //CR5_PLANNER_INTERPOLATION_H
