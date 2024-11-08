//
// Created by robert on 23-4-21.
//

#ifndef RRT_PLANNER_INTERPOLATION_H
#define RRT_PLANNER_INTERPOLATION_H

#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
namespace rrt_planner{

double basis_function(const std::vector<double>& nodes, int i, int p, double u);

std::vector<double> create_nodes(const std::vector<VectorXd> &data_points);

MatrixXd find_control_points(const std::vector<VectorXd> &data_points, std::vector<double> &nodes);

std::vector<VectorXd> interplote(const std::vector<VectorXd> &data_points, int inter_num, int dof);

}   // namespace rrt_planner

#endif //RRT_PLANNER_INTERPOLATION_H
