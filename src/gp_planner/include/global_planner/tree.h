//
// Created by robert on 9/4/22.
//
#ifndef RRT_PLANNER_TREE_H
#define RRT_PLANNER_TREE_H

#pragma once

#include "header.h"

namespace rrt_planner{

class Tree
{
public:
    std::vector<VectorXd> points;
    std::vector<VectorXd> path;
    int dof;
public:
    Tree();
    void set_dof(int DOF);
    VectorXd generate_random_point(const VectorXd& goal_point) const;
    double cal_dist(const VectorXd& p1,const VectorXd& p2) const;
    VectorXd find_nearest_point(const VectorXd& p);
    void add_point(const VectorXd& p);
    void find_path(const VectorXd& start, const VectorXd& goal);
    double calculate_path_length();
    void renew();
    bool is_same(VectorXd p1,VectorXd p2) const;
    ~Tree();

};

}   // namespace rrt_planner

#endif //RRT_PLANNER_TREE_H
