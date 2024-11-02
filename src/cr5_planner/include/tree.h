//
// Created by robert on 9/4/22.
//
#ifndef CR5_PLANNER_TREE_H
#define CR5_PLANNER_TREE_H

#include "header.h"

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


#endif //CR5_PLANNER_TREE_H
