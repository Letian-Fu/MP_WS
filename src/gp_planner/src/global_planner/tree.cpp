//
// Created by robert on 9/4/22.
//

#include "tree.h"
namespace rrt_planner{

Tree::Tree() {
    dof=0;
}

void Tree::set_dof(int DOF) {
    dof=DOF;
}

VectorXd Tree::generate_random_point(const VectorXd& goal_point) const {
    VectorXd random_point;
    random_point.resize(goal_point.size());
    double p_rand,p_goal;
    p_goal = 0.01;
    p_rand = rand() / double(RAND_MAX);
    if(p_rand<=p_goal)
        random_point = goal_point;
    else
    {
        for (int i = 0; i < dof; i++)
            random_point(i) = 2*M_PI * ((2 * (rand() / double(RAND_MAX))) - 1);
    }
    return random_point;
}

double Tree::cal_dist(const VectorXd& p1,const VectorXd& p2) const {
    double dist;
    VectorXd temp;
    temp.resize(dof);
    for(int i=0;i<dof;i++)
        temp(i)=p1(i)-p2(i);
    dist = temp.norm();
    return dist;
}

VectorXd Tree::find_nearest_point(const VectorXd& p) {
    VectorXd nearest_point;
    double nearest_dist;
    nearest_point = points[0];
    nearest_dist = 100000000;
    for (auto & point : points)
    {
        double temp_dist;
        temp_dist = cal_dist(point,p);
        if (temp_dist < nearest_dist)
        {
            nearest_dist = temp_dist;
            nearest_point = point;
        }
    }
    return nearest_point;
}

void Tree::add_point(const VectorXd& p) {
    points.push_back(p);
    if(points.size()==1)
        points[0](dof+1)=-1;
}

void Tree::find_path(const VectorXd& start, const VectorXd& goal) {
    path.push_back(goal);
    while(path[path.size()-1]!=start)
    {
        if(path[path.size()-1](dof+1)>0)
            path.push_back(points[path[path.size()-1](dof+1)-1]);
        else {
            std::cout << "Not Found" << std::endl;
            break;
        }
    }
    std::reverse(path.begin(), path.end());
}

double Tree::calculate_path_length() {
    double length=0;
    for(int i=1;i<path.size();i++)
    {
        length+= cal_dist(path[i],path[i-1]);
//        for(int j=0;j<dof;j++)
//            length+= abs(path[i](j)-path[i-1](j));
    }
    return length;
}

void Tree::renew() {
    points.resize(0);
    path.resize(0);
    dof=0;
}

bool Tree::is_same(VectorXd p1, VectorXd p2) const {
    bool flag=true;
    for(int i=0;i<dof;i++)
    {
        if(p1(i)!=p2(i))
        {
            flag=false;
            break;
        }
    }
    return flag;
}

Tree::~Tree() = default;

}   // namespace rrt_planner
