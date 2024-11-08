//
// Created by robert on 9/4/22.
//

#ifndef RRT_PLANNER_ENV_H
#define RRT_PLANNER_ENV_H

#pragma once

#include "header.h"
#include "kinematics.h"
#include "planner.h"

namespace rrt_planner{

void CreateFloor(const MyPlanner& planner);

void CreateBox(const MyPlanner& planner,
               const std::string& box_name,
               double box_x,
               double box_y,
               double box_width,
               double box_length,
               double box_height,
               int target_flag);

std::vector<VectorXd> CreateRandomTargets(const MyPlanner& planner,
                                   const std::string& box_name,
                                   double box_x,
                                   double box_y,
                                   int target_num);

void CreateObs(const MyPlanner& planner,
               const std::string& obs_name,
               double obs_x,
               double obs_y,
               double box_width,
               double box_length,
               double box_height);

void CreateObs2(const MyPlanner& planner,
                const std::string& obs_name,
                double obs_x,
                double obs_y,
                double obs_z,
                double box_width,
                double box_length,
                double box_height);

VectorXd CatchTest_fixed_targets(MyPlanner planner,
                   const planning_scene::PlanningScenePtr& scene,
                   VectorXd box_pos,
                   VectorXd final_pos,
                   const std::string& box_name,
                   int execute_flag,
                   int print_flag);

VectorXd CatchTest_random_targets(MyPlanner planner,
                   const planning_scene::PlanningScenePtr& scene,
                   const VectorXd& box_pos,
                   VectorXd final_pos,
                   const std::string& box_name,
                   std::vector<VectorXd> target_pos,
                   int execute_flag,
                   int print_flag);

}   // namespace rrt_planner
#endif //RRT_PLANNER_ENV_H
