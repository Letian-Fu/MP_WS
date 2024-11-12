#include "rrtplanner.h"
#include <memory>
#include <string.h>

namespace rrt_planner{
RRTPlanner::RRTPlanner(ros::NodeHandle nh, robot_model::RobotModelPtr kinematic_model)
:   kinematic_model_(kinematic_model){
    NUM_INITIATION_=1;
    is_plan_success_ = false;
    tree_=Tree();
    is_adapt_step_=0;
    is_adapt_fai_=0;
    if (nh.hasParam("robot/DOF"))
        nh.getParam("robot/DOF", dof_);
    else{
        dof_=6;
    }
    if (nh.hasParam("rrt_planner/step_rate_"))
        nh.getParam("rrt_planner/step_rate_", step_rate_);
    else
        step_rate_=0.03;
    if (nh.hasParam("rrt_planner/fai_rate_"))
        nh.getParam("rrt_planner/fai_rate_", fai_rate_);
    else
        fai_rate_=1.01;
    if (nh.hasParam("rrt_planner/MAX_TREE_NUM_"))
        nh.getParam("rrt_planner/MAX_TREE_NUM_", MAX_TREE_NUM_);
    else
        MAX_TREE_NUM_=2000;
    dh_alpha_.resize(dof_);
    dh_a_.resize(dof_);
    dh_d_.resize(dof_);
    dh_theta_.resize(dof_);
    start_conf_.resize(dof_);
    goal_conf_.resize(dof_);
    arm_pos_.resize(dof_);
    dh_alpha_<<M_PI_2,0.0,0.0,M_PI_2,-M_PI_2,0.0;
    dh_a_<<0.0,-0.427,-0.357,0.0,0.0,0.0;
    dh_d_<<0.147,0.0,0.0,0.141,0.116,0.105;
    dh_theta_<<0.0,-M_PI_2,0.0,-M_PI_2,0.0,0.0;
    step_ = step_rate_ * 2 * M_PI;
    fai_=fai_rate_*step_;
    MAX_FAIL_NUM_ = 5*MAX_TREE_NUM_;
    inter_num_ = 50;
    joint_model_group_ = kinematic_model_->getJointModelGroup("arm");
    joint_all_names_ = joint_model_group_->getJointModelNames();
    kinematic_state_=std::make_shared<robot_state::RobotState>(kinematic_model_);
    kinematic_state_->setToDefaultValues();
}

bool RRTPlanner::CheckCollision(VectorXd proper_point, VectorXd new_point, const planning_scene::PlanningScenePtr& col_scene, double inter_step) const {
    bool is_collision = false;
    std::vector<double> joint_positions;
    robot_state::RobotState &current_state = col_scene->getCurrentStateNonConst();
    current_state.copyJointGroupPositions(joint_model_group_, joint_positions);
    robot_state::RobotState state(col_scene->getRobotModel());

    VectorXd n_p,p_p;
    n_p.resize(dof_);
    p_p.resize(dof_);
    for (int i = 0; i < dof_; i++)
    {
        p_p(i)=proper_point(i);
        n_p(i)=new_point(i);
    }

    std::vector<double> temp;
    for (int i = 0; i < dof_; i++)
        temp.push_back(abs(n_p(i) - p_p(i)) / inter_step);
    int num = floor(*std::max_element(temp.begin(), temp.end()));
    if (num < 1)
        num = 1;

    VectorXd delta;
    delta = (n_p - p_p) / num;
    for (int i = 0; i < num + 1; i++) {
        VectorXd temp_vec;
        temp_vec = n_p - i * delta;
        for (int j = 0; j < temp_vec.size(); j++)
            joint_positions[j] = temp_vec(j);
        state.setVariablePositions(joint_all_names_, joint_positions);
        current_state.setJointGroupPositions(joint_model_group_, joint_positions);
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        col_scene->checkCollision(collision_request, collision_result, current_state);
        if (collision_result.collision == 1)
            is_collision = true;
        if (is_collision)
            break;
    }
    return is_collision;
}

bool RRTPlanner::RRT_Plan(const planning_scene::PlanningScenePtr& col_scene, const VectorXd& start_conf, const VectorXd& end_conf, std::vector<VectorXd>& results) {
    step_rate_=0.02;
    fai_rate_=1.01;
    step_=step_rate_*2*M_PI;
    fai_=fai_rate_*step_;
    Tree tree1,tree2;
    tree_.renew();
    tree_.set_dof(dof_);
    tree1.renew();
    tree1.set_dof(dof_);
    tree2.renew();
    tree2.set_dof(dof_);
    VectorXd start_point,goal_point;
    VectorXd current_point1,current_point2,sample_point,proper_point1,proper_point2;
    int idx1=1;
    int idx2=1;

    sample_point.resize(dof_+2);
    start_point.resize(dof_+2);
    goal_point.resize(dof_+2);
    current_point1.resize(dof_+2);
    proper_point1.resize(dof_+2);
    current_point2.resize(dof_+2);
    proper_point2.resize(dof_+2);

    for(int i=0;i<dof_;i++) {
        start_point(i) = start_conf(i);
        goal_point(i) = end_conf(i);
    }
    start_point(dof_)=idx1;
    start_point(dof_+1)=-1;
    goal_point(dof_)=idx2;
    goal_point(dof_+1)=-1;

    current_point1 = start_point;
    current_point2 = goal_point;
    tree1.add_point(start_point);
    tree2.add_point(goal_point);
    double bias=0.5;
    double sig_x = 0;
    double sig_delta_x = 0.2;
    double a=1;
    double c=1;
    double sig_y = bias + a/ (1 + exp(-c*sig_x));

    double delta_fai_add = 0.1;
    double delta_fai_minus = 0.2;
    double fai_max=1.4;
    double fai_min=0.01;

    int pathFound=0;
    int num_initiation =0;
    int fail_num = 0;
    int temp_success=0;
    int swap = 0;
    double check_inter_step=M_PI/45;
    int is_print_step=0;
    int is_print_fai=0;
    if(is_print_step==1)
        std::cout<<sig_y<<std::endl;
    if(is_print_fai==1)
        std::cout<<fai_rate_<<std::endl;
    is_plan_success_ = true;
    while (tree_.cal_dist(current_point1,current_point2) >= step_ ||
           CheckCollision(current_point1, current_point2, col_scene, check_inter_step))
    {
        if((swap%2)==0)
        {
            sample_point = tree1.generate_random_point(goal_point);
            proper_point1 = tree1.find_nearest_point(sample_point);
            if (!tree1.is_same(sample_point, proper_point1)) {
                VectorXd new_point1;
                new_point1.resize(sample_point.size());
                double dist1 = tree1.cal_dist(sample_point,proper_point1);
                double dist2 = tree1.cal_dist(goal_point,proper_point1);
                new_point1=proper_point1+step_/dist1*(sample_point-proper_point1)+fai_/dist2*(goal_point-proper_point1);
                bool is_collision = CheckCollision(proper_point1, new_point1, col_scene, check_inter_step);
                if (!is_collision) {
//                    fai_rate_=origin_fai_rate;
                    if(is_adapt_fai_==1){
                        fai_rate_+=delta_fai_add;
                        if(fai_rate_>fai_max)
                            fai_rate_=fai_max;
                        fai_=fai_rate_*step_;
                        if(is_print_fai==1)
                            std::cout<<fai_rate_<<std::endl;
                    }
                    else{
                        temp_success+=1;
                        if(temp_success>1)
                            fai_ = fai_rate_ * step_;
                    }

                    idx1 += 1;
                    new_point1(dof_) = idx1;
                    new_point1(dof_ + 1) = proper_point1(dof_);
                    tree1.add_point(new_point1);
                    current_point1 = new_point1;
                    proper_point2 = tree2.find_nearest_point(new_point1);
                    double dist = tree2.cal_dist(new_point1, proper_point2);
                    if (dist < step_) {
                        if (!CheckCollision(proper_point2, new_point1, col_scene, check_inter_step))
                            pathFound = 1;
                    }
                    else {
                        if(is_adapt_step_==1)
                        {
                            sig_x += sig_delta_x;
                            sig_y = bias + a/ (1 + exp(-c*sig_x));
                            step_ = step_rate_*2*M_PI*sig_y;
                            if(is_print_step==1)
                                std::cout<<sig_y<<std::endl;
                        }
                        else
                            step_=step_*1.0;
                        VectorXd new_point2, new_point3;
                        new_point2.resize(sample_point.size());
                        new_point3.resize(sample_point.size());
                        new_point2 = proper_point2 + step_ / dist * (new_point1 - proper_point2);
                        is_collision = CheckCollision(proper_point2, new_point2, col_scene, check_inter_step);
                        if (!is_collision) {
                            idx2 += 1;
                            new_point2(dof_) = idx2;
                            new_point2(dof_ + 1) = proper_point2(dof_);
                            tree2.add_point(new_point2);
                            current_point2 = new_point2;
                            if(is_adapt_step_==1)
                            {
                                sig_x += sig_delta_x;
                                sig_y = bias + a/ (1 + exp(-c*sig_x));
                                step_ = step_rate_*2*M_PI*sig_y;
                                if(is_print_step==1)
                                    std::cout<<sig_y<<std::endl;
                            }
                            else
                                step_=step_*1.0;
                            while (tree2.cal_dist(new_point2, new_point1) > step_) {
                                dist = tree2.cal_dist(new_point2, new_point1);
                                new_point3 = new_point2 + step_ / dist * (new_point1 - new_point2);
                                is_collision = CheckCollision(new_point3, new_point2, col_scene, check_inter_step);
                                if (!is_collision) {
                                    idx2 += 1;
                                    new_point3(dof_) = idx2;
                                    new_point3(dof_ + 1) = idx2 - 1;
                                    tree2.add_point(new_point3);
                                    current_point2 = new_point3;
                                    new_point2 = new_point3;
                                    if(is_adapt_step_==1) {
                                        sig_x += sig_delta_x;
                                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                                        step_ = step_rate_*2*M_PI*sig_y;
                                        if(is_print_step==1)
                                            std::cout<<sig_y<<std::endl;
                                    }
                                    else
                                        step_=step_*1.0;
                                } else{
                                    if(is_adapt_step_==1) {
                                        sig_x -= sig_delta_x;
                                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                                        step_ = step_rate_*2*M_PI*sig_y;
                                        if(is_print_step==1)
                                            std::cout<<sig_y<<std::endl;
                                    }
                                    else
                                        step_=step_rate_*2*M_PI;
                                    fail_num+=1;
                                    break;
                                }
                            }
                        } else {
                            if(is_adapt_step_==1) {
                                sig_x -= sig_delta_x;
                                sig_y = bias + a/ (1 + exp(-c*sig_x));
                                step_ = step_rate_*2*M_PI*sig_y;
                                if(is_print_step==1)
                                    std::cout<<sig_y<<std::endl;
                            }
                            else
                                step_=step_rate_*2*M_PI;
                            fail_num += 1;
                        }
                    }
                } else {
                    if(is_adapt_step_==1){
                        sig_x -= sig_delta_x;
                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                        step_ = step_rate_*2*M_PI*sig_y;
                        if(is_print_step==1)
                            std::cout<<sig_y<<std::endl;
                    } else
                        step_=step_rate_*2*M_PI;
                    if(is_adapt_fai_==1) {
                        fai_rate_-=delta_fai_minus;
                        if(fai_rate_<fai_min)
                            fai_rate_=fai_min;
                        fai_=fai_rate_*step_;
                        if(is_print_fai==1)
                            std::cout<<fai_rate_<<std::endl;
                    } else
                        fai_=1.01*step_;
                    fail_num += 1;
                    temp_success=0;
                }
            }
        } else if((swap%2)==1) {
            sample_point = tree2.generate_random_point(start_point);
            proper_point2 = tree2.find_nearest_point(sample_point);
            if (!tree2.is_same(sample_point, proper_point2)) {
                VectorXd new_point2;
                new_point2.resize(sample_point.size());
                double dist1 = tree2.cal_dist(sample_point,proper_point2);
                double dist2 = tree2.cal_dist(start_point,proper_point2);
                new_point2=proper_point2+step_/dist1*(sample_point-proper_point2)+fai_/dist2*(start_point-proper_point2);
                bool is_collision = CheckCollision(proper_point2, new_point2, col_scene, check_inter_step);
                if (!is_collision) {
//                    fai_rate_=origin_fai_rate;
                    if(is_adapt_fai_==1) {
                        fai_rate_+=delta_fai_add;
                        if(fai_rate_>fai_max)
                            fai_rate_=fai_max;
                        fai_=fai_rate_*step_;
                        if(is_print_fai==1)
                            std::cout<<fai_rate_<<std::endl;
                    } else {
                        temp_success+=1;
                        if(temp_success>1)
                            fai_ = fai_rate_ * step_;
                    }
                    idx2 += 1;
                    new_point2(dof_) = idx2;
                    new_point2(dof_ + 1) = proper_point2(dof_);
                    tree2.add_point(new_point2);
                    current_point2 = new_point2;
                    proper_point1 = tree1.find_nearest_point(new_point2);
                    double dist = tree1.cal_dist(new_point2, proper_point1);
                    if (dist < step_) {
                        if (!CheckCollision(proper_point1, new_point2, col_scene, check_inter_step))
                            pathFound = 1;
                    } else {
                        if(is_adapt_step_==1) {
                            sig_x += sig_delta_x;
                            sig_y = bias + a/ (1 + exp(-c*sig_x));
                            step_ = step_rate_*2*M_PI*sig_y;
                            if(is_print_step==1)
                                std::cout<<sig_y<<std::endl;
                        } else
                            step_=step_*1.0;
                        VectorXd new_point1, new_point3;
                        new_point1.resize(sample_point.size());
                        new_point3.resize(sample_point.size());
                        new_point1 = proper_point1 + step_ / dist * (new_point2 - proper_point1);
                        is_collision = CheckCollision(proper_point1, new_point1, col_scene, check_inter_step);
                        if (!is_collision) {
                            idx1 += 1;
                            new_point1(dof_) = idx1;
                            new_point1(dof_ + 1) = proper_point1(dof_);
                            tree1.add_point(new_point1);
                            current_point1 = new_point1;
                            if(is_adapt_step_==1) {
                                sig_x += sig_delta_x;
                                sig_y = bias + a/ (1 + exp(-c*sig_x));
                                step_ = step_rate_*2*M_PI*sig_y;
                                if(is_print_step==1)
                                    std::cout<<sig_y<<std::endl;
                            } else
                                step_=step_*1.0;
                            while (tree1.cal_dist(new_point1, new_point2) > step_) {
                                dist = tree1.cal_dist(new_point2, new_point1);
                                new_point3 = new_point1 + step_ / dist * (new_point2 - new_point1);
                                is_collision = CheckCollision(new_point3, new_point1, col_scene, check_inter_step);
                                if (!is_collision) {
                                    idx1 += 1;
                                    new_point3(dof_) = idx1;
                                    new_point3(dof_ + 1) = idx1 - 1;
                                    tree1.add_point(new_point3);
                                    current_point1 = new_point3;
                                    new_point1 = new_point3;
                                    if(is_adapt_step_==1) {
                                        sig_x += sig_delta_x;
                                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                                        step_ = step_rate_*2*M_PI*sig_y;
                                        if(is_print_step==1)
                                            std::cout<<sig_y<<std::endl;
                                    } else
                                        step_=step_*1.0;
                                } else{
                                    if(is_adapt_step_==1) {
                                        sig_x -= sig_delta_x;
                                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                                        step_ = step_rate_*2*M_PI*sig_y;
                                        if(is_print_step==1)
                                            std::cout<<sig_y<<std::endl;
                                    } else
                                        step_=step_rate_*2*M_PI;
                                    fail_num+=1;
                                    break;
                                }
                            }
                        } else {
                            if(is_adapt_step_==1) {
                                sig_x -= sig_delta_x;
                                sig_y = bias + a/ (1 + exp(-c*sig_x));
                                step_ = step_rate_*2*M_PI*sig_y;
                                if(is_print_step==1)
                                    std::cout<<sig_y<<std::endl;
                            } else
                                step_=step_rate_*2*M_PI;
                            fail_num += 1;
                        }
                    }
                } else {
                    if(is_adapt_step_==1) {
                        sig_x -= sig_delta_x;
                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                        step_ = step_rate_*2*M_PI*sig_y;
                        if(is_print_step==1)
                            std::cout<<sig_y<<std::endl;
                    } else
                        step_=step_rate_*2*M_PI;
                    if(is_adapt_fai_==1) {
                        fai_rate_-=delta_fai_minus;
                        if(fai_rate_<fai_min)
                            fai_rate_=fai_min;
                        fai_=fai_rate_*step_;
                        if(is_print_fai==1)
                            std::cout<<fai_rate_<<std::endl;
                    } else
                        fai_=1.01*step_;
                    fail_num += 1;
                    temp_success=0;
                }
            }

        }
        swap+=1;
        if(tree1.points.size()>=MAX_TREE_NUM_||tree2.points.size()>=MAX_TREE_NUM_) {
            std::cout<<"RRT_Plan Failed Type 1"<<std::endl;
            tree1.renew();
            tree1.set_dof(dof_);
            tree2.renew();
            tree2.set_dof(dof_);
            current_point1 = start_point;
            tree1.add_point(start_point);
            current_point2 = goal_point;
            tree2.add_point(goal_point);
            num_initiation +=1;
            idx1=1;
            idx2=1;
            is_plan_success_=false;
            break;
        }
        if(fail_num>=MAX_FAIL_NUM_) {
            std::cout<<"RRT_Plan Failed Type 2"<<std::endl;
            tree1.renew();
            tree1.set_dof(dof_);
            tree2.renew();
            tree2.set_dof(dof_);
            current_point1 = start_point;
            tree1.add_point(start_point);
            current_point2 = goal_point;
            tree2.add_point(goal_point);
            num_initiation +=1;
            idx1=1;
            idx2=1;
        }
        if(pathFound==1)
            break;
        if(num_initiation>=NUM_INITIATION_) {
            is_plan_success_ = false;
            break;
        }

    }
    if(is_plan_success_==1) {
        tree1.find_path(start_point, tree1.points[idx1 - 1]);
        tree2.find_path(goal_point, tree2.points[idx2 - 1]);
        std::reverse(tree2.path.begin(), tree2.path.end());
        tree_.points.insert(tree_.points.end(), tree1.points.begin(), tree1.points.end());
        tree_.points.insert(tree_.points.end(), tree2.points.begin(), tree2.points.end());
        tree_.path.insert(tree_.path.end(), tree1.path.begin(), tree1.path.end());
        tree_.path.insert(tree_.path.end(), tree2.path.begin(), tree2.path.end());
        std::vector<VectorXd> prue_path = Pruning_path(col_scene);
        int path_node_num = prue_path.size();
        int exec_step = path_node_num + (path_node_num - 1) * inter_num_;
        results= interplote(prue_path,inter_num_,dof_);
    }
    return is_plan_success_;

}

std::vector<VectorXd> RRTPlanner::Pruning_path(const planning_scene::PlanningScenePtr& col_scene) {
    double path_cost1=0;
    double path_cost2=0;
    int path_nodes_num=tree_.path.size();
    std::vector<VectorXd> result;
    result.push_back(tree_.path[0]);
    int start_idx,current_idx;
    double check_inter_step=M_PI/45;
    start_idx=0;
    while(tree_.is_same(result[result.size()-1],tree_.path[tree_.path.size()-1])==0)
    {
        VectorXd start_point;
        start_point =tree_.path[start_idx];
        current_idx=start_idx+1;
        for(int i=current_idx;i<tree_.path.size();i++)
        {
            VectorXd temp_point=tree_.path[i];
            bool is_collision= CheckCollision(start_point,temp_point,col_scene, check_inter_step);
            if(is_collision==0) {
                current_idx += 1;
                if(current_idx-start_idx>path_nodes_num/4)
                {
                    result.push_back(tree_.path[current_idx-1]);
                    start_idx=current_idx-1;
                    break;
                }
            }
            else
            {
                result.push_back(tree_.path[current_idx-1]);
                start_idx=current_idx-1;
                break;
            }
            if(current_idx==tree_.path.size())
            {
                result.push_back(tree_.path[current_idx-1]);
                break;
            }
        }
    }
    for(int i=1;i<result.size();i++)
        path_cost1+=tree_.cal_dist(result[i], result[i-1]);
    std::vector<VectorXd> rev_path=tree_.path;
    std::reverse(rev_path.begin(), rev_path.end());
    std::vector<VectorXd> result2;
    result2.push_back(rev_path[0]);
    int start_idx2,current_idx2;
    start_idx2=0;
    while(tree_.is_same(result2[result2.size()-1],rev_path[rev_path.size()-1])==0)
    {
        VectorXd start_point;
        start_point =rev_path[start_idx2];
        current_idx2=start_idx2+1;
        for(int i=current_idx2;i<rev_path.size();i++)
        {
            VectorXd temp_point=rev_path[i];
            bool is_collision= CheckCollision(start_point,temp_point,col_scene, check_inter_step);
            if(is_collision==0) {
                current_idx2 += 1;
                if(current_idx2-start_idx2>path_nodes_num/4)
                {
                    result2.push_back(rev_path[current_idx2-1]);
                    start_idx2=current_idx2-1;
                    break;
                }
            }
            else
            {
                result2.push_back(rev_path[current_idx2-1]);
                start_idx2=current_idx2-1;
                break;
            }
            if(current_idx2==rev_path.size())
            {
                result2.push_back(rev_path[current_idx2-1]);
                break;
            }
        }
    }
    std::reverse(result2.begin(), result2.end());
    for(int i=1;i<result2.size();i++)
        path_cost2+=tree_.cal_dist(result2[i], result2[i-1]);
    std::vector<VectorXd> final_result;
    if(path_cost1<=path_cost2)
        final_result=result;
    else
        final_result=result2;
    return final_result;
}

// void RRTPlanner::armStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
//     for (int i=0;i<dof_;i++)
//         arm_pos_(i)=msg->position[i];
// }

// void RRTPlanner::planCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg) {
//     is_plan_success_=false;
//     planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
//     monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
//     planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
//     ps->getCurrentStateNonConst().update();
//     planning_scene::PlanningScenePtr col_scene = ps->diff();
//     col_scene->decoupleParent();
//     for(int i=0;i<dof_;i++)
//     {
//         start_conf_(i) = arm_pos_(i);
//         goal_conf_(i) = msg->goal.request.goal_constraints[0].joint_constraints[i].position;
//     }
//     is_plan_success_ = RRT_Plan(col_scene,start_conf_,goal_conf_,global_results_);
// }


RRTPlanner::~RRTPlanner() = default;

}   // namespace rrt_planner
