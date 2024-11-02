//
// Created by robert on 9/4/22.
//
#include "planner.h"

#include <memory>


MyPlanner::MyPlanner(ros::NodeHandle nh) {
    ROS_INFO("SRRTPlanner Initialize!");
    NUM_INITIATION=1;
    plan_flag = 0;
    is_plan_success = false;
    if (nh.hasParam("robot/DOF"))
        nh.getParam("robot/DOF", dof);
    else
        dof=6;
    start_conf.resize(dof);
    goal_conf.resize(dof);
    arm_pos.resize(dof);
    start_conf.setZero();
    goal_conf.setZero();
    arm_pos.setZero();
    if (nh.hasParam("robot/arm_state_topic"))
        nh.getParam("robot/arm_state_topic", arm_state_topic);
    if (nh.hasParam("robot/arm_joint_names"))
        nh.getParam("robot/arm_joint_names", arm_joint_names);
    arm_state_sub = nh.subscribe(arm_state_topic, 1, &MyPlanner::armStateCallback, this);
    plan_sub = nh.subscribe("move_group/goal", 1, &MyPlanner::planCallback,this);
    dh_alpha.resize(6);
    dh_a.resize(6);
    dh_d.resize(6);
    dh_theta.resize(6);
    dh_alpha<<M_PI_2,0.0,0.0,M_PI_2,-M_PI_2,0.0;
    dh_a<<0.0,-0.427,-0.357,0.0,0.0,0.0;
    dh_d<<0.147,0.0,0.0,0.141,0.116,0.105;
    dh_theta<<0.0,-M_PI_2,0.0,-M_PI_2,0.0,0.0;
    tree=Tree();
    move_group = new Move_Group("arm");
    planning_scene = new Scene();
    robot_model_loader = new Robot_Model_Loader("robot_description");
    kinematic_model = robot_model_loader->getModel();
    kinematic_state=std::make_shared<robot_state::RobotState>(kinematic_model);
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("arm");

    is_adapt_step=0;
    is_adapt_fai=0;

    if (nh.hasParam("planner/step_rate"))
        nh.getParam("planner/step_rate", step_rate);
    else
        step_rate=0.03;
    if (nh.hasParam("planner/fai_rate"))
        nh.getParam("planner/fai_rate", fai_rate);
    else
        fai_rate=1.01;
    if (nh.hasParam("planner/MAX_TREE_NUM"))
        nh.getParam("planner/MAX_TREE_NUM", MAX_TREE_NUM);
    else
        MAX_TREE_NUM=2000;
//    std::cout<<"Input Step Rate:"<<std::endl;
//    std::cin>>step_rate;
//    std::cout<<"Input Fai Rate:"<<std::endl;
//    std::cin>>fai_rate;
//    std::cout<<"Input MAX_TREE_NUM:"<<std::endl;
//    int max_tree_num;
//    std::cin>>max_tree_num;
    step = step_rate * 2 * M_PI;
    fai=fai_rate*step;
    MAX_FAIL_NUM = 5*MAX_TREE_NUM;
//    std::cout<<"MAX_TREE_NUM: "<<MAX_TREE_NUM<<std::endl;
//    std::cout<<"MAX_FAIL_NUM: "<<MAX_FAIL_NUM<<std::endl;

    if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
    }
    else
        std::cout <<"create socket successed " << std::endl;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr= inet_addr("192.168.5.1");
    servaddr.sin_port = htons(6001);
    std::cout<<"Initialize Finished!"<<std::endl;
}

void MyPlanner::armStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    for (int i=0;i<dof;i++)
        arm_pos(i)=msg->position[i];
}

void MyPlanner::planCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg) {
    clock_t start_init,finish_init;
//    std::cout<<"Set Planning Problem"<<std::endl;
//    start_init = clock();
//    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
//    monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
//    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
//    ps->getCurrentStateNonConst().update();
//    planning_scene::PlanningScenePtr col_scene = ps->diff();
//    col_scene->decoupleParent();
//    finish_init = clock();
//    double duration=(double)(finish_init - start_init) * 1000 / CLOCKS_PER_SEC;
//    std::cout<<"Initial Time:"<<duration<<"ms"<<std::endl;
//    for(int i=0;i<dof;i++)
//    {
//        start_conf(i) = arm_pos(i);
//        goal_conf(i) = msg->goal.request.goal_constraints[0].joint_constraints[i].position;
//    }
////    std::cout<<"MyPlan"<<std::endl;
//    VectorXd result=MyPlan(col_scene);
//    std::cout<<"MyPlan Success:"<<result(0)<<std::endl;
//    std::cout<<"MyPlan Time:"<<result(1)<<" ms"<<std::endl;
//    std::cout<<"MyPlan Sample Point Num:"<<result(2)<<std::endl;
//    std::cout<<"MyPlan Path Point Num:"<<result(3)<<std::endl;
//    std::cout<<"MyPlan Plan Fail Nodes:"<<result(4)<<std::endl;
//    std::cout<<"MyPlan Path Points: "<<std::endl;
//    for(int i=0;i<tree.path.size();i++)
//    {
//        std::cout<<(tree.path[i].transpose()*180/M_PI)<<std::endl;
//    }
}

bool MyPlanner::CheckCollision(VectorXd proper_point, VectorXd new_point, const planning_scene::PlanningScenePtr& col_scene, double inter_step) const {
    bool is_collision = false;
    const std::vector<std::string> &joint_all_names = joint_model_group->getJointModelNames();
    std::vector<std::string> joint_names;
    for (const auto & joint_all_name : joint_all_names)
        joint_names.push_back(joint_all_name);
    std::vector<double> joint_positions;
    robot_state::RobotState &current_state = col_scene->getCurrentStateNonConst();
    current_state.copyJointGroupPositions(joint_model_group, joint_positions);
    robot_state::RobotState state(col_scene->getRobotModel());

    VectorXd n_p,p_p;
    n_p.resize(dof);
    p_p.resize(dof);
    for (int i = 0; i < dof; i++)
    {
        p_p(i)=proper_point(i);
        n_p(i)=new_point(i);
    }

    std::vector<double> temp;
    for (int i = 0; i < dof; i++)
        temp.push_back(abs(n_p(i) - p_p(i)) / inter_step);
    int num = floor(*std::max_element(temp.begin(), temp.end()));
    if (num < 1)
        num = 1;

    VectorXd delta;
    delta = (n_p - p_p) / num;
    for (int i = 0; i < num + 1; i++) {
        VectorXd temp_vec;
//            temp_vec = p_p + i * delta;
        temp_vec = n_p - i * delta;
        for (int j = 0; j < temp_vec.size(); j++)
            joint_positions[j] = temp_vec(j);
        state.setVariablePositions(joint_names, joint_positions);
//            scene->setCurrentState(state);
        current_state.setJointGroupPositions(joint_model_group, joint_positions);
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

void MyPlanner::execute() {
    if(plan_flag==1)
    {
        if(is_plan_success)
        {
            int path_node_num = tree.path.size();
            double total_time = 0.5 * path_node_num;
//            int inter_num = (int)(3 * (path_node_num/20.0));
            int inter_num = 50;
            int exec_step = path_node_num + (path_node_num - 1) * inter_num;
            std::vector<VectorXd> exec_values;
            exec_values= interplote(tree.path,inter_num,dof);
//            std::cout<<"exec_values:"<<std::endl;
//            for(int i=0;i<exec_values.size();i++)
//                std::cout<<exec_values[i].transpose()*180/M_PI<<std::endl;
//            std::cout<<"path: \n"<<std::endl;
//            for(auto & i : tree.path)
//                std::cout<<i.transpose()<<std::endl;
//            std::cout<<"inter_points: \n"<<std::endl;
//            for(auto & i : exec_values)
//                std::cout<<i.transpose()<<std::endl;

//            exec_values.resize(exec_step);
//            for (int i = 0; i < path_node_num - 1; i++) {
//                VectorXd delta = (tree.path[i + 1] - tree.path[i]) / (inter_num + 1);
//                for (int j = 0; j < inter_num + 1; j++) {
//                    int idx = i + inter_num * i + j;
//                    exec_values[idx] = tree.path[i] + delta * j;
//                }
//            }
//            exec_values[exec_step - 1] = tree.path[path_node_num - 1];

            moveit_msgs::RobotTrajectory plan_traj = moveit_msgs::RobotTrajectory();
            plan_traj.joint_trajectory.header.frame_id = move_group->getPlanningFrame();
            plan_traj.joint_trajectory.joint_names = move_group->getActiveJoints();
            plan_traj.joint_trajectory.points.resize(exec_step);
            for (int i = 0; i < exec_step; i++) {
                plan_traj.joint_trajectory.points[i].positions.resize(6);
                ros::Duration time_from_start;
                time_from_start = ros::Duration(total_time * i / exec_step);
                for (int j = 0; j < dof; j++)
                    plan_traj.joint_trajectory.points[i].positions[j] = exec_values[i](j);
                std::cout<<exec_values[i].transpose()<<std::endl;
                plan_traj.joint_trajectory.points[i].time_from_start = time_from_start;
            }
            move_group->execute(plan_traj);
            move_group->clearPoseTargets();
            move_group->setStartState(*move_group->getCurrentState());
            move_group->stop();
//            ROS_INFO("You Can Start Planning!");
        }
        plan_flag = 0;
    }
}

void MyPlanner::execute_real() {
    if(plan_flag==1)
    {
        if(is_plan_success)
        {
            int path_node_num = tree.path.size();
            double total_time = 5.0 * (path_node_num/20.0);
            int inter_num = (int)(2 * (path_node_num/20.0));
            int exec_step = path_node_num + (path_node_num - 1) * inter_num;
            std::vector<VectorXd> exec_values;
            exec_values.resize(exec_step);
            for (int i = 0; i < path_node_num - 1; i++) {
                VectorXd delta = (tree.path[i + 1] - tree.path[i]) / (inter_num + 1);
                for (int j = 0; j < inter_num + 1; j++) {
                    int idx = i + inter_num * i + j;
                    exec_values[idx] = tree.path[i] + delta * j;
                }
            }
            exec_values[exec_step - 1] = tree.path[path_node_num - 1];
            char buf_send[BUFSIZ],buf_recv[BUFSIZ];
            int buf_len;
            strcpy(buf_send,"Move");
//            std::cout<<exec_step<<std::endl;
            buf_len=send(sockfd,buf_send,strlen(buf_send),0);
            buf_len=recv(sockfd,buf_recv,BUFSIZ,0);
            std::cout<<buf_recv<<std::endl;
            if(buf_recv[0]=='S')
            {
                for (int i = 0; i < exec_step; i++) {
                    std::string temp;
                    temp=std::to_string(exec_values[i](0)*180/M_PI)+","
                            +std::to_string(exec_values[i](1)*180/M_PI)+","
                            +std::to_string(exec_values[i](2)*180/M_PI)+","
                            +std::to_string(exec_values[i](3)*180/M_PI)+","
                            +std::to_string(exec_values[i](4)*180/M_PI)+","
                            +std::to_string(exec_values[i](5)*180/M_PI);
                    strcpy(buf_send,temp.c_str());
//                    std::cout<<buf_send<<std::endl;
                    buf_len=send(sockfd,buf_send,strlen(buf_send),0);
                    buf_len=recv(sockfd,buf_recv,BUFSIZ,0);
                    for(int j=0;j<1000;j++);
//                    sleep(1);
//                    std::cout<<"i: "<<i<<std::endl;
                }
                strcpy(buf_send,"MoveF");
                //            std::cout<<exec_step<<std::endl;
                buf_len=send(sockfd,buf_send,strlen(buf_send),0);
            }
            std::cout<<"Finish Sending"<<std::endl;
        }
        plan_flag = 0;
    }
}

VectorXd MyPlanner::MyPlan(const planning_scene::PlanningScenePtr& col_scene) {
    step_rate=0.02;
//    step_rate=0.01;
    fai_rate=1.01;
    step=step_rate*2*M_PI;
    fai=fai_rate*step;
    Tree tree1,tree2;
    tree.renew();
    tree.set_dof(dof);
    tree1.renew();
    tree1.set_dof(dof);
    tree2.renew();
    tree2.set_dof(dof);
    VectorXd start_point,goal_point;
    VectorXd current_point1,current_point2,sample_point,proper_point1,proper_point2;
    int idx1=1;
    int idx2=1;

    sample_point.resize(dof+2);
    start_point.resize(dof+2);
    goal_point.resize(dof+2);
    current_point1.resize(dof+2);
    proper_point1.resize(dof+2);
    current_point2.resize(dof+2);
    proper_point2.resize(dof+2);

    for(int i=0;i<dof;i++) {
        start_point(i) = start_conf(i);
        goal_point(i) = goal_conf(i);
    }
    start_point(dof)=idx1;
    start_point(dof+1)=-1;
    goal_point(dof)=idx2;
    goal_point(dof+1)=-1;

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
//    double a=1;
//    double sig_x = -5.0;
//    double sig_delta_x = 0.2;
//    double sig_y = bias + a/ (1 + exp(-1*sig_x));


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
        std::cout<<fai_rate<<std::endl;
    is_plan_success = true;
    clock_t start,finish;
    start = clock();
    while (tree.cal_dist(current_point1,current_point2) >= step ||
           CheckCollision(current_point1, current_point2, col_scene, check_inter_step))
    {
//        std::cout<<sig_y<<std::endl;
//        std::cout<<fai_rate<<std::endl;
        if((swap%2)==0)
        {
            sample_point = tree1.generate_random_point(goal_point);
            proper_point1 = tree1.find_nearest_point(sample_point);
            if (!tree1.is_same(sample_point, proper_point1)) {
                VectorXd new_point1;
                new_point1.resize(sample_point.size());
                double dist1 = tree1.cal_dist(sample_point,proper_point1);
                double dist2 = tree1.cal_dist(goal_point,proper_point1);
                new_point1=proper_point1+step/dist1*(sample_point-proper_point1)+fai/dist2*(goal_point-proper_point1);
                bool is_collision = CheckCollision(proper_point1, new_point1, col_scene, check_inter_step);
                if (!is_collision) {
//                    fai_rate=origin_fai_rate;
                    if(is_adapt_fai==1)
                    {
                        fai_rate+=delta_fai_add;
                        if(fai_rate>fai_max)
                            fai_rate=fai_max;
                        fai=fai_rate*step;
                        if(is_print_fai==1)
                            std::cout<<fai_rate<<std::endl;
                    }
                    else
                    {
                        temp_success+=1;
                        if(temp_success>1)
                            fai = fai_rate * step;
                    }

                    idx1 += 1;
                    new_point1(dof) = idx1;
                    new_point1(dof + 1) = proper_point1(dof);
                    tree1.add_point(new_point1);
                    current_point1 = new_point1;
                    proper_point2 = tree2.find_nearest_point(new_point1);
                    double dist = tree2.cal_dist(new_point1, proper_point2);
                    if (dist < step) {
                        if (!CheckCollision(proper_point2, new_point1, col_scene, check_inter_step))
                            pathFound = 1;
                    }
                    else {
                        if(is_adapt_step==1)
                        {
                            sig_x += sig_delta_x;
                            sig_y = bias + a/ (1 + exp(-c*sig_x));
                            step = step_rate*2*M_PI*sig_y;
                            if(is_print_step==1)
                                std::cout<<sig_y<<std::endl;
                        }
                        else
                            step=step*1.0;
                        VectorXd new_point2, new_point3;
                        new_point2.resize(sample_point.size());
                        new_point3.resize(sample_point.size());
                        new_point2 = proper_point2 + step / dist * (new_point1 - proper_point2);
                        is_collision = CheckCollision(proper_point2, new_point2, col_scene, check_inter_step);
                        if (!is_collision) {
                            idx2 += 1;
                            new_point2(dof) = idx2;
                            new_point2(dof + 1) = proper_point2(dof);
                            tree2.add_point(new_point2);
                            current_point2 = new_point2;
                            if(is_adapt_step==1)
                            {
                                sig_x += sig_delta_x;
                                sig_y = bias + a/ (1 + exp(-c*sig_x));
                                step = step_rate*2*M_PI*sig_y;
                                if(is_print_step==1)
                                    std::cout<<sig_y<<std::endl;
                            }
                            else
                                step=step*1.0;
                            while (tree2.cal_dist(new_point2, new_point1) > step) {
                                dist = tree2.cal_dist(new_point2, new_point1);
                                new_point3 = new_point2 + step / dist * (new_point1 - new_point2);
                                is_collision = CheckCollision(new_point3, new_point2, col_scene, check_inter_step);
                                if (!is_collision) {
                                    idx2 += 1;
                                    new_point3(dof) = idx2;
                                    new_point3(dof + 1) = idx2 - 1;
                                    tree2.add_point(new_point3);
                                    current_point2 = new_point3;
                                    new_point2 = new_point3;
                                    if(is_adapt_step==1)
                                    {
                                        sig_x += sig_delta_x;
                                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                                        step = step_rate*2*M_PI*sig_y;
                                        if(is_print_step==1)
                                            std::cout<<sig_y<<std::endl;
                                    }
                                    else
                                        step=step*1.0;
                                }
                                else{
                                    if(is_adapt_step==1)
                                    {
                                        sig_x -= sig_delta_x;
                                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                                        step = step_rate*2*M_PI*sig_y;
                                        if(is_print_step==1)
                                            std::cout<<sig_y<<std::endl;
                                    }
                                    else
                                        step=step_rate*2*M_PI;
                                    fail_num+=1;
                                    break;
                                }
                            }
                        }
                        else {
                            if(is_adapt_step==1)
                            {
                                sig_x -= sig_delta_x;
                                sig_y = bias + a/ (1 + exp(-c*sig_x));
                                step = step_rate*2*M_PI*sig_y;
                                if(is_print_step==1)
                                    std::cout<<sig_y<<std::endl;
                            }
                            else
                                step=step_rate*2*M_PI;
                            fail_num += 1;
                        }
                    }
                }
                else {
                    if(is_adapt_step==1)
                    {
                        sig_x -= sig_delta_x;
                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                        step = step_rate*2*M_PI*sig_y;
                        if(is_print_step==1)
                            std::cout<<sig_y<<std::endl;
                    }
                    else
                        step=step_rate*2*M_PI;
                    if(is_adapt_fai==1)
                    {
                        fai_rate-=delta_fai_minus;
                        if(fai_rate<fai_min)
                            fai_rate=fai_min;
                        fai=fai_rate*step;
                        if(is_print_fai==1)
                            std::cout<<fai_rate<<std::endl;
                    }
                    else
                        fai=1.01*step;
                    fail_num += 1;
                    temp_success=0;
                }
            }
        }
        else if((swap%2)==1)
        {
            sample_point = tree2.generate_random_point(start_point);
            proper_point2 = tree2.find_nearest_point(sample_point);
            if (!tree2.is_same(sample_point, proper_point2)) {
                VectorXd new_point2;
                new_point2.resize(sample_point.size());
                double dist1 = tree2.cal_dist(sample_point,proper_point2);
                double dist2 = tree2.cal_dist(start_point,proper_point2);
                new_point2=proper_point2+step/dist1*(sample_point-proper_point2)+fai/dist2*(start_point-proper_point2);
                bool is_collision = CheckCollision(proper_point2, new_point2, col_scene, check_inter_step);
                if (!is_collision) {
//                    fai_rate=origin_fai_rate;
                    if(is_adapt_fai==1)
                    {
                        fai_rate+=delta_fai_add;
                        if(fai_rate>fai_max)
                            fai_rate=fai_max;
                        fai=fai_rate*step;
                        if(is_print_fai==1)
                            std::cout<<fai_rate<<std::endl;
                    }
                    else
                    {
                        temp_success+=1;
                        if(temp_success>1)
                            fai = fai_rate * step;
                    }
                    idx2 += 1;
                    new_point2(dof) = idx2;
                    new_point2(dof + 1) = proper_point2(dof);
                    tree2.add_point(new_point2);
                    current_point2 = new_point2;
                    proper_point1 = tree1.find_nearest_point(new_point2);
                    double dist = tree1.cal_dist(new_point2, proper_point1);
                    if (dist < step) {
                        if (!CheckCollision(proper_point1, new_point2, col_scene, check_inter_step))
                            pathFound = 1;
                    }
                    else {
                        if(is_adapt_step==1)
                        {
                            sig_x += sig_delta_x;
                            sig_y = bias + a/ (1 + exp(-c*sig_x));
                            step = step_rate*2*M_PI*sig_y;
                            if(is_print_step==1)
                                std::cout<<sig_y<<std::endl;
                        }
                        else
                            step=step*1.0;
                        VectorXd new_point1, new_point3;
                        new_point1.resize(sample_point.size());
                        new_point3.resize(sample_point.size());
                        new_point1 = proper_point1 + step / dist * (new_point2 - proper_point1);
                        is_collision = CheckCollision(proper_point1, new_point1, col_scene, check_inter_step);
                        if (!is_collision) {
                            idx1 += 1;
                            new_point1(dof) = idx1;
                            new_point1(dof + 1) = proper_point1(dof);
                            tree1.add_point(new_point1);
                            current_point1 = new_point1;
                            if(is_adapt_step==1)
                            {
                                sig_x += sig_delta_x;
                                sig_y = bias + a/ (1 + exp(-c*sig_x));
                                step = step_rate*2*M_PI*sig_y;
                                if(is_print_step==1)
                                    std::cout<<sig_y<<std::endl;
                            }
                            else
                                step=step*1.0;
                            while (tree1.cal_dist(new_point1, new_point2) > step) {
                                dist = tree1.cal_dist(new_point2, new_point1);
                                new_point3 = new_point1 + step / dist * (new_point2 - new_point1);
                                is_collision = CheckCollision(new_point3, new_point1, col_scene, check_inter_step);
                                if (!is_collision) {
                                    idx1 += 1;
                                    new_point3(dof) = idx1;
                                    new_point3(dof + 1) = idx1 - 1;
                                    tree1.add_point(new_point3);
                                    current_point1 = new_point3;
                                    new_point1 = new_point3;
                                    if(is_adapt_step==1)
                                    {
                                        sig_x += sig_delta_x;
                                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                                        step = step_rate*2*M_PI*sig_y;
                                        if(is_print_step==1)
                                            std::cout<<sig_y<<std::endl;
                                    }
                                    else
                                        step=step*1.0;
                                }
                                else{
                                    if(is_adapt_step==1)
                                    {
                                        sig_x -= sig_delta_x;
                                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                                        step = step_rate*2*M_PI*sig_y;
                                        if(is_print_step==1)
                                            std::cout<<sig_y<<std::endl;
                                    }
                                    else
                                        step=step_rate*2*M_PI;
                                    fail_num+=1;
                                    break;
                                }
                            }
                        }
                        else {
                            if(is_adapt_step==1)
                            {
                                sig_x -= sig_delta_x;
                                sig_y = bias + a/ (1 + exp(-c*sig_x));
                                step = step_rate*2*M_PI*sig_y;
                                if(is_print_step==1)
                                    std::cout<<sig_y<<std::endl;
                            }
                            else
                                step=step_rate*2*M_PI;
                            fail_num += 1;
                        }
                    }
                }
                else {
                    if(is_adapt_step==1)
                    {
                        sig_x -= sig_delta_x;
                        sig_y = bias + a/ (1 + exp(-c*sig_x));
                        step = step_rate*2*M_PI*sig_y;
                        if(is_print_step==1)
                            std::cout<<sig_y<<std::endl;
                    }
                    else
                        step=step_rate*2*M_PI;
                    if(is_adapt_fai==1)
                    {
                        fai_rate-=delta_fai_minus;
                        if(fai_rate<fai_min)
                            fai_rate=fai_min;
                        fai=fai_rate*step;
                        if(is_print_fai==1)
                            std::cout<<fai_rate<<std::endl;
                    }
                    else
                        fai=1.01*step;
                    fail_num += 1;
                    temp_success=0;
                }
            }

        }
        swap+=1;
        if(tree1.points.size()>=MAX_TREE_NUM||tree2.points.size()>=MAX_TREE_NUM)
        {
            std::cout<<"MyPlan Failed Type 1"<<std::endl;
            tree1.renew();
            tree1.set_dof(dof);
            tree2.renew();
            tree2.set_dof(dof);
            current_point1 = start_point;
            tree1.add_point(start_point);
            current_point2 = goal_point;
            tree2.add_point(goal_point);
            num_initiation +=1;
            idx1=1;
            idx2=1;
            is_plan_success=false;
            break;
        }
        if(fail_num>=MAX_FAIL_NUM)
        {
            std::cout<<"MyPlan Failed Type 2"<<std::endl;
            tree1.renew();
            tree1.set_dof(dof);
            tree2.renew();
            tree2.set_dof(dof);
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
        if(num_initiation>=NUM_INITIATION)
        {
            is_plan_success = false;
            break;
        }

    }
    if(is_plan_success==1) {
        tree1.find_path(start_point, tree1.points[idx1 - 1]);
        tree2.find_path(goal_point, tree2.points[idx2 - 1]);
        std::reverse(tree2.path.begin(), tree2.path.end());
        tree.points.insert(tree.points.end(), tree1.points.begin(), tree1.points.end());
        tree.points.insert(tree.points.end(), tree2.points.begin(), tree2.points.end());
        tree.path.insert(tree.path.end(), tree1.path.begin(), tree1.path.end());
        tree.path.insert(tree.path.end(), tree2.path.begin(), tree2.path.end());
        clock_t prue_s, prue_f;
        prue_s = clock();
        std::vector<VectorXd> prue_path = Pruning_path(col_scene);
        prue_f = clock();
//        tree.path = prue_path;
        double path_length = tree.calculate_path_length();
        finish = clock();
        double prue_duration = (double) (prue_f - prue_s) * 1000 / CLOCKS_PER_SEC;
        double duration = (double) (finish - start) * 1000 / CLOCKS_PER_SEC;
        double plan_duration = duration - prue_duration;
        plan_flag = 1;
        VectorXd result;
        result.resize(6);
        result(0) = is_plan_success;
        result(1) = plan_duration;
        result(2) = tree.points.size();
        result(3) = tree.path.size();
        result(4) = fail_num;
        result(5) = path_length;
        return result;
    }
    else
    {
        VectorXd result;
        result.resize(6);
        result.setZero();
        return result;
    }
}

VectorXd MyPlanner::MyPlan_RRTConnect(const planning_scene::PlanningScenePtr& col_scene) {
    step_rate=0.02;
    step=step_rate*2*M_PI;
    Tree tree1,tree2;
    tree.renew();
    tree.set_dof(dof);
    tree1.renew();
    tree1.set_dof(dof);
    tree2.renew();
    tree2.set_dof(dof);
    VectorXd start_point,goal_point;
    VectorXd current_point1,current_point2,sample_point,proper_point1,proper_point2;
    int idx1=1;
    int idx2=1;
    sample_point.resize(dof+2);
    start_point.resize(dof+2);
    goal_point.resize(dof+2);
    current_point1.resize(dof+2);
    proper_point1.resize(dof+2);
    current_point2.resize(dof+2);
    proper_point2.resize(dof+2);
    for(int i=0;i<dof;i++) {
        start_point(i) = start_conf(i);
        goal_point(i) = goal_conf(i);
    }
    start_point(dof)=idx1;
    start_point(dof+1)=-1;
    goal_point(dof)=idx2;
    goal_point(dof+1)=-1;
    current_point1 = start_point;
    current_point2 = goal_point;
    tree1.add_point(start_point);
    tree2.add_point(goal_point);

    int pathFound=0;
    int num_initiation =0;
    int fail_num = 0;
    int swap = 0;
    double check_inter_step=M_PI/45;
    is_plan_success = true;
    clock_t start,finish;
    start = clock();
    while (tree.cal_dist(current_point1,current_point2) >= step ||
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
                new_point1=proper_point1+step/dist1*(sample_point-proper_point1);
                bool is_collision = CheckCollision(proper_point1, new_point1, col_scene, check_inter_step);
                if (!is_collision) {
                    idx1 += 1;
                    new_point1(dof) = idx1;
                    new_point1(dof + 1) = proper_point1(dof);
                    tree1.add_point(new_point1);
                    current_point1 = new_point1;
                    proper_point2 = tree2.find_nearest_point(new_point1);
                    double dist = tree2.cal_dist(new_point1, proper_point2);
                    if (dist < step) {
                        if (!CheckCollision(proper_point2, new_point1, col_scene, check_inter_step))
                            pathFound = 1;
                    }
                    else {
                        VectorXd new_point2, new_point3;
                        new_point2.resize(sample_point.size());
                        new_point3.resize(sample_point.size());
                        new_point2 = proper_point2 + step / dist * (new_point1 - proper_point2);
                        is_collision = CheckCollision(proper_point2, new_point2, col_scene, check_inter_step);
                        if (!is_collision) {
                            idx2 += 1;
                            new_point2(dof) = idx2;
                            new_point2(dof + 1) = proper_point2(dof);
                            tree2.add_point(new_point2);
                            current_point2 = new_point2;
                            while (tree2.cal_dist(new_point2, new_point1) > step) {
                                dist = tree2.cal_dist(new_point2, new_point1);
                                new_point3 = new_point2 + step / dist * (new_point1 - new_point2);
                                is_collision = CheckCollision(new_point3, new_point2, col_scene, check_inter_step);
                                if (!is_collision) {
                                    idx2 += 1;
                                    new_point3(dof) = idx2;
                                    new_point3(dof + 1) = idx2 - 1;
                                    tree2.add_point(new_point3);
                                    current_point2 = new_point3;
                                    new_point2 = new_point3;
                                }
                                else{
                                    fail_num+=1;
                                    break;
                                }
                            }
                        }
                        else {
                            fail_num += 1;
                        }
                    }
                }
                else {
                    fail_num += 1;
                }
            }
        }
        else if((swap%2)==1)
        {
            sample_point = tree2.generate_random_point(start_point);
            proper_point2 = tree2.find_nearest_point(sample_point);
            if (!tree2.is_same(sample_point, proper_point2)) {
                VectorXd new_point2;
                new_point2.resize(sample_point.size());
                double dist1 = tree2.cal_dist(sample_point,proper_point2);
                new_point2=proper_point2+step/dist1*(sample_point-proper_point2);
                bool is_collision = CheckCollision(proper_point2, new_point2, col_scene, check_inter_step);
                if (!is_collision) {
                    idx2 += 1;
                    new_point2(dof) = idx2;
                    new_point2(dof + 1) = proper_point2(dof);
                    tree2.add_point(new_point2);
                    current_point2 = new_point2;
                    proper_point1 = tree1.find_nearest_point(new_point2);
                    double dist = tree1.cal_dist(new_point2, proper_point1);
                    if (dist < step) {
                        if (!CheckCollision(proper_point1, new_point2, col_scene, check_inter_step))
                            pathFound = 1;
                    }
                    else {
                        VectorXd new_point1, new_point3;
                        new_point1.resize(sample_point.size());
                        new_point3.resize(sample_point.size());
                        new_point1 = proper_point1 + step / dist * (new_point2 - proper_point1);
                        is_collision = CheckCollision(proper_point1, new_point1, col_scene, check_inter_step);
                        if (!is_collision) {
                            idx1 += 1;
                            new_point1(dof) = idx1;
                            new_point1(dof + 1) = proper_point1(dof);
                            tree1.add_point(new_point1);
                            current_point1 = new_point1;
                            while (tree1.cal_dist(new_point1, new_point2) > step) {
                                dist = tree1.cal_dist(new_point2, new_point1);
                                new_point3 = new_point1 + step / dist * (new_point2 - new_point1);
                                is_collision = CheckCollision(new_point3, new_point1, col_scene, check_inter_step);
                                if (!is_collision) {
                                    idx1 += 1;
                                    new_point3(dof) = idx1;
                                    new_point3(dof + 1) = idx1 - 1;
                                    tree1.add_point(new_point3);
                                    current_point1 = new_point3;
                                    new_point1 = new_point3;
                                }
                                else{
                                    fail_num+=1;
                                    break;
                                }
                            }
                        }
                        else {
                            fail_num += 1;
                        }
                    }
                }
                else {
                    fail_num += 1;
                }
            }

        }
        swap+=1;
        if(tree1.points.size()>=MAX_TREE_NUM||tree2.points.size()>=MAX_TREE_NUM)
        {
            std::cout<<"MyPlan Failed Type 1"<<std::endl;
            tree1.renew();
            tree1.set_dof(dof);
            tree2.renew();
            tree2.set_dof(dof);
            current_point1 = start_point;
            tree1.add_point(start_point);
            current_point2 = goal_point;
            tree2.add_point(goal_point);
            num_initiation +=1;
            idx1=1;
            idx2=1;
            is_plan_success=false;
            break;
        }
        if(fail_num>=MAX_FAIL_NUM)
        {
            std::cout<<"MyPlan Failed Type 2"<<std::endl;
            tree1.renew();
            tree1.set_dof(dof);
            tree2.renew();
            tree2.set_dof(dof);
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
        if(num_initiation>=NUM_INITIATION)
        {
            is_plan_success = false;
            break;
        }

    }
    if(is_plan_success==1) {
        tree1.find_path(start_point, tree1.points[idx1 - 1]);
        tree2.find_path(goal_point, tree2.points[idx2 - 1]);
        std::reverse(tree2.path.begin(), tree2.path.end());
        tree.points.insert(tree.points.end(), tree1.points.begin(), tree1.points.end());
        tree.points.insert(tree.points.end(), tree2.points.begin(), tree2.points.end());
        tree.path.insert(tree.path.end(), tree1.path.begin(), tree1.path.end());
        tree.path.insert(tree.path.end(), tree2.path.begin(), tree2.path.end());
        clock_t prue_s, prue_f;
        prue_s = clock();
        std::vector<VectorXd> prue_path = Pruning_path(col_scene);
        prue_f = clock();
//        tree.path = prue_path;
        double path_length = tree.calculate_path_length();
        finish = clock();
        double prue_duration = (double) (prue_f - prue_s) * 1000 / CLOCKS_PER_SEC;
        double duration = (double) (finish - start) * 1000 / CLOCKS_PER_SEC;
        double plan_duration = duration - prue_duration;
        plan_flag = 1;
        VectorXd result;
        result.resize(6);
        result(0) = is_plan_success;
        result(1) = plan_duration;
        result(2) = tree.points.size();
        result(3) = tree.path.size();
        result(4) = fail_num;
        result(5) = path_length;
        return result;
    }
    else
    {
        VectorXd result;
        result.resize(6);
        result.setZero();
        return result;
    }

}

VectorXd MyPlanner::MyPlan_BG(const planning_scene::PlanningScenePtr& col_scene) {
    step_rate=0.02;
    fai_rate=1.01;
    step=step_rate*2*M_PI;
    fai=fai_rate*step;
    Tree tree1,tree2;
    tree.renew();
    tree.set_dof(dof);
    tree1.renew();
    tree1.set_dof(dof);
    tree2.renew();
    tree2.set_dof(dof);
    VectorXd start_point,goal_point;
    VectorXd current_point1,current_point2,sample_point,proper_point1,proper_point2;
    int idx1=1;
    int idx2=1;

    sample_point.resize(dof+2);
    start_point.resize(dof+2);
    goal_point.resize(dof+2);
    current_point1.resize(dof+2);
    proper_point1.resize(dof+2);
    current_point2.resize(dof+2);
    proper_point2.resize(dof+2);

    for(int i=0;i<dof;i++) {
        start_point(i) = start_conf(i);
        goal_point(i) = goal_conf(i);
    }
    start_point(dof)=idx1;
    start_point(dof+1)=-1;
    goal_point(dof)=idx2;
    goal_point(dof+1)=-1;

    current_point1 = start_point;
    current_point2 = goal_point;
    tree1.add_point(start_point);
    tree2.add_point(goal_point);

    int pathFound=0;
    int num_initiation =0;
    int fail_num = 0;
    int swap = 0;
    double check_inter_step=M_PI/45;
    is_plan_success = true;
    clock_t start,finish;
    start = clock();
    while (tree.cal_dist(current_point1,current_point2) >= step ||
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
                new_point1=proper_point1+step/dist1*(sample_point-proper_point1)+fai/dist2*(goal_point-proper_point1);
                bool is_collision = CheckCollision(proper_point1, new_point1, col_scene, check_inter_step);
                if (!is_collision) {
                    idx1 += 1;
                    new_point1(dof) = idx1;
                    new_point1(dof + 1) = proper_point1(dof);
                    tree1.add_point(new_point1);
                    current_point1 = new_point1;
                    proper_point2 = tree2.find_nearest_point(new_point1);
                    double dist = tree2.cal_dist(new_point1, proper_point2);
                    if (dist < step) {
                        if (!CheckCollision(proper_point2, new_point1, col_scene, check_inter_step))
                            pathFound = 1;
                    }
                    else {
                        VectorXd new_point2, new_point3;
                        new_point2.resize(sample_point.size());
                        new_point3.resize(sample_point.size());
                        new_point2 = proper_point2 + step / dist * (new_point1 - proper_point2);
                        is_collision = CheckCollision(proper_point2, new_point2, col_scene, check_inter_step);
                        if (!is_collision) {
                            idx2 += 1;
                            new_point2(dof) = idx2;
                            new_point2(dof + 1) = proper_point2(dof);
                            tree2.add_point(new_point2);
                            current_point2 = new_point2;
                            while (tree2.cal_dist(new_point2, new_point1) > step) {
                                dist = tree2.cal_dist(new_point2, new_point1);
                                new_point3 = new_point2 + step / dist * (new_point1 - new_point2);
                                is_collision = CheckCollision(new_point3, new_point2, col_scene, check_inter_step);
                                if (!is_collision) {
                                    idx2 += 1;
                                    new_point3(dof) = idx2;
                                    new_point3(dof + 1) = idx2 - 1;
                                    tree2.add_point(new_point3);
                                    current_point2 = new_point3;
                                    new_point2 = new_point3;
                                }
                                else{
                                    fail_num+=1;
                                    break;
                                }
                            }
                        }
                        else {
                            fail_num += 1;
                        }
                    }
                }
                else {
                    fail_num += 1;
                }
            }
        }
        else if((swap%2)==1)
        {
            sample_point = tree2.generate_random_point(start_point);
            proper_point2 = tree2.find_nearest_point(sample_point);
            if (!tree2.is_same(sample_point, proper_point2)) {
                VectorXd new_point2;
                new_point2.resize(sample_point.size());
                double dist1 = tree2.cal_dist(sample_point,proper_point2);
                double dist2 = tree2.cal_dist(start_point,proper_point2);
                new_point2=proper_point2+step/dist1*(sample_point-proper_point2)+fai/dist2*(start_point-proper_point2);
                bool is_collision = CheckCollision(proper_point2, new_point2, col_scene, check_inter_step);
                if (!is_collision) {
                    idx2 += 1;
                    new_point2(dof) = idx2;
                    new_point2(dof + 1) = proper_point2(dof);
                    tree2.add_point(new_point2);
                    current_point2 = new_point2;
                    proper_point1 = tree1.find_nearest_point(new_point2);
                    double dist = tree1.cal_dist(new_point2, proper_point1);
                    if (dist < step) {
                        if (!CheckCollision(proper_point1, new_point2, col_scene, check_inter_step))
                            pathFound = 1;
                    }
                    else {
                        VectorXd new_point1, new_point3;
                        new_point1.resize(sample_point.size());
                        new_point3.resize(sample_point.size());
                        new_point1 = proper_point1 + step / dist * (new_point2 - proper_point1);
                        is_collision = CheckCollision(proper_point1, new_point1, col_scene, check_inter_step);
                        if (!is_collision) {
                            idx1 += 1;
                            new_point1(dof) = idx1;
                            new_point1(dof + 1) = proper_point1(dof);
                            tree1.add_point(new_point1);
                            current_point1 = new_point1;
                            while (tree1.cal_dist(new_point1, new_point2) > step) {
                                dist = tree1.cal_dist(new_point2, new_point1);
                                new_point3 = new_point1 + step / dist * (new_point2 - new_point1);
                                is_collision = CheckCollision(new_point3, new_point1, col_scene, check_inter_step);
                                if (!is_collision) {
                                    idx1 += 1;
                                    new_point3(dof) = idx1;
                                    new_point3(dof + 1) = idx1 - 1;
                                    tree1.add_point(new_point3);
                                    current_point1 = new_point3;
                                    new_point1 = new_point3;
                                }
                                else{
                                    fail_num+=1;
                                    break;
                                }
                            }
                        }
                        else {
                            fail_num += 1;
                        }
                    }
                }
                else {
                    fail_num += 1;
                }
            }

        }
        swap+=1;
        if(tree1.points.size()>=MAX_TREE_NUM||tree2.points.size()>=MAX_TREE_NUM)
        {
            std::cout<<"MyPlan Failed Type 1"<<std::endl;
            tree1.renew();
            tree1.set_dof(dof);
            tree2.renew();
            tree2.set_dof(dof);
            current_point1 = start_point;
            tree1.add_point(start_point);
            current_point2 = goal_point;
            tree2.add_point(goal_point);
            num_initiation +=1;
            idx1=1;
            idx2=1;
            is_plan_success=false;
            break;
        }
        if(fail_num>=MAX_FAIL_NUM)
        {
            std::cout<<"MyPlan Failed Type 2"<<std::endl;
            tree1.renew();
            tree1.set_dof(dof);
            tree2.renew();
            tree2.set_dof(dof);
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
        if(num_initiation>=NUM_INITIATION)
        {
            is_plan_success = false;
            break;
        }

    }
    if(is_plan_success==1) {
        tree1.find_path(start_point, tree1.points[idx1 - 1]);
        tree2.find_path(goal_point, tree2.points[idx2 - 1]);
        std::reverse(tree2.path.begin(), tree2.path.end());
        tree.points.insert(tree.points.end(), tree1.points.begin(), tree1.points.end());
        tree.points.insert(tree.points.end(), tree2.points.begin(), tree2.points.end());
        tree.path.insert(tree.path.end(), tree1.path.begin(), tree1.path.end());
        tree.path.insert(tree.path.end(), tree2.path.begin(), tree2.path.end());
        clock_t prue_s, prue_f;
        prue_s = clock();
        std::vector<VectorXd> prue_path = Pruning_path(col_scene);
        prue_f = clock();
//        tree.path = prue_path;
        double path_length = tree.calculate_path_length();
        finish = clock();
        double prue_duration = (double) (prue_f - prue_s) * 1000 / CLOCKS_PER_SEC;
        double duration = (double) (finish - start) * 1000 / CLOCKS_PER_SEC;
        double plan_duration = duration - prue_duration;
        plan_flag = 1;
        VectorXd result;
        result.resize(6);
        result(0) = is_plan_success;
        result(1) = plan_duration;
        result(2) = tree.points.size();
        result(3) = tree.path.size();
        result(4) = fail_num;
        result(5) = path_length;
        return result;
    }
    else
    {
        VectorXd result;
        result.resize(6);
        result.setZero();
        return result;
    }
}

std::vector<VectorXd> MyPlanner::Pruning_path(const planning_scene::PlanningScenePtr& col_scene) {
    double path_cost1=0;
    double path_cost2=0;
    int path_nodes_num=tree.path.size();
    std::vector<VectorXd> result;
    result.push_back(tree.path[0]);
    int start_idx,current_idx;
    double check_inter_step=M_PI/45;
    start_idx=0;
    while(tree.is_same(result[result.size()-1],tree.path[tree.path.size()-1])==0)
    {
        VectorXd start_point;
        start_point =tree.path[start_idx];
        current_idx=start_idx+1;
        for(int i=current_idx;i<tree.path.size();i++)
        {
            VectorXd temp_point=tree.path[i];
            bool is_collision= CheckCollision(start_point,temp_point,col_scene, check_inter_step);
            if(is_collision==0) {
                current_idx += 1;
                if(current_idx-start_idx>path_nodes_num/4)
                {
                    result.push_back(tree.path[current_idx-1]);
                    start_idx=current_idx-1;
                    break;
                }
            }
            else
            {
                result.push_back(tree.path[current_idx-1]);
                start_idx=current_idx-1;
                break;
            }
            if(current_idx==tree.path.size())
            {
                result.push_back(tree.path[current_idx-1]);
                break;
            }
        }
    }
    for(int i=1;i<result.size();i++)
        path_cost1+=tree.cal_dist(result[i], result[i-1]);
    std::vector<VectorXd> rev_path=tree.path;
    std::reverse(rev_path.begin(), rev_path.end());
    std::vector<VectorXd> result2;
    result2.push_back(rev_path[0]);
    int start_idx2,current_idx2;
    start_idx2=0;
    while(tree.is_same(result2[result2.size()-1],rev_path[rev_path.size()-1])==0)
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
        path_cost2+=tree.cal_dist(result2[i], result2[i-1]);
    std::vector<VectorXd> final_result;
    if(path_cost1<=path_cost2)
        final_result=result;
    else
        final_result=result2;
    return final_result;
}

MyPlanner::~MyPlanner() = default;

