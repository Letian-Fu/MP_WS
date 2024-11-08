//
// Created by robert on 08/11/24.
//
#include "header.h"
#include "planner.h"
#include "env.h"
#include "interpolation.h"

using namespace rrt_planner;
using namespace std;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "plan_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;
    srand(time(nullptr));

//    std::vector<VectorXd> data_points;
//    data_points.resize(5);
//    for(auto & data_point : data_points)
//        data_point.resize(2);
//    data_points[0] << -1.0, 1.0;
//    data_points[1] << -1.0, -0.5;
//    data_points[2] << 1.0, -1.0;
//    data_points[3] << 1.0, 1.0;
//    data_points[4] << 0.2, 0.8;
//    data_points.resize(6);
//    for(auto & data_point : data_points)
//        data_point.resize(6);
//    data_points[0] << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//    data_points[1] << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0；
//    data_points[2] << 2.0, -1.0, 0.0, 1.0, 2.0, 3.0；
//    data_points[3] << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0；
//    data_points[4] << 4.0, 1.0, 2.0, 3.0, 4.0, 5.0；
//    data_points[5] << 4.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//    data_points.resize(5);
//    for(auto & data_point : data_points)
//        data_point.resize(6);
//    data_points[0] << -1.0, 1.0, 1,0.2,0.3,0.1;
//    data_points[1] << -1.0, -0.5, 1.2,0.4,0.6,0.2;
//    data_points[2] << 1.0, -1.0, 1.4,0.6,0.9,0.3;
//    data_points[3] << 1.0, 1.0, 1.6,0.8,1.2,0.4;
//    data_points[4] << 0.2, 0.8, 1.8,1.0,1.5,0.5;
//
//    // 反算控制点
////    std::vector<double> nodes = create_nodes(data_points);
////    MatrixXd control_points = find_control_points(data_points, nodes);
//    int inter_num=5;
//    std::vector<VectorXd> inter_points= interplote(data_points, inter_num);
//    std::cout<<"inter_points:\n"<<std::endl;
//    for(auto & inter_point : inter_points)
//        std::cout<<inter_point.transpose()<<std::endl;


    MyPlanner my_planner(n);
    int mode;
    std::cout << "1 Obs Test"<< "  "<< "2 Catch Test"<< "  "<< "3 Real Catch"<< "  "<< "4 Experiments" << std::endl;
    std::cout << "Choose Mode:" << std::endl;
    std::cin >> mode;

    if(mode==1) {
        std::cout<<"Obs Test"<<std::endl;
        std::cout << "Set Planning Problem" << std::endl;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        moveit_msgs::CollisionObject obs;
        obs.header.frame_id = my_planner.move_group->getPlanningFrame();
        obs.id = "obs_1";
//        shape_msgs::SolidPrimitive primitive;
//        primitive.type = primitive.SPHERE;
//        primitive.dimensions.resize(1);
//        primitive.dimensions[0] = 0.1;
//        //设置障碍物位置
//        geometry_msgs::Pose pose;
//        pose.orientation.w = 1.0;
//        pose.position.x = 0.5;
//        pose.position.y = 0.0;
//        pose.position.z = 0.3;
//        //将障碍物属性、位置添加到障碍物实例中
//        obs.primitives.push_back(primitive);
//        obs.primitive_poses.push_back(pose);
//        obs.operation = obs.ADD;
//        collision_objects.push_back(obs);

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.18;
        primitive.dimensions[1] = 0.18;
        primitive.dimensions[2] = 0.5;
        //设置障碍物位置
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.5;
        pose.position.y = 0.0;
        pose.position.z = 0.25;
        //将障碍物属性、位置添加到障碍物实例中
        obs.primitives.push_back(primitive);
        obs.primitive_poses.push_back(pose);
        obs.operation = obs.ADD;
        collision_objects.push_back(obs);

        my_planner.planning_scene->addCollisionObjects(collision_objects);
        clock_t start_init, finish_init;
        start_init = clock();
        planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                "robot_description");
        monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
        planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
        ps->getCurrentStateNonConst().update();
        planning_scene::PlanningScenePtr scene = ps->diff();
        scene->decoupleParent();
        finish_init = clock();
        double duration = (double) (finish_init - start_init) * 1000 / CLOCKS_PER_SEC;
        std::cout << "Initial Time:" << duration << "ms" << std::endl;

        int test_num;
        std::cout << "Test Num:" << std::endl;
        std::cin >> test_num;

        int excute_flag=0;
        std::cout << "excute_flag:" << std::endl;
        std::cin >> excute_flag;

        VectorXd conf_s(6),conf_g(6);
        //Simple Experiments
        conf_s<<-36,-63,-48,118,36,0;
        conf_g<<58,-59,-56,10,-58,9;
//        //Complex Experimets
//        conf_s<<-160,-47,-42,102,159,18;
//        conf_g<<55,-71,101,-94,-72,-22;
        conf_s=conf_s*M_PI/180;
        conf_g=conf_g*M_PI/180;

        if(excute_flag==1)
        {
            my_planner.start_conf=my_planner.arm_pos;
////        my_planner.goal_conf<<-0.625371,-0.788809,-1.29437,2.08364,0.625325,-0.000457022;
            my_planner.goal_conf=conf_s;
            my_planner.MyPlan(scene);
            my_planner.execute();
        }
        VectorXd result(6);
        result.setZero();
        for (int i = 0; i < test_num; i++) {
            std::cout<<"Round "<<i+1<<std::endl;
            if(excute_flag==1)
                my_planner.start_conf=my_planner.arm_pos;
            else
                my_planner.start_conf << -0.525371, -0.788809, -1.29437, 2.08364, 0.625325, -0.000457022;
            my_planner.goal_conf << 0.815875, -1.11172, -0.897325, 2.00869, -0.81592, 0.000160027;
            my_planner.goal_conf=conf_g;
//            std::cout << "start_conf: " << my_planner.start_conf.transpose() * 180 / M_PI << std::endl;
//            std::cout << "goal_conf: " << my_planner.goal_conf.transpose() * 180 / M_PI << std::endl;
            result += my_planner.MyPlan(scene);
            if(excute_flag==1)
                my_planner.execute();
            sleep(1);
            if(excute_flag==1)
                my_planner.start_conf=my_planner.arm_pos;
            else
                my_planner.start_conf << 0.915875, -1.11172, -0.897325, 2.00869, -0.81592, 0.000160027;
//            my_planner.goal_conf << -0.625371, -0.788809, -1.29437, 2.08364, 0.625325, -0.000457022;
            my_planner.goal_conf=conf_g;
//            std::cout << "start_conf: " << my_planner.start_conf.transpose() * 180 / M_PI << std::endl;
//            std::cout << "goal_conf: " << my_planner.goal_conf.transpose() * 180 / M_PI << std::endl;
            result += my_planner.MyPlan(scene);
            if(excute_flag==1)
                my_planner.execute();
            sleep(1);
        }

        for(int i=1;i<6;i++)
            result(i)=result(i)/result(0);
        std::cout << std::endl << std::endl;
        std::cout<<"Algorithm"<<"   "<<"Success_Num"<<"   "<<"Average_Plan_Time"<<"   "<<"Average_Nodes"<<"   "<<"Average_Path_Nodes"<<"   "<<"Average_Fail_Nodes"<<"   "<<"Average_Path_Length"<<std::endl;
        std::cout<<"MyPlan"<<"   "<<result(0)<<"   "<<result(1)<<"ms"<<"   "<<result(2)<<"   "<<result(3)<<"   "<<result(4)<<"   "<<result(5)<<std::endl;
    }
    else if(mode==2)
    {
        std::cout<<"Catch Test!"<<std::endl;
        int mode2;
        int obs_flag=0;
        int execute_flag;
        int print_flag;
        int target_num;
//        std::cout << "Obs_flag:" << std::endl;
//        std::cin >> obs_flag;
        std::cout << "Target_num:" << std::endl;
        std::cin >> target_num;
        std::cout << "Execute_flag:" << std::endl;
        std::cin >> execute_flag;
        std::cout << "Print_flag:" << std::endl;
        std::cin >> print_flag;
        std::cout << "1 Test Once"<< "  "<< "2 Few Tests"<<std::endl;
        std::cout << "Choose Mode:" << std::endl;
        std::cin >> mode2;

        if(obs_flag==1)
        {
            CreateObs(my_planner,"obs_1",0.0,0.5,0.2,0.2,0.5);
            sleep(1.0);
            CreateObs(my_planner,"obs_2",0.0,-0.5,0.2,0.2,0.5);
            sleep(1.0);
            CreateObs(my_planner,"obs_3",0.5,0.0,0.2,0.2,0.5);
            sleep(1.0);
            CreateObs(my_planner,"obs_4",-0.5,0.0,0.2,0.2,0.5);
            sleep(1.0);
        }

        std::vector<VectorXd> final_pos;
        final_pos.resize(3);
        final_pos[0].resize(2);
        final_pos[0]<<0.3,-0.3;
        final_pos[1].resize(2);
        final_pos[1]<<0.3,-0.4;
        final_pos[2].resize(2);
        final_pos[2]<<0.3,-0.5;

        if(mode2==1) {
            std::vector<VectorXd> box_pos;
            box_pos.resize(3);
            sleep(1.0);
            CreateFloor(my_planner);
            sleep(1.0);
            CreateBox(my_planner,"box_1",-0.5,-0.5,0.6,0.6,0.15,0);
            sleep(1.0);
            std::vector<VectorXd> box_1_target_pos = CreateRandomTargets(my_planner,"box_1",-0.5,-0.5,target_num);
            sleep(1.0);
            box_pos[0].resize(2);
            box_pos[0]<<-0.5,-0.5;
            sleep(1.0);
            CreateBox(my_planner,"box_2",-0.5,0.5,0.6,0.6,0.15,0);
            sleep(1.0);
            std::vector<VectorXd> box_2_target_pos = CreateRandomTargets(my_planner,"box_2",-0.5,0.5,target_num);
            sleep(1.0);
            box_pos[1].resize(2);
            box_pos[1]<<-0.5,0.5;
            sleep(1.0);
            CreateBox(my_planner,"box_3",0.5,0.5,0.6,0.6,0.15,0);
            sleep(1.0);
            std::vector<VectorXd> box_3_target_pos = CreateRandomTargets(my_planner,"box_3",0.5,0.5,target_num);
            sleep(1.0);
            box_pos[2].resize(2);
            box_pos[2]<<0.5,0.5;
            sleep(1.0);

            planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
            monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
            planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
            ps->getCurrentStateNonConst().update();
            planning_scene::PlanningScenePtr scene = ps->diff();
            scene->decoupleParent();

            VectorXd result(4);
            result.setZero();
            result += CatchTest_random_targets(my_planner, scene, box_pos[0], final_pos[0], "box_1",box_1_target_pos, execute_flag, print_flag);
            std::cout<<std::endl;
            result += CatchTest_random_targets(my_planner, scene, box_pos[1], final_pos[1], "box_2",box_2_target_pos, execute_flag, print_flag);
            std::cout<<std::endl;
            result += CatchTest_random_targets(my_planner, scene, box_pos[2], final_pos[2], "box_3",box_3_target_pos, execute_flag, print_flag);

//            std::cout << std::endl << "MyPlan: "<< std::endl;
            std::cout<<std::endl;
            std::cout << "Success Num: " << result(0) << std::endl;
            std::cout << "Average Time: " << result(1) / result(0) << " ms" << std::endl;
//            std::cout << "Average Nodes: " << result(2) / result(0)<<std::endl;
//            std::cout << "Average Fail Nodes: " << result(3) / result(0)<<std::endl;
        }
        else
        {
            int test_num;
            std::cout << "Test Num:" << std::endl;
            std::cin >> test_num;

            VectorXd result(4);
            VectorXd temp_result(4);
            result.setZero();
            for(int i=0;i<test_num;i++)
            {
                std::cout << std::endl << "Round " << i+1 << std::endl;
                temp_result.setZero();
                std::vector<VectorXd> box_pos;
                box_pos.resize(3);
                sleep(1.0);
                CreateFloor(my_planner);
                sleep(1.0);
                CreateBox(my_planner,"box_1",-0.5,-0.5,0.6,0.6,0.15,0);
                sleep(1.0);
                std::vector<VectorXd> box_1_target_pos = CreateRandomTargets(my_planner,"box_1",-0.5,-0.5,target_num);
                sleep(1.0);
                box_pos[0].resize(2);
                box_pos[0]<<-0.5,-0.5;
                sleep(1.0);
                CreateBox(my_planner,"box_2",-0.5,0.5,0.6,0.6,0.15,0);
                sleep(1.0);
                std::vector<VectorXd> box_2_target_pos = CreateRandomTargets(my_planner,"box_2",-0.5,0.5,target_num);
                sleep(1.0);
                box_pos[1].resize(2);
                box_pos[1]<<-0.5,0.5;
                sleep(1.0);
                CreateBox(my_planner,"box_3",0.5,0.5,0.6,0.6,0.15,0);
                sleep(1.0);
                std::vector<VectorXd> box_3_target_pos = CreateRandomTargets(my_planner,"box_3",0.5,0.5,target_num);
                sleep(1.0);
                box_pos[2].resize(2);
                box_pos[2]<<0.5,0.5;
                sleep(1.0);

                planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
                monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
                planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
                ps->getCurrentStateNonConst().update();
                planning_scene::PlanningScenePtr scene = ps->diff();
                scene->decoupleParent();
                
                temp_result += CatchTest_random_targets(my_planner, scene, box_pos[0], final_pos[0], "box_1",box_1_target_pos, execute_flag, print_flag);
                temp_result += CatchTest_random_targets(my_planner, scene, box_pos[1], final_pos[1], "box_2",box_2_target_pos, execute_flag, print_flag);
                temp_result += CatchTest_random_targets(my_planner, scene, box_pos[2], final_pos[2], "box_3",box_3_target_pos, execute_flag, print_flag);
                result+=temp_result;
//                std::cout << std::endl << "MyPlan: "<< std::endl;
                std::cout << "Success Num: " << temp_result(0) << std::endl;
                std::cout << "Average Time: " << temp_result(1) / temp_result(0) << " ms" << std::endl;
//                std::cout << "Average Nodes: " << temp_result(2) / temp_result(0)<<std::endl;
//                std::cout << "Average Fail Nodes: " << temp_result(3) / temp_result(0)<<std::endl;
            }
//            std::cout << std::endl << "MyPlan: "<< std::endl;
            std::cout << "Final Result: "<<std::endl;
            std::cout << "Success Num: " << result(0) << std::endl;
            std::cout << "Average Time: " << result(1) / result(0) << " ms" << std::endl;
//            std::cout << "Average Nodes: " << result(2) / result(0)<<std::endl;
//            std::cout << "Average Fail Nodes: " << result(3) / result(0)<<std::endl;
        }
    }
    else if(mode==3)
    {
        CreateFloor(my_planner);
        sleep(1.0);
        CreateObs2(my_planner,"obs_1",0.2,-0.7,0.5,0.1,0.1,1.0);
        sleep(1.0);
        CreateObs2(my_planner,"obs_2",0.0,-0.62,0.65,0.4,0.12,0.20);
        sleep(1.0);
        CreateObs2(my_planner,"obs_3",-0.20,-0.45,0.15,0.10,0.4,0.2);
        sleep(1.0);
        CreateObs2(my_planner,"obs_4",-0.20,-0.45,0.34,0.10,0.20,0.2);
        sleep(1.0);
        CreateObs2(my_planner,"obs_5",-0,-0.35,0.04,1.0,0.45,0.08);
        sleep(1.0);
        planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
        planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
        ps->getCurrentStateNonConst().update();
        planning_scene::PlanningScenePtr scene = ps->diff();
        scene->decoupleParent();
        char buf_send[BUFSIZ],buf_recv[BUFSIZ];
        int buf_len;
        char command;
        double x,y,z,yaw,pitch,roll;
        MatrixXd OriMat;
        VectorXd joint_goal,now_joint(6),target_xyzrpy(6);


        //TCP Test
        int connect_flag=1;
        if(::connect(my_planner.sockfd,(struct sockaddr *)&my_planner.servaddr,sizeof(my_planner.servaddr)) != 0)
        {
            std::cout << "connect error" << std::endl;
            connect_flag=0;
        }
        else
            std::cout<<"connected to server"<<std::endl;
        while(connect_flag==1)
        {
            buf_len=recv(my_planner.sockfd,buf_recv,BUFSIZ,0);
            std::cout<<buf_recv<<std::endl;
            if(buf_recv[0]=='P')
            {
                strcpy(buf_send,"GN");
                buf_len=send(my_planner.sockfd,buf_send,strlen(buf_send),0);
                buf_len=recv(my_planner.sockfd,buf_recv,BUFSIZ,0);
                std::cout<<buf_recv<<std::endl;
                std::vector<char*> temp;
                const char split[] = ",";
                char* res = strtok(buf_recv, split);//image_name必须为char[]
                while (res != NULL)
                {
                    temp.push_back(res);
                    res = strtok(NULL, split);
                }
                if(temp.size()==6)
                {
                    now_joint(0)=atof(temp[0]) * M_PI / 180;
                    now_joint(1)=atof(temp[1]) * M_PI / 180;
                    now_joint(2)=atof(temp[2]) * M_PI / 180;
                    now_joint(3)=atof(temp[3]) * M_PI / 180;
                    now_joint(4)=atof(temp[4]) * M_PI / 180;
                    now_joint(5)=atof(temp[5]) * M_PI / 180;
                    std::cout<<now_joint.transpose()<<std::endl;
                }
                my_planner.start_conf=my_planner.arm_pos;
                std::cout<<"start_conf:"<<my_planner.start_conf.transpose()<<std::endl;
                my_planner.goal_conf = now_joint;
                std::cout<<"goal_conf:"<<my_planner.goal_conf.transpose()<<std::endl;
                my_planner.MyPlan(scene);
                my_planner.execute();
                sleep(1);

                strcpy(buf_send,"GT");
                buf_len=send(my_planner.sockfd,buf_send,strlen(buf_send),0);
                buf_len=recv(my_planner.sockfd,buf_recv,BUFSIZ,0);
        //        std::cout<<buf_len<<std::endl;
                std::vector<char*> temp2;
                char* res2 = strtok(buf_recv, split);//image_name必须为char[]
                while (res2 != NULL)
                {
                    temp2.push_back(res2);
                    res2 = strtok(NULL, split);
                }
                if(temp.size()==6)
                {
                    target_xyzrpy(0)=atof(temp[0]) / 1000;
                    target_xyzrpy(1)=atof(temp[1]) / 1000;
                    target_xyzrpy(2)=atof(temp[2]) / 1000;
                    target_xyzrpy(3)=atof(temp[3]) * M_PI / 180;
                    target_xyzrpy(4)=atof(temp[4]) * M_PI / 180;
                    target_xyzrpy(5)=atof(temp[5]) * M_PI / 180+M_PI_2;
                    std::cout<<"target_xyzrpy: "<<target_xyzrpy.transpose()<<std::endl;
                }
                //
                //前往过渡点
                std::cout<<std::endl<<"前往过渡点"<<std::endl;
                my_planner.start_conf=my_planner.arm_pos;
                std::cout<<"start_conf:"<<my_planner.start_conf.transpose()<<std::endl;
                x=target_xyzrpy[0];
                y=target_xyzrpy[1];
                z=target_xyzrpy[2];
                roll=target_xyzrpy[3];
                pitch=target_xyzrpy[4];
                yaw=target_xyzrpy[5];
                OriMat = orientationMatrix(x, y, z, yaw, pitch, roll);
                joint_goal = ReverseKinematics_Collision_Nearest(my_planner.dh_alpha,my_planner.dh_a,my_planner.dh_d,my_planner.dh_theta,
                                                                 OriMat,scene,my_planner.joint_model_group,my_planner.start_conf);
        //        if(joint_goal.size()==6)
        //            my_planner.goal_conf = joint_goal;
        //        else
        //            my_planner.goal_conf = my_planner.start_conf;
                std::cout<<"joint_goal: "<<joint_goal.transpose()<<std::endl;
                joint_goal.resize(6);
                joint_goal<<-64.5647,
                3.4883,
                -109.3200,
                15.7949,
                89.666580,
                -74.7543;
                joint_goal=joint_goal*M_PI/180;
                my_planner.goal_conf = joint_goal;
                std::cout<<"goal_conf:"<<my_planner.goal_conf.transpose()<<std::endl;
                my_planner.MyPlan(scene);
                my_planner.execute_real();
                my_planner.plan_flag=1;
                my_planner.is_plan_success=1;
                my_planner.execute();
                sleep(2);
            }
            buf_len=recv(my_planner.sockfd,buf_recv,BUFSIZ,0);
            std::cout<<buf_recv<<std::endl;
            if(buf_recv[0]=='A')
            {
                //前往放置点
                std::cout<<std::endl<<"前往放置点"<<std::endl;
                my_planner.start_conf=my_planner.arm_pos;
                std::cout<<"start_conf:"<<my_planner.start_conf.transpose()<<std::endl;
                joint_goal.resize(6);
                joint_goal<<-125.8004,
                -6.6283,
                -116.9481,
                32.7908,
                89.1402,
                -105.3470;
                joint_goal=joint_goal*M_PI/180;
                my_planner.goal_conf = joint_goal;
                std::cout<<"goal_conf:"<<my_planner.goal_conf.transpose()<<std::endl;
                my_planner.MyPlan(scene);
                my_planner.execute_real();
                my_planner.plan_flag=1;
                my_planner.is_plan_success=1;
                my_planner.execute();
                sleep(5);
            }
            buf_len=recv(my_planner.sockfd,buf_recv,BUFSIZ,0);
            std::cout<<buf_recv<<std::endl;
            if(buf_recv[0]=='B')
            {
                //前往拍照点
                std::cout<<std::endl<<"前往拍照点"<<std::endl;
                my_planner.start_conf=my_planner.arm_pos;
                std::cout<<"start_conf:"<<my_planner.start_conf.transpose()<<std::endl;
                joint_goal.resize(6);
                joint_goal<<241.974411-360,
                0.988003,
                -98.265465,
                7.240437,
                89.639008,
                -116.252197;
                joint_goal=joint_goal*M_PI/180;
                my_planner.goal_conf = joint_goal;
                std::cout<<"goal_conf:"<<my_planner.goal_conf.transpose()<<std::endl;
                my_planner.MyPlan(scene);
                my_planner.execute_real();
                my_planner.plan_flag=1;
                my_planner.is_plan_success=1;
                my_planner.execute();
                sleep(5);
            }
        }
    }
    else if(mode==4)
    {
        std::cout<<"Experiments"<<std::endl;
//            std::cout << "Set Planning Problem" << std::endl;
        std::vector<moveit_msgs::CollisionObject> collision_objects;

        double obs_size = 0.1;
        double obs_num = 12;
        std::vector<VectorXd> obs_pos;
        int create_obs=2;
        if(create_obs==1) {
            for (int i = 0; i < obs_num; i++) {
                moveit_msgs::CollisionObject obs;
                obs.header.frame_id = my_planner.move_group->getPlanningFrame();
                obs.id = "obs_" + std::to_string(i + 1);
//                std::cout << obs.id << std::endl;
                shape_msgs::SolidPrimitive primitive_obs;
                primitive_obs.type = primitive_obs.SPHERE;
                primitive_obs.dimensions.resize(1);
                primitive_obs.dimensions[0] = obs_size;
                geometry_msgs::Pose pose_obs;
                pose_obs.orientation.w = 1.0;
//            while(true) {
                pose_obs.position.x = 0.75 * ((2 * (rand() / double(RAND_MAX))) - 1);
                pose_obs.position.y = 0.75 * ((2 * (rand() / double(RAND_MAX))) - 1);
                pose_obs.position.z = 0.6 + 0.3 * ((2 * (rand() / double(RAND_MAX))) - 1);
//                int count=0;
//                for(auto & obs_po : obs_pos)
//                {
//                    VectorXd temp(3);
//                    temp<<obs_po(0)-pose_obs.position.x,obs_po(1)-pose_obs.position.y-obs_po(2)-pose_obs.position.z;
//                    double dist=temp.norm();
//                    if(dist > obs_size * 2)
//                        count++;
//                }
//                if(count==obs_pos.size())
//                    break;
//            }
                //将障碍物属性、位置添加到障碍物实例中
                obs.primitives.push_back(primitive_obs);
                obs.primitive_poses.push_back(pose_obs);
                obs.operation = obs.ADD;
                collision_objects.push_back(obs);
                VectorXd pos(3);
                pos << pose_obs.position.x, pose_obs.position.y, pose_obs.position.z;
                obs_pos.push_back(pos);
            }
        }
        if(create_obs==2)
        {
            moveit_msgs::CollisionObject obs;
            obs.header.frame_id = my_planner.move_group->getPlanningFrame();
            obs.id = "obs_1";
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.18;
            primitive.dimensions[1] = 0.18;
            primitive.dimensions[2] = 0.45;
//            primitive.dimensions[0] = 0.15;
//            primitive.dimensions[1] = 0.15;
//            primitive.dimensions[2] = 0.40;
            //设置障碍物位置
            geometry_msgs::Pose pose;
            pose.orientation.w = 1.0;
            pose.position.x = 0.5;
            pose.position.y = 0.0;
            pose.position.z = primitive.dimensions[2]/2;
            //将障碍物属性、位置添加到障碍物实例中
            obs.primitives.push_back(primitive);
            obs.primitive_poses.push_back(pose);
            obs.operation = obs.ADD;
            collision_objects.push_back(obs);
        }

        my_planner.planning_scene->addCollisionObjects(collision_objects);

        clock_t start_init, finish_init;
        start_init = clock();
        planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                "robot_description");
        monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
        planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
        ps->getCurrentStateNonConst().update();
        planning_scene::PlanningScenePtr scene = ps->diff();
        scene->decoupleParent();
        finish_init = clock();
        double duration = (double) (finish_init - start_init) * 1000 / CLOCKS_PER_SEC;
//            std::cout << "Initial Time:" << duration << "ms" << std::endl;

        int test_num;
        std::cout << "Test Num:" << std::endl;
        std::cin >> test_num;
        VectorXd conf_s(6),conf_g(6);
        //Simple Experiments
        conf_s<<-36,-63,-48,118,36,0;
        conf_g<<58,-59,-56,10,-58,9;

//        //Complex Experimets
//        conf_s<<-160,-47,-42,102,159,18;
//        conf_g<<55,-71,101,-94,-72,-22;

        conf_s=conf_s*M_PI/180;
        conf_g=conf_g*M_PI/180;

        my_planner.start_conf=my_planner.arm_pos;
//        my_planner.goal_conf<<-0.625371,-0.788809,-1.29437,2.08364,0.625325,-0.000457022;
        my_planner.goal_conf=conf_s;
        my_planner.is_adapt_step=1;
        my_planner.is_adapt_fai=1;
        my_planner.MyPlan(scene);
//        my_planner.execute();

        VectorXd result(6);
        result.setZero();
        VectorXd result_rrtconnect(6);
        result_rrtconnect.setZero();
        VectorXd result_bg(6);
        result_bg.setZero();
        VectorXd result_adapt_step(6);
        result_adapt_step.setZero();
        VectorXd result_adapt_fai(6);
        result_adapt_fai.setZero();
        VectorXd result_adapt_step_fai(6);
        result_adapt_step_fai.setZero();
        for (int i = 0; i < test_num; i++) {
            std::cout<<"Round "<<i+1<<std::endl;
//            my_planner.start_conf=my_planner.arm_pos;
            my_planner.start_conf=conf_s;
            my_planner.goal_conf=conf_g;
//            result_rrtconnect += my_planner.MyPlan_RRTConnect(scene);
//            result_bg += my_planner.MyPlan_BG(scene);
////            my_planner.is_adapt_step=0;
////            my_planner.is_adapt_fai=0;
////            result += my_planner.MyPlan(scene);
//            my_planner.is_adapt_step=1;
//            my_planner.is_adapt_fai=0;
//            result_adapt_step += my_planner.MyPlan(scene);
//            my_planner.is_adapt_step=0;
//            my_planner.is_adapt_fai=1;
//            result_adapt_fai += my_planner.MyPlan(scene);
            my_planner.is_adapt_step=1;
            my_planner.is_adapt_fai=1;
            result_adapt_step_fai += my_planner.MyPlan(scene);
            my_planner.execute();

////            my_planner.start_conf=my_planner.arm_pos;
//            my_planner.start_conf=conf_g;
//            my_planner.goal_conf=conf_s;
//            result_rrtconnect += my_planner.MyPlan_RRTConnect(scene);
//            result_bg += my_planner.MyPlan_BG(scene);
////            my_planner.is_adapt_step=0;
////            my_planner.is_adapt_fai=0;
////            result += my_planner.MyPlan(scene);
//            my_planner.is_adapt_step=1;
//            my_planner.is_adapt_fai=0;
//            result_adapt_step += my_planner.MyPlan(scene);
//            my_planner.is_adapt_step=0;
//            my_planner.is_adapt_fai=1;
//            result_adapt_fai += my_planner.MyPlan(scene);
//            my_planner.is_adapt_step=1;
//            my_planner.is_adapt_fai=1;
//            result_adapt_step_fai += my_planner.MyPlan(scene);
////            my_planner.execute();
        }
        for(int i=1;i<6;i++) {
            result_rrtconnect(i) = result_rrtconnect(i) / result_rrtconnect(0);
            result_bg(i) = result_bg(i) / result_bg(0);
            result(i) = result(i) / result(0);
            result_adapt_step(i) = result_adapt_step(i) / result_adapt_step(0);
            result_adapt_fai(i) = result_adapt_fai(i) / result_adapt_fai(0);
            result_adapt_step_fai(i) = result_adapt_step_fai(i) / result_adapt_step_fai(0);
        }
        std::cout << std::endl << std::endl;
        std::cout<<"Algorithm"<<"   "<<"Success_Num"<<"   "<<"Average_Plan_Time"<<"   "<<"Average_Nodes"<<"   "<<"Average_Path_Nodes"<<"   "<<"Average_Fail_Nodes"<<"   "<<"Average_Path_Length"<<std::endl;
        std::cout<<"RRTConnet"<<"   "<<result_rrtconnect(0)<<"   "<<result_rrtconnect(1)<<"ms"<<"   "<<result_rrtconnect(2)<<"   "<<result_rrtconnect(3)<<"   "<<result_rrtconnect(4)<<"   "<<result_rrtconnect(5)<<std::endl;
        std::cout<<"MyPlan_BG"<<"   "<<result_bg(0)<<"   "<<result_bg(1)<<"ms"<<"   "<<result_bg(2)<<"   "<<result_bg(3)<<"   "<<result_bg(4)<<"   "<<result_bg(5)<<std::endl;
//        std::cout<<"MyPlan"<<"   "<<result(0)<<"   "<<result(1)<<"ms"<<"   "<<result(2)<<"   "<<result(3)<<"   "<<result(4)<<"   "<<result(5)<<std::endl;
        std::cout<<"MyPlan_Adapt_Step"<<"   "<<result_adapt_step(0)<<"   "<<result_adapt_step(1)<<"ms"<<"   "<<result_adapt_step(2)<<"   "<<result_adapt_step(3)<<"   "<<result_adapt_step(4)<<"   "<<result_adapt_step(5)<<std::endl;
        std::cout<<"MyPlan_Adapt_Fai"<<"   "<<result_adapt_fai(0)<<"   "<<result_adapt_fai(1)<<"ms"<<"   "<<result_adapt_fai(2)<<"   "<<result_adapt_fai(3)<<"   "<<result_adapt_fai(4)<<"   "<<result_adapt_fai(5)<<std::endl;
        std::cout<<"MyPlan_Adapt_Step_Fai"<<"   "<<result_adapt_step_fai(0)<<"   "<<result_adapt_step_fai(1)<<"ms"<<"   "<<result_adapt_step_fai(2)<<"   "<<result_adapt_step_fai(3)<<"   "<<result_adapt_step_fai(4)<<"   "<<result_adapt_step_fai(5)<<std::endl;
    }
    else {
        ROS_INFO_STREAM("You Can Start Planning!");
        while (ros::ok()) {
            ros::spinOnce();
            my_planner.execute();
        }
    }


    ros::shutdown();
    return 0;
}
