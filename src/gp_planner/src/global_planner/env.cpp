//
// Created by robert on 9/4/22.
//
#include "env.h"
namespace rrt_planner{

void CreateFloor(const MyPlanner& planner)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::cout<<"Create Floor"<<std::endl;
    moveit_msgs::CollisionObject floor;
    floor.header.frame_id = planner.move_group->getPlanningFrame();
    floor.id = "floor";
    //设置障碍物外形、尺寸等属性
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 0.1;
    //设置障碍物位置
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = -0.06;
    //将障碍物属性、位置添加到障碍物实例中
    floor.primitives.push_back(primitive);
    floor.primitive_poses.push_back(pose);
    floor.operation = floor.ADD;
    collision_objects.push_back(floor);
    planner.planning_scene->addCollisionObjects(collision_objects);
}

void CreateBox(const MyPlanner& planner,
               const std::string& box_name,
               double box_x,
               double box_y,
               double box_width,
               double box_length,
               double box_height,
               int target_flag){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::cout<<"Create Box "<<box_name<<std::endl;
    double thickness = 0.02;
    double target_size = 0.04;

    moveit_msgs::CollisionObject box;
    box.header.frame_id = planner.move_group->getPlanningFrame();
    box.id = box_name;
    shape_msgs::SolidPrimitive primitive_part1;
    primitive_part1.type = primitive_part1.BOX;
    primitive_part1.dimensions.resize(3);
    primitive_part1.dimensions[0] = thickness;
    primitive_part1.dimensions[1] = box_length+thickness*2;
    primitive_part1.dimensions[2] = box_height;
    geometry_msgs::Pose pose_part1;
    pose_part1.orientation.w = 1.0;
    pose_part1.orientation.x=0;
    pose_part1.orientation.y=0;
    pose_part1.orientation.z=0;
    pose_part1.position.x = box_x - box_width/2 - thickness/2;
    pose_part1.position.y = box_y;
    pose_part1.position.z = primitive_part1.dimensions[2]/2;
    //将障碍物属性、位置添加到障碍物实例中
    box.primitives.push_back(primitive_part1);
    box.primitive_poses.push_back(pose_part1);

    shape_msgs::SolidPrimitive primitive_part2;
    primitive_part2.type = primitive_part2.BOX;
    primitive_part2.dimensions.resize(3);
    primitive_part2.dimensions[0] = thickness;
    primitive_part2.dimensions[1] = box_length+thickness*2;
    primitive_part2.dimensions[2] = box_height;
    geometry_msgs::Pose pose_part2;
    pose_part2.orientation.w = 1.0;
    pose_part2.orientation.x=0;
    pose_part2.orientation.y=0;
    pose_part2.orientation.z=0;
    pose_part2.position.x = box_x + box_width/2 + thickness/2;
    pose_part2.position.y = box_y;
    pose_part2.position.z = primitive_part2.dimensions[2]/2;
    //将障碍物属性、位置添加到障碍物实例中
    box.primitives.push_back(primitive_part2);
    box.primitive_poses.push_back(pose_part2);

    shape_msgs::SolidPrimitive primitive_part3;
    primitive_part3.type = primitive_part3.BOX;
    primitive_part3.dimensions.resize(3);
    primitive_part3.dimensions[0] = box_width;
    primitive_part3.dimensions[1] = thickness;
    primitive_part3.dimensions[2] = box_height;
    geometry_msgs::Pose pose_part3;
    pose_part3.orientation.w = 1.0;
    pose_part3.orientation.x=0;
    pose_part3.orientation.y=0;
    pose_part3.orientation.z=0;
    pose_part3.position.x = box_x;
    pose_part3.position.y = box_y + box_length/2 + thickness/2;
    pose_part3.position.z = primitive_part3.dimensions[2]/2;
    //将障碍物属性、位置添加到障碍物实例中
    box.primitives.push_back(primitive_part3);
    box.primitive_poses.push_back(pose_part3);

    shape_msgs::SolidPrimitive primitive_part4;
    primitive_part4.type = primitive_part4.BOX;
    primitive_part4.dimensions.resize(3);
    primitive_part4.dimensions[0] = box_width;
    primitive_part4.dimensions[1] = thickness;
    primitive_part4.dimensions[2] = box_height;
    geometry_msgs::Pose pose_part4;
    pose_part4.orientation.w = 1.0;
    pose_part4.orientation.x=0;
    pose_part4.orientation.y=0;
    pose_part4.orientation.z=0;
    pose_part4.position.x = box_x;
    pose_part4.position.y = box_y - box_length/2 - thickness/2;
    pose_part4.position.z = primitive_part4.dimensions[2]/2;
    //将障碍物属性、位置添加到障碍物实例中
    box.primitives.push_back(primitive_part4);
    box.primitive_poses.push_back(pose_part4);

    shape_msgs::SolidPrimitive primitive_part5;
    primitive_part5.type = primitive_part5.BOX;
    primitive_part5.dimensions.resize(3);
    primitive_part5.dimensions[0] = box_width+2*thickness;
    primitive_part5.dimensions[1] = box_length+2*thickness;
    primitive_part5.dimensions[2] = thickness;
    geometry_msgs::Pose pose_part5;
    pose_part5.orientation.w = 1.0;
    pose_part5.orientation.x=0;
    pose_part5.orientation.y=0;
    pose_part5.orientation.z=0;
    pose_part5.position.x = box_x;
    pose_part5.position.y = box_y;
    pose_part5.position.z = -thickness/2;
    //将障碍物属性、位置添加到障碍物实例中
    box.primitives.push_back(primitive_part5);
    box.primitive_poses.push_back(pose_part5);

    box.operation = box.ADD;
    collision_objects.push_back(box);

    if(target_flag==1)
    {
        moveit_msgs::CollisionObject target1;
        target1.header.frame_id = planner.move_group->getPlanningFrame();
        target1.id = box_name + "_" + "target1";
        shape_msgs::SolidPrimitive primitive_target1;
        primitive_target1.type = primitive_target1.BOX;
        primitive_target1.dimensions.resize(3);
        primitive_target1.dimensions[0] = target_size;
        primitive_target1.dimensions[1] = target_size;
        primitive_target1.dimensions[2] = target_size;
        geometry_msgs::Pose pose_target1;
        pose_target1.orientation.w = 1.0;
//        pose_target1.position.x = box_x + 0.25 * ((2 * (rand() / double(RAND_MAX))) - 1);
//        pose_target1.position.y = box_y + 0.25 * ((2 * (rand() / double(RAND_MAX))) - 1);
        pose_target1.position.x = box_x - 0.06;
        pose_target1.position.y = box_y + 0.06;
        pose_target1.position.z = primitive_target1.dimensions[2]/2;
        //将障碍物属性、位置添加到障碍物实例中
        target1.primitives.push_back(primitive_target1);
        target1.primitive_poses.push_back(pose_target1);
        target1.operation = target1.ADD;
        collision_objects.push_back(target1);

        moveit_msgs::CollisionObject target2;
        target2.header.frame_id = planner.move_group->getPlanningFrame();
        target2.id = box_name + "_" + "target2";
        shape_msgs::SolidPrimitive primitive_target2;
        primitive_target2.type = primitive_target2.BOX;
        primitive_target2.dimensions.resize(3);
        primitive_target2.dimensions[0] = target_size;
        primitive_target2.dimensions[1] = target_size;
        primitive_target2.dimensions[2] = target_size;
        geometry_msgs::Pose pose_target2;
        pose_target2.orientation.w = 1.0;
//        pose_target2.position.x = box_x + 0.25 * ((2 * (rand() / double(RAND_MAX))) - 1);
//        pose_target2.position.y = box_y + 0.25 * ((2 * (rand() / double(RAND_MAX))) - 1);
        pose_target2.position.x = box_x + 0.06;
        pose_target2.position.y = box_y + 0.06;
        pose_target2.position.z = primitive_target2.dimensions[2]/2;
        //将障碍物属性、位置添加到障碍物实例中
        target2.primitives.push_back(primitive_target2);
        target2.primitive_poses.push_back(pose_target2);
        target2.operation = target2.ADD;
        collision_objects.push_back(target2);

        moveit_msgs::CollisionObject target3;
        target3.header.frame_id = planner.move_group->getPlanningFrame();
        target3.id = box_name + "_" + "target3";
        shape_msgs::SolidPrimitive primitive_target3;
        primitive_target3.type = primitive_target3.BOX;
        primitive_target3.dimensions.resize(3);
        primitive_target3.dimensions[0] = target_size;
        primitive_target3.dimensions[1] = target_size;
        primitive_target3.dimensions[2] = target_size;
        geometry_msgs::Pose pose_target3;
        pose_target3.orientation.w = 1.0;
//        pose_target3.position.x = box_x + 0.25 * ((2 * (rand() / double(RAND_MAX))) - 1);
//        pose_target3.position.y = box_y + 0.25 * ((2 * (rand() / double(RAND_MAX))) - 1);
        pose_target3.position.x = box_x + 0.06;
        pose_target3.position.y = box_y - 0.06;
        pose_target3.position.z = primitive_target3.dimensions[2]/2;
        //将障碍物属性、位置添加到障碍物实例中
        target3.primitives.push_back(primitive_target3);
        target3.primitive_poses.push_back(pose_target3);
        target3.operation = target3.ADD;
        collision_objects.push_back(target3);

        moveit_msgs::CollisionObject target4;
        target4.header.frame_id = planner.move_group->getPlanningFrame();
        target4.id = box_name + "_" + "target4";
        shape_msgs::SolidPrimitive primitive_target4;
        primitive_target4.type = primitive_target4.BOX;
        primitive_target4.dimensions.resize(3);
        primitive_target4.dimensions[0] = target_size;
        primitive_target4.dimensions[1] = target_size;
        primitive_target4.dimensions[2] = target_size;
        geometry_msgs::Pose pose_target4;
        pose_target4.orientation.w = 1.0;
//        pose_target4.position.x = box_x + 0.25 * ((2 * (rand() / double(RAND_MAX))) - 1);
//        pose_target4.position.y = box_y + 0.25 * ((2 * (rand() / double(RAND_MAX))) - 1);
        pose_target4.position.x = box_x - 0.06;
        pose_target4.position.y = box_y - 0.06;
        pose_target4.position.z = primitive_target4.dimensions[2]/2;
        //将障碍物属性、位置添加到障碍物实例中
        target4.primitives.push_back(primitive_target4);
        target4.primitive_poses.push_back(pose_target4);
        target4.operation = target4.ADD;
        collision_objects.push_back(target4);
    }

    planner.planning_scene->addCollisionObjects(collision_objects);
}

std::vector<VectorXd> CreateRandomTargets(const MyPlanner& planner,
                   const std::string& box_name,
                   double box_x,
                   double box_y,
                   int target_num){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<VectorXd> target_pos;
//    std::cout<<"Create Targets "<<box_name<<std::endl;
    double target_size = 0.02;
    for(int i=0;i<target_num;i++)
    {
        moveit_msgs::CollisionObject target;
        target.header.frame_id = planner.move_group->getPlanningFrame();
        target.id = box_name + "_" + "target" + std::to_string(i+1);
        shape_msgs::SolidPrimitive primitive_target;
        primitive_target.type = primitive_target.BOX;
        primitive_target.dimensions.resize(3);
        primitive_target.dimensions[0] = target_size;
        primitive_target.dimensions[1] = target_size;
        primitive_target.dimensions[2] = target_size;
        geometry_msgs::Pose pose_target;
        pose_target.orientation.w = 1.0;
//        pose_target.orientation.x=0;
//        pose_target.orientation.y=0;
//        pose_target.orientation.z=0;
        while(true) {
            pose_target.position.x = box_x + 0.125 * ((2 * (rand() / double(RAND_MAX))) - 1);
            pose_target.position.y = box_y + 0.125 * ((2 * (rand() / double(RAND_MAX))) - 1);
            int count=0;
            for(auto & target_po : target_pos)
            {
                if(abs(pose_target.position.x-target_po(0))>(target_size+0.01))
                    if(abs(pose_target.position.y-target_po(1))>(target_size+0.01))
                        count+=1;
            }
            if(count==target_pos.size())
                break;
        }
        pose_target.position.z = primitive_target.dimensions[2]/2;
        //将障碍物属性、位置添加到障碍物实例中
        target.primitives.push_back(primitive_target);
        target.primitive_poses.push_back(pose_target);
        target.operation = target.ADD;
        collision_objects.push_back(target);
        VectorXd pos(3);
        pos<<pose_target.position.x,pose_target.position.y,pose_target.position.z;
        target_pos.push_back(pos);
    }
    planner.planning_scene->addCollisionObjects(collision_objects);
    return target_pos;
}

void CreateObs(const MyPlanner& planner,
               const std::string& obs_name,
               double obs_x,
               double obs_y,
               double box_width,
               double box_length,
               double box_height)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::cout<<"Create Obs "<<obs_name<<std::endl;
    moveit_msgs::CollisionObject obs;
    obs.header.frame_id = planner.move_group->getPlanningFrame();
    obs.id = obs_name;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = box_width;
    primitive.dimensions[1] = box_length;
    primitive.dimensions[2] = box_height;
    //设置障碍物位置
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = obs_x;
    pose.position.y = obs_y;
    pose.position.z = primitive.dimensions[2]/2;
    //将障碍物属性、位置添加到障碍物实例中
    obs.primitives.push_back(primitive);
    obs.primitive_poses.push_back(pose);
    obs.operation = obs.ADD;
    collision_objects.push_back(obs);
    planner.planning_scene->addCollisionObjects(collision_objects);
}

void CreateObs2(const MyPlanner& planner,
               const std::string& obs_name,
               double obs_x,
               double obs_y,
               double obs_z,
               double box_width,
               double box_length,
               double box_height)
               {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::cout<<"Create Obs "<<obs_name<<std::endl;
    moveit_msgs::CollisionObject obs;
    obs.header.frame_id = planner.move_group->getPlanningFrame();
    obs.id = obs_name;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = box_width;
    primitive.dimensions[1] = box_length;
    primitive.dimensions[2] = box_height;
    //设置障碍物位置
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = obs_x;
    pose.position.y = obs_y;
    pose.position.z = obs_z;
    //将障碍物属性、位置添加到障碍物实例中
    obs.primitives.push_back(primitive);
    obs.primitive_poses.push_back(pose);
    obs.operation = obs.ADD;
    collision_objects.push_back(obs);
    planner.planning_scene->addCollisionObjects(collision_objects);
               }

VectorXd CatchTest_fixed_targets(MyPlanner planner,
                   const planning_scene::PlanningScenePtr& scene,
                   VectorXd box_pos,
                   VectorXd final_pos,
                   const std::string& box_name,
                   int execute_flag,
                   int print_flag){
    double yaw=M_PI_2;
    double pitch=M_PI;
    double roll=0.0;
    double x,y,z;

    Matrix<double,4,4> OriMat;
    VectorXd joint_goal;
    double avg_time=0;
    int success_num=0;
    double nodes=0;
    double fail_nodes=0;
    double plan_time=0;

    if(print_flag==1)
        std::cout<<"Algorithm"<<"  "<<"Success"<<"  "<<"Time"<<"  "<<"Nodes"<<"  "<<"Path Nodes"<<"  "<<"Fail Nodes"<<std::endl;
    for(int i=0;i<4;i++)
    {
        clock_t start,finish;
        start=clock();
        VectorXd result_plan(8);
        result_plan.setZero();
        std::string target_name = box_name + "_" + "target" + std::to_string(i+1);
        VectorXd coefficient(2);
        if(i==0)
            coefficient<<-1,1;
        else if(i==1)
            coefficient<<1,1;
        else if(i==2)
            coefficient<<1,-1;
        else
            coefficient<<-1,-1;
        x = box_pos(0)+coefficient(0)*0.06;
        y = box_pos(1)+coefficient(1)*0.06;
        z = 0.08;
        OriMat = orientationMatrix(x, y, z, yaw, pitch, roll);
        joint_goal = ReverseKinematics_Collision(planner.dh_alpha,planner.dh_a,planner.dh_d,planner.dh_theta,OriMat,scene,planner.joint_model_group);
        planner.start_conf = planner.arm_pos;
        planner.goal_conf = joint_goal;
        result_plan = planner.MyPlan(scene);
        finish = clock();
        if(result_plan(0)==1)
        {
            success_num += 1;
            planner.arm_pos = planner.goal_conf;
            avg_time+=(double)(finish - start) * 1000 / CLOCKS_PER_SEC;
            nodes+=result_plan(2);
            fail_nodes+=result_plan(4);
            plan_time+=result_plan(1);
            if(execute_flag==1) {
                planner.execute();
                planner.move_group->attachObject(target_name);
                planner.move_group->setStartState(*planner.move_group->getCurrentState());
                planner.move_group->stop();
                planner.move_group->clearPoseTargets();
            }
        }
        else
            planner.arm_pos = planner.goal_conf;
        if(print_flag==1)
            std::cout << "MyPlan" << "   " << result_plan(0) << "   " << result_plan(1) << "   " << result_plan(2) << "   "<< result_plan(3) << "   " << result_plan(4)<< std::endl;
        start=clock();
        x = final_pos(0) + i * 0.08;
        y = final_pos(1);
        z = 0.08;
        OriMat = orientationMatrix(x, y, z, yaw, pitch, roll);
        joint_goal = ReverseKinematics_Collision(planner.dh_alpha,planner.dh_a,planner.dh_d,planner.dh_theta,OriMat,scene,planner.joint_model_group);
        planner.start_conf = planner.arm_pos;
        planner.goal_conf = joint_goal;
        result_plan = planner.MyPlan(scene);
        finish = clock();
        if(result_plan(0)==1)
        {
            success_num += 1;
            planner.arm_pos = planner.goal_conf;
            avg_time+=(double)(finish - start) * 1000 / CLOCKS_PER_SEC;
            nodes+=result_plan(2);
            fail_nodes+=result_plan(4);
            plan_time+=result_plan(1);
            if(execute_flag==1) {
                planner.execute();
                planner.move_group->detachObject(target_name);
                planner.move_group->setStartState(*planner.move_group->getCurrentState());
                planner.move_group->stop();
                planner.move_group->clearPoseTargets();
            }
        }
        else
            planner.arm_pos = planner.goal_conf;
        if(print_flag==1)
        {
            std::cout << "MyPlan" << "   " << result_plan(0) << "   " << result_plan(1) << "   " << result_plan(2) << "   "<< result_plan(3) << "   " << result_plan(4)<< std::endl;
        }
    }
    avg_time = avg_time/success_num;
    if(print_flag==1) {
        std::cout << "Success Num: " << success_num << std::endl;
        std::cout << "Average Time: " << avg_time << " ms" << std::endl;
    }
    VectorXd result(4);
    result<<success_num,avg_time*success_num,nodes,fail_nodes;
    return result;
}

VectorXd CatchTest_random_targets(MyPlanner planner,
                    const planning_scene::PlanningScenePtr& scene,
                    const VectorXd& box_pos,
                    VectorXd final_pos,
                    const std::string& box_name,
                    std::vector<VectorXd> target_pos,
                    int execute_flag,
                    int print_flag){
    std::cout<<"Catch Test "<<box_name<<std::endl;
    double yaw=M_PI_2;
    double pitch=M_PI;
    double roll=0.0;
    double x,y,z;

    Matrix<double,4,4> OriMat;
    VectorXd joint_goal;
    double avg_time=0;
    int success_num=0;
    double nodes=0;
    double fail_nodes=0;
    double plan_time=0;
    

    if(print_flag==1)
        std::cout<<"Algorithm"<<"  "<<"Success"<<"  "<<"Time"<<std::endl;
//        std::cout<<"Algorithm"<<"  "<<"Success"<<"  "<<"Time"<<"  "<<"Nodes"<<"  "<<"Path Nodes"<<"  "<<"Fail Nodes"<<std::endl;
    for(int i=0;i<target_pos.size();i++)
    {
        clock_t start,finish;
//        sleep(1.0);
        start=clock();
        VectorXd result_plan(8);
        result_plan.setZero();
        std::string target_name = box_name + "_" + "target" + std::to_string(i+1);
        VectorXd coefficient(2);
        if(i==0)
            coefficient<<-1,1;
        else if(i==1)
            coefficient<<1,1;
        else if(i==2)
            coefficient<<1,-1;
        else
            coefficient<<-1,-1;
        x = target_pos[i](0);
        y = target_pos[i](1);
        z = target_pos[i](2)+0.14;
        OriMat = orientationMatrix(x, y, z, yaw, pitch, roll);
        joint_goal = ReverseKinematics_Collision(planner.dh_alpha,planner.dh_a,planner.dh_d,planner.dh_theta,OriMat,scene,planner.joint_model_group);
        planner.start_conf = planner.arm_pos;
        planner.goal_conf = joint_goal;
//        std::cout<<joint_goal.transpose()*180/M_PI<<std::endl;
        if(joint_goal.size()==6)
            planner.goal_conf = joint_goal;
        else
            planner.goal_conf=planner.start_conf;
        result_plan = planner.MyPlan(scene);
        finish = clock();

        if(print_flag==1)
            std::cout << "MyPlan" << "   " << result_plan(0) << "   " << result_plan(1)<<"ms"<< std::endl;
//            std::cout << "MyPlan" << "   " << result_plan(0) << "   " << result_plan(1)<<"ms" << "   " << result_plan(2) << "   "<< result_plan(3) << "   " << result_plan(4)<< std::endl;
        if(result_plan(0)==1)
        {
            success_num += 1;
            planner.arm_pos = planner.goal_conf;
            avg_time+=(double)(finish - start) * 1000 / CLOCKS_PER_SEC;
            nodes+=result_plan(2);
            fail_nodes+=result_plan(4);
            plan_time+=result_plan(1);
            if(execute_flag==1) {
                planner.execute();
//                sleep(1.0);
                planner.move_group->attachObject(target_name);
                planner.move_group->clearPoseTargets();
                planner.move_group->setStartState(*planner.move_group->getCurrentState());
                planner.move_group->stop();
            }
        }
        else
            planner.arm_pos = planner.goal_conf;

//        geometry_msgs::Pose target;
//        target.orientation.w = 1;
//        target.orientation.x= 0;
//        target.orientation.y = 0;
//        target.orientation.z = 0;
//        target.position.x = x;
//        target.position.y = y;
//        target.position.z = z;
//        planner.move_group->setPoseTarget(target);
//        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//        start=clock();
//        moveit::core::MoveItErrorCode success_target=planner.move_group->plan(my_plan);
//        finish=clock();
//        if(success_target)
//        {
//            std::cout<<"OMPL Plan Time: "<<(double)(finish - start) * 1000 / CLOCKS_PER_SEC<< "ms"<<std::endl;
//            // planner.move_group.execute(my_plan);
//        }
//        sleep(1.0);
        start=clock();
        x = final_pos(0) + i * 0.06;
        y = final_pos(1);
        z = target_pos[i](2)+0.14;
        OriMat = orientationMatrix(x, y, z, yaw, pitch, roll);
        joint_goal = ReverseKinematics_Collision(planner.dh_alpha,planner.dh_a,planner.dh_d,planner.dh_theta,OriMat,scene,planner.joint_model_group);
        planner.start_conf = planner.arm_pos;
//        std::cout<<joint_goal.transpose()*180/M_PI<<std::endl;
        if(joint_goal.size()==6)
            planner.goal_conf = joint_goal;
        else
            planner.goal_conf=planner.start_conf;
        result_plan = planner.MyPlan(scene);
        finish = clock();
        if(print_flag==1)
            std::cout << "MyPlan" << "   " << result_plan(0) << "   " << result_plan(1)<<"ms"<< std::endl;
//            std::cout << "MyPlan" << "   " << result_plan(0) << "   " << result_plan(1)<<"ms" << "   " << result_plan(2) << "   "<< result_plan(3) << "   " << result_plan(4)<< std::endl;
        if(result_plan(0)==1)
        {
            success_num += 1;
            planner.arm_pos = planner.goal_conf;
            avg_time+=(double)(finish - start) * 1000 / CLOCKS_PER_SEC;
            nodes+=result_plan(2);
            fail_nodes+=result_plan(4);
            plan_time+=result_plan(1);
            if(execute_flag==1) {
                planner.execute();
//                sleep(1.0);
                planner.move_group->detachObject(target_name);
                planner.move_group->clearPoseTargets();
                planner.move_group->setStartState(*planner.move_group->getCurrentState());
                planner.move_group->stop();
            }
        }
        else
            planner.arm_pos = planner.goal_conf;
    }
    avg_time = avg_time/success_num;
    if(print_flag==1) {
        std::cout << "Success Num: " << success_num << std::endl;
        std::cout << "Average Time: " << avg_time << " ms" << std::endl;
    }
    VectorXd result(4);
    result<<success_num,avg_time*success_num,nodes,fail_nodes;
    return result;

}

}   //namespace rrt_planner
