//
// Created by robert on 9/4/22.
//
#include "header.h"
#include "planner.h"
#include "env.h"
#include "interpolation.h"

using namespace rrt_planner;
using namespace std;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "dynamic_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;
    srand(time(nullptr));

    ros::Publisher pub = n.advertise<geometry_msgs::Pose>("chatter", 1000);
    ros::Rate loop_rate(10);
    MyPlanner my_planner(n);
    sleep(3.0);
    int count = 0;
    std::vector<std::string> object_ids;
	object_ids.push_back("obs");
    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject obs;
    obs.id = "obs";
    obs.header.frame_id = my_planner.move_group->getPlanningFrame();
    obs.header.stamp = ros::Time::now();

    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;
    // 设置障碍物的位置
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  1.0;
    pose.position.y = -0.7;
    pose.position.z =  0.4;

    // 将障碍物的属性、位置加入到障碍物的实例中
    obs.primitives.push_back(primitive);
    obs.primitive_poses.push_back(pose);
    obs.operation = obs.ADD;

    // 创建一个障碍物的列表，把之前创建的障碍物实例加入其中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(obs);

    // 所有障碍物加入列表后（这里只有一个障碍物），再把障碍物加入到当前的情景中，如果要删除障碍物，使用removeCollisionObjects(collision_objects)
    // my_planner.planning_scene->addCollisionObjects(collision_objects);
	
	// obs.primitives.clear();//Clear primitives required for MOVE operation (Only to avoid a warning)
	// obs.operation = obs.MOVE; //change operation to MOVE
	
    geometry_msgs::Pose pose_moving;
	pose_moving.orientation.w = 1.0;
    pose_moving.position.x =  1.0;
    pose_moving.position.y = -0.7;
    pose_moving.position.z =  0.4;
    double step = 0.1;

//     VectorXd conf_s(6),conf_g(6);
//     //Simple Experiments
//     conf_s<<-36,-63,-48,118,36,0;
//     conf_g<<58,-59,-56,10,-58,9;
// //        //Complex Experimets
// //        conf_s<<-160,-47,-42,102,159,18;
// //        conf_g<<55,-71,101,-94,-72,-22;
//     conf_s=conf_s*M_PI/180;
//     conf_g=conf_g*M_PI/180;

    // my_planner.planning_scene->addCollisionObjects(collision_objects);
    // planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    //         "robot_description");
    // monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
    // planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
    // ps->getCurrentStateNonConst().update();
    // planning_scene::PlanningScenePtr scene = ps->diff();
    // scene->decoupleParent();

    while (ros::ok()) {
        collision_objects.clear(); 
        
        // obs.id = "obs";
        pose_moving.position.x =  pose_moving.position.x + step;
        if(pose_moving.position.x > 2.0 || pose_moving.position.x <- 2.0)
            step = -step;
        obs.primitives.clear();
        obs.primitives.push_back(primitive);
        obs.primitive_poses.clear();
        obs.primitive_poses.push_back(pose_moving);
        // obs.operation = obs.MOVE; //change operation to MOVE
        collision_objects.push_back(obs);
        my_planner.planning_scene->applyCollisionObjects(collision_objects); 
        //applyCollisionObjects apply changes to planning interface

        ros::Duration(0.5).sleep(); //ZzZzZ 
        //publish topic:position of moving obstacle
        pub.publish(pose_moving);
        // my_planner.start_conf=my_planner.arm_pos;
        // // my_planner.goal_conf<<-0.625371,-0.788809,-1.29437,2.08364,0.625325,-0.000457022;
        // my_planner.goal_conf=conf_s;
        // my_planner.MyPlan(scene);
        // my_planner.execute();    
    }
    my_planner.planning_scene->removeCollisionObjects(object_ids);
    ros::shutdown();
    return 0;
}
