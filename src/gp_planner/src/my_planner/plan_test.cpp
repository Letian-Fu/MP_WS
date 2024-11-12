#include "my_planner/MyPlanner.h"
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "plan_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;
    MyPlanner my_planner(n);
    while (ros::ok()) {
        ros::spinOnce();
        // my_planner.execute();
    }


    return 0;
}