# Motion_Planning

运行
```
roslaunch gp_planner cr5_dobot_bringup.launch

roslaunch dobot_moveit moveit.launch

roslaunch gp_planner cr5_obstacle_update.launch

roslaunch gp_planner cr5_plan_test.launch

```

感知 

```
conda activate yolo
rosrun gp_planner realtime_predict.py

rosrun gp_planner get_depth

```