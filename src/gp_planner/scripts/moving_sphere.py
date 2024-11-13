#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
import math
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
import numpy as np


def move_obstacle():
    # 初始化ROS节点
    rospy.init_node('obstacle_circle_mover')

    # 创建一个SetModelState服务的客户端
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # 障碍物的名称
    model_name = 'moving_sphere'  # 替换为你的障碍物模型名称
    rate = rospy.Rate(20)  # 控制循环的频率

    # 创建发布者
    pub = rospy.Publisher('/obstacle_info', Float64MultiArray, queue_size=10)

    # 创建MoveIt PlanningSceneInterface对象
    planning_scene_interface = PlanningSceneInterface()
    obs_size = 0.5

    # 运动参数
    mode = 'helical'  # 'helical', 'linear', or 'parabolic'
    if mode == 'helical':
        # 螺旋运动参数
        radius = 1.0  # 圆的半径
        height_per_rev = 0.1  # 每转一圈上升的高度
        angular_speed = 0.5  # 角速度
    elif mode == 'linear':
        # 直线往复运动参数
        amplitude_x = 1.0  # x方向往复距离的一半
        amplitude_y = 0.5  # y方向往复距离的一半
        frequency = 0.5  # 往复频率
        direction = np.array([1.0, 0.0, 0.0])  # 运动方向
    elif mode == 'parabolic':
        # 抛物线运动参数
        amplitude_x = 1.0  # x方向抛物线高度
        amplitude_y = 0.5  # y方向抛物线高度
        frequency = 0.5  # 抛物线频率
        direction = np.array([1.0, 0.0, 0.0])  # 抛物线方向

    while not rospy.is_shutdown():
        t = rospy.get_time()
        if mode == 'helical':
            # 螺旋运动
            theta = angular_speed * t
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)
            z = height_per_rev * theta / (2 * math.pi)
            linear_speed = np.sqrt((radius * angular_speed) ** 2 + (height_per_rev * angular_speed / (2 * math.pi)) ** 2)
            direction = np.array([-math.sin(theta), math.cos(theta), height_per_rev / (2 * math.pi)])
        elif mode == 'linear':
            # 直线往复运动
            x = amplitude_x * math.sin(2 * math.pi * frequency * t)
            y = amplitude_y * math.sin(2 * math.pi * frequency * t)
            z = 0.0
            linear_speed = np.sqrt((amplitude_x * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)) ** 2 +
                                   (amplitude_y * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)) ** 2)
            direction = np.array([amplitude_x * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t),
                                  amplitude_y * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t), 0.0])
        elif mode == 'parabolic':
            # 抛物线运动
            x = amplitude_x * math.sin(2 * math.pi * frequency * t)
            y = amplitude_y * math.sin(2 * math.pi * frequency * t) ** 2
            z = 0.0
            linear_speed = np.sqrt((amplitude_x * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)) ** 2 +
                                   (2 * amplitude_y * 2 * math.pi * frequency * math.sin(2 * math.pi * frequency * t) * math.cos(2 * math.pi * frequency * t)) ** 2)
            direction = np.array([amplitude_x * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t),
                                  2 * amplitude_y * 2 * math.pi * frequency * math.sin(2 * math.pi * frequency * t) * math.cos(2 * math.pi * frequency * t), 0.0])

        # 设置障碍物的位姿
        model_pose = Pose()
        model_pose.position.x = x
        model_pose.position.y = y
        model_pose.position.z = z
        model_pose.orientation.x = 0
        model_pose.orientation.y = 0
        model_pose.orientation.z = 0
        model_pose.orientation.w = 1

        # 创建ModelState消息并设置位姿
        state = ModelState()
        state.model_name = model_name
        state.pose = model_pose

        # 调用服务设置障碍物的位姿
        set_state(state)

        # 更新MoveIt中的规划场景
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = rospy.Time.now()
        collision_object.id = model_name

        # 假设障碍物是一个球体
        primitive = SolidPrimitive()
        primitive.type = primitive.SPHERE
        primitive.dimensions.append(0.1)

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(model_pose)
        collision_object.operation = CollisionObject.ADD

        # 应用碰撞对象到规划场景
        planning_scene_interface.add_object(collision_object)

        # 发布障碍物的位置和大小信息
        obstacle_info = Float64MultiArray()
        obstacle_info.data = [x, y, z, obs_size, linear_speed, direction[0], direction[1],direction[2]]
        pub.publish(obstacle_info)

        # 控制循环频率
        rate.sleep()

if __name__ == '__main__':
    try:
        move_obstacle()
    except rospy.ROSInterruptException:
        pass