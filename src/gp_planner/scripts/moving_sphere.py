#!/usr/bin/env python
import rospy
import math
import numpy as np
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped


def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
        return v
    return v / norm

def move_obstacle():
    # 初始化ROS节点
    rospy.init_node('obstacle_circle_mover')
    # 创建一个SetModelState服务的客户端
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    # 障碍物的名称
    model_name = 'moving_sphere'  # 替换为你的障碍物模型名称
    rate = rospy.Rate(15)  # 控制循环的频率
    # 创建发布者
    pub = rospy.Publisher('/obstacle_info', Float64MultiArray, queue_size=10)
    # 创建MoveIt PlanningSceneInterface对象
    planning_scene_interface = PlanningSceneInterface()

    # 更新MoveIt中的规划场景(地板)
    floor = CollisionObject()
    floor.header.frame_id = "world"
    floor.header.stamp = rospy.Time.now()
    floor.id = "floor"

    # 假设障碍物是一个长方体
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions=[3.0, 3.0, 0.1]

    # 设置长方体的位置
    floor_pose = Pose()
    floor_pose.position.x = 0.0
    floor_pose.position.y = 0.0
    floor_pose.position.z = -0.15 - 0.05  # 长方体中心在-0.15，高度0.1，所以底部在-0.2
    floor_pose.orientation.w = 1.0

    floor.primitives.append(primitive)
    floor.primitive_poses.append(floor_pose)
    floor.operation = CollisionObject.ADD

    # # 应用碰撞对象到规划场景
    planning_scene_interface.add_object(floor)


    obs_size = 0.08

    # 运动参数
    mode = 'vertical'  # 'helical', 'linear', 'vertical', 'horizontal', or 'combined'
    if mode == 'helical':
        # 螺旋运动参数
        radius = 1.0  # 圆的半径
        height_amplitude = 0.75  # 上下往复运动的振幅
        height_frequency = 0.5  # 上下往复运动的频率
        angular_speed = 0.5  # 角速度
    elif mode == 'linear':
        # 水平往复运动参数
        amplitude_x = 1.5  # x方向往复距离的一半
        amplitude_y = 1.5  # y方向往复距离的一半
        frequency = 0.5  # 往复频率
    elif mode == 'vertical':
        # 上下往复运动参数
        amplitude_z = 0.3  # z方向往复距离的一半
        frequency_z = 0.05  # 往复频率
    elif mode == 'horizontal':
        # 水平往复运动参数
        amplitude_x = 1.5  # x方向往复距离的一半
        amplitude_y = 1.5  # y方向往复距离的一半
        frequency = 0.5  # 往复频率
    elif mode == 'combined':
        # 组合运动参数
        radius = 1.0  # 圆周运动半径
        height_amplitude = 0.75  # 上下往复运动的振幅
        height_frequency = 0.5  # 上下往复运动的频率
        angular_speed = 0.5  # 角速度

    t = 0.0
    initial_height = 0.45
    while not rospy.is_shutdown():
        t += 0.05  # 增加时间步长，以便观察运动

        if mode == 'helical':
            # 螺旋运动
            theta = angular_speed * t
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)
            z = initial_height+height_amplitude * math.sin(2 * math.pi * height_frequency * t)
            linear_speed = np.sqrt((radius * angular_speed) ** 2 + (height_amplitude * 2 * math.pi * height_frequency * math.cos(2 * math.pi * height_frequency * t)) ** 2)
            direction = normalize_vector(np.array([-math.sin(theta), math.cos(theta), 2 * math.pi * height_frequency * math.cos(2 * math.pi * height_frequency * t)]))
        elif mode == 'linear':
            # 水平往复运动
            x = amplitude_x * math.sin(2 * math.pi * frequency * t)
            y = amplitude_y * math.sin(2 * math.pi * frequency * t)
            z = initial_height
            linear_speed = np.sqrt((amplitude_x * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)) ** 2 +
                                   (amplitude_y * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)) ** 2)
            direction = normalize_vector(np.array([amplitude_x * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t),
                                                   amplitude_y * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t), 0.0]))
        elif mode == 'vertical':
            # 上下往复运动
            x = 0.0
            y = -0.5
            z = initial_height + amplitude_z * math.sin(2 * math.pi * frequency_z * t)
            linear_speed = amplitude_z * 2 * math.pi * frequency_z * math.cos(2 * math.pi * frequency_z * t)
            direction = normalize_vector(np.array([0.0, 0.0, 2 * math.pi * frequency_z * math.cos(2 * math.pi * frequency_z * t)]))
            # z = initial_height
            # linear_speed = 0
            # direction = np.array([0.0, 0.0, 0])
        elif mode == 'horizontal':
            # 水平往复运动
            x = amplitude_x * math.sin(2 * math.pi * frequency * t)
            y = amplitude_y * math.sin(2 * math.pi * frequency * t)
            z = 0.0
            linear_speed = np.sqrt((amplitude_x * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)) ** 2 +
                                   (amplitude_y * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)) ** 2)
            direction = normalize_vector(np.array([amplitude_x * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t),
                                                   amplitude_y * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t), 0.0]))
        elif mode == 'combined':
            # 组合运动：圆周运动 + 上下往复运动
            theta = angular_speed * t
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)
            z = initial_height+height_amplitude * math.sin(2 * math.pi * height_frequency * t)
            linear_speed = np.sqrt((radius * angular_speed) ** 2 + (height_amplitude * 2 * math.pi * height_frequency * math.cos(2 * math.pi * height_frequency * t)) ** 2)
            direction = normalize_vector(np.array([-math.sin(theta), math.cos(theta), 2 * math.pi * height_frequency * math.cos(2 * math.pi * height_frequency * t)]))

        # 限制位置在指定范围内
        x = np.clip(x, -1.5, 1.5)
        y = np.clip(y, -1.5, 1.5)
        z = np.clip(z, 0, 1.5)

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
        primitive.dimensions.append(obs_size)

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(model_pose)
        collision_object.operation = CollisionObject.ADD

        # # 应用碰撞对象到规划场景
        planning_scene_interface.add_object(collision_object)

        # 发布障碍物的位置和大小信息
        obstacle_info = Float64MultiArray()
        obstacle_info.data = [x, y, z, obs_size+0.05, linear_speed, direction[0], direction[1],direction[2]]
        pub.publish(obstacle_info)

        # 控制循环频率
        rate.sleep()

if __name__ == '__main__':
    try:
        move_obstacle()
    except rospy.ROSInterruptException:
        pass