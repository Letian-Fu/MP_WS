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
from visualization_msgs.msg import Marker


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
    rate_ = 50
    rate = rospy.Rate(rate_)  # 控制循环的频率
    # 创建发布者
    pub = rospy.Publisher('/obstacle_info', Float64MultiArray, queue_size=10)
    pub_mpc = rospy.Publisher('/obstacle_info_mpc', Float64MultiArray, queue_size=10)
    pub_marker = rospy.Publisher('/obstacle_trajectory', Marker, queue_size=10)

    # 定义Marker消息
    marker = Marker()
    marker.header.frame_id = "world"  # 使用 Gazebo 的世界坐标系
    marker.type = Marker.LINE_STRIP  # 选择轨迹类型为线条
    marker.action = Marker.ADD
    marker.scale.x = 0.02  # 线条宽度
    marker.color.r = 0.0  # 轨迹颜色为红色
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0  # 颜色透明度
    marker.pose.orientation.w = 1.0  # 轨迹的全局姿态
    marker.points = []  # 存储轨迹点的数组
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
    floor_pose.position.z = -0.08  # 长方体中心在-0.15，高度0.1，所以底部在-0.2
    floor_pose.orientation.w = 1.0

    floor.primitives.append(primitive)
    floor.primitive_poses.append(floor_pose)
    floor.operation = CollisionObject.ADD

    # # 应用碰撞对象到规划场景
    planning_scene_interface.add_object(floor)
    obs_size = 0.10

    # 运动参数
    mode = 'vertical'  # 'helical', 'linear', 'vertical', 'horizontal', or 'combined'
    if mode == 'vertical':
        # 上下往复运动参数
        start = np.array([-0.6, 0.0, 0.3])  # 垂直方向起点
        end = np.array([-0.6, 0.0, 0.9])    # 垂直方向终点
        linear_speed = 0.3  # 匀速运动的速度 (m/s)
    elif mode == 'horizontal':
        # 水平往复运动参数
        start = np.array([-1.0, 0.0, 0.5])  # 水平方向起点
        end = np.array([1.0, 0.0, 0.5])    # 水平方向终点
        linear_speed = 0.2  # 匀速运动的速度 (m/s)
    # 计算运动方向和周期
    direction = (end - start) / np.linalg.norm(end - start)  # 单位方向向量
    position = start.copy()  # 障碍物的初始位置
    moving_forward = True  # 初始状态为“向终点移动”
    distance = np.linalg.norm(end - start)
    period = 2*distance / linear_speed
    delta_t = 1.0 / rate_                                      # 时间步长（假设帧率为 30 FPS）
    rospy.loginfo("Obstacle will move back and forth between start and end.")
    t = 0
    while not rospy.is_shutdown():
        # 计算当前时间在周期中的相对位置
        t = (t + delta_t) % period  # 保证时间在 [0, period) 内循环
        if t < period / 2:
            # 前半周期：从起点到终点
            direction = (end - start) / np.linalg.norm(end - start)  # 起点到终点的单位方向
            position = start + direction * linear_speed * t
        else:
            # 后半周期：从终点返回起点
            t_back = t - period / 2  # 后半周期的相对时间
            direction = (start - end) / np.linalg.norm(start - end)  # 终点到起点的单位方向
            position = end + direction * linear_speed * t_back
        # if moving_forward:
        #     # 向终点移动
        #     position += direction * linear_speed / 30  # 每帧移动一点
        #     # 检查是否到达终点
        #     if np.linalg.norm(position - end) < 1e-3:  # 允许一个小误差
        #         position = end.copy()  # 确保位置精确为终点
        #         direction = -direction  # 反转方向
        #         moving_forward = False  # 切换为返回阶段
        # else:
        #     # 返回起点
        #     position += direction * linear_speed / 30  # 每帧移动一点
        #     # 检查是否到达起点
        #     if np.linalg.norm(position - start) < 1e-3:  # 允许一个小误差
        #         position = start.copy()  # 确保位置精确为起点
        #         direction = -direction  # 反转方向
        #         moving_forward = True  # 切换为前进阶段
        # 位置解包
        x, y, z = position
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
        obstacle_info.data = [x, y, z, obs_size, linear_speed, direction[0], direction[1],direction[2]]
        print(f"Linear Speed: {linear_speed:.2f}, Direction: {direction}")
        print(f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, obs_size: {obs_size:.2f}")
        pub.publish(obstacle_info)
        pub_mpc.publish(obstacle_info)
        rospy.logdebug(f"Obstacle position: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

        # 添加当前位置到轨迹点
        point = Pose()
        point.position.x = x
        point.position.y = y
        point.position.z = z
        marker.points.append(point.position)

        # 限制轨迹点的数量（防止内存过大）
        if len(marker.points) > 1000:  # 保留最近 1000 个点
            marker.points.pop(0)

        # 发布轨迹
        marker.header.stamp = rospy.Time.now()
        pub_marker.publish(marker)
        # 控制循环频率
        rate.sleep()

if __name__ == '__main__':

    try:
        move_obstacle()
    except rospy.ROSInterruptException:
        pass