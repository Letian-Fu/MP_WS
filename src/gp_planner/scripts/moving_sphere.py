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
    rate_ = 20
    rate = rospy.Rate(rate_)  # 控制循环的频率
    # 创建发布者
    pub = rospy.Publisher('/obstacle_info', Float64MultiArray, queue_size=10)
    pub_mpc = rospy.Publisher('/obstacle_info_mpc', Float64MultiArray, queue_size=10)
    pub_marker = rospy.Publisher('/obstacle_trajectory', Marker, queue_size=10)

    # 定义Marker消息
    marker = Marker()
    # marker.header.frame_id = "world"  # 使用 Gazebo 的世界坐标系
    marker.header.frame_id = "base_link"  # 使用 Gazebo 的世界坐标系
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
    floor.header.frame_id = "base_link"
    floor.header.stamp = rospy.Time.now()
    floor.id = "floor"

    # 假设障碍物是一个长方体
    floor_primitive = SolidPrimitive()
    floor_primitive.type = floor_primitive.BOX
    floor_primitive.dimensions=[3.0, 3.0, 0.1]

    # 设置长方体的位置
    floor_pose = Pose()
    floor_pose.position.x = 0.0
    floor_pose.position.y = 0.0
    floor_pose.position.z = -0.08  # 长方体中心在-0.15，高度0.1，所以底部在-0.2
    floor_pose.orientation.w = 1.0

    floor.primitives.append(floor_primitive)
    floor.primitive_poses.append(floor_pose)
    floor.operation = CollisionObject.ADD

    # 应用碰撞对象到规划场景
    planning_scene_interface.add_object(floor)

    # # 更新MoveIt中的规划场景
    # shebei = CollisionObject()
    # shebei.header.frame_id = "base_link"
    # shebei.header.stamp = rospy.Time.now()
    # shebei.id = "shebei"

    # # 假设障碍物是一个长方体
    # shebei_primitive = SolidPrimitive()
    # shebei_primitive.type = shebei_primitive.BOX
    # shebei_primitive.dimensions=[0.25, 0.25, 0.1]

    # # 设置长方体的位置
    # shebei_pose = Pose()
    # shebei_pose.position.x = -0.95
    # shebei_pose.position.y = -0.105
    # shebei_pose.position.z = 0.05 
    # shebei_pose.orientation.w = 1.0


    # shebei.primitives.append(shebei_primitive)
    # shebei.primitive_poses.append(shebei_pose)
    # shebei.operation = CollisionObject.ADD

    # # 应用碰撞对象到规划场景
    # planning_scene_interface.add_object(shebei)

    # # 更新MoveIt中的规划场景(地板)
    # shebei2 = CollisionObject()
    # shebei2.header.frame_id = "base_link"
    # shebei2.header.stamp = rospy.Time.now()
    # shebei2.id = "shebei2"

    # # 假设障碍物是一个长方体
    # shebei2_primitive = SolidPrimitive()
    # shebei2_primitive.type = shebei2_primitive.BOX
    # shebei2_primitive.dimensions=[0.35, 0.35, 0.4]

    # # 设置长方体的位置
    # shebei2_pose = Pose()
    # shebei2_pose.position.x = -0.75
    # shebei2_pose.position.y = 0
    # shebei2_pose.position.z = 0.05 
    # shebei2_pose.orientation.w = 1.0


    # shebei2.primitives.append(shebei2_primitive)
    # shebei2.primitive_poses.append(shebei2_pose)
    # shebei2.operation = CollisionObject.ADD

    # # 应用碰撞对象到规划场景
    # planning_scene_interface.add_object(shebei2)

    # # 更新MoveIt中的规划场景(地板)
    # pingmu = CollisionObject()
    # pingmu.header.frame_id = "base_link"
    # pingmu.header.stamp = rospy.Time.now()
    # pingmu.id = "pingmu"

    # # 假设障碍物是一个长方体
    # pingmu_primitive = SolidPrimitive()
    # pingmu_primitive.type = pingmu_primitive.BOX
    # pingmu_primitive.dimensions=[0.3, 0.1, 0.3]

    # # 设置长方体的位置
    # pingmu_pose = Pose()
    # pingmu_pose.position.x = -0.42
    # pingmu_pose.position.y = -0.7
    # pingmu_pose.position.z = 0.15 
    # pingmu_pose.orientation.w = 1.0

    # pingmu.primitives.append(pingmu_primitive)
    # pingmu.primitive_poses.append(pingmu_pose)
    # pingmu.operation = CollisionObject.ADD

    # # 应用碰撞对象到规划场景
    # planning_scene_interface.add_object(pingmu)

    # # 更新MoveIt中的规划场景(地板)
    # zawu = CollisionObject()
    # zawu.header.frame_id = "base_link"
    # zawu.header.stamp = rospy.Time.now()
    # zawu.id = "zawu"

    # # 假设障碍物是一个长方体
    # zawu_primitive = SolidPrimitive()
    # zawu_primitive.type = zawu_primitive.BOX
    # zawu_primitive.dimensions=[0.4, 0.1, 0.3]

    # # 设置长方体的位置
    # zawu_pose = Pose()
    # zawu_pose.position.x = -0.25
    # zawu_pose.position.y = 0.7
    # zawu_pose.position.z = 0.15  # 长方体中心在-0.15，高度0.1，所以底部在-0.2
    # zawu_pose.orientation.w = 1.0

    # zawu.primitives.append(zawu_primitive)
    # zawu.primitive_poses.append(zawu_pose)
    # zawu.operation = CollisionObject.ADD

    # # 应用碰撞对象到规划场景
    # planning_scene_interface.add_object(zawu)


    obs_size = 0.10

    # 运动参数
    mode = 'vertical'  # 'helical', 'linear', 'vertical', 'horizontal', or 'combined'
    if mode == 'vertical':
        # 上下往复运动参数
        start = np.array([-0.6, 0.0, 0.2])  # 垂直方向起点
        end = np.array([-0.6, 0.0, 1.0])    # 垂直方向终点
        linear_speed = 0.6  # 匀速运动的速度 (m/s)
    elif mode == 'horizontal':
        # 水平往复运动参数
        start = np.array([-0.8, 0, 0.6])  # 水平方向起点
        end = np.array([-0.2, 0.0, 0.6])    # 水平方向终点
        linear_speed = 0.3  # 匀速运动的速度 (m/s)
    elif mode == 'diagonal':
        # **斜向往复运动参数**
        start = np.array([-0.5, -0.3, 0.3])  # 斜向起点
        end = np.array([-0.5, 0.3, 0.8])      # 斜向终点
        linear_speed = 0.3  # 匀速运动的速度 (m/s)
        # 计算运动方向和周期
    direction = (end - start) / np.linalg.norm(end - start)  # 单位方向向量
    position = start.copy()  # 障碍物的初始位置
    distance = np.linalg.norm(end - start)
    amplitude = distance / 2                                # 运动范围的一半
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
        # # 使用正弦函数计算障碍物位置
        # alpha = 0.5 * (1 - math.cos(2 * math.pi * t / period))  # 平滑系数 (0 到 1)
        # position = start + alpha * (end - start)

        # # 使用正弦函数的导数计算速度
        # velocity = direction * amplitude * (2 * math.pi / period) * math.cos(2 * math.pi * t / period)
        # linear_speed_actual = np.linalg.norm(velocity)  # 瞬时速度大小
        # linear_speed = linear_speed_actual
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
        collision_object.header.frame_id = "base_link"
        collision_object.header.stamp = rospy.Time.now()
        collision_object.id = model_name

        # 假设障碍物是一个球体
        primitive = SolidPrimitive()
        primitive.type = primitive.SPHERE
        primitive.dimensions.append(obs_size)

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(model_pose)
        collision_object.operation = CollisionObject.ADD

        # 应用碰撞对象到规划场景
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