#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_commander import PlanningSceneInterface
import math

# # 回调函数，处理接收到的ModelStates消息
# def model_states_callback(msg):
#     for name in msg.name:
#         if name == "my_obstacle":  # 替换为你的障碍物模型名称
#             # 获取障碍物的位置和姿态
#             pose = msg.pose[msg.name.index(name)]
#             print("Obstacle Position:")
#             print("X:", pose.position.x)
#             print("Y:", pose.position.y)
#             print("Z:", pose.position.z)

# # 订阅/gazebo/model_states话题
# rospy.Subscriber("/gazebo/model_states", ModelState, model_states_callback)


def move_obstacle_in_circle():
    # 初始化ROS节点
    rospy.init_node('obstacle_circle_mover')

    # 创建一个SetModelState服务的客户端
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # 障碍物的名称
    model_name = 'moving_sphere'  # 替换为你的障碍物模型名称

    # 圆形轨迹的参数
    radius = 0.6  # 圆的半径
    center_x = 0.0  # 圆心的x坐标
    center_y = 0.0  # 圆心的y坐标
    height = 0.7  # 障碍物的z坐标
    angular_speed = 0.5  # 角速度，单位：弧度/秒
    rate = rospy.Rate(30)  # 控制循环的频率

    # 创建MoveIt PlanningSceneInterface对象
    planning_scene_interface = PlanningSceneInterface()

    while not rospy.is_shutdown():
        # 计算障碍物当前的位置
        theta = rospy.get_time() * angular_speed
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)

        # 设置障碍物的位姿
        model_pose = Pose()
        model_pose.position.x = x
        model_pose.position.y = y
        model_pose.position.z = height
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

        # 控制循环频率
        rate.sleep()

if __name__ == '__main__':
    try:
        move_obstacle_in_circle()
    except rospy.ROSInterruptException:
        pass