#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import math

def move_obstacle_in_circle():
    # 初始化ROS节点
    rospy.init_node('obstacle_circle_mover')

    # 创建一个SetModelState服务的客户端
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # 障碍物的名称
    model_name = 'moving_sphere'  # 替换为你的障碍物模型名称

    # 圆形轨迹的参数
    radius = 1.0  # 圆的半径
    center_x = 0.0  # 圆心的x坐标
    center_y = 0.0  # 圆心的y坐标
    height = 1.0  # 障碍物的z坐标
    angular_speed = 0.5  # 角速度，单位：弧度/秒
    rate = rospy.Rate(30)  # 控制循环的频率

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

        # 控制循环频率
        rate.sleep()

if __name__ == '__main__':
    try:
        move_obstacle_in_circle()
    except rospy.ROSInterruptException:
        pass