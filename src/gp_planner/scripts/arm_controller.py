#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import *
from control_msgs.msg import *
import actionlib
from sensor_msgs.msg import JointState

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# global num_points, traj_pos, traj_vel, delta_t, control_inter
# num_points = 0
# traj_pos = []
# traj_vel = []
# delta_t = 6.0 / 10
# control_inter = 5

# def move():
#     #goal就是我们向发送的关节运动数据，实例化为FollowJointTrajectoryGoal()类
#     goal = FollowJointTrajectoryGoal()

#     #goal当中的trajectory就是我们要操作的，其余的Header之类的不用管
#     goal.trajectory = JointTrajectory()
#     #goal.trajectory底下一共还有两个成员，分别是joint_names和points，先给joint_names赋值
#     goal.trajectory.joint_names = JOINT_NAMES

#     #从joint_state话题上获取当前的关节角度值，因为后续要移动关节时第一个值要为当前的角度值
#     joint_states = rospy.wait_for_message("joint_states",JointState)
#     joints_pos = joint_states.position

#     #给trajectory中的第二个成员points赋值
#     #points中有四个变量，positions,velocities,accelerations,effort，我们给前三个中的全部或者其中一两个赋值就行了
#     goal.trajectory.points=[0]*num_points
#     for i in range(num_points):
#         traj_point = JointTrajectoryPoint()
#         traj_point.positions = traj_pos[i]
#         traj_point.velocities = traj_vel[i]
#         time_from_start = (i * delta_t / control_inter)
#         traj_point.time_from_start = time_from_start
#         goal.trajectory.points[i] = traj_point
    
#     #发布goal，注意这里的client还没有实例化，ros节点也没有初始化，我们在后面的程序中进行如上操作
#     client.send_goal(goal)
#     client.wait_for_result()
 
# def pub_traj():
#     global client
#     #实例化一个action的类，命名为client，与上述client对应，话题为/eff_joint_traj_controller/follow_joint_trajectory，消息类型为FollowJointTrajectoryAction
#     client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
#     print("Waiting for server...")
#     #等待server
#     client.wait_for_server()
#     print("Connect to server......")
#     #执行move函数，发布action
#     move()


# def planning_result_callback(data):
#     # 假设每个点有6个关节位置
#     point_size = 6
#     num_points = len(data.data) // point_size
#     for i in range(0, len(data.data), point_size):
#         point_msg = JointTrajectoryPoint()
#         traj_pos.append(data.data[i:i+point_size])
#         traj_vel.append(data.data[i+point_size:i+2*point_size])
#     pub_traj()

# if __name__ == '__main__':
#     rospy.init_node('trajectory_publisher')
#     planning_result_sub = rospy.Subscriber('/local_planning_result', Float64MultiArray, planning_result_callback)
#     while not rospy.is_shutdown():
#         rospy.spin()

class TrajectoryPublisher:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()
        rospy.Subscriber('/local_planning_result', Float64MultiArray, self.planning_result_callback)
        self.traj_pos = []
        self.traj_vel = []
        self.num_points = 0
        self.delta_t = 6.0 / 10
        self.control_inter = 5

    def planning_result_callback(self, data):
        point_size = 6
        self.num_points = len(data.data) // (point_size * 2)
        self.traj_pos = []
        self.traj_vel = []
        for i in range(0, len(data.data), point_size * 2):
            self.traj_pos.append(data.data[i:i+point_size])
            self.traj_vel.append(data.data[i+point_size:i+point_size*2])
        self.pub_traj()

    def pub_traj(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_NAMES

        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        goal.trajectory.points = [JointTrajectoryPoint() for _ in range(self.num_points)]
        for i in range(self.num_points):
            traj_point = JointTrajectoryPoint()
            traj_point.positions = self.traj_pos[i]
            traj_point.velocities = self.traj_vel[i]
            time_from_start = rospy.Duration(i * self.delta_t / self.control_inter)
            traj_point.time_from_start = time_from_start
            goal.trajectory.points[i] = traj_point

        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('trajectory_publisher')
    publisher = TrajectoryPublisher()
    rospy.spin()