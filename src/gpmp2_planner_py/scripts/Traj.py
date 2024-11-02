#!/usr/bin/env python
# coding:utf-8
import numpy as np
import rospy
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gpmp2 import *
from gtsam.gtsam import Values, symbol

import Problem
import Robot


class Traj:
	def __init__(self):
		flag = rospy.has_param('robot/est_traj_pub_topic')
		if flag == True:
			self.est_traj_pub_topic = rospy.get_param('robot/est_traj_pub_topic')
			self.est_traj_pub = rospy.Publisher(self.est_traj_pub_topic, JointTrajectory, queue_size=1)
			# print ("Robot est_traj_pub_topic:")
			# print (self.est_traj_pub_topic)
			flag = False
		else:
			print ("No Robot trajectory_control_topic")

		flag = rospy.has_param('robot/plan_traj_pub_topic')
		if flag == True:
			self.plan_traj_pub_topic = rospy.get_param('robot/plan_traj_pub_topic')
			self.plan_traj_pub = rospy.Publisher(self.plan_traj_pub_topic, JointTrajectory, queue_size=1)
			# print ("Robot trajectory_control_topic:")
			# print (self.plan_traj_pub_topic)
			flag = False
		else:
			print ("No Robot plan_traj_pub_topic")

		flag = rospy.has_param('robot/trajectory_control_topic')
		if flag == True:
			self.trajectory_control_topic = rospy.get_param('robot/trajectory_control_topic')
			# print ("Robot trajectory_control_topic:")
			# print (self.trajectory_control_topic)
			flag = False
		else:
			print ("No Robot trajectory_control_topic")

		self.traj_client = SimpleActionClient(self.trajectory_control_topic, FollowJointTrajectoryAction)
		if not self.traj_client.wait_for_server(rospy.Duration(5)):
			print("Waiting for trajectory_control server...")
			if not self.traj_client.wait_for_server(rospy.Duration(5)):
				print("Cannot find trajectory_control server")
				print("Quiting")
				rospy.signal_shutdown("Cannot find trajectory_control server")
		# else:
			print ("Robot traj_client:")
			print (self.traj_client)

		self.traj = FollowJointTrajectoryGoal()
		# self.tarj.trajectory = JointTrajectory()
		self.traj.trajectory.points = JointTrajectoryPoint()

		flag = rospy.has_param('robot/arm_joint_names')
		if flag == True:
			self.arm_joint_names = rospy.get_param('robot/arm_joint_names')
			# print ("Robot arm_joint_names:")
			# print (self.arm_joint_names)
			self.traj.trajectory.joint_names = self.arm_joint_names
			# print ("Robot traj:")
			# print self.traj
			flag = False
		else:
			print ("No Robot arm_joint_names")

	def initializeTrajectory(self, init_values, problem):
		init_values.clear()
		# print("Initializing trajectory.")
		# avg_vel = (problem.goal_conf - problem.start_conf)/problem.total_time
		# print(problem.goal_conf)
		# print(problem.start_conf)
		avg_vel = (problem.goal_conf - problem.start_conf) / (problem.total_step - 1)
		for i in range(0, problem.total_step):
			ratio = np.double(i)/np.double(problem.total_step - 1)
			conf = (1.0 - ratio)*problem.start_conf + ratio * problem.goal_conf
			init_values.insert(symbol("x", i), conf)
			init_values.insert(symbol("v", i), avg_vel)
		# print ("Initializing Finished")


	def executeTrajectory(self, exec_values, problem, exec_step):
		DOF = problem.robot.getDOF
		self.traj.trajectory.points = [0]*exec_step
		for i in range(0, exec_step):
			conf = exec_values.atVector(symbol("x", i))
			vel = exec_values.atVector(symbol("v", i))
			time = rospy.Duration(i * problem.delta_t/(problem.control_inter + 1))
			self.traj.trajectory.points[i] = JointTrajectoryPoint(positions=np.asarray(conf),
																  velocities=np.asarray(vel),
																  time_from_start=time)
		self.traj.trajectory.header.stamp = rospy.Time.now()
		self.traj_client.send_goal(self.traj)
		self.traj_client.wait_for_result()

	def publisEstimatedTrajectory(self, values, problem, step):
		est_traj = JointTrajectory()
		est_traj.points = [0] * (step + 1)
		for i in range(0, step + 1):
			conf = values.atVector(symbol("x", i))
			est_traj.points[i] = JointTrajectoryPoint(positions=conf)
		self.est_traj_pub.publish(est_traj)

	def publisPlannedTrajectory(self, values, problem, step):
		plan_traj = JointTrajectory()
		plan_traj.points = [0] * (problem.total_step - step)
		for i in range(step, problem.total_step):
			conf = values.atVector(symbol("x", i))
			plan_traj.points[i - step] = JointTrajectoryPoint(positions=conf)
		self.plan_traj_pub.publish(plan_traj)




