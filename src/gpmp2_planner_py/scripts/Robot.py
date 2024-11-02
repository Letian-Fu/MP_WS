#!/usr/bin/env python
# coding:utf-8
import numpy as np
import rospy

from gpmp2 import *
from gtsam.gtsam import Pose3, Rot3
from gtsam import Point3


class Robot:
	def __init__(self):
		# print ("Loading Robot Parameters.")
		flag = rospy.has_param('robot/DOF')
		if flag == True:
			self.DOF = int(rospy.get_param('robot/DOF'))
			# print ("Robot DOF:")
			# print (self.DOF)
			flag = False
		else:
			print ("No DOF Data Loaded!")

		flag = rospy.has_param('robot/arm_base/orientation')
		if flag == True:
			self.orientation = np.asarray(rospy.get_param('robot/arm_base/orientation'))
			# print ("Robot Orientation:")
			# print (self.orientation)
			flag = False
		else:
			print ("No orientation Data Loaded! Set Default Orientation!")
			self.orientation = np.asarray([0, 0, 0, 1])
			print ("Robot Orientation:")
			print (self.orientation)

		flag = rospy.has_param('robot/arm_base/position')
		if flag == True:
			self.position = np.asarray(rospy.get_param('robot/arm_base/position'))
			# print ("Robot Position:")
			# print (self.position)
			flag = False
		else:
			print ("No Position Data! Set Default Position!")
			self.position = np.asarray([0, 0, 0])
			print ("Robot Position:")
			print (self.position)
		self.arm_base = Pose3(
			Rot3.Quaternion(self.orientation[3], self.orientation[0], self.orientation[1], self.orientation[2]),
			Point3(self.position))
		# print("arm base")
		# print (self.arm_base)

		flag = rospy.has_param('robot/DH')
		if flag == True:
			self.a = np.asarray(rospy.get_param('robot/DH/a'))
			self.alpha = np.asarray(rospy.get_param('robot/DH/alpha'))
			self.d = np.asarray(rospy.get_param('robot/DH/d'))
			self.theta = np.asarray(rospy.get_param('robot/DH/theta'))
			# print("Robot DH Data:")
			# print("a:")
			# print (self.a)
			# print("alpha:")
			# print (self.alpha)
			# print("d:")
			# print (self.d)
			# print("theta:")
			# print (self.theta)
			flag = False
		else:
			print ("No DH Data!")

		self.abstract_arm = Arm(self.DOF, self.a, self.alpha, self.d, self.arm_base, self.theta)
		flag = rospy.has_param('robot/spheres')
		if flag == True:
			# self.spheres_data = np.asarray(rospy.get_param('robot/spheres'))
			self.js = np.asarray(rospy.get_param('robot/spheres/js'))
			self.xs = np.asarray(rospy.get_param('robot/spheres/xs'))
			self.ys = np.asarray(rospy.get_param('robot/spheres/ys'))
			self.zs = np.asarray(rospy.get_param('robot/spheres/zs'))
			self.rs = np.asarray(rospy.get_param('robot/spheres/rs'))
			# print("Robot spheres theta:")
			# print("js:")
			# print self.js
			# print("xs:")
			# print self.xs
			# print("ys:")
			# print self.ys
			# print("zs:")
			# print self.zs
			# print("rs:")
			# print self.rs
			flag = False
		else:
			print ("No Spheres Data!")
		# self.spheres_data = [
		# 	[0, 0.0, -0.1, 0.00, 0.08],
		# 	[0, 0.0, 0.0, 0.00, 0.08],
		# 	[1, 0.0, 0.0, 0.10, 0.06],
		# 	[1, 0.105, 0.0, 0.10, 0.06],
		# 	[1, 0.210, 0.0, 0.10, 0.06],
		# 	[1, 0.315, 0.0, 0.10, 0.06],
		# 	[1, 0.420, 0.0, 0.10, 0.06],
		# 	[1, 0.0, 0.0, 0.0, 0.08],
		# 	[2, 0.11, 0.0, 0.00, 0.06],
		# 	[2, 0.22, 0.0, 0.00, 0.06],
		# 	[2, 0.0, 0.0, 0.00, 0.08],
		# 	[3, 0.0, 0.0, 0.00, 0.07],
		# 	[4, 0.0, 0.0, 0.00, 0.06],
		# 	[5, 0.0, 0.0, 0.00, 0.06],
		# ]
		# self.spheres_data = np.asarray(self.spheres_data)
		nr_body = self.js.shape[0]
		# print nr_body
		self.spheres_data = []
		for i in range(0, nr_body):
			self.spheres_data.append([])
			self.spheres_data[i] = np.zeros(5)
			self.spheres_data[i][0] = self.js[i]
			self.spheres_data[i][1] = self.xs[i]
			self.spheres_data[i][2] = self.ys[i]
			self.spheres_data[i][3] = self.zs[i]
			self.spheres_data[i][4] = self.rs[i]
		self.spheres_data = np.asarray(self.spheres_data)
		# print (self.spheres_data)
		sphere_vec = BodySphereVector()
		for i in range(nr_body):
			sphere_vec.push_back(
				BodySphere(
					int(self.spheres_data[i, 0]), self.spheres_data[i, 4], Point3(self.spheres_data[i, 1:4])
				)
			)
			# print self.spheres_data[i, 0]
			# print self.spheres_data[i, 4]
			# # print self.spheres_data[i, 1:4]
			# print Point3(self.spheres_data[i, 1:4])
		self.arm = ArmModel(self.abstract_arm, sphere_vec)
		# self.spheres_data = BodySphereVector()
		# for i in range(0, self.js.size):
		# 	self.spheres_data.push_back(BodySphere(self.js[i], self.rs[i], Point3(self.xs[i], self.ys[i], self.zs[i])))
		#
		# self.arm = ArmModel(Arm(self.DOF, self.a, self.alpha, self.d, self.arm_base, self.theta), self.spheres_data)
		# print ("Finish Robot Loading")

	def getDOF(self):
		return self.DOF



