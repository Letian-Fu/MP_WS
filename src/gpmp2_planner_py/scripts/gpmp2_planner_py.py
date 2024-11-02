#!/usr/bin/env python
# coding:utf-8
import sys
import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import MoveGroupActionGoal, CollisionObject
import moveit_commander
import pymoveit_core.planning_scene
import moveit_ros_planning_interface

from gpmp2 import *
from gtsam import *
from gtsam.gtsam import Values, symbol, Pose3, Rot3, noiseModel
from gtsam import Point3
from gpmp2.datasets.generate3Ddataset import Dataset, add_obstacle
from gpmp2.utils.signedDistanceField3D import signedDistanceField3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from gpmp2.utils.plot_utils import *
from scipy import ndimage

import Robot
import Problem
import Traj


class GPMP2Planner:
    def __init__(self):
        self.problem = Problem.Problem()
        self.traj = Traj.Traj()
        self.init_values = Values()
        self.batch_values = Values()
        self.exec_values = Values()
        self.replan_values = Values()
        self.plan_flag = 0
        self.replan_flag = 0
        self.replan_pose_idx = 0
        self.move_flag = 0
        self.replan_goal_conf = np.zeros(self.problem.robot.getDOF())
        self.isam_optimizer = ISAM2TrajOptimizer3DArm(self.problem.robot.arm, self.problem.sdf, self.problem.opt_setting)
        self.replan_sub = rospy.Subscriber("move_group/goal", MoveGroupActionGoal, queue_size=1000, callback=self.plancallback)
        self.col_flag = 0

        flag = rospy.has_param('robot/arm_state_topic')
        if flag == True:
            self.arm_state_topic = rospy.get_param('robot/arm_state_topic')
            self.arm_state_sub = rospy.Subscriber(self.arm_state_topic, JointState, queue_size=1, callback=self.armStateCallback)
            self.arm_pos = np.zeros(self.problem.robot.getDOF())
            self.arm_pos_time = rospy.Time.now()
            print("Robot arm_state_topic:")
            print(self.arm_state_topic)
            flag = False
        else:
            print("No Robot arm_state_topic")
        rospy.sleep(1)
        if not rospy.has_param("start_conf"):
            self.problem.start_conf = self.arm_pos
        print("GPMP2 Planner Initialize Finished")

    def optimize(self, start_conf, goal_conf, init_values):
        DOF = self.problem.robot.getDOF()
        pose_fix_model = noiseModel.Isotropic.Sigma(DOF, self.problem.fix_pose_sigma)
        vel_fix_model = noiseModel.Isotropic.Sigma(DOF, self.problem.fix_vel_sigma)
        ## init optimization
        graph = NonlinearFactorGraph()
        graph_obs = NonlinearFactorGraph()

        for i in range(total_time_step + 1):
            key_pos = symbol("x", i)
            key_vel = symbol("v", i)

            # priors
            if i == 0:
                graph.push_back(PriorFactorVector(key_pos, start_conf, pose_fix_model))
                graph.push_back(PriorFactorVector(key_vel, np.zeros(DOF), vel_fix_model))
            elif i == total_time_step:
                graph.push_back(PriorFactorVector(key_pos, goal_conf, pose_fix_model))
                graph.push_back(PriorFactorVector(key_vel, np.zeros(DOF), vel_fix_model))

            # GP priors and cost factor
            if i > 0:
                key_pos1 = symbol("x", i - 1)
                key_pos2 = symbol("x", i)
                key_vel1 = symbol("v", i - 1)
                key_vel2 = symbol("v", i)
                graph.push_back(
                    GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2,
                                               self.problem.delta_t, self.problem.Qc_model))

                # cost factor
                graph.push_back(
                    ObstacleSDFFactorArm(key_pos, self.problem.robot.arm, sdf, self.problem.cost_sigma,
                                         self.problem.epsilon))
                graph_obs.push_back(
                    ObstacleSDFFactorArm(key_pos, self.problem.robot.arm, sdf, self.problem.cost_sigma,
                                         self.problem.epsilon))

                # GP cost factor
                if check_inter > 0:
                    for j in range(1, self.problem.obs_check_inter + 1):
                        tau = j * (self.problem.total_time / self.problem.total_step * (
                                    self.problem.obs_check_inter + 1))
                        graph.push_back(
                            ObstacleSDFFactorGPArm(
                                key_pos1,
                                key_vel1,
                                key_pos2,
                                key_vel2,
                                self.problem.robot.arm,
                                self.problem.sdf,
                                self.problem.cost_sigma,
                                self.problem.epsilon,
                                self.problem.Qc_model,
                                self.problem.delta_t,
                                tau,
                            ))
                        graph_obs.push_back(
                            ObstacleSDFFactorGPArm(
                                key_pos1,
                                key_vel1,
                                key_pos2,
                                key_vel2,
                                self.problem.robot.arm,
                                self.problem.sdf,
                                self.problem.cost_sigma,
                                self.problem.epsilon,
                                self.problem.Qc_model,
                                self.problem.delta_t,
                                tau,
                            ))

        ## optimize!
        if self.problem.opt_type == "LM":
            parameters = LevenbergMarquardtParams()  # TODO: check why this fails
            parameters.setVerbosity("ERROR")
            # parameters.setVerbosityLM('LAMBDA');
            parameters.setlambdaInitial(1000.0)
            optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters)
        elif self.problem.opt_type == "Dogleg":
            parameters = DoglegParams()
            parameters.setVerbosity("ERROR")
            optimizer = DoglegOptimizer(graph, init_values, parameters)
        else:
            parameters = GaussNewtonParams()
            parameters.setVerbosity("ERROR")
            optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

        print(f"Initial Error = {graph.error(init_values)}\n")
        print(f"Initial Collision Cost: {graph_obs.error(init_values)}\n")

        # optimizer.optimizeSafely()
        result = optimizer.values()

        print(f"Error = {graph.error(result)}\n")
        print(f"Collision Cost End: {graph_obs.error(result)}\n")
        return result

    def interploteTrajectory(self, values):
        results = Values()
        inter_dt = self.problem.delta_t / (self.problem.control_inter + 1)
        result_idx = 0
        for i in range(0, self.problem.total_step - 1):
            results.insert(symbol("x", result_idx), values.atVector(symbol("x", i)))
            results.insert(symbol("v", result_idx), values.atVector(symbol("v", i)))
            for j in range(1, self.problem.control_inter + 1):
                result_idx = result_idx + 1
                tau = j * inter_dt
                gp_inter = GaussianProcessInterpolatorLinear(self.problem.Qc_model, self.problem.delta_t, tau)
                conf1 = values.atVector(symbol("x", i))
                vel1 = values.atVector(symbol("v", i))
                conf2 = values.atVector(symbol("x", i + 1))
                vel2 = values.atVector(symbol("v", i + 1))
                conf = gp_inter.interpolatePose(conf1, vel1, conf2, vel2)
                vel = gp_inter.interpolateVelocity(conf1, vel1, conf2, vel2)
                results.insert(symbol("x", result_idx), conf)
                results.insert(symbol("v", result_idx), vel)
            result_idx = result_idx + 1
        results.insert(symbol("x", result_idx), values.atVector(symbol("x", self.problem.total_step - 1)))
        results.insert(symbol("v", result_idx), values.atVector(symbol("v", self.problem.total_step - 1)))
        return results

    def plan(self):
        if self.plan_flag == 1 and self.replan_flag == 0:
            tic = time.time()

            self.traj.initializeTrajectory(self.init_values, self.problem)
            # self.init_values.print_("Init Values")
            # init_values = Values()
            # init_values = initArmTrajStraightLine(self.problem.start_conf, self.problem.goal_conf, self.problem.total_step - 1)
            # print init_values
            DOF = self.problem.robot.getDOF()
            print("Optimizing...")

            # self.batch_values = BatchTrajOptimize3DArm(self.problem.robot.arm,
            #                                            self.problem.sdf,
            #                                            self.problem.start_conf,
            #                                            np.zeros(DOF),
            #                                            self.problem.goal_conf,
            #                                            np.zeros(DOF),
            #                                            self.init_values,
            #                                            self.problem.opt_setting)


            self.batch_values = self.optimize(start_conf=self.problem.start_conf,
                                              goal_conf=self.problem.goal_conf,
                                              init_values=self.init_values)
            # print("GPMP2 Batch Values")
            # print (self.batch_values)
            print("Batch GPMP2 optimization complete.")
            toc = time.time()
            print("Plan time:")
            print(toc - tic)
            # self.isam_optimizer = ISAM2TrajOptimizer3DArm(self.problem.robot.arm,
            #                                               self.problem.sdf,
            #                                               self.problem.opt_setting)
            # self.isam_optimizer.initFactorGraph(self.problem.start_conf,
            #                                     np.zeros(self.problem.robot.getDOF()),
            #                                     self.problem.goal_conf,
            #                                     np.zeros(self.problem.robot.getDOF()))
            # self.isam_optimizer.initValues(self.batch_values)
            # self.isam_optimizer.update()
            # self.replan_values = self.isam_optimizer.values()
            # self.init_values.clear()
            self.plan_flag = 2
            # self.plan_flag = 0
            # self.exec_values = interpolateArmTraj(self.batch_values,
            #                                       self.problem.Qc_model,
            #                                       self.problem.delta_t,
            #                                       self.problem.control_inter,
            #                                       0,
            #                                       self.problem.total_step - 1)
            # print("GPMP2 Exec Values:")
            # print self.exec_values
            # exec_step = self.problem.total_step + self.problem.control_inter * (self.problem.total_step - 1)
            # print("Exec Step:")
            # print exec_step
            # # exec_angle = []
            # print("Exec angle")
            # for i in range(0, exec_step):
            #     conf = self.exec_values.atVector(symbol("x", i))
            #     angle = 180 * conf / 3.1415926
            #     print(i)
            #     print angle
            #     exec_angle.append(angle)
            # print("Exec Angle")
            # print exec_angle


    def execute(self):
        if self.plan_flag == 2:
            print("Checking for collision.")
            # self.exec_values = interpolateArmTraj(self.batch_values,
            #                                       self.problem.Qc_model,
            #                                       self.problem.delta_t,
            #                                       self.problem.control_inter)
            # self.exec_values = interpolateArmTraj(self.batch_values,
            #                                       self.problem.Qc_model,
            #                                       self.problem.delta_t,
            #                                       self.problem.control_inter,
            #                                       0,
            #                                       self.problem.total_step - 1)
            self.exec_values = self.interploteTrajectory(values=self.batch_values)

            exec_step = self.problem.total_step + self.problem.control_inter * (self.problem.total_step - 1)
            print("Exec Step")
            print(exec_step)
            # print("Batch Exec angle")
            # for j in range(0, exec_step):
            #     conf = self.exec_values.atVector(symbol("x", j))
            #     angle = [0, 0, 0, 0, 0, 0]
            #     angle[0] = int(180 * conf[0] / 3.1415926)
            #     angle[1] = int(180 * conf[1] / 3.1415926)
            #     angle[2] = int(180 * conf[2] / 3.1415926)
            #     angle[3] = int(180 * conf[3] / 3.1415926)
            #     angle[4] = int(180 * conf[4] / 3.1415926)
            #     angle[5] = int(180 * conf[5] / 3.1415926)
            #     print(j)
            #     print angle
            coll_cost = CollisionCost3DArm(self.problem.robot.arm,
                                           self.problem.sdf,
                                           self.exec_values,
                                           self.problem.opt_setting)
            print("Cost:")
            print (coll_cost)
            # coll_cost = 0
            if coll_cost != 0:
                print("Plan is not collision free!")
                self.plan_flag = 0
                self.replan_flag = 0
                self.move_flag = 0
                print("You can start planning!")
                # rospy.signal_shutdown("Plan is not collision free")
            elif coll_cost == 0:
                print("Executing GPMP2 planned trajectory open-loop...")
                # curr_conf = np.asarray(self.arm_pos)
                # print("Curr Angle")
                # angle = [0, 0, 0, 0, 0, 0]
                # angle[0] = int(180 * curr_conf[0] / 3.1415926)
                # angle[1] = int(180 * curr_conf[1] / 3.1415926)
                # angle[2] = int(180 * curr_conf[2] / 3.1415926)
                # angle[3] = int(180 * curr_conf[3] / 3.1415926)
                # angle[4] = int(180 * curr_conf[4] / 3.1415926)
                # angle[5] = int(180 * curr_conf[5] / 3.1415926)
                # print(angle)
                # print self.DistanceCalculate(self.arm_pos, self.problem.goal_conf)
                # print self.DistanceCalculate(self.replan_goal_conf, self.arm_pos)
                # self.traj.traj.trajectory.points = [0] * exec_step
                # for i in range(0, exec_step):
                #     conf = self.exec_values.atVector(symbol("x", i))
                #     print ("step")
                #     print (i)
                #     angle = [0, 0, 0, 0, 0, 0]
                #     angle[0] = int(180 * conf[0] / 3.1415926)
                #     angle[1] = int(180 * conf[1] / 3.1415926)
                #     angle[2] = int(180 * conf[2] / 3.1415926)
                #     angle[3] = int(180 * conf[3] / 3.1415926)
                #     angle[4] = int(180 * conf[4] / 3.1415926)
                #     angle[5] = int(180 * conf[5] / 3.1415926)
                #     print angle
                #     vel = self.exec_values.atVector(symbol("v", i))
                #     time_from_start = rospy.Duration(i * self.problem.delta_t / (self.problem.control_inter + 1))
                #     self.traj.traj.trajectory.points[i] = JointTrajectoryPoint(positions=np.asarray(conf),
                #                                                                velocities=np.asarray(vel),
                #                                                                time_from_start=time_from_start)
                # self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                # self.traj.traj_client.send_goal(self.traj.traj)
                # self.traj.traj_client.wait_for_result()
                # self.traj.executeTrajectory(exec_values=self.exec_values, problem=self.problem, exec_step=exec_step)
                while self.DistanceCalculate(self.arm_pos, self.problem.goal_conf) > 1e-6 and self.DistanceCalculate(self.replan_goal_conf, self.arm_pos) > 1e-6 and self.plan_flag == 2:
                    if self.move_flag == 0:
                        self.traj.traj.trajectory.points = [0] * exec_step
                        # print("Len")
                        # print self.traj.traj.trajectory.points.__len__()
                        for j in range(0, exec_step):
                            conf = self.exec_values.atVector(symbol("x", j))
                            # print ("step")
                            # print (j)
                            # angle = [0, 0, 0, 0, 0, 0]
                            # angle[0] = int(180 * conf[0] / 3.1415926)
                            # angle[1] = int(180 * conf[1] / 3.1415926)
                            # angle[2] = int(180 * conf[2] / 3.1415926)
                            # angle[3] = int(180 * conf[3] / 3.1415926)
                            # angle[4] = int(180 * conf[4] / 3.1415926)
                            # angle[5] = int(180 * conf[5] / 3.1415926)
                            # print angle
                            vel = self.exec_values.atVector(symbol("v", j))
                            time_from_start = rospy.Duration(j * self.problem.delta_t / (self.problem.control_inter + 1))
                            self.traj.traj.trajectory.points[j] = JointTrajectoryPoint(positions=np.asarray(conf),
                                                                                       velocities=np.asarray(vel),
                                                                                       time_from_start=time_from_start)
                        self.move_flag = 1
                        self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                        self.traj.traj_client.send_goal(self.traj.traj)
                        # self.traj.traj_client.wait_for_result()
                        # curr_conf = np.asarray(self.arm_pos)
                        # print("Curr Angle")
                        # angle = [0, 0, 0, 0, 0, 0]
                        # angle[0] = int(180 * curr_conf[0] / 3.1415926)
                        # angle[1] = int(180 * curr_conf[1] / 3.1415926)
                        # angle[2] = int(180 * curr_conf[2] / 3.1415926)
                        # angle[3] = int(180 * curr_conf[3] / 3.1415926)
                        # angle[4] = int(180 * curr_conf[4] / 3.1415926)
                        # angle[5] = int(180 * curr_conf[5] / 3.1415926)
                        # print(angle)
                        # print self.DistanceCalculate(self.arm_pos, self.problem.goal_conf)
                        # print self.DistanceCalculate(self.replan_goal_conf, self.arm_pos)
                        # time.sleep(5)

                        # self.traj.traj_client.wait_for_result()
                        # for i in range(0, self.problem.total_step):
                        #     if i == self.problem.total_step - 1:
                        #         self.traj.traj.trajectory.points = [0] * 1
                        #         conf = self.exec_values.atVector(symbol("x", i * self.problem.control_inter + i))
                        #         print ("step")
                        #         print (i * self.problem.control_inter + i)
                        #         angle = [0, 0, 0, 0, 0, 0]
                        #         angle[0] = int(180 * conf[0] / 3.1415926)
                        #         angle[1] = int(180 * conf[1] / 3.1415926)
                        #         angle[2] = int(180 * conf[2] / 3.1415926)
                        #         angle[3] = int(180 * conf[3] / 3.1415926)
                        #         angle[4] = int(180 * conf[4] / 3.1415926)
                        #         angle[5] = int(180 * conf[5] / 3.1415926)
                        #         print angle
                        #         vel = self.exec_values.atVector(symbol('v', i * self.problem.control_inter + i))
                        #         time_from_start = rospy.Duration(i*self.problem.control_inter+i)*self.problem.delta_t/(self.problem.control_inter+1)
                        #         self.traj.traj.trajectory.points[0] = JointTrajectoryPoint(positions=np.asarray(conf),
                        #                                                                    velocities=np.asarray(vel),
                        #                                                                    time_from_start=time_from_start)
                        #         self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                        #         self.traj.traj_client.send_goal(self.traj.traj)
                        #         self.move_flag = 1
                        #         # self.traj.traj_client.wait_for_result()
                        #     else:
                        #         for j in range(0, self.problem.control_inter + 1):
                        #             self.traj.traj.trajectory.points = [0] * 1
                        #             conf = self.exec_values.atVector(symbol("x", i * self.problem.control_inter + j + i))
                        #             print ("step")
                        #             print (i * self.problem.control_inter + j + i)
                        #             angle = [0, 0, 0, 0, 0, 0]
                        #             angle[0] = int(180 * conf[0] / 3.1415926)
                        #             angle[1] = int(180 * conf[1] / 3.1415926)
                        #             angle[2] = int(180 * conf[2] / 3.1415926)
                        #             angle[3] = int(180 * conf[3] / 3.1415926)
                        #             angle[4] = int(180 * conf[4] / 3.1415926)
                        #             angle[5] = int(180 * conf[5] / 3.1415926)
                        #             print angle
                        #             vel = self.exec_values.atVector(symbol("v", i * self.problem.control_inter + j + i))
                        #             time_from_start = rospy.Duration(i * self.problem.control_inter + j + i) * self.problem.delta_t / (self.problem.control_inter + 1)
                        #             self.traj.traj.trajectory.points[0] = JointTrajectoryPoint(positions=np.asarray(conf),
                        #                                                                        velocities=np.asarray(vel),
                        #                                                                        time_from_start=time_from_start)
                        #             self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                        #             self.traj.traj_client.send_goal(self.traj.traj)
                        #             # self.traj.traj_client.wait_for_result()
                        #     # self.traj.traj_client.wait_for_result()
                    if self.replan_flag == 1 and self.move_flag == 1:
                        self.Replan(self.problem.sdf)
                        # print("Replan Values Origin")
                        # print self.replan_values
                        tic = time.time()
                        curr_conf = np.asarray(self.arm_pos)
                        print("Replan Curr Angle")
                        angle = [0, 0, 0, 0, 0, 0]
                        angle[0] = int(180 * curr_conf[0] / 3.1415926)
                        angle[1] = int(180 * curr_conf[1] / 3.1415926)
                        angle[2] = int(180 * curr_conf[2] / 3.1415926)
                        angle[3] = int(180 * curr_conf[3] / 3.1415926)
                        angle[4] = int(180 * curr_conf[4] / 3.1415926)
                        angle[5] = int(180 * curr_conf[5] / 3.1415926)
                        print(angle)
                        curr_vel = self.exec_values.atVector(symbol("v", self.replan_pose_idx * self.problem.control_inter + self.replan_pose_idx))
                        # init_values = initArmTrajStraightLine(curr_conf, self.replan_goal_conf, self.problem.total_step - 1)
                        self.traj.initializeTrajectory(self.init_values, self.problem)
                        # print init_values
                        DOF = self.problem.robot.getDOF()
                        new_batch_values = self.optimize(start_conf=curr_conf,
                                                         goal_conf=self.replan_goal_conf,
                                                         init_values=self.init_values)
                        self.exec_values = self.interploteTrajectory(values=new_batch_values)

                        # new_batch_values = BatchTrajOptimize3DArm(self.problem.robot.arm,
                        #                                           self.problem.sdf,
                        #                                           curr_conf,
                        #                                           np.zeros(DOF),
                        #                                           self.replan_goal_conf,
                        #                                           np.zeros(DOF),
                        #                                           init_values,
                        #                                           self.problem.opt_setting)
                        #
                        # self.exec_values = interpolateArmTraj(new_batch_values,
                        #                                       self.problem.Qc_model,
                        #                                       self.problem.delta_t,
                        #                                       self.problem.control_inter,
                        #                                       0,
                        #                                       self.problem.total_step - 1)
                        toc = time.time()
                        print("Replan time:")
                        print (toc - tic)
                        coll_cost = CollisionCost3DArm(self.problem.robot.arm,
                                                       self.problem.sdf,
                                                       self.exec_values,
                                                       self.problem.opt_setting)
                        print("Cost:")
                        print (coll_cost)
                        if coll_cost != 0:
                            print("Plan is not collision free!")
                            self.plan_flag = 0
                            self.replan_flag = 0
                            self.move_flag = 0
                            print("You can start planning!")
                            # rospy.signal_shutdown("Plan is not collision free")
                        elif coll_cost == 0:
                            for j in range(0, exec_step):
                                conf = self.exec_values.atVector(symbol("x", j))
                                # print ("step")
                                # print (j)
                                # angle = [0, 0, 0, 0, 0, 0]
                                # angle[0] = int(180 * conf[0] / 3.1415926)
                                # angle[1] = int(180 * conf[1] / 3.1415926)
                                # angle[2] = int(180 * conf[2] / 3.1415926)
                                # angle[3] = int(180 * conf[3] / 3.1415926)
                                # angle[4] = int(180 * conf[4] / 3.1415926)
                                # angle[5] = int(180 * conf[5] / 3.1415926)
                                # print angle
                                vel = self.exec_values.atVector(symbol("v", j))
                                time_from_start = rospy.Duration(j * self.problem.delta_t / (self.problem.control_inter + 1))
                                self.traj.traj.trajectory.points[j] = JointTrajectoryPoint(positions=np.asarray(conf),
                                                                                           velocities=np.asarray(vel),
                                                                                           time_from_start=time_from_start)
                            # self.move_flag = 1
                            self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                            self.traj.traj_client.send_goal(self.traj.traj)

                            self.replan_flag = 0
                        # self.isam_optimizer.fixConfigAndVel(self.replan_pose_idx, curr_conf, curr_vel)
                        # self.isam_optimizer.changeGoalConfigAndVel(self.replan_goal_conf, np.zeros(self.problem.robot.getDOF()))
                        # self.isam_optimizer.update()
                        # self.replan_values = self.isam_optimizer.values()
                        # print("Replan Values After")
                        # print self.replan_values

                        # print("Replan Exec angle")
                        # for j in range(0, exec_step):
                        #     conf = self.exec_values.atVector(symbol("x", j))
                        #     angle = [0, 0, 0, 0, 0, 0]
                        #     angle[0] = int(180 * conf[0] / 3.1415926)
                        #     angle[1] = int(180 * conf[1] / 3.1415926)
                        #     angle[2] = int(180 * conf[2] / 3.1415926)
                        #     angle[3] = int(180 * conf[3] / 3.1415926)
                        #     angle[4] = int(180 * conf[4] / 3.1415926)
                        #     angle[5] = int(180 * conf[5] / 3.1415926)
                        #     print(j)
                        #     print angle
                        # print("Len")
                        # print self.traj.traj.trajectory.points.__len__()
                        # self.traj.traj.trajectory.points = [0] * (exec_step - 18)
                        # # print("Len")
                        # # print self.traj.traj.trajectory.points.__len__()
                        # for j in range(0, exec_step - 18):
                        #     conf = self.exec_values.atVector(symbol("x", j + 18))
                        #     if j == exec_step - 19:
                        #         print ("step")
                        #         print (j)
                        #         angle = [0, 0, 0, 0, 0, 0]
                        #         angle[0] = int(180 * conf[0] / 3.1415926)
                        #         angle[1] = int(180 * conf[1] / 3.1415926)
                        #         angle[2] = int(180 * conf[2] / 3.1415926)
                        #         angle[3] = int(180 * conf[3] / 3.1415926)
                        #         angle[4] = int(180 * conf[4] / 3.1415926)
                        #         angle[5] = int(180 * conf[5] / 3.1415926)
                        #         print angle
                        #     vel = self.exec_values.atVector(symbol("v", j + 18))
                        #     time_from_start = rospy.Duration(
                        #         j * self.problem.delta_t / (self.problem.control_inter + 1))
                        #     self.traj.traj.trajectory.points[j] = JointTrajectoryPoint(positions=np.asarray(conf),
                        #                                                                velocities=np.asarray(vel),
                        #                                                                time_from_start=time_from_start)
                        # self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                        # self.traj.traj_client.send_goal(self.traj.traj)


                        # tic = time.time()
                        # curr_conf = self.batch_values.atVector(symbol("x", self.replan_pose_idx))
                        # curr_vel = self.exec_values.atVector(symbol("v", self.replan_pose_idx))
                        # # curr_conf = np.asarray(self.arm_pos)
                        # # curr_vel = self.exec_values.atVector(symbol("v", self.replan_pose_idx * self.problem.control_inter + self.replan_pose_idx))
                        # # # self.replan_goal_conf = np.asarray(np.zeros(self.problem.robot.getDOF()))
                        # self.isam_optimizer.fixConfigAndVel(self.replan_pose_idx, curr_conf, curr_vel)
                        # self.isam_optimizer.changeGoalConfigAndVel(self.replan_goal_conf, np.zeros(self.problem.robot.getDOF()))
                        # self.isam_optimizer.update()
                        # self.replan_values = self.isam_optimizer.values()
                        # # print("Replan Values")
                        # # print self.replan_values
                        # self.exec_values = interpolateArmTraj(self.replan_values,
                        #                                       self.problem.Qc_model,
                        #                                       self.problem.delta_t,
                        #                                       self.problem.control_inter,
                        #                                       0,
                        #                                       self.problem.total_step - 1)
                        # # print("Replan Exec angle")
                        # # for i in range(0, exec_step):
                        # #     conf = self.exec_values.atVector(symbol("x", i))
                        # #     angle = [0, 0, 0, 0, 0, 0]
                        # #     angle[0] = int(180 * conf[0] / 3.1415926)
                        # #     angle[1] = int(180 * conf[1] / 3.1415926)
                        # #     angle[2] = int(180 * conf[2] / 3.1415926)
                        # #     angle[3] = int(180 * conf[3] / 3.1415926)
                        # #     angle[4] = int(180 * conf[4] / 3.1415926)
                        # #     angle[5] = int(180 * conf[5] / 3.1415926)
                        # #     print(i)
                        # #     print angle
                        # self.replan_flag = 0
                        # toc = time.time()
                        # print("Replan time:")
                        # print(toc - tic)
                        # print("Replan trajectory")
                        # self.traj.traj_client.cancel_all_goals()
                        # for i in range(self.replan_pose_idx, self.problem.total_step):
                        #     if i == self.problem.total_step - 1:
                        #         self.traj.traj.trajectory.points = [0] * 1
                        #         conf = self.exec_values.atVector(symbol("x", i * self.problem.control_inter + i))
                        #         vel = self.exec_values.atVector(symbol("v", i * self.problem.control_inter + i))
                        #         time_from_start = rospy.Duration(i * self.problem.control_inter + i) * self.problem.delta_t / (
                        #                     self.problem.control_inter + 1)
                        #         self.traj.traj.trajectory.points[0] = JointTrajectoryPoint(positions=np.asarray(conf),
                        #                                                                    velocities=np.asarray(vel),
                        #                                                                    time_from_start=time_from_start)
                        #         self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                        #         self.traj.traj_client.send_goal(self.traj.traj)
                        #         self.move_flag = 1
                        #         # self.traj.traj_client.wait_for_result()
                        #     else:
                        #         for j in range(0, self.problem.control_inter + 1):
                        #             self.traj.traj.trajectory.points = [0] * 1
                        #             conf = self.exec_values.atVector(symbol("x", i * self.problem.control_inter + j + i))
                        #             vel = self.exec_values.atVector(symbol("v", i * self.problem.control_inter + j + i))
                        #             time_from_start = rospy.Duration(i * self.problem.control_inter + j + i) * self.problem.delta_t / (
                        #                         self.problem.control_inter + 1)
                        #             self.traj.traj.trajectory.points[0] = JointTrajectoryPoint(positions=np.asarray(conf),
                        #                                                                        velocities=np.asarray(vel),
                        #                                                                        time_from_start=time_from_start)
                        #             self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                        #             self.traj.traj_client.send_goal(self.traj.traj)
                                    # self.traj.traj_client.wait_for_result()
                    # if self.move_flag == 1:
                    #     curr_conf = np.asarray(self.arm_pos)
                    #     print("Curr Angle")
                    #     angle = [0, 0, 0, 0, 0, 0]
                    #     angle[0] = int(180 * curr_conf[0] / 3.1415926)
                    #     angle[1] = int(180 * curr_conf[1] / 3.1415926)
                    #     angle[2] = int(180 * curr_conf[2] / 3.1415926)
                    #     angle[3] = int(180 * curr_conf[3] / 3.1415926)
                    #     angle[4] = int(180 * curr_conf[4] / 3.1415926)
                    #     angle[5] = int(180 * curr_conf[5] / 3.1415926)
                    #     print(angle)
                    #     print self.DistanceCalculate(self.arm_pos, self.problem.goal_conf)
                    #     print self.DistanceCalculate(self.replan_goal_conf, self.arm_pos)
                print("Finish Executing")
                self.plan_flag = 0
                self.replan_flag = 0
                self.move_flag = 0
                print("You can start planning!")
        # else:
        #     print("Plan First")

    def Replan(self, sdf):
        self.replan_pose_idx = int(3)


    def armStateCallback(self, msg):
        # for j in range(0, self.problem.robot.getDOF()):
        # self.arm_pos = msg.position
        # print(msg.position)
        # self.arm_pos = msg.position[2:8]
        self.arm_pos = msg.position
        self.arm_pos_time = rospy.Time.now()
        # curr_conf = np.asarray(self.arm_pos)
        # print("Curr Angle")
        # angle = [0, 0, 0, 0, 0, 0]
        # angle[0] = int(180 * curr_conf[0] / 3.1415926)
        # angle[1] = int(180 * curr_conf[1] / 3.1415926)
        # angle[2] = int(180 * curr_conf[2] / 3.1415926)
        # angle[3] = int(180 * curr_conf[3] / 3.1415926)
        # angle[4] = int(180 * curr_conf[4] / 3.1415926)
        # angle[5] = int(180 * curr_conf[5] / 3.1415926)
        # print(angle)
        # print (self.arm_pos)

    def plancallback(self, msg):
        # print ("Plan")
        if self.plan_flag == 0 and self.replan_flag == 0:
            print("Plan")
            self.problem.start_conf = np.asarray(self.arm_pos)
            goal_values = np.zeros(self.problem.robot.getDOF())
            i = 0
            # print msg
            for constraint in msg.goal.request.goal_constraints[0].joint_constraints:
                goal_values[i] = constraint.position
                i += 1
            # print goal_values
            self.replan_goal_conf = np.asarray(goal_values)
            self.problem.goal_conf = np.asarray(goal_values)
            print("Plan Start Conf:")
            print(self.problem.start_conf)
            print("Goal Angle")
            print("Plan Goal Conf:")
            print (self.problem.goal_conf)
            print("Goal Angle")
            angle = [0, 0, 0, 0, 0, 0]
            angle[0] = int(180 * goal_values[0] / 3.1415926)
            angle[1] = int(180 * goal_values[1] / 3.1415926)
            angle[2] = int(180 * goal_values[2] / 3.1415926)
            angle[3] = int(180 * goal_values[3] / 3.1415926)
            angle[4] = int(180 * goal_values[4] / 3.1415926)
            angle[5] = int(180 * goal_values[5] / 3.1415926)
            print(angle)
            self.plan_flag = 1
        elif self.plan_flag == 1 or self.plan_flag == 2:
            print("Replan")
            goal_values = np.zeros(self.problem.robot.getDOF())
            i = 0
            for constraint in msg.goal.request.goal_constraints[0].joint_constraints:
                goal_values[i] = constraint.position
                i += 1
            self.replan_goal_conf = np.asarray(goal_values)
            self.problem.goal_conf = np.asarray(goal_values)
            print ("Replan Goal Conf")
            print (self.replan_goal_conf)
            print("Replan Goal Angle")
            angle = [0, 0, 0, 0, 0, 0]
            angle[0] = int(180 * goal_values[0] / 3.1415926)
            angle[1] = int(180 * goal_values[1] / 3.1415926)
            angle[2] = int(180 * goal_values[2] / 3.1415926)
            angle[3] = int(180 * goal_values[3] / 3.1415926)
            angle[4] = int(180 * goal_values[4] / 3.1415926)
            angle[5] = int(180 * goal_values[5] / 3.1415926)
            print(angle)
            self.replan_flag = 1
            # print self.replan_flag

    def DistanceCalculate(self, a1, b):
        temp_a = np.asarray(a1)
        temp_b = np.asarray(b)
        dis = 0
        if temp_a.size == temp_b.size:
            for j in range(0, temp_a.size):
                while temp_a[j] > 3.1415926 or temp_a[j] < -3.1415926:
                    if temp_a[j] > 3.1415926:
                        temp_a[j] -= 3.1415926
                    if temp_a[j] < - 3.1415926:
                        temp_a[j] += 3.1415926
                while temp_b[j] > 3.1415926 or temp_b[j] < -3.1415926:
                    if temp_b[j] > 3.1415926:
                        temp_b[j] -= 3.1415926
                    if temp_b[j] < - 3.1415926:
                        temp_b[j] += 3.1415926
            if temp_a.size == temp_b.size:
                for j in range(0, temp_a.size):
                    dis += (temp_a[j] - temp_b[j]) * (temp_a[j] - temp_b[j])
        else:
            print("Array size error!")
        return dis


if __name__ == '__main__':
    # robot = Robot.Robot()
    # problem = Problem.Problem()
    # traj = Traj.Traj()
    # robot = moveit_commander.RobotCommander()
    # group_names = robot.get_group_names()
    # print "============ Robot Groups:", robot.get_group_names()
    # group_name = "arm"
    # group = moveit_commander.MoveGroupCommander(group_name)
    # planning_frame = group.get_planning_frame()
    print("Start!\n")
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(3.0)
    # #obs
    collision_object = CollisionObject()
    collision_object.header.frame_id = "world"
    collision_object.header.stamp = rospy.Time.now()
    collision_object.id = "box"
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions.append(0.2)
    primitive.dimensions.append(1.0)
    primitive.dimensions.append(0.2)
    box_pose = Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.x = 0.0
    box_pose.position.y = -0.5
    box_pose.position.z = 0.5
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = collision_object.ADD
    # collision_objects = ()
    # collision_objects.append(collision_object)
    scene.add_object(collision_object)
    # scene.add_box(name="box1",
    #               pose=box_pose,
    #               size=(0.2, 1.0, 0.2))


    dataset = Dataset()
    dataset.cols = 30
    dataset.rows = 30
    dataset.z = 20
    dataset.origin_x = -1.5
    dataset.origin_y = -1.5
    dataset.origin_z = -0.35
    dataset.cell_size = 0.1
    dataset.map = np.zeros((dataset.rows, dataset.cols, dataset.z))
    dataset.corner_idx = None
    #
    cell_size = dataset.cell_size
    origin = np.asarray([dataset.origin_x, dataset.origin_y, dataset.origin_z])
    origin_point3 = Point3(origin)
    tic = time.time()
    [dataset.map, dataset.corner_idx] = add_obstacle([15, 15, 2], [30, 30, 3], dataset.map, dataset.corner_idx)
    # [dataset.map, dataset.corner_idx] = add_obstacle([15, 10, 8], [3, 3, 10], dataset.map, dataset.corner_idx)
    # [dataset.map, dataset.corner_idx] = add_obstacle([20, 15, 8], [3, 3, 10], dataset.map, dataset.corner_idx)

    # print ("Map")
    # print dataset.map
    # curr_map = dataset.map > 0.75
    # inv_map = 1 - curr_map
    # map_dist = ndimage.distance_transform_edt(inv_map)
    # print ("Map_dist")
    # print map_dist
    # inv_map_dist = ndimage.distance_transform_edt(curr_map)
    # print ("Inv_Map_dist")
    # print inv_map_dist
    # field = map_dist - inv_map_dist
    # field = field * cell_size
    # print ("Field")
    # print field

    print("calculating signed distance field ...")
    field = signedDistanceField3D(dataset.map, dataset.cell_size)
    field = np.asarray(field)
    # print field
    print("calculating signed distance field done")
    sdf = SignedDistanceField(origin_point3, cell_size, field.shape[0], field.shape[1], field.shape[2])

    for z in range(0, field.shape[2]):
        temp = np.asarray(field[:, :, z])
        sdf.initFieldData(z, temp.transpose())
    sdf.print("SDF")
    sdf.saveSDF('test_sdf.bin')
    # sdf.saveSDF('test.bin')
    toc = time.time()
    print("SDF TIME")
    print(toc - tic)
    rospy.init_node('gpmp2_planner_py', anonymous=True)
    gpmp2_plan = GPMP2Planner()
    gpmp2_plan.problem.sdf = sdf
    print("You can start planning!")


    # dataset

    # sdf1 = SignedDistanceField()
    # sdf1.loadSDF('/home/letianfu/catkin_ws/src/gpmp2_planner_py/sdf/obs_test_one_box.bin')
    # # sdf1.loadSDF('/home/robert/sdffile/obs_test_one_box.bin')
    #
    # sdf1.print_("SDF1")
    # count = 0
    # for i in range(0, dataset.cols):
    #     for j in range(0, dataset.rows):
    #         for k in range(0, dataset.z):
    #             curr_x = float(dataset.origin_x + cell_size * i)
    #             curr_y = float(dataset.origin_y + cell_size * j)
    #             curr_z = float(dataset.origin_z + cell_size * k)
    #             if k == dataset.z - 1:
    #                 p1 = Point3(curr_x, curr_y, curr_z - 0.0001)
    #             else:
    #                 p1 = Point3(curr_x, curr_y, curr_z)
    #             # print p1
    #             # print("sdf point")
    #             # print sdf.getSignedDistance(p1)
    #             # print("sdf1 point")
    #             # print sdf1.getSignedDistance(p1)
    #             error = sdf1.getSignedDistance(p1) - sdf.getSignedDistance(p1)
    #             # print error
    #             if error > 1e-4:
    #                 count += 1
    # print(count)

    # batch_values = Values()
    alpha = [1.5708, 0, 0, 1.5708, -1.5708, 0]
    # print alpha
    alpha = np.asarray(alpha)
    a = [0, -0.427, -0.357, 0, 0, 0]
    a = np.asarray(a)
    # print a
    d = [0.147, 0, 0, 0.141, 0.116, 0.105]
    d = np.asarray(d)
    # print d
    base_pose = Pose3(Rot3(np.eye(3)), Point3(np.asarray([0, 0, 0])))
    # print base_pose
    theta = [0, -1.5708, 0, -1.5708, 0, 0]
    theta = np.asarray(theta)
    # print theta
    abstract_arm = Arm(6, a, alpha, d, base_pose, theta)
    spheres_data = [[0, 0.0, - 0.1,  0.00, 0.08],
                    [0, 0.0,  0.0,  0.00, 0.08],
                    [1, 0.0,  0.0,  0.10, 0.06],
                    [1, 0.105,  0.0,  0.10, 0.06],
                    [1, 0.210,  0.0, 0.10,  0.06],
                    [1, 0.315,  0.0, 0.10, 0.06],
                    [1, 0.420,  0.0, 0.10, 0.06],
                    [1, 0.0,  0.0,  0.00, 0.08],
                    [2, 0.11, 0.0,  0.00, 0.06],
                    [2, 0.22,  0.0,  0.00, 0.06],
                    [2, 0.0,  0.0, 0.00, 0.08],
                    [3, 0.0,  0.0,  0.00, 0.07],
                    [4, 0.0,  0.00,  0.00, 0.06],
                    [5, 0.0,  0.0,  0.00, 0.06],
                    ]
    spheres_data = np.asarray(spheres_data)
    # print spheres_data
    nr_body = spheres_data.shape[0]
    # print nr_body
    sphere_vec = BodySphereVector()

    for i in range(nr_body):
        sphere_vec.push_back(
            BodySphere(
                int(spheres_data[i, 0]), spheres_data[i, 4], Point3(spheres_data[i, 1:4])
            )
        )
        # print spheres_data[i, 0]
        # print spheres_data[i, 4]
        # print Point3(spheres_data[i, 1:4])
    arm_model = ArmModel(abstract_arm, sphere_vec)
    # sdf = SignedDistanceField()
    # sdf.loadSDF('/home/robert/sdffile/obs_test_one_box.bin')
    # sdf.print_("SDF")
    total_time = 10.0
    total_time_step = 9
    total_check_step = 90
    delta_t = total_time / total_time_step
    check_inter = total_check_step / total_time_step - 1
    Qc = 1 * np.eye(6)
    Qc_model = noiseModel.Gaussian.Covariance(Qc)
    cost_sigma = 0.02
    epsilon_dist = 0.2
    pose_fix_sigma = 0.0001
    vel_fix_sigma = 0.0001
    opt_setting = TrajOptimizerSetting(6)
    opt_setting.set_total_step(total_time_step)
    opt_setting.set_total_time(total_time)
    opt_setting.set_epsilon(epsilon_dist)
    opt_setting.set_cost_sigma(cost_sigma)
    opt_setting.set_obs_check_inter(int(check_inter))
    opt_setting.set_conf_prior_model(pose_fix_sigma)
    opt_setting.set_vel_prior_model(vel_fix_sigma)
    opt_setting.set_Qc_model(Qc)
    opt_setting.setLM()

    # start_conf = gpmp2_plan.problem.start_conf
    # end_conf = gpmp2_plan.problem.goal_conf
    # start_conf = np.asarray([2.0944, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    # end_conf = np.asarray([0.5236, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    start_conf = np.asarray([0, 0, 0, 0, 0, 0])
    end_conf = np.asarray([2.0944, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    start_vel = np.zeros(6)
    end_vel = np.zeros(6)
    init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step)
    # print("Init 1")
    # print init_values
    # batch_values = BatchTrajOptimize3DArm(arm_model,
    #                                       sdf,
    #                                       start_conf,
    #                                       start_vel,
    #                                       end_conf,
    #                                       end_vel,
    #                                       init_values,
    #                                       opt_setting)
    # print("Batch 1")
    # print batch_values
    # # plot_inter = 5
    # # plot_values = interpolateArmTraj(batch_values, Qc_model, delta_t, plot_inter)
    # # print("Plot Values")
    # # print plot_values
    # #
    # sdf.loadSDF('/home/robert/test2.bin')
    # batch_values = BatchTrajOptimize3DArm(arm_model,
    #                                       sdf1,
    #                                       start_conf,
    #                                       start_vel,
    #                                       end_conf,
    #                                       end_vel,
    #                                       init_values,
    #                                       opt_setting)
    # print("Batch 2")
    # print batch_values
    # # #
    # # gpmp2_plan.problem.sdf = sdf
    # # # # gpmp2_plan.problem.start_conf = np.asarray([2.0944, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    # # # # gpmp2_plan.problem.goal_conf = np.asarray([0.5236, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    # # gpmp2_plan.problem.start_conf = np.asarray([0, 0, 0, 0, 0, 0])
    # # gpmp2_plan.problem.goal_conf = np.asarray([2.0944, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    # # gpmp2_plan.plan_flag = 1
    # # gpmp2_plan.plan()
    # # print("Checking for collision.")
    # # gpmp2_plan.exec_values = interpolateArmTraj(gpmp2_plan.batch_values,
    # #                                             gpmp2_plan.problem.Qc_model,
    # #                                             gpmp2_plan.problem.delta_t,
    # #                                             gpmp2_plan.problem.control_inter,
    # #                                             0,
    # #                                             gpmp2_plan.problem.total_step - 1)
    # # exec_step = gpmp2_plan.problem.total_step + gpmp2_plan.problem.control_inter * (gpmp2_plan.problem.total_step - 1)
    # # coll_cost = CollisionCost3DArm(gpmp2_plan.problem.robot.arm,
    # #                                gpmp2_plan.problem.sdf,
    # #                                gpmp2_plan.exec_values,
    # #                                gpmp2_plan.problem.opt_setting)
    # # print("Cost:")
    # # print (coll_cost)
    #
    # # sdf = SignedDistanceField()
    # # sdf.loadSDF('/home/robert/obs_test_one_box.bin')
    # # # gpmp2_plan.problem.robot.arm = arm_model
    # # # gpmp2_plan.problem.sdf = sdf
    # # # gpmp2_plan.problem.start_conf = start_conf
    # # # gpmp2_plan.problem.goal_conf = end_conf
    # # gpmp2_plan.init_values.clear()
    # # gpmp2_plan.traj.initializeTrajectory(gpmp2_plan.init_values, gpmp2_plan.problem)
    # # gpmp2_plan.init_values.print_("Init Values")
    # # # gpmp2_plan.init_values = init_values
    # # # gpmp2_plan.problem.opt_setting = opt_setting
    # # batch_values2 = BatchTrajOptimize3DArm(gpmp2_plan.problem.robot.arm,
    # #                                       gpmp2_plan.problem.sdf,
    # #                                       gpmp2_plan.problem.start_conf,
    # #                                       np.zeros(6),
    # #                                       gpmp2_plan.problem.goal_conf,
    # #                                       np.zeros(6),
    # #                                       gpmp2_plan.init_values,
    # #                                       gpmp2_plan.problem.opt_setting)
    # # print("Batch 2")
    # # print batch_values2
    # # plot_inter = 5
    # # plot_values2 = interpolateArmTraj(batch_values2, Qc_model, delta_t, plot_inter)
    # # print("Plot Values 2")
    # # print plot_values2
    # #
    # gpmp2_plan.problem.sdf = sdf
    # gpmp2_plan.problem.sdf.print_("gpmp2 sdf")
    # gpmp2_plan.problem.start_conf = np.asarray([2.0944, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    # gpmp2_plan.problem.goal_conf = np.asarray([0.5236, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    # gpmp2_plan.problem.start_conf = np.asarray([0, 0, 0, 0, 0, 0])
    # gpmp2_plan.problem.goal_conf = np.asarray([2.0944, 0.4363, 1.5708, -2.0071, -2.0944, 0])
    # gpmp2_plan.init_values.clear()
    # gpmp2_plan.traj.initializeTrajectory(gpmp2_plan.init_values, gpmp2_plan.problem)
    # gpmp2_plan.init_values.print_("Init Values")
    # batch_values = BatchTrajOptimize3DArm(gpmp2_plan.problem.robot.arm,
    #                                       gpmp2_plan.problem.sdf,
    #                                       gpmp2_plan.problem.start_conf,
    #                                       np.zeros(6),
    #                                       gpmp2_plan.problem.goal_conf,
    #                                       np.zeros(6),
    #                                       gpmp2_plan.init_values,
    #                                       gpmp2_plan.problem.opt_setting)
    # print("Batch 3")
    # print batch_values
    #
    #
    #
    # # # state_sub = rospy.Subscriber("move_group/goal", MoveGroupActionGoal, queue_size=1000, callback=plancallback)
    # # rospy.spin()
    #
    while not rospy.is_shutdown():
        gpmp2_plan.plan()
        # print("2222")
        gpmp2_plan.execute()

    sys.exit(0)


