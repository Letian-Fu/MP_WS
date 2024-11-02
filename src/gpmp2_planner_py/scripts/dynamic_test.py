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
from moveit_msgs.msg import MoveGroupActionGoal, CollisionObject, PlanningScene, PlanningSceneComponents
import moveit_commander
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
# import pymoveit_core.planning_scene
import moveit_ros_planning_interface
from moveit_python.planning_scene_interface import PlanningSceneInterface


from gpmp2 import *
from gtsam import *
from gtsam.gtsam import Values, symbol, Pose3, Rot3, noiseModel
from gtsam import Point3
from gpmp2.datasets.generate3Ddataset import Dataset, add_obstacle
from gpmp2.utils.signedDistanceField3D import signedDistanceField3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d , Axes3D
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
        self.exec_step = 0

        flag = rospy.has_param('robot/arm_state_topic')
        if flag == True:
            self.arm_state_topic = rospy.get_param('robot/arm_state_topic')
            self.arm_state_sub = rospy.Subscriber(self.arm_state_topic, JointState, queue_size=1, callback=self.armStateCallback)
            self.arm_pos = np.zeros(self.problem.robot.getDOF())
            self.arm_pos_time = rospy.Time.now()
            # print("Robot arm_state_topic:")
            # print(self.arm_state_topic)
            flag = False
        else:
            print("No Robot arm_state_topic")
        rospy.sleep(1)
        if not rospy.has_param("start_conf"):
            self.problem.start_conf = self.arm_pos
        # print("GPMP2 Planner Initialize Finished")

    def optimize(self, start_conf, goal_conf, init_values):
        DOF = self.problem.robot.getDOF()
        pose_fix_model = noiseModel.Isotropic.Sigma(DOF, self.problem.fix_pose_sigma)
        vel_fix_model = noiseModel.Isotropic.Sigma(DOF, self.problem.fix_vel_sigma)
        ## init optimization
        graph = NonlinearFactorGraph()
        graph_obs = NonlinearFactorGraph()

        for i in range(self.problem.total_step):
            key_pos = symbol("x", i)
            key_vel = symbol("v", i)

            # priors
            if i == 0:
                graph.push_back(PriorFactorVector(key_pos, start_conf, pose_fix_model))
                graph.push_back(PriorFactorVector(key_vel, np.zeros(DOF), vel_fix_model))
            elif i == self.problem.total_step:
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
                if self.problem.obs_check_inter > 0:
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
            # self.init_values = initArmTrajStraightLine(self.problem.start_conf, self.problem.goal_conf, self.problem.total_step)
            # print init_values
            DOF = self.problem.robot.getDOF()
            # print("Optimizing...")
            # self.batch_values = self.optimize(start_conf=self.problem.start_conf,
            #                                   goal_conf=self.problem.goal_conf,
            #                                   init_values=self.init_values)
            self.batch_values = BatchTrajOptimize3DArm(self.problem.robot.arm,
                                                      self.problem.sdf,
                                                      self.problem.start_conf,
                                                      np.zeros(DOF),
                                                      self.problem.goal_conf,
                                                      np.zeros(DOF),
                                                      self.init_values,
                                                      self.problem.opt_setting)
            # print("GPMP2 Batch Values")
            # print (self.batch_values)
            # print("Batch GPMP2 optimization complete.")
            toc = time.time()
            print("Plan time:")
            print(toc - tic)
            self.plan_flag = 2
            self.exec_values = self.interploteTrajectory(values=self.batch_values)
            self.exec_step = self.problem.total_step + self.problem.control_inter * (self.problem.total_step - 1)
            coll_cost = CollisionCost3DArm(self.problem.robot.arm,
                                           self.problem.sdf,
                                           self.exec_values,
                                           self.problem.opt_setting)
            # print("Cost:")
            # print(coll_cost)
            # coll_cost = 0
            # if coll_cost != 0:
            #     print("Plan is not collision free!")
            #     self.plan_flag = 0
            #     self.move_flag = 0
            # else:
                # gpmp2_planner.traj.traj.trajectory.points = [0] * gpmp2_planner.exec_step
                # print('Start moving!')

    def execute(self):
        if self.plan_flag == 2:
            print("Checking for collision.")
            self.exec_values = self.interploteTrajectory(values=self.batch_values)

            self.exec_step = self.problem.total_step + self.problem.control_inter * (self.problem.total_step - 1)
            print("Exec Step")
            print(self.exec_step)
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
                while self.DistanceCalculate(self.arm_pos, self.problem.goal_conf) > 1e-6 and self.DistanceCalculate(self.replan_goal_conf, self.arm_pos) > 1e-6 and self.plan_flag == 2:
                    if self.move_flag == 0:
                        self.traj.traj.trajectory.points = [0] * self.exec_step
                        # print("Len")
                        # print self.traj.traj.trajectory.points.__len__()
                        for j in range(0, self.exec_step):
                            conf = self.exec_values.atVector(symbol("x", j))
                            vel = self.exec_values.atVector(symbol("v", j))
                            time_from_start = rospy.Duration(j * self.problem.delta_t / (self.problem.control_inter + 1))
                            self.traj.traj.trajectory.points[j] = JointTrajectoryPoint(positions=np.asarray(conf),
                                                                                       velocities=np.asarray(vel),
                                                                                       time_from_start=time_from_start)
                        self.move_flag = 1
                        self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                        self.traj.traj_client.send_goal(self.traj.traj)
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

                        toc = time.time()
                        print("Replan time:")
                        print(toc - tic)
                        coll_cost = CollisionCost3DArm(self.problem.robot.arm,
                                                       self.problem.sdf,
                                                       self.exec_values,
                                                       self.problem.opt_setting)
                        print("Cost:")
                        print(coll_cost)
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
                                vel = self.exec_values.atVector(symbol("v", j))
                                time_from_start = rospy.Duration(j * self.problem.delta_t / (self.problem.control_inter + 1))
                                self.traj.traj.trajectory.points[j] = JointTrajectoryPoint(positions=np.asarray(conf),
                                                                                           velocities=np.asarray(vel),
                                                                                           time_from_start=time_from_start)
                            # self.move_flag = 1
                            self.traj.traj.trajectory.header.stamp = rospy.Time.now()
                            self.traj.traj_client.send_goal(self.traj.traj)

                            self.replan_flag = 0
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

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("dynamic_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    rospy.sleep(3.0)
    gpmp2_planner = GPMP2Planner()
    # pub = rospy.Publisher('chatter', Pose, queue_size=1000)
    # #obs
    collision_object = CollisionObject()
    collision_object.header.frame_id = "world"
    collision_object.header.stamp = rospy.Time.now()
    collision_object.id = "obs"
    primitive = SolidPrimitive()

    primitive.type = primitive.SPHERE
    primitive.dimensions.append(0.1)

    # primitive.type = primitive.BOX
    # primitive.dimensions.append(0.1)
    # primitive.dimensions.append(0.1)
    # primitive.dimensions.append(0.1)
    obs_pose = Pose()
    obs_pose.orientation.w = 1.0
    obs_pose.position.x = 0.0
    obs_pose.position.y = -0.45
    obs_pose.position.z = 0.5
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(obs_pose)
    collision_object.operation = collision_object.ADD
    # collision_objects = ()
    # collision_objects.append(collision_object)
    scene.add_object(collision_object)

    workspace = np.array([3, 3, 3])
    dataset = Dataset()
    dataset.origin_x = -1.0
    dataset.origin_y = -1.0
    dataset.origin_z = -0.5
    dataset.cell_size = 0.05
    dataset.cols = 40
    dataset.rows = 40
    dataset.z = 40
    dataset.map = np.zeros((dataset.rows, dataset.cols, dataset.z))
    dataset.corner_idx = None
    #
    cell_size = dataset.cell_size
    origin = np.asarray([dataset.origin_x, dataset.origin_y, dataset.origin_z])
    origin_point3 = Point3(origin)
    tic = time.time()
    [dataset.map, dataset.corner_idx] = add_obstacle([20, 9, 10], [3, 3, 3], dataset.map, dataset.corner_idx)
    [dataset.map, dataset.corner_idx] = add_obstacle([20, 20, 5], [30, 30, 3], dataset.map, dataset.corner_idx)

    print("calculating signed distance field ...")
    field = signedDistanceField3D(dataset.map, dataset.cell_size)
    field = np.asarray(field)
    # print field
    print("calculating signed distance field done")
    sdf = SignedDistanceField(origin_point3, cell_size, field.shape[0], field.shape[1], field.shape[2])

    for z in range(0, field.shape[2]):
        temp = np.asarray(field[:, :, z])
        sdf.initFieldData(z, temp.transpose())
    # sdf.print("SDF")
    # sdf.saveSDF('test_sdf.bin')
    # sdf.saveSDF('test.bin')
    toc = time.time()
    print("SDF TIME")
    print(toc - tic)
    gpmp2_planner.problem.sdf = sdf

    print("Start Dynamic Test!")

    start_conf = np.array([-136, -42, -43, 85, 136, 0])
    start_conf = start_conf * np.pi / 180
    goal_conf = np.array([-35, -70, 54, 17, 35, 0])
    # goal_conf = np.array([-35, -42, -43, 85, 136, 0])
    goal_conf = goal_conf * np.pi / 180


    gpmp2_planner.problem.start_conf = np.asarray(gpmp2_planner.arm_pos)
    # gpmp2_planner.problem.goal_conf = np.asarray(goal_conf)
    gpmp2_planner.problem.goal_conf = np.asarray(start_conf)
    gpmp2_planner.plan_flag = 1
    gpmp2_planner.plan()
    upper_limit = 0.85
    lower_limit = 0.25
    update_fre = 3
    step = 0.02
    current_exec_idx = 0
    count = 0

    exec_values = gpmp2_planner.exec_values
    t = time.perf_counter()
    while gpmp2_planner.DistanceCalculate(gpmp2_planner.arm_pos, gpmp2_planner.problem.goal_conf) > 1e-3 and gpmp2_planner.plan_flag == 2:
        if count > update_fre:
            print(f'update time:{time.perf_counter()-t:.8f}s')
            scene.remove_world_object('obs')
            collision_object.primitive_poses[0].position.z += step
            if collision_object.primitive_poses[0].position.z > upper_limit or collision_object.primitive_poses[0].position.z < lower_limit:
                step = -step
            scene.add_object(collision_object)
            count = 0
            t = time.perf_counter()
            t1 = time.time()
            dataset = Dataset()
            dataset.origin_x = -1.0
            dataset.origin_y = -1.0
            dataset.origin_z = -0.5
            dataset.cell_size = 0.05
            dataset.cols = 40
            dataset.rows = 40
            dataset.z = 40
            dataset.map = np.zeros((dataset.rows, dataset.cols, dataset.z))
            dataset.corner_idx = None
            #
            cell_size = dataset.cell_size
            origin = np.asarray([dataset.origin_x, dataset.origin_y, dataset.origin_z])
            origin_point3 = Point3(origin)
            tic = time.time()
            current_z = int((collision_object.primitive_poses[0].position.z + 0.5) / cell_size)
            [dataset.map, dataset.corner_idx] = add_obstacle([20, 9, current_z], [3, 3, 3], dataset.map, dataset.corner_idx)
            [dataset.map, dataset.corner_idx] = add_obstacle([20, 20, 5], [30, 30, 3], dataset.map, dataset.corner_idx)
            # print("calculating signed distance field ...")
            field = signedDistanceField3D(dataset.map, dataset.cell_size)
            field = np.asarray(field)
            # print field
            # print("calculating signed distance field done")
            sdf = SignedDistanceField(origin_point3, cell_size, field.shape[0], field.shape[1], field.shape[2])

            for z in range(0, field.shape[2]):
                temp = np.asarray(field[:, :, z])
                sdf.initFieldData(z, temp.transpose())
            gpmp2_planner.problem.sdf = sdf
            t2 = time.time()
            print(f'sdf time:{t2-t1:.8f}s')
        count = count + 1

        coll_cost = CollisionCost3DArm(gpmp2_planner.problem.robot.arm,
                                       gpmp2_planner.problem.sdf,
                                       exec_values,
                                       gpmp2_planner.problem.opt_setting)
        if coll_cost == 0:
            if current_exec_idx < gpmp2_planner.exec_step:
                gpmp2_planner.traj.traj.trajectory.points = [0] * 1
                conf = exec_values.atVector(symbol("x", current_exec_idx))
                vel = exec_values.atVector(symbol("v", current_exec_idx))
                time_from_start = rospy.Duration(current_exec_idx * gpmp2_planner.problem.delta_t / (gpmp2_planner.problem.control_inter + 1))
                gpmp2_planner.traj.traj.trajectory.points[0] = JointTrajectoryPoint(positions=np.asarray(conf),
                                                                           velocities=np.asarray(vel),
                                                                           time_from_start=time_from_start)
                gpmp2_planner.traj.traj.trajectory.header.stamp = rospy.Time.now()
                gpmp2_planner.traj.traj_client.send_goal(gpmp2_planner.traj.traj)
                current_exec_idx = current_exec_idx + 1
        else:
            print('col_cost: ', coll_cost)
            # print("collision not free!")
            planner = GPMP2Planner()
            planner.problem.sdf = sdf
            planner.problem.start_conf = np.asarray(gpmp2_planner.arm_pos)
            planner.problem.goal_conf = gpmp2_planner.problem.goal_conf
            planner.plan_flag = 1
            planner.plan()
            exec_values = planner.exec_values
            current_exec_idx = 0
        rospy.sleep(0.1)

    # rospy.sleep(0.3)
    planner2 = GPMP2Planner()
    planner2.problem.sdf = gpmp2_planner.problem.sdf
    planner2.problem.start_conf = np.asarray(planner2.arm_pos)
    # gpmp2_planner.problem.goal_conf = np.asarray(goal_conf)
    planner2.problem.goal_conf = np.asarray(goal_conf)
    planner2.plan_flag = 1
    planner2.plan()
    current_exec_idx = 0
    count = 0
    exec_values = planner2.exec_values
    while planner2.DistanceCalculate(planner2.arm_pos, planner2.problem.goal_conf) > 1e-3 and planner2.plan_flag == 2:
        if count > update_fre:
            scene.remove_world_object('obs')
            collision_object.primitive_poses[0].position.z += step
            if collision_object.primitive_poses[0].position.z > upper_limit or collision_object.primitive_poses[0].position.z < lower_limit:
                step = -step
            scene.add_object(collision_object)
            count = 0

            dataset = Dataset()
            dataset.origin_x = -1.0
            dataset.origin_y = -1.0
            dataset.origin_z = -0.5
            dataset.cell_size = 0.05
            dataset.cols = 40
            dataset.rows = 40
            dataset.z = 40
            dataset.map = np.zeros((dataset.rows, dataset.cols, dataset.z))
            dataset.corner_idx = None
            #
            cell_size = dataset.cell_size
            origin = np.asarray([dataset.origin_x, dataset.origin_y, dataset.origin_z])
            origin_point3 = Point3(origin)
            tic = time.time()
            current_z = int((collision_object.primitive_poses[0].position.z + 0.5) / cell_size)
            [dataset.map, dataset.corner_idx] = add_obstacle([20, 9, current_z], [3, 3, 3], dataset.map, dataset.corner_idx)
            [dataset.map, dataset.corner_idx] = add_obstacle([20, 20, 5], [30, 30, 3], dataset.map, dataset.corner_idx)
            # print("calculating signed distance field ...")
            field = signedDistanceField3D(dataset.map, dataset.cell_size)
            field = np.asarray(field)
            # print field
            # print("calculating signed distance field done")
            sdf = SignedDistanceField(origin_point3, cell_size, field.shape[0], field.shape[1], field.shape[2])

            for z in range(0, field.shape[2]):
                temp = np.asarray(field[:, :, z])
                sdf.initFieldData(z, temp.transpose())
            planner2.problem.sdf = sdf

        count = count + 1

        coll_cost = CollisionCost3DArm(planner2.problem.robot.arm,
                                       planner2.problem.sdf,
                                       exec_values,
                                       planner2.problem.opt_setting)
        if coll_cost == 0:
            if current_exec_idx < planner2.exec_step:
                planner2.traj.traj.trajectory.points = [0] * 1
                conf = exec_values.atVector(symbol("x", current_exec_idx))
                vel = exec_values.atVector(symbol("v", current_exec_idx))
                time_from_start = rospy.Duration(current_exec_idx * planner2.problem.delta_t / (gpmp2_planner.problem.control_inter + 1))
                planner2.traj.traj.trajectory.points[0] = JointTrajectoryPoint(positions=np.asarray(conf),
                                                                           velocities=np.asarray(vel),
                                                                           time_from_start=time_from_start)
                planner2.traj.traj.trajectory.header.stamp = rospy.Time.now()
                planner2.traj.traj_client.send_goal(planner2.traj.traj)
                current_exec_idx = current_exec_idx + 1
        else:
            print('col_cost: ', coll_cost)
            # print("collision not free!")
            planner = GPMP2Planner()
            planner.problem.sdf = sdf
            planner.problem.start_conf = np.asarray(planner2.arm_pos)
            planner.problem.goal_conf = planner2.problem.goal_conf
            planner.plan_flag = 1
            planner.plan()
            exec_values = planner.exec_values
            current_exec_idx = 0
        rospy.sleep(0.1)
    rospy.spin()
    rospy.shutdown()
    sys.exit(0)


