#!/usr/bin/env python
# coding:utf-8
import os
import sys
import numpy as np
import rospy
import math
import rospkg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

from gpmp2 import *
from gtsam import *
from gpmp2.utils.plot_utils import *
from gpmp2.utils.signedDistanceField3D import signedDistanceField3D
from gtsam.gtsam import noiseModel
from gtsam import Point3

import Robot


class Dataset:
    """docstring for Dataset"""

    def __init__(self):
        self.cols = None
        self.rows = None
        self.z = None
        self.origin_x = None
        self.origin_y = None
        self.origin_z = None
        self.cell_size = None
        self.map = None
        self.corner_idx = None


def add_obstacle(position, size, map, corner):
    half_size_row = int(math.floor((size[0] - 1) / 2))
    half_size_col = int(math.floor((size[1] - 1) / 2))
    half_size_z = int(math.floor((size[2] - 1) / 2))

    # occupency grid
    map[position[0] - half_size_row - 1:position[0] + half_size_row,
        position[1] - half_size_col - 1:position[1] + half_size_col,
        position[2] - half_size_z - 1:position[2] + half_size_z, ] = np.ones(
            (2 * half_size_row + 1, 2 * half_size_col + 1,
             2 * half_size_z + 1))

    # corner
    temp = np.asarray([
        position[0] - half_size_row - 1,
        position[0] + half_size_row - 1,
        position[1] - half_size_col - 1,
        position[1] + half_size_col - 1,
        position[2] - half_size_z - 1,
        position[2] + half_size_z - 1,
    ]).reshape(1, 6)

    if corner is None:
        corner = temp
    else:
        corner = np.concatenate((corner, temp), axis=0)

    return [map, corner]


def generate3Ddataset(dataset_str):
    # GENERATE3DDATASET enerate 3D dataset evidence grid
    #
    #   Usage: dataset = GENERATE3DDATASET(dataset_str)
    #   @dataset_str       dataset string, existing datasets:
    #                      'WAMDeskDataset'
    #
    #   Dataset Format:
    #   dataset.map        ground truth evidence grid
    #   dataset.rows       number of rows (x)
    #   dataset.cols       number of cols (y)
    #   dataset.z          number of depth (z)
    #   dataset.origin_x   origin of map x
    #   dataset.origin_y   origin of map y
    #   dataset.origin_z   origin of map z
    #   dataset.cell_size  cell size
    #   dataset.corner_idx corner index to visualize edges

    dataset = Dataset()
    # dataset 1: small dataset for demo
    if dataset_str is "SmallDemo":
        # params
        dataset.cols = 200
        dataset.rows = 200
        dataset.z = 200
        dataset.origin_x = -1
        dataset.origin_y = -1
        dataset.origin_z = -1
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols, dataset.z))
        # obstacles
        dataset.corner_idx = None
        dataset.map, dataset.corner_idx = add_obstacle([150, 150, 150],
                                                       [20, 20, 20],
                                                       dataset.map,
                                                       dataset.corner_idx)

    # dataset 2: desk dataset for WAM WAMDeskDataset
    elif dataset_str is "WAMDeskDataset":
        # params
        dataset.cols = 300
        dataset.rows = 300
        dataset.z = 300
        dataset.origin_x = -1.5
        dataset.origin_y = -1.5
        dataset.origin_z = -1.5
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols, dataset.z))
        # obstacles
        dataset.corner_idx = None
        [dataset.map,
         dataset.corner_idx] = add_obstacle([170, 220, 130], [140, 60, 5],
                                            dataset.map, dataset.corner_idx)
        [dataset.map,
         dataset.corner_idx] = add_obstacle([105, 195, 90], [10, 10, 80],
                                            dataset.map, dataset.corner_idx)
        [dataset.map,
         dataset.corner_idx] = add_obstacle([235, 195, 90], [10, 10, 80],
                                            dataset.map, dataset.corner_idx)
        [dataset.map,
         dataset.corner_idx] = add_obstacle([105, 245, 90], [10, 10, 80],
                                            dataset.map, dataset.corner_idx)
        [dataset.map,
         dataset.corner_idx] = add_obstacle([235, 245, 90], [10, 10, 80],
                                            dataset.map, dataset.corner_idx)

        [dataset.map,
         dataset.corner_idx] = add_obstacle([250, 190, 145], [60, 5, 190],
                                            dataset.map, dataset.corner_idx)
        [dataset.map,
         dataset.corner_idx] = add_obstacle([250, 90, 145], [60, 5, 190],
                                            dataset.map, dataset.corner_idx)

        [dataset.map,
         dataset.corner_idx] = add_obstacle([200, 190, 145], [40, 5, 190],
                                            dataset.map, dataset.corner_idx)
        # [dataset.map, dataset.corner_idx] = add_obstacle([130 40 95], [60, 5, 190], dataset.map, dataset.corner_idx);

        [dataset.map,
         dataset.corner_idx] = add_obstacle([250, 140, 240], [60, 100, 5],
                                            dataset.map, dataset.corner_idx)
        [dataset.map,
         dataset.corner_idx] = add_obstacle([250, 140, 190], [60, 100, 5],
                                            dataset.map, dataset.corner_idx)
        [dataset.map,
         dataset.corner_idx] = add_obstacle([250, 140, 140], [60, 100, 5],
                                            dataset.map, dataset.corner_idx)
        [dataset.map,
         dataset.corner_idx] = add_obstacle([250, 140, 90], [60, 100, 5],
                                            dataset.map, dataset.corner_idx)
    return dataset


class Problem:
    def __init__(self):
        self.robot = Robot.Robot()
        DOF = self.robot.getDOF()
        # print("Starting Loading Problem Parameters:")
        flag = rospy.has_param('start_conf')
        if flag == True:
            self.start_conf = np.asarray(rospy.get_param('start_conf'))
            # print ("Start_Conf:")
            # print (self.start_conf)
            flag = False
        else:
            print ("No Start_Conf! Set Default Start_Conf")
            self.start_conf = np.asarray(np.zeros(self.robot.DOF))
            print ("Start_Conf:")
            print (self.start_conf)

        flag = rospy.has_param('goal_conf')
        if flag == True:
            self.goal_conf = np.asarray(rospy.get_param('goal_conf'))
            # print ("Goal_Conf:")
            # print (self.goal_conf)
            flag = False
        else:
            print ("No Start_Conf! Set Default Goal_Conf")
            self.goal_conf = np.asarray(np.zeros(self.robot.DOF))
            print ("Goal_Conf:")
            print (self.goal_conf)

        flag = rospy.has_param('sdf_file')
        if flag == True:
            temp_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            self.sdf_file = temp_path + '/' + rospy.get_param('sdf_file')
            # self.sdf_file = '/home/robert/MP_ws/src/gpmp2_planner_py' + '/' + rospy.get_param('sdf_file')
            # print ("SDF File:")
            # print (self.sdf_file)
            flag = False
        else:
            print ("No SDF File")

        # dataset = generate3Ddataset("SmallDemo")
        # origin = np.asarray([dataset.origin_x, dataset.origin_y, dataset.origin_z])
        # origin_point3 = Point3(origin)
        # cell_size = dataset.cell_size
        #
        # # sdf
        # # print("calculating signed distance field ...")
        # field = signedDistanceField3D(dataset.map, dataset.cell_size)
        # # print("calculating signed distance field done")
        # self.sdf = SignedDistanceField(origin_point3, cell_size, field.shape[0],
        #                           field.shape[1], field.shape[2])
        # for z in range(field.shape[2]):
        #     self.sdf.initFieldData(
        #         z, field[:, :, z])
        self.sdf = SignedDistanceField()
        # self.sdf.loadSDF(filename=self.sdf_file)
        # self.sdf.loadSDF(self.sdf_file)
        # print("SDF Data:")
        # print (self.sdf)

        self.total_time = np.double(rospy.get_param('total_time'))
        self.total_step = int(rospy.get_param('total_step'))
        self.obs_check_inter = int(rospy.get_param('obs_check_inter'))
        self.control_inter = int(rospy.get_param('control_inter'))
        self.cost_sigma = np.double(rospy.get_param('cost_sigma'))
        self.epsilon = np.double(rospy.get_param('epsilon'))
        self.Qc = np.double(rospy.get_param('Qc'))
        self.fix_pose_sigma = float(rospy.get_param('fix_pose_sigma'))
        self.fix_vel_sigma = float(rospy.get_param('fix_vel_sigma'))
        self.opt_type = rospy.get_param('opt_type')
        self.Qc_model = noiseModel.Gaussian.Covariance(self.Qc * np.identity(DOF))
        self.delta_t = self.total_time/(self.total_step - 1)

        # print("Problem Parameters:")
        # print("total_time:")
        # print(self.total_time)
        # print("total_step:")
        # print(self.total_step)
        # print("obs_check_inter:")
        # print(self.obs_check_inter)
        # print("control_inter:")
        # print(self.control_inter)
        # print("cost_sigma:")
        # print(self.cost_sigma)
        # print("total_time:")
        # print(self.total_time)
        # print("epsilon:")
        # print(self.epsilon)
        # print("Qc:")
        # print(self.Qc)
        # print("fix_pose_sigma:")
        # print(self.fix_pose_sigma)
        # print("fix_vel_sigma:")
        # print(self.fix_vel_sigma)
        # print("opt_type:")
        # print(self.opt_type)

        self.opt_setting = TrajOptimizerSetting(DOF)
        self.opt_setting.set_total_time(self.total_time)
        self.opt_setting.set_total_step(self.total_step - 1)
        self.opt_setting.set_obs_check_inter(self.obs_check_inter)
        self.opt_setting.set_cost_sigma(self.cost_sigma)
        self.opt_setting.set_epsilon(self.epsilon)
        self.opt_setting.set_Qc_model(self.Qc*np.identity(DOF))
        self.opt_setting.set_conf_prior_model(self.fix_pose_sigma)
        self.opt_setting.set_vel_prior_model(self.fix_vel_sigma)
        if self.opt_type == "LM":
            self.opt_setting.setLM()
        elif self.opt_type == "Dogleg":
            self.opt_setting.setDogleg()
        elif self.opt_type == "GaussNewton":
            self.opt_setting.setGaussNewton()
        else:
            print("Optimization type error!")

        # print("Finish Proble Loading.")

