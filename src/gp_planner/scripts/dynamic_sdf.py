#!/usr/bin/env python
import numpy as np
from scipy import ndimage
import math
import rospy
from std_msgs.msg import Float64MultiArray
import datetime
import threading


class DynamicSDF:
    def __init__(self):
        self.cols = 40
        self.rows = 40
        self.z = 30
        self.origin = np.array([-1.0, -1.0, -0.35])
        self.cell_size = 0.05
        self.map = np.zeros((self.rows, self.cols, self.z))
        self.sdf = np.zeros((self.rows, self.cols, self.z))
        self.prob_map = np.ones((self.rows, self.cols, self.z))
        self.epsilon = 0.05
        self.total_time = 1.0
        self.opt_setting_ = type('opt_setting', (object,), {'epsilon': self.epsilon})()

        rospy.init_node('dynamic_sdf')
        self.obs_sub_ = rospy.Subscriber("/obstacle_info", Float64MultiArray, self.obstacleCallback)
        self.is_rading_sub_ = rospy.Subscriber("/is_reading", Float64MultiArray, self.readingCallback)
        self.is_reading = 0
        self.pub = rospy.Publisher('/map_updated', Float64MultiArray, queue_size=10)
        self.lock = threading.Lock()  # 创建互斥锁
        self.timer = rospy.Timer(rospy.Duration(0.2), self.updateSDF)  # 创建定时器

    def updateSDF(self, event):
        with self.lock:  # 使用互斥锁
            if self.is_reading == 0:
                self.is_reading = 1
                # 模拟更新SDF
                output_file = "/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_dynamic_py.txt"
                self.saveSDFToSingleFile(self.sdf, output_file)
                print(f"save sdf at {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                msg = Float64MultiArray()
                msg.data = [3]
                self.pub.publish(msg)
                self.is_reading = 0

    def signedDistanceField3D(self, ground_truth_map, cell_size):
        # regularize unknow area to open area
        cur_map = ground_truth_map > 0.75
        cur_map = cur_map.astype(int)

        if np.amax(cur_map) == 0:
            return np.ones(ground_truth_map.shape) * 1000

        # inverse map
        inv_map = 1 - cur_map

        # get signed distance from map and inverse map
        # since bwdist(foo) = ndimage.distance_transform_edt(1-foo)
        map_dist = ndimage.distance_transform_edt(inv_map)
        inv_map_dist = ndimage.distance_transform_edt(cur_map)

        field = map_dist - inv_map_dist

        # metric
        field = field * cell_size
        field = field.astype(float)

        return field

    def saveSDFToSingleFile(self, sdf, output_file):
        with open(output_file, 'w') as file:
            msg = Float64MultiArray()
            msg.data = [1]
            self.pub.publish(msg)
            # Write metadata
            file.write(f"Origin: {self.origin[0]} {self.origin[1]} {self.origin[2]}\n")
            file.write(f"FieldRows: {sdf.shape[0]}\n")
            file.write(f"FieldCols: {sdf.shape[1]}\n")
            file.write(f"FieldZ: {sdf.shape[2]}\n")
            file.write(f"CellSize: {self.cell_size}\n")
            for z in range(sdf.shape[2]):
                # Write each layer
                for x in range(sdf.shape[1]):
                    for y in range(sdf.shape[0]):
                        file.write(f"{sdf[x, y, z]:.2f} ")
                    file.write("\n")
                file.write("\n")  # Add an extra newline for clarity between layers


    def add_obstacle(self, position, size, map):
        half_size_row = int(math.floor((size[0] - 1) / 2))
        half_size_col = int(math.floor((size[1] - 1) / 2))
        half_size_z = int(math.floor((size[2] - 1) / 2))

        # occupency grid
        map[position[0] - half_size_row - 1:position[0] + half_size_row,
            position[1] - half_size_col - 1:position[1] + half_size_col,
            position[2] - half_size_z - 1:position[2] + half_size_z, ] = np.ones(
                (2 * half_size_row + 1, 2 * half_size_col + 1,
                2 * half_size_z + 1))

        return map

    def add_dynamic_obstacle(self,position, size, velocity, direction, total_time, map, prob_map):
        # 计算障碍物的半径
        half_size_row = int(np.floor((size - 1) / 2))
        half_size_col = int(np.floor((size - 1) / 2))
        half_size_z = int(np.floor((size - 1) / 2))
        # velocity = 0
        # 计算障碍物的未来位置
        future_position = [
            int(round(position[0] + velocity * direction[0] * total_time)),
            int(round(position[1] + velocity * direction[1] * total_time)),
            int(round(position[2] + velocity * direction[2] * total_time))
        ]
        # 计算障碍物的移动距离
        distance_moved = velocity * total_time
        if distance_moved != 0:
            # 遍历障碍物的每个单元格，并在地图上设置为障碍物或衰减值
            for i in range(future_position[0] - half_size_row, future_position[0] + half_size_row + 1):
                for j in range(future_position[1] - half_size_col, future_position[1] + half_size_col + 1):
                    for k in range(future_position[2] - half_size_z, future_position[2] + half_size_z + 1):
                        if 0 <= i < map.shape[0] and 0 <= j < map.shape[1] and 0 <= k < map.shape[2]:
                            distance_to_future = np.sqrt((i - future_position[0]) ** 2 +
                                                        (j - future_position[1]) ** 2 +
                                                        (k - future_position[2]) ** 2)
                            decayed_distance = self.epsilon * (1.0 - distance_to_future / distance_moved)
                            probability = 1.0 - min(1.0, distance_to_future / distance_moved)
                            probability = max(0.0, min(1.0, probability))
                            prob_map[i, j, k] = probability
                            if decayed_distance > 0:
                                map[i, j, k] = 1.0  # 障碍物位置
                            else:
                                map[i, j, k] = max(0.0, map[i, j, k] - 0.1)  # 衰减值
        else:
            for i in range(future_position[0] - half_size_row, future_position[0] + half_size_row + 1):
                for j in range(future_position[1] - half_size_col, future_position[1] + half_size_col + 1):
                    for k in range(future_position[2] - half_size_z, future_position[2] + half_size_z + 1):
                        if 0 <= i < map.shape[0] and 0 <= j < map.shape[1] and 0 <= k < map.shape[2]:
                                map[i, j, k] = 1.0  # 障碍物位置

    def readingCallback(self, msg):
        if msg.data[0] == 1:
            self.is_reading = 1
        elif msg.data[0] == 0:
            self.is_reading = 0

    def obstacleCallback(self, msg):
        if len(msg.data) == 8:
            x = msg.data[0]
            y = msg.data[1]
            z = msg.data[2]
            obs_size = msg.data[3] + self.epsilon / 2
            linear_speed = msg.data[4]
            direction_x = msg.data[5]
            direction_y = msg.data[6]
            direction_z = msg.data[7]
            # 将全局坐标转换为栅格坐标
            grid_c = int(np.floor((x - self.origin[0]) / self.cell_size))
            grid_r = int(np.floor((y - self.origin[1]) / self.cell_size))
            grid_z = int(np.floor((z - self.origin[2]) / self.cell_size))

            position = [grid_r, grid_c, grid_z]
            direction = [direction_y, direction_x, direction_z]
            velocity = linear_speed / self.cell_size
            size = 2 * obs_size / self.cell_size
            if velocity != 0:
                # 调用add_dynamic_obstacle函数
                self.add_dynamic_obstacle(position, size, velocity, direction, self.total_time, self.map, self.prob_map)
                self.sdf = self.signedDistanceField3D(self.map, self.cell_size)
                # if self.is_reading == 0:
                #     # Save the SDF to a single file
                #     output_file = "/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_dynamic_py.txt"
                #     self.saveSDFToSingleFile(sdf, output_file)
                #     # 打印带有时间戳的信息
                #     print(f"save sdf at {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                #     # 发布消息
                #     msg = Float64MultiArray()
                #     msg.data = [3]
                #     self.pub.publish(msg)
            else :
                msg = Float64MultiArray()
                msg.data = [2]
                self.pub.publish(msg)


# Example usage
if __name__ == "__main__":
    dynamicsdf = DynamicSDF()
    dynamicsdf.map = dynamicsdf.add_obstacle([20, 20, 4], [30, 30, 3], dynamicsdf.map)
    dynamicsdf.sdf = dynamicsdf.signedDistanceField3D(dynamicsdf.map,dynamicsdf.cell_size)
    rospy.spin()

    