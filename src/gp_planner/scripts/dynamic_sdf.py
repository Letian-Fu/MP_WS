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
        # self.cols = 80
        # self.rows = 80
        # self.z = 60
        # self.origin = np.array([-1.0, -1.0, -0.35])
        # self.cell_size = 0.025
        self.map = np.zeros((self.rows, self.cols, self.z))
        self.sdf = np.zeros((self.rows, self.cols, self.z))
        self.prob_map = np.ones((self.rows, self.cols, self.z))
        self.epsilon = 0
        self.total_time = 1.2
        self.total_steps = 10
        self.opt_setting_ = type('opt_setting', (object,), {'epsilon': self.epsilon})()
        self.update_flag = False

        rospy.init_node('dynamic_sdf')
        self.obs_sub_ = rospy.Subscriber("/obstacle_info", Float64MultiArray, self.obstacleCallback)
        # self.obs_sub_detected_ = rospy.Subscriber("/obstacle_info_detected", Float64MultiArray, self.obstacleDetectedCallback)
        self.is_rading_sub_ = rospy.Subscriber("/is_reading", Float64MultiArray, self.readingCallback)
        self.is_reading = 0
        self.pub = rospy.Publisher('/map_updated', Float64MultiArray, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.updateSDF)  # 创建定时器

    def updateSDF(self, event):
        # 模拟更新SDF
        if self.update_flag == True:
            output_file = "/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_dynamic_py.txt"
            self.saveSDFToSingleFile(self.sdf, output_file)
            print(f"save sdf at {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            msg = Float64MultiArray()
            msg.data = [3]
            self.pub.publish(msg)
            self.is_reading = 0
            self.update_flag = False

    def signedDistanceField3D(self, ground_truth_map, cell_size, prob_map):
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
        field = field * prob_map
        field = np.maximum(field, 0).astype(float)
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
                        file.write(f"{sdf[x, y, z]:.4f} ")
                    file.write("\n")
                file.write("\n")  # Add an extra newline for clarity between layers

    def add_obstacle(self, position, size, map):
        half_size_row = int(math.floor((size[0] - 1) / 2))
        half_size_col = int(math.floor((size[1] - 1) / 2))
        half_size_z = int(math.floor((size[2] - 1) / 2))

        start_row = max(0, position[0] - half_size_row - 1)
        end_row = min(map.shape[0], position[0] + half_size_row)

        start_col = max(0, position[1] - half_size_col - 1)
        end_col = min(map.shape[1], position[1] + half_size_col)

        start_z = max(0, position[2] - half_size_z - 1)
        end_z = min(map.shape[2], position[2] + half_size_z)

        fill_size_row = end_row - start_row
        fill_size_col = end_col - start_col
        fill_size_z = end_z - start_z
        map[start_row:end_row, start_col:end_col, start_z:end_z] = np.ones((fill_size_row, fill_size_col, fill_size_z))

        # # occupency grid
        # map[position[0] - half_size_row - 1:position[0] + half_size_row,
        #     position[1] - half_size_col - 1:position[1] + half_size_col,
        #     position[2] - half_size_z - 1:position[2] + half_size_z] = np.ones(
        #         (2 * half_size_row + 1, 2 * half_size_col + 1,
        #         2 * half_size_z + 1))

        return map

    def add_dynamic_obstacle(self,position, size, velocity, direction, total_time, total_steps, map, prob_map):
        # 计算障碍物的半径
        half_size_row = int(np.floor((size - 1) / 2))
        half_size_col = int(np.floor((size - 1) / 2))
        half_size_z = int(np.floor((size - 1) / 2))
        time_step = total_time/total_steps
        for step in range(total_steps):
            epsilon = self.epsilon * (1 - step/total_steps)
            # 计算障碍物的未来位置
            current_position = [
                int(round(position[0] + velocity * direction[0] * step * time_step)),
                int(round(position[1] + velocity * direction[1] * step * time_step)),
                int(round(position[2] + velocity * direction[2] * step * time_step))
            ]
            # current_position = [
            #     int(round(position[0] + velocity * direction[0] * step * time_step + epsilon / self.cell_size)),
            #     int(round(position[1] + velocity * direction[1] * step * time_step + epsilon / self.cell_size)),
            #     int(round(position[2] + velocity * direction[2] * step * time_step + epsilon / self.cell_size))
            # ]
            # 检查当前点是否超出边界
            if (current_position[0] < 0 or current_position[0] >= map.shape[0] or
                current_position[1] < 0 or current_position[1] >= map.shape[1] or
                current_position[2] < 0 or current_position[2] >= map.shape[2]):
                print(f"Skipping out-of-bounds position: {current_position}")
                continue
            # 计算障碍物的移动距离
            # distance_moved = velocity * time_step
            probability = 1.0 - (0.4 * step / total_steps)
            start_row = max(0, current_position[0] - half_size_row - 1)
            end_row = min(map.shape[0], current_position[0] + half_size_row)

            start_col = max(0, current_position[1] - half_size_col - 1)
            end_col = min(map.shape[1], current_position[1] + half_size_col)

            start_z = max(0, current_position[2] - half_size_z - 1)
            end_z = min(map.shape[2], current_position[2] + half_size_z)

            # 动态计算填充值大小
            fill_size_row = end_row - start_row
            fill_size_col = end_col - start_col
            fill_size_z = end_z - start_z
            map[start_row:end_row, start_col:end_col, start_z:end_z] = np.ones((fill_size_row, fill_size_col, fill_size_z))
            prob_map[start_row:end_row, start_col:end_col, start_z:end_z] = probability * np.ones((fill_size_row, fill_size_col, fill_size_z))

    def add_dynamic_obstacle_static(self,position, size, velocity, direction, total_time, total_steps, map, prob_map):
        # 计算障碍物的半径
        half_size_row = int(np.floor((size - 1) / 2))
        half_size_col = int(np.floor((size - 1) / 2))
        half_size_z = int(np.floor((size - 1) / 2))
        # 计算障碍物的未来位置
        current_position = [
            int(round(position[0])),
            int(round(position[1])),
            int(round(position[2]))
        ]
        # 计算障碍物的移动距离
        # distance_moved = velocity * time_step
        start_row = max(0, current_position[0] - half_size_row - 1)
        end_row = min(map.shape[0], current_position[0] + half_size_row)

        start_col = max(0, current_position[1] - half_size_col - 1)
        end_col = min(map.shape[1], current_position[1] + half_size_col)

        start_z = max(0, current_position[2] - half_size_z - 1)
        end_z = min(map.shape[2], current_position[2] + half_size_z)

        # 动态计算填充值大小
        fill_size_row = end_row - start_row
        fill_size_col = end_col - start_col
        fill_size_z = end_z - start_z
        map[start_row:end_row, start_col:end_col, start_z:end_z] = np.ones((fill_size_row, fill_size_col, fill_size_z))

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
            obs_size = msg.data[3]
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
            size = 2 * obs_size / self.cell_size + self.epsilon / self.cell_size
            self.map = np.zeros((self.rows, self.cols, self.z))
            self.sdf = np.zeros((self.rows, self.cols, self.z))
            self.prob_map = np.ones((self.rows, self.cols, self.z))
            # floor
            self.add_obstacle([20, 20, 5], [30, 30, 3], self.map)
            # pingmu
            self.add_obstacle([6, 12, 10], [2, 6, 6], self.map)
            # zawu
            self.add_obstacle([34, 15, 10], [2, 8, 6], self.map)
            # shebei
            self.add_obstacle([18, 1, 9], [6, 6, 4], self.map)

            if velocity != 0:
                # 调用add_dynamic_obstacle函数
                self.add_dynamic_obstacle(position, size, velocity, direction, self.total_time, self.total_steps,self.map, self.prob_map)
                # self.add_dynamic_obstacle_static(position, size, velocity, direction, self.total_time, self.total_steps,self.map, self.prob_map)
                self.sdf = self.signedDistanceField3D(self.map, self.cell_size,self.prob_map)
                self.update_flag = True
            else :
                msg = Float64MultiArray()
                msg.data = [2]
                self.pub.publish(msg)

    def obstacleDetectedCallback(self, msg):
        if len(msg.data) == 8:
            x = msg.data[0]
            y = msg.data[1]
            z = msg.data[2]
            obs_size = msg.data[3]
            linear_speed = msg.data[4]
            direction_x = msg.data[5]
            direction_y = msg.data[6]
            direction_z = msg.data[7]
            # 打印提取的数据
            print(f"x: {x}, y: {y}, z: {z}, obs_size: {obs_size}, linear_speed: {linear_speed}, direction: ({direction_x}, {direction_y}, {direction_z})")

            # 将全局坐标转换为栅格坐标
            grid_c = int(np.floor((x - self.origin[0]) / self.cell_size))
            grid_r = int(np.floor((y - self.origin[1]) / self.cell_size))
            grid_z = int(np.floor((z - self.origin[2]) / self.cell_size))
            position = [grid_r, grid_c, grid_z]
            direction = [direction_y, direction_x, direction_z]
            velocity = linear_speed / self.cell_size
            size = obs_size / self.cell_size
            self.map = np.zeros((self.rows, self.cols, self.z))
            self.sdf = np.zeros((self.rows, self.cols, self.z))
            self.prob_map = np.ones((self.rows, self.cols, self.z))
            # floor
            self.add_obstacle([20, 20, 5], [30, 30, 3], self.map)
            # pingmu
            self.add_obstacle([8, 12, 10], [2, 6, 6], self.map)
            # zawu
            self.add_obstacle([34, 15, 10], [2, 8, 6], self.map)
            # shebei
            self.add_obstacle([18, 2, 8], [5, 5, 2], self.map)
            if velocity != 0:
                # 调用add_dynamic_obstacle函数
                self.add_dynamic_obstacle(position, size, velocity, direction, self.total_time, self.total_steps,self.map, self.prob_map)
                # self.add_dynamic_obstacle_static(position, size, velocity, direction, self.total_time, self.total_steps,self.map, self.prob_map)
                self.sdf = self.signedDistanceField3D(self.map, self.cell_size,self.prob_map)
                self.update_flag = True
            else :
                msg = Float64MultiArray()
                msg.data = [2]
                self.pub.publish(msg)


# Example usage
if __name__ == "__main__":
    dynamicsdf = DynamicSDF()
    # dynamicsdf.map = dynamicsdf.add_obstacle([20, 20, 5], [30, 30, 3], dynamicsdf.map)
    dynamicsdf.sdf = dynamicsdf.signedDistanceField3D(dynamicsdf.map,dynamicsdf.cell_size,dynamicsdf.prob_map)
    rospy.spin()

    