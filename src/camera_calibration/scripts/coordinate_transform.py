import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_coordinate_system(ax, R, t, label, scale=0.2):
    """
    绘制一个坐标系
    :param ax: matplotlib 3D 坐标系
    :param R: 旋转矩阵 (3x3)
    :param t: 平移向量 (3x1 或 1x3)
    :param label: 坐标系标签
    :param scale: 坐标轴长度比例
    """
    origin = t.flatten()  # 展平平移向量为1D数组
    x_axis = origin + scale * R[:, 0]
    y_axis = origin + scale * R[:, 1]
    z_axis = origin + scale * R[:, 2]

    # 绘制坐标轴
    ax.quiver(*origin, *(x_axis - origin), color='r', label=f'{label} x', arrow_length_ratio=0.1)  # X轴
    ax.quiver(*origin, *(y_axis - origin), color='g', label=f'{label} y', arrow_length_ratio=0.1)  # Y轴
    ax.quiver(*origin, *(z_axis - origin), color='b', label=f'{label} z', arrow_length_ratio=0.1)  # Z轴
    ax.text(*origin, f'{label}', color='black')  # 添加标签

# 手眼标定结果
R_camera_to_base = np.array([[0.001452523670781414, 0.4303348593991797, 0.9026681555039339],
                             [-0.7848845283470771, 0.5598214048445991, -0.265624305814068],
                             [-0.6196403532181971, -0.7081044438949013, 0.3385763269927777]])
t_camera_to_base = np.array([-1.208500077908652, 0.4514269013863701, 0.2478105949936489])

# 创建3D绘图
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# 绘制基坐标系 (Identity 矩阵表示基坐标系)
plot_coordinate_system(ax, np.eye(3), np.zeros(3), 'Base', scale=0.3)

# 绘制相机坐标系
plot_coordinate_system(ax, R_camera_to_base, t_camera_to_base, 'Camera', scale=0.3)

# 设置显示范围和标签
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Coordinate System Visualization')
plt.legend()
plt.show()