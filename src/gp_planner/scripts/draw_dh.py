import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
font_prop = font_manager.FontProperties(fname=font_path)
rcParams['font.sans-serif'] = font_prop.get_name()
rcParams['axes.unicode_minus'] = False

# 设置全局字体大小
rcParams['axes.labelsize'] = 16  # 坐标轴标签字体大小
rcParams['xtick.labelsize'] = 14  # x轴刻度字体大小
rcParams['ytick.labelsize'] = 14  # y轴刻度字体大小
rcParams['legend.fontsize'] = 14  # 图例字体大小
rcParams['figure.titlesize'] = 18  # 标题字体大小


def dh_to_transformation_matrix(theta, d, a, alpha):
    """
    根据DH参数计算齐次变换矩阵
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def compute_joint_positions(dh_params):
    """
    计算每个关节的位置
    """
    joint_positions = [np.eye(4)]  # 初始变换矩阵
    current_transform = np.eye(4)  # 初始变换矩阵为单位矩阵

    for i in range(len(dh_params)):
        theta, d, a, alpha = dh_params[i]
        # 计算当前关节的齐次变换矩阵
        transform = dh_to_transformation_matrix(theta, d, a, alpha)
        # 累积变换
        current_transform = current_transform @ transform
        joint_positions.append(current_transform)

    return joint_positions


def draw_cylinder(ax, start, end, radius=0.03, color='gray'):
    """
    绘制圆柱体表示连杆
    """
    # 计算圆柱体的方向向量
    direction = end - start
    height = np.linalg.norm(direction)
    direction = direction / height

    # 生成圆柱体的参数
    t = np.linspace(0, 2 * np.pi, 30)
    z = np.linspace(0, height, 20)
    t, z = np.meshgrid(t, z)

    # 圆柱体的横截面
    x = radius * np.cos(t)
    y = radius * np.sin(t)

    # 旋转到圆柱体的方向
    v = direction
    up = np.array([0, 0, 1])  # 默认向上的方向
    if not np.allclose(v, up):
        axis = np.cross(up, v)
        angle = np.arccos(np.dot(up, v))
        rotation_matrix = rotation_matrix_from_axis_angle(axis, angle)
        points = np.dot(rotation_matrix, np.array([x.flatten(), y.flatten(), z.flatten()]))
    else:
        points = np.array([x.flatten(), y.flatten(), z.flatten()])

    # 平移到起点
    points[0, :] += start[0]
    points[1, :] += start[1]
    points[2, :] += start[2]

    # 恢复形状并绘制
    x, y, z = points[0, :].reshape(t.shape), points[1, :].reshape(t.shape), points[2, :].reshape(t.shape)
    ax.plot_surface(x, y, z, color=color, alpha=0.8)


def rotation_matrix_from_axis_angle(axis, angle):
    """
    计算旋转矩阵 (Rodrigues 公式)
    """
    axis = axis / np.linalg.norm(axis)
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    cross_prod_matrix = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    return cos_angle * np.eye(3) + sin_angle * cross_prod_matrix + (1 - cos_angle) * np.outer(axis, axis)


def draw_coordinate_system(ax, origin, rotation, scale=0.1):
    """
    在给定位置绘制坐标系
    """
    x_axis = origin + scale * rotation[:, 0]
    y_axis = origin + scale * rotation[:, 1]
    z_axis = origin + scale * rotation[:, 2]

    # 绘制箭头
    ax.quiver(*origin, *(x_axis - origin), color='red', linewidth=2)
    ax.quiver(*origin, *(y_axis - origin), color='green', linewidth=2)
    ax.quiver(*origin, *(z_axis - origin), color='blue', linewidth=2)


def draw_mechanism(joint_positions):
    """
    绘制机械臂示意图，包括圆柱体和坐标轴
    """
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制连杆和关节
    for i in range(len(joint_positions) - 1):
        start = joint_positions[i][:3, 3]
        end = joint_positions[i + 1][:3, 3]
        draw_cylinder(ax, start, end)

    for i, transform in enumerate(joint_positions):
        origin = transform[:3, 3]
        rotation = transform[:3, :3]
        draw_coordinate_system(ax, origin, rotation)
        ax.scatter(*origin, color='black', s=50)  # 关节位置

    # 设置坐标系范围
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-0.2, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("CR5 机械臂示意图")
    plt.show()


# 根据表格定义 DH 参数
dh_params = [
    [0, 0.147, 0, 0],               # Joint 1
    [-np.pi/2, 0, 0, np.pi/2],      # Joint 2
    [0, 0, -0.427, 0],              # Joint 3
    [-np.pi/2, 0.141, -0.357, 0],   # Joint 4
    [0, 0.116, 0, np.pi/2],         # Joint 5
    [0, 0.105, 0, -np.pi/2]         # Joint 6
]

# 计算关节位置
joint_positions = compute_joint_positions(dh_params)

# 绘制机械臂
draw_mechanism(joint_positions)