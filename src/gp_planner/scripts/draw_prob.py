import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 参数设置
Tp = 5  # 持续时间
t = np.linspace(0, Tp, 100)  # 时间
x = np.linspace(-5, 5, 100)  # 障碍物沿 X 轴的路径
T, X = np.meshgrid(t, x)  # 创建网格

# 概率分布公式
P_d = 1 - T / Tp

# 绘制 3D 曲面图
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 绘制曲面
surf = ax.plot_surface(T, X, P_d, cmap='viridis', edgecolor='none', alpha=0.8)

# 添加标题和标签
ax.set_title("Dynamic Obstacle Probability Distribution", fontsize=14)
ax.set_xlabel("Time $t$ (s)", fontsize=12)
ax.set_ylabel("Position $x$ (m)", fontsize=12)
ax.set_zlabel("Probability $P_d(t)$", fontsize=12)

# 添加颜色条
fig.colorbar(surf, shrink=0.5, aspect=10, label="Probability $P_d(t)$")

# 调整视角
ax.view_init(elev=30, azim=220)

plt.show()