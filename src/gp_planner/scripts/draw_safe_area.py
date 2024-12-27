import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 参数设置
r0 = 10  # 起始安全半径
rf = 2   # 最终安全半径
Tp = 5   # 时间持续长度（水平距离）

# 创建时间和角度的网格
x = np.linspace(0, Tp, 100)  # 沿 X 轴的距离，从 0 到 Tp
theta = np.linspace(0, 2 * np.pi, 100)  # 角度，从 0 到 2π
X, Theta = np.meshgrid(x, theta)

# 计算安全半径 r_safe(x)
R = r0 - (X / Tp) * (r0 - rf)  # 半径随 X 轴位置线性减小

# 将极坐标 (R, Theta) 转换为笛卡尔坐标 (Y, Z, X)
Y = R * np.cos(Theta)
Z = R * np.sin(Theta)

# 绘制 3D 图
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 绘制锥形区域
ax.plot_surface(X, Y, Z, alpha=0.7, cmap='viridis', edgecolor='none')

# 添加标签和标题
ax.set_title("3D Safe Radius Reduction (Horizontal Direction)", fontsize=14)
ax.set_xlabel("X (Time/Distance)", fontsize=12)
ax.set_ylabel("Y (m)", fontsize=12)
ax.set_zlabel("Z (m)", fontsize=12)

# 设置视角
ax.view_init(elev=30, azim=240)

# 显示图形
plt.show()