import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib import font_manager

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

# 定义 Sigmoid 函数
def sigmoid(x):
    return 1 / (1 + np.exp(-x))

# 生成 x 数据
x = np.linspace(-10, 10, 100)  # 在 -10 到 10 之间生成 100 个点
y = sigmoid(x)  # 计算 Sigmoid 函数值

# 绘制 Sigmoid 函数
plt.figure(figsize=(8, 6))  # 定义画布大小
plt.plot(x, y, label=r"$\sigma(x) = \frac{1}{1+e^{-x}}$", color="blue", linewidth=2)

# 添加标题和标签
plt.title("Sigmoid 函数", fontsize=20)
plt.xlabel("x", fontsize=16)
plt.ylabel(r"$\sigma(x)$", fontsize=16)

# 添加网格和图例
plt.grid(alpha=0.3)
plt.axhline(0.5, color="gray", linestyle="--", linewidth=1, label="y = 0.5")  # y = 0.5 的线
plt.legend(fontsize=12)

# 显示图形
plt.show()