import numpy as np
import matplotlib.pyplot as plt

# 定义分段函数
def L_hinge(d, epsilon):
    if d > epsilon:
        return 0
    elif 0 <= d <= epsilon:
        return (1 / (2 * epsilon)) * (d - epsilon) ** 2
    else:  # d < 0
        return (1 / 2) * epsilon - d

# 设置 epsilon 参数
epsilon = 1.0

# 创建输入范围 d(c)
d_values = np.linspace(-2 * epsilon, 2 * epsilon, 500)  # 从 -2ε 到 2ε 的范围
L_values = [L_hinge(d, epsilon) for d in d_values]     # 计算每个点的函数值

# 绘制函数
plt.figure(figsize=(8, 6))
plt.plot(d_values, L_values, label=r"$L_{\mathrm{hinge}}(d, \epsilon)$", color="blue", linewidth=2)
plt.axhline(0, color="black", linestyle="--", linewidth=0.8)
plt.axvline(0, color="black", linestyle="--", linewidth=0.8)
plt.title(r"$L_{\mathrm{hinge}}(d, \epsilon)$ Function", fontsize=14)
plt.xlabel(r"$d(c)$", fontsize=12)
plt.ylabel(r"$L_{\mathrm{hinge}}(c, \epsilon)$", fontsize=12)
plt.grid(alpha=0.3)
plt.legend(fontsize=12)
plt.show()