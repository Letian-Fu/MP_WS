import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams
from matplotlib import font_manager

# # 打印所有系统可用字体路径
# for font in font_manager.findSystemFonts(fontpaths=None, fontext='ttf'):
#     print(font)
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

# 从文件读取数据
filename = "/home/roboert/MP_WS/src/gp_planner/exp_data/real_test_2.txt"
data = []
with open(filename, 'r') as file:
    for line in file:
        try:
            values = list(map(float, line.strip().split(',')))
            data.append(values)
        except ValueError:
            # 忽略可能的空行或无效行
            continue

# 将数据转换为 NumPy 数组
data = np.array(data)

# 检查数据是否正确
if data.shape[1] != 6:
    print("数据格式不正确，应该有 6 列（6 个关节角度）。")
else:
    # 绘制每个关节的角度变化
    time_steps = range(data.shape[0])  # 时间步长
    
    plt.figure(figsize=(12, 8))
    joint_labels = [f"关节 {i+1}" for i in range(data.shape[1])]

    for i in range(data.shape[1]):
        plt.plot(time_steps, data[:, i], label=joint_labels[i])

    plt.title("关节角度变化", fontsize=20)  # 手动设置标题字体大小
    plt.xlabel("步数", fontsize=18)  # 手动设置x轴标签字体大小
    plt.ylabel("关节角度 (度)", fontsize=18)  # 手动设置y轴标签字体大小
    # 设置坐标轴刻度字体大小
    plt.tick_params(axis='both', which='major', labelsize=20)  # 主刻度字体大小
    plt.tick_params(axis='both', which='minor', labelsize=16)  # 次刻度字体大小（如果有次刻度）

    plt.legend(fontsize=16)  # 设置图例字体大小
    plt.grid(True)

    # 保存图片或显示
    plt.savefig("/home/roboert/MP_WS/src/gp_planner/exp_data/joint_angles_variation.png", dpi=300)  # 保存为图片
    plt.show()  # 显示图片