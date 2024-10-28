import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取CSV文件
file_path = r"C:\Users\sunaxiang\Desktop\ROS_Tools\csv-pointcloud-visualizer\point_cloud_data - 4.csv"  # 替换成你的CSV文件路径
data = pd.read_csv(file_path, delimiter=",")  # 根据文件实际情况调整分隔符

# 动态获取坐标和属性列名
x_column = data.columns[data.columns.str.contains("x", case=False)].tolist()[0]
y_column = data.columns[data.columns.str.contains("y", case=False)].tolist()[0]
z_column = data.columns[data.columns.str.contains("z", case=False)].tolist()[0]
snr_column = data.columns[data.columns.str.contains("snr", case=False)].tolist()[0]
velocity_column = data.columns[
    data.columns.str.contains("velocity", case=False)
].tolist()[0]
noise_column = data.columns[data.columns.str.contains("noise", case=False)].tolist()[0]

# 提取坐标和属性
x = data[x_column]
y = data[y_column]
z = data[z_column]
snr = data[snr_column]  # 用于颜色映射
velocity = data[velocity_column]  # 用于点大小
noise = data[noise_column]  # 用作透明度

# 动态调整点的透明度
alpha = 1 - (noise - noise.min()) / (noise.max() - noise.min())
alpha = alpha.clip(0, 1)  # 确保透明度在 0 到 1 之间

# 可视化点云
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# 绘制散点图，颜色映射到 SNR 值，点的大小映射到 velocity
# sc = ax.scatter(
#     x, y, z, c=snr, s=velocity * 10, cmap="viridis", alpha=alpha
# )  # 使用 'viridis' 颜色映射，点大小与速度成正比

point_size = 5  # 设定一个固定的点大小
sc = ax.scatter(x, y, z, c=snr, s=point_size, cmap="viridis", alpha=alpha)


# 添加颜色条
cbar = plt.colorbar(sc, ax=ax, label="SNR")

# 设置坐标轴标签
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# 设置图形标题
ax.set_title("Point Cloud Visualization")

# 交互功能
plt.ion()  # 开启交互模式
plt.show()

# 让图形保持在屏幕上
plt.pause(0.001)
input("Press Enter to continue...")
