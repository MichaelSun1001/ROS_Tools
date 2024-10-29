import rosbag
import pandas as pd
import sensor_msgs.point_cloud2 as pc2

# 输入的 bag 文件和输出的 CSV 文件路径
bag_file = 'bag.bag'
output_csv = 'output.csv'
topic_name = '/hugin_raf_1/radar_data'

# 用于存储所有点云数据和额外信息
all_data = []

# 读取 bag 文件
with rosbag.Bag(bag_file, 'r') as bag:
    frame_id = 0  # 帧序号
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # 提取时间戳
        timestamp = msg.header.stamp.to_sec()  # 将时间戳转换为秒

        # 获取字段名称
        if frame_id == 0:
            columns = ['frame_id', 'timestamp'] + \
                [field.name for field in msg.fields]

        # 提取点云数据
        pc_gen = pc2.read_points(
            msg, field_names=columns[2:], skip_nans=True)  # 从第三列开始提取点云数据
        for point in pc_gen:
            # 将帧序号、时间戳和点云信息组合在一起
            all_data.append((frame_id, timestamp) + point)

        frame_id += 1  # 更新帧序号

# 创建 DataFrame
df = pd.DataFrame(all_data, columns=columns)

# 保存为 CSV 文件
df.to_csv(output_csv, index=False)
print(
    f"PointCloud2 data with frame ID and timestamp has been saved to {output_csv}")
