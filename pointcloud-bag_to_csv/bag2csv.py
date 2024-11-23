import os
import rosbag
import pandas as pd
import sensor_msgs.point_cloud2 as pc2

# 定义多个 bag 文件路径
bag_files = [
    # '/media/sax/新加卷/2024年11月23日-速度试验/front-34s.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/front-41s.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/front-56s.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/front-goback-fast.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/front-goback-slow.bag',
    
    # '/media/sax/新加卷/2024年11月23日-速度试验/lateral-35s.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/lateral-43s.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/lateral-49s.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/lateral-59s.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/lateral-goback-fast.bag',
    # '/media/sax/新加卷/2024年11月23日-速度试验/lateral-goback-slow.bag',
    
    '/media/sax/新加卷/2024年11月23日-速度试验/new-can.bag',
    '/media/sax/新加卷/2024年11月23日-速度试验/new-can+new-tplink.bag',

    # '/media/sax/新加卷/2024年11月23日-速度试验/front-34s.bag',

]  # 可以添加更多 bag 文件路径

# 定义要读取的 topic
topics_to_check = ['/ars548', '/hugin_raf_1/radar_data',  '/ars548_process/point_cloud','/ars548_process/point_cloud2',
                   '/ars548_process/detection_point_cloud', '/radar/PointCloudDetection', '/point_cloud_raw']  # 根据实际的 topic 名称修改


def process_bag_file(bag_file):
    # 检查文件是否存在
    if not os.path.isfile(bag_file):
        print(f"Error: Bag file '{bag_file}' not found.")
        return

    # 从 bag 文件名生成 CSV 和 TXT 文件名
    bag_base_name = os.path.splitext(os.path.basename(bag_file))[0]
    output_csv_pc2 = f'{bag_base_name}_PointCloud2.csv'
    output_txt_pc2 = f'{bag_base_name}_PointCloud2.txt'
    output_csv_pc = f'{bag_base_name}_PointCloud.csv'
    output_txt_pc = f'{bag_base_name}_PointCloud.txt'

    # 用于存储不同类型的数据
    all_data_pc2 = []
    all_data_pc = []

    # 初始化列名变量
    columns_pc2 = None
    columns_pc = None

    # 初始化每个话题的帧序号
    frame_id_pc2 = -1  # PointCloud2
    frame_id_pc = -1   # PointCloud

    # 读取当前 bag 文件
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=topics_to_check):
            timestamp = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}"

            # 处理 PointCloud2 数据
            if msg._type == 'sensor_msgs/PointCloud2':
                frame_id_pc2 += 1  # 仅递增 PointCloud2 的帧序号
                if columns_pc2 is None:  # 初始化列名
                    columns_pc2 = ['frame_id', 'timestamp'] + \
                        [field.name for field in msg.fields]
                # 提取所有点的数据
                pc_gen = pc2.read_points(
                    msg, field_names=[field.name for field in msg.fields], skip_nans=True)
                for point in pc_gen:
                    all_data_pc2.append((frame_id_pc2, timestamp) + tuple(point))

            # 处理 PointCloud 数据
            elif msg._type == 'sensor_msgs/PointCloud':
                frame_id_pc += 1  # 仅递增 PointCloud 的帧序号
                if columns_pc is None:  # 初始化列名
                    columns_pc = ['frame_id', 'timestamp', 'x', 'y',
                                  'z'] + [chan.name for chan in msg.channels]
                # 提取所有点的数据
                for i, point in enumerate(msg.points):
                    point_data = (point.x, point.y, point.z) + \
                        tuple(chan.values[i] for chan in msg.channels)
                    all_data_pc.append((frame_id_pc, timestamp) + point_data)

    # 保存 PointCloud2 数据到文件
    if all_data_pc2:
        df_pc2 = pd.DataFrame(all_data_pc2, columns=columns_pc2)
        df_pc2.to_csv(output_csv_pc2, index=False)  # 禁用索引
        print(f"PointCloud2 data has been saved to {output_csv_pc2}")
        df_pc2.to_csv(output_txt_pc2, sep=' ', index=False, header=True)
        print(f"PointCloud2 data has been saved to {output_txt_pc2}")

    # 保存 PointCloud 数据到文件
    if all_data_pc:
        df_pc = pd.DataFrame(all_data_pc, columns=columns_pc)
        df_pc.to_csv(output_csv_pc, index=False)  # 禁用索引
        print(f"PointCloud data has been saved to {output_csv_pc}")
        df_pc.to_csv(output_txt_pc, sep=' ', index=False, header=True)
        print(f"PointCloud data has been saved to {output_txt_pc}")



# 处理每个 bag 文件
for bag_file in bag_files:
    process_bag_file(bag_file)
