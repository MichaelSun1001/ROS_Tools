import os
import rosbag
import pandas as pd
import sensor_msgs.point_cloud2 as pc2

# 定义 bag 文件夹路径
bag_folder = "/home/sax/rosbags/test"
# 定义固定的输出路径
output_base_dir = "/home/sax/rosbags/test"  # 替换为您指定的路径

# 动态读取文件夹中的所有 .bag 文件
bag_files = [
    os.path.join(bag_folder, f) for f in os.listdir(bag_folder) if f.endswith(".bag")
]

# 定义要读取的 topic
topics_to_check = [
    "/ars548_process/point_cloud",
    "/ars548_process/point_cloud2",
    "/ars548",
    "/hugin_raf_1/radar_data",
]  # 根据实际的 topic 名称修改


def process_bag_file(bag_file):
    # 检查文件是否存在
    if not os.path.isfile(bag_file):
        print(f"Error: Bag file '{bag_file}' not found.")
        return

    # 从 bag 文件名生成文件夹结构
    bag_base_name = os.path.splitext(os.path.basename(bag_file))[0]

    # 假设从 bag 文件名中获取需要的场景信息，例如 "548-scene1-front"
    scene_info = (
        bag_base_name.split("-")[0] + "-" + bag_base_name.split("-")[1]
    )  # 例如 "548-scene1"

    # 输出目录结构: /media/sax/新加卷1/bag/test/548-scene1-front/话题/话题类型/
    output_topic_base_dir = os.path.join(output_base_dir, scene_info)

    # 确保文件夹存在
    os.makedirs(output_topic_base_dir, exist_ok=True)

    # 用于存储不同类型的数据
    all_data_pc2 = []
    all_data_pc = []

    # 初始化列名变量
    columns_pc2 = None
    columns_pc = None

    # 初始化每个话题的帧序号
    frame_id_pc2 = -1  # PointCloud2
    frame_id_pc = -1  # PointCloud

    # 读取当前 bag 文件
    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=topics_to_check):
            timestamp = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}"

            # 根据话题名和消息类型来决定保存路径
            topic_name = topic.strip("/").replace("/", "-")
            topic_type_dir = os.path.join(output_topic_base_dir, topic_name)

            # 确保话题文件夹存在
            os.makedirs(topic_type_dir, exist_ok=True)

            # 创建对应的目录分别存放 CSV 和 TXT 文件
            if msg._type == "sensor_msgs/PointCloud2":
                # 创建 PointCloud2 的保存路径
                output_csv_pc2 = os.path.join(
                    topic_type_dir, f"{bag_base_name}_PointCloud2.csv"
                )
                output_txt_pc2 = os.path.join(
                    topic_type_dir, f"{bag_base_name}_PointCloud2.txt"
                )
                os.makedirs(
                    os.path.dirname(output_csv_pc2), exist_ok=True
                )  # 确保目录存在
                os.makedirs(os.path.dirname(output_txt_pc2), exist_ok=True)

                # 处理 PointCloud2 数据
                frame_id_pc2 += 1
                if columns_pc2 is None:
                    columns_pc2 = ["frame_id", "timestamp"] + [
                        field.name for field in msg.fields
                    ]
                pc_gen = pc2.read_points(
                    msg,
                    field_names=[field.name for field in msg.fields],
                    skip_nans=True,
                )
                for point in pc_gen:
                    all_data_pc2.append(
                        (frame_id_pc2, timestamp) + tuple(point))

            elif msg._type == "sensor_msgs/PointCloud":
                # 创建 PointCloud 的保存路径
                output_csv_pc = os.path.join(
                    topic_type_dir, f"{bag_base_name}_PointCloud.csv"
                )
                output_txt_pc = os.path.join(
                    topic_type_dir, f"{bag_base_name}_PointCloud.txt"
                )
                os.makedirs(
                    os.path.dirname(output_csv_pc), exist_ok=True
                )  # 确保目录存在
                os.makedirs(os.path.dirname(output_txt_pc), exist_ok=True)

                # 处理 PointCloud 数据
                frame_id_pc += 1
                if columns_pc is None:
                    columns_pc = ["frame_id", "timestamp", "x", "y", "z"] + [
                        chan.name for chan in msg.channels
                    ]
                for i, point in enumerate(msg.points):
                    point_data = (point.x, point.y, point.z) + tuple(
                        chan.values[i] for chan in msg.channels
                    )
                    all_data_pc.append((frame_id_pc, timestamp) + point_data)

    # 保存 PointCloud2 数据到文件
    if all_data_pc2:
        df_pc2 = pd.DataFrame(all_data_pc2, columns=columns_pc2)
        df_pc2.to_csv(output_csv_pc2, index=False)  # 禁用索引
        print(f"PointCloud2 data has been saved to {output_csv_pc2}")
        df_pc2.to_csv(output_txt_pc2, sep=" ", index=False, header=True)
        print(f"PointCloud2 data has been saved to {output_txt_pc2}")

    # 保存 PointCloud 数据到文件
    if all_data_pc:
        df_pc = pd.DataFrame(all_data_pc, columns=columns_pc)
        df_pc.to_csv(output_csv_pc, index=False)  # 禁用索引
        print(f"PointCloud data has been saved to {output_csv_pc}")
        df_pc.to_csv(output_txt_pc, sep=" ", index=False, header=True)
        print(f"PointCloud data has been saved to {output_txt_pc}")


# 处理每个 bag 文件
for bag_file in bag_files:
    process_bag_file(bag_file)
