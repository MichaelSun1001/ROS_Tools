import os
import rosbag


def save_pointcloud2_to_txt(msg, filename, message_count):
    with open(filename, "a") as file:
        file.write(f"Message {message_count}:\n")

        # 写入头信息
        file.write("Header:\n")
        file.write(f"  seq: {msg.header.seq}\n")
        file.write(f"  stamp: {msg.header.stamp}\n")
        file.write(f"  frame_id: {msg.header.frame_id}\n")

        # 写入 PointCloud2 的基本信息
        file.write(f"Height: {msg.height}\n")
        file.write(f"Width: {msg.width}\n")

        # 写入字段信息
        file.write("Fields:\n")
        for field in msg.fields:
            file.write(f"  - name: {field.name}\n")
            file.write(f"    offset: {field.offset}\n")
            file.write(f"    datatype: {field.datatype}\n")
            file.write(f"    count: {field.count}\n")

        file.write(f"Is Big Endian: {'true' if msg.is_bigendian else 'false'}\n")
        file.write(f"Point Step: {msg.point_step}\n")
        file.write(f"Row Step: {msg.row_step}\n")
        file.write(f"Is Dense: {'true' if msg.is_dense else 'false'}\n")

        # 写入点云数据（十六进制表示），分块显示
        file.write("Data (hex):\n")
        for i in range(0, len(msg.data), 16):  # 每 16 个字节一行
            chunk = msg.data[i : i + 16]
            file.write(" ".join(f"{byte:02x}" for byte in chunk) + "\n")
        file.write("\n")


def save_pointcloud_to_txt(msg, filename, message_count):
    with open(filename, "a") as file:
        file.write(f"Message {message_count}:\n")

        # 写入头信息
        file.write("Header:\n")
        file.write(f"  seq: {msg.header.seq}\n")
        file.write(f"  stamp: {msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}\n")
        file.write(f"  frame_id: {msg.header.frame_id}\n")

        # 写入点信息
        file.write("Points:\n")
        for point in msg.points:
            file.write(f"  - x: {point.x}, y: {point.y}, z: {point.z}\n")

        # 写入通道信息
        file.write("Channels:\n")
        for channel in msg.channels:
            file.write(f"  - name: {channel.name}\n")
            file.write("    values:\n")
            for value in channel.values:
                file.write(f"      - {value}\n")

        file.write("\n")


def process_bag_file(bag_file, topics_to_check):
    if not os.path.isfile(bag_file):
        print(f"Error: Bag file '{bag_file}' not found.")
        return

    bag_base_name = os.path.splitext(os.path.basename(bag_file))[0]
    output_txt_pc2 = f"{bag_base_name}_PointCloud2.txt"
    output_txt_pc = f"{bag_base_name}_PointCloud.txt"

    with rosbag.Bag(bag_file, "r") as bag:
        message_count = 0
        for topic, msg, t in bag.read_messages(topics=topics_to_check):
            message_count += 1

            # 处理 PointCloud2
            if msg._type == "sensor_msgs/PointCloud2":
                save_pointcloud2_to_txt(msg, output_txt_pc2, message_count)

            # 处理 PointCloud
            elif msg._type == "sensor_msgs/PointCloud":
                save_pointcloud_to_txt(msg, output_txt_pc, message_count)

        print(f"Processed {message_count} messages from {bag_file}")


# 定义 bag 文件路径和 topics
bag_files = [
    "/media/sax/新加卷/2024年11月23日-速度试验/front-34s.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/front-41s.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/front-56s.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/front-goback-fast.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/front-goback-slow.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/lateral-35s.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/lateral-43s.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/lateral-49s.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/lateral-59s.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/lateral-goback-fast.bag",
    "/media/sax/新加卷/2024年11月23日-速度试验/lateral-goback-slow.bag",
]
topics_to_check = [
    "/ars548",
    "/hugin_raf_1/radar_data",
    "/ars548_process/point_cloud",
    "/ars548_process/point_cloud2",
    "/ars548_process/detection_point_cloud",
    "/radar/PointCloudDetection",
    "/point_cloud_raw",
]

# 执行处理
for bag_file in bag_files:
    process_bag_file(bag_file, topics_to_check)
