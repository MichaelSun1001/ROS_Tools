import csv
import rosbag
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math  # 用于角度转弧度

# CSV 文件路径
csv_file_path = '/media/sax/新加卷/常工航天光学/2024-11-5.csv'

# 创建 rosbag 文件
bag = rosbag.Bag('imu_data.bag', 'w')

# 打开 CSV 文件并读取数据
with open(csv_file_path, 'r') as csvfile:
    reader = csv.reader(csvfile)

    # 跳过第一行（表头）
    header_row = next(reader)

    # 获取第一个时间戳，作为基准时间（单位 ms）
    first_row = next(reader)
    first_timestamp_ms = int(first_row[7])  # 假设时间戳在第 8 列

    # 写入第一行数据
    for row in reader:
        current_timestamp_ms = int(row[7])  # 当前行的时间戳（单位 ms）

        # 计算相对于第一个时间戳的时间
        elapsed_time_ms = current_timestamp_ms - first_timestamp_ms

        # 转换为 ROS 时间戳格式（秒和纳秒）
        secs = elapsed_time_ms // 1000
        nsecs = (elapsed_time_ms % 1000) * 1_000_000

        # 创建 IMU 消息
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.frame_id = 'imu_frame'
        imu_msg.header.stamp.secs = secs
        imu_msg.header.stamp.nsecs = nsecs

        # 填充 IMU 数据
        imu_msg.linear_acceleration.x = float(row[1])
        imu_msg.linear_acceleration.y = float(row[2])
        imu_msg.linear_acceleration.z = float(row[3])
        imu_msg.angular_velocity.x = float(row[4]) * math.pi / 180  # 转换为 rad/s
        imu_msg.angular_velocity.y = float(row[5]) * math.pi / 180  # 转换为 rad/s
        imu_msg.angular_velocity.z = float(row[6]) * math.pi / 180  # 转换为 rad/s

        # 写入 Bag 文件
        bag.write('/imu/data', imu_msg)

# 关闭 Bag 文件
bag.close()
