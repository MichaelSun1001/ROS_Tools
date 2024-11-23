import os
import rosbag
import pandas as pd

# 定义多个 bag 文件路径
bag_files = [
    '/media/sax/新加卷/FDI-dynamic.bag',
    '/media/sax/新加卷/FDI_static.bag',
    '/media/sax/新加卷/待测548数据集/jf_test3.bag'
]  # 可以添加更多 bag 文件路径

# 定义要读取的 topic
topics_to_check = ['/imu', '/imu/data_raw', '/sensor/imu']  # 根据实际的 topic 名称修改


def process_bag_file(bag_file):
    # 检查文件是否存在
    if not os.path.isfile(bag_file):
        print(f"Error: Bag file '{bag_file}' not found.")
        return

    # 从 bag 文件名生成 CSV 和 TXT 文件名
    bag_base_name = os.path.splitext(os.path.basename(bag_file))[0]
    output_csv_imu = f'{bag_base_name}_IMU.csv'
    output_txt_imu = f'{bag_base_name}_IMU.txt'

    # 用于存储 IMU 数据
    all_data_imu = []

    # 读取当前 bag 文件
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=topics_to_check):
            if msg._type == 'sensor_msgs/Imu':
                # 提取 IMU 数据
                # 合并秒和纳秒部分
                timestamp = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}"
                frame_id = msg.header.frame_id  # 帧 ID
                orientation = (msg.orientation.x, msg.orientation.y,
                               msg.orientation.z, msg.orientation.w)  # 四元数
                orientation_covariance = tuple(
                    msg.orientation_covariance)  # 方向的协方差矩阵 (9个元素)
                angular_velocity = (
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)  # 角速度
                angular_velocity_covariance = tuple(
                    msg.angular_velocity_covariance)  # 角速度的协方差矩阵 (9个元素)
                linear_acceleration = (
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)  # 线性加速度
                linear_acceleration_covariance = tuple(
                    msg.linear_acceleration_covariance)  # 线性加速度的协方差矩阵 (9个元素)

                # 将数据添加到列表中
                all_data_imu.append((timestamp, frame_id) + orientation + orientation_covariance +
                                    angular_velocity + angular_velocity_covariance +
                                    linear_acceleration + linear_acceleration_covariance)

    # 如果有 IMU 数据，保存到文件
    if all_data_imu:
        columns_imu = ['timestamp', 'frame_id', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                       'orientation_covariance_0', 'orientation_covariance_1', 'orientation_covariance_2',
                       'orientation_covariance_3', 'orientation_covariance_4', 'orientation_covariance_5',
                       'orientation_covariance_6', 'orientation_covariance_7', 'orientation_covariance_8',
                       'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                       'angular_velocity_covariance_0', 'angular_velocity_covariance_1', 'angular_velocity_covariance_2',
                       'angular_velocity_covariance_3', 'angular_velocity_covariance_4', 'angular_velocity_covariance_5',
                       'angular_velocity_covariance_6', 'angular_velocity_covariance_7', 'angular_velocity_covariance_8',
                       'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
                       'linear_acceleration_covariance_0', 'linear_acceleration_covariance_1', 'linear_acceleration_covariance_2',
                       'linear_acceleration_covariance_3', 'linear_acceleration_covariance_4', 'linear_acceleration_covariance_5',
                       'linear_acceleration_covariance_6', 'linear_acceleration_covariance_7', 'linear_acceleration_covariance_8']

        # 保存为 CSV 文件
        df_imu = pd.DataFrame(all_data_imu, columns=columns_imu)
        df_imu.to_csv(output_csv_imu, index=False)
        print(f"IMU data has been saved to {output_csv_imu}")

        # 保存为 TXT 文件
        df_imu.to_csv(output_txt_imu, sep=' ', index=False, header=False)
        print(f"IMU data has been saved to {output_txt_imu}")


# 处理每个 bag 文件
for bag_file in bag_files:
    process_bag_file(bag_file)
