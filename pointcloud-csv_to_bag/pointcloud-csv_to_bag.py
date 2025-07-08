import rospy
import pandas as pd
import sensor_msgs.msg
import std_msgs.msg
import rosbag
import numpy as np
from sensor_msgs import point_cloud2


def csv_to_pointcloud(csv_file, bag_file):
    # 读取 CSV 文件
    df = pd.read_csv(csv_file)

    # 检查数据中是否有时间戳列
    if 'timestamp' not in df.columns:
        raise ValueError("CSV文件必须包含'timestamp'列用于标识点云帧")

    # 初始化 ROS 节点
    rospy.init_node('csv_to_pointcloud_node', anonymous=True)

    # 创建 ROS bag 文件
    with rosbag.Bag(bag_file, 'w') as bag:
        # 按时间戳分组处理数据
        grouped = df.groupby('timestamp')

        # 遍历每个时间戳（帧）
        for timestamp_val, frame_data in grouped:
            # 将时间戳转换为浮点数
            timestamp_sec = float(timestamp_val)

            # 创建 ROS 时间对象
            secs = int(timestamp_sec)
            nsecs = int((timestamp_sec - secs) * 1e9)
            current_time = rospy.Time(secs, nsecs)

            # 创建 PointCloud2 消息头
            header = std_msgs.msg.Header()
            header.stamp = current_time
            header.frame_id = 'radar'  # 根据实际需要修改坐标系

            # 提取点坐标和属性
            points = frame_data[['X', 'Y', 'Z']].values
            rcs = frame_data['RCS'].values

            # 组合点云数据（XYZ + RCS）
            structured_data = []
            for i in range(len(points)):
                x, y, z = points[i]
                structured_data.append([x, y, z, rcs[i]])

            # 定义 PointCloud2 消息字段
            fields = [
                point_cloud2.PointField(name='x', offset=0,
                                        datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='y', offset=4,
                                        datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='z', offset=8,
                                        datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='rcs', offset=12,
                                        datatype=point_cloud2.PointField.FLOAT32, count=1)
            ]

            # 创建点云消息
            cloud = point_cloud2.create_cloud(header, fields, structured_data)

            # 写入ROS bag
            bag.write('/point_cloud', cloud, current_time)

            print(
                f"Processed timestamp: {timestamp_val} with {len(frame_data)} points")


if __name__ == '__main__':
    csv_file = '/home/sax/rosbags/train_head.csv'  # CSV文件路径
    bag_file = '/home/sax/rosbags/train_head.bag'     # 输出bag文件路径

    try:
        csv_to_pointcloud(csv_file, bag_file)
        print("Successfully created bag file with original timestamps")
    except Exception as e:
        print(f"Error: {str(e)}")
