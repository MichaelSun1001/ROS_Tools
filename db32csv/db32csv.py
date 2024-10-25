import pandas as pd
import rclpy
from rclpy.node import Node
from rosbag2_api import Rosbag2Reader
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def extract_pointcloud_data(db3_file, output_csv):
    rclpy.init()
    reader = Rosbag2Reader()
    reader.open(db3_file)

    pointcloud_data = []
    columns = []

    while reader.has_next():
        topic, msg, timestamp = reader.read_next()

        if topic == '/your_pointcloud_topic':  # 替换为你的点云话题
            # 获取字段名称（仅在第一次读取时设置）
            if not columns:
                columns = [field.name for field in msg.fields]

            # 读取点云数据
            pc_gen = pc2.read_points(msg, field_names=columns, skip_nans=True)
            pointcloud_data.extend(list(pc_gen))

    # 将点云数据转换为 DataFrame
    df = pd.DataFrame(pointcloud_data, columns=columns)

    # 保存为 CSV 文件
    df.to_csv(output_csv, index=False)
    print(f'PointCloud2 data has been saved to {output_csv}')

    rclpy.shutdown()


if __name__ == '__main__':
    db3_file = 'your_file.db3'  # 替换为你的 db3 文件名
    output_csv = 'output.csv'     # 输出 CSV 文件名
    extract_pointcloud_data(db3_file, output_csv)
