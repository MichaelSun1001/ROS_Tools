import open3d as o3d
import numpy as np


def pcd_to_csv(pcd_file, csv_file):
    # 读取 PCD 文件
    point_cloud = o3d.io.read_point_cloud(pcd_file)

    # 将点云数据转换为 NumPy 数组
    points = np.asarray(point_cloud.points)

    # 打开 CSV 文件进行写操作
    with open(csv_file, 'w') as csv_file:
        # 写入 CSV 文件的列标题
        csv_file.write("x,y,z\n")

        # 遍历点云并写入 CSV 文件
        for point in points:
            csv_file.write(f"{point[0]},{point[1]},{point[2]}\n")

    print("PCD to CSV conversion complete.")


# 替换以下文件路径为实际的文件路径
input_pcd_file = "/home/sax/n008-2018-08-01-15-16-36-0400__RADAR_BACK_LEFT__1533151061567861.pcd"
output_csv_file = "/home/sax/output_file.csv"

# 执行转换
pcd_to_csv(input_pcd_file, output_csv_file)
