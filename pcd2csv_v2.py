import open3d as o3d
import numpy as np
import os


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

    print(f"Converted {pcd_file} to {csv_file}")


def convert_pcd_folder_to_csv(input_folder, output_folder):
    # 如果输出文件夹不存在，则创建它
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 遍历输入文件夹中的所有 PCD 文件
    for file_name in os.listdir(input_folder):
        if file_name.endswith('.pcd'):
            input_pcd_file = os.path.join(input_folder, file_name)
            output_csv_file = os.path.join(
                output_folder, f"{file_name[:-4]}.csv")  # 使用相同的文件名，仅更改扩展名
            pcd_to_csv(input_pcd_file, output_csv_file)


# 替换以下文件夹路径为实际的文件夹路径
input_folder = "/media/sax/00426AEBE77FC6E9/v1.0-trainval01_blobs/samples/RADAR_FRONT"
output_folder = "/media/sax/00426AEBE77FC6E9/v1.0-trainval01_blobs/samples/RADAR_FRONT/csv"
# output_folder = "/home/sax/csv"

# 执行转换
convert_pcd_folder_to_csv(input_folder, output_folder)

print("PCD to CSV conversion complete.")
