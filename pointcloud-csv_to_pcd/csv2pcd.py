import pandas as pd


def csv_to_pcd(csv_file_path, pcd_file_path, required_columns):
    try:
        # 读取CSV文件
        df = pd.read_csv(csv_file_path)

        # 检查是否包含所需的列
        for col in required_columns:
            if col not in df.columns:
                raise ValueError(f"Missing required column: {col}")

        # 提取点云数据
        points = df[required_columns].values

        # 构建FIELDS、SIZE和TYPE字符串
        fields_str = " ".join(required_columns)
        size_str = " ".join(["4"] * len(required_columns))
        type_str = " ".join(["F"] * len(required_columns))

        # 定义PCD文件头
        pcd_header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS {fields_str}
SIZE {size_str}
TYPE {type_str}
COUNT {"1 " * len(required_columns)}
WIDTH {points.shape[0]}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {points.shape[0]}
DATA ascii
"""

        # 写入PCD文件
        with open(pcd_file_path, "w") as f:
            f.write(pcd_header)
            for point in points:
                f.write(" ".join(map(str, point)) + "\n")

        print(f"PCD file saved to {pcd_file_path}")

    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    # 指定CSV文件路径和输出PCD文件路径
    csv_file_path = "/home/sax/db3_2_pcd/underground-human_static/rosbag2_2023_10_14-21_22_54_0.db/radar_points.csv"
    pcd_file_path = "/home/sax/db3_2_pcd/underground-human_static/rosbag2_2023_10_14-21_22_54_0.db/radar_points.pcd"

    # 指定需要的列
    required_columns = [
        "X_m",
        "Y_m",
        "Z_m",
        "Vx_ms",
        "RCS_dbm2",
        "Time_ms",
        "probability",
        "snr",
    ]

    # 调用转换函数
    csv_to_pcd(csv_file_path, pcd_file_path, required_columns)
