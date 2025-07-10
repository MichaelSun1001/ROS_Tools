#!/usr/bin/env python3
import os
import rosbag
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct


def convert_pointcloud2_to_o3d(ros_cloud):
    """将ROS PointCloud2消息转换为Open3D点云"""
    fields = [field.name for field in ros_cloud.fields]
    if 'rgb' in fields or 'rgba' in fields:
        cloud_points = pc2.read_points(
            ros_cloud, skip_nans=True,
            field_names=("x", "y", "z", "rgb" if 'rgb' in fields else "rgba")
        )
        points = []
        colors = []
        for point in cloud_points:
            x, y, z, color = point
            r = (color >> 16) & 0x0000ff
            g = (color >> 8) & 0x0000ff
            b = color & 0x0000ff
            points.append([x, y, z])
            colors.append([r / 255.0, g / 255.0, b / 255.0])
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points)
        o3d_cloud.colors = o3d.utility.Vector3dVector(colors)
    else:
        cloud_points = pc2.read_points(
            ros_cloud, skip_nans=True, field_names=("x", "y", "z")
        )
        points = list(cloud_points)
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points)
    return o3d_cloud


def extract_and_merge_pointclouds_from_bag():
    """从ROS bag文件中提取并合并点云，保存为单个PCD文件"""
    # 参数设置
    bag_file = "/home/sax/rosbags/RGBD_Comparsion/BerxelP100R/desk_2025-07-10-16-03-43.bag"
    # bag_file = "/home/sax/rosbags/RGBD_Comparsion/BerxelP100R/screen_2025-07-10-16-05-39.bag"
    # bag_file = "/home/sax/rosbags/RGBD_Comparsion/BerxelP100R/flatfond_2025-07-10-16-04-24.bag"

    # bag_file = "/home/sax/rosbags/RGBD_Comparsion/InuchipR132/desk_2025-07-10-15-42-47.bag"
    # bag_file = "/home/sax/rosbags/RGBD_Comparsion/InuchipR132/flatfond_2025-07-10-15-41-04.bag"
    # bag_file = "/home/sax/rosbags/RGBD_Comparsion/InuchipR132/screen_2025-07-10-15-42-01.bag"

    # bag_file = "/home/sax/rosbags/RGBD_Comparsion/RealSenseD435/flatfond_2025-07-10-14-29-39.bag
    # bag_file = "/home/sax/rosbags/RGBD_Comparsion/RealSenseD435/desk_2025-07-10-14-44-00.bag"
    # bag_file = "/home/sax/rosbags/RGBD_Comparsion/RealSenseD435/screen_2025-07-10-14-49-17.bag"

    # topic = "/berxel_camera/depth/berxel_cloudpoint"
    # topic = "/depth/depth2pc"
    topic = "/camera/depth/color/points"

    frame_count = 3  # 不再使用，仅做显示
    output_path = "/home/sax/ROS_Tools/pointcloud-bag_to_pcd/output"

    print("PointCloud Extraction and Merging Parameters:")
    print(f"  Bag File:     {bag_file}")
    print(f"  Topic:        {topic}")
    print(f"  Frame Count:  {frame_count}")
    print(f"  Output Path:  {output_path}")

    os.makedirs(output_path, exist_ok=True)

    bag_base = os.path.splitext(os.path.basename(bag_file))[0]

    try:
        bag = rosbag.Bag(bag_file, 'r')
    except Exception as e:
        print(f"Error opening bag file: {str(e)}")
        return

    processed_count = 0
    saved = False

    for topic, msg, t in bag.read_messages(topics=[topic]):
        if msg._type == 'sensor_msgs/PointCloud2':
            processed_count += 1
            if processed_count == 6:
                try:
                    o3d_cloud = convert_pointcloud2_to_o3d(msg)
                    filename = os.path.join(
                        output_path, f"{bag_base}_pointcloud_{processed_count}.pcd")
                    o3d.io.write_point_cloud(filename, o3d_cloud)
                    print(f"Saved frame {processed_count} to {filename}")
                    saved = True
                    break  # 只保存第6帧，立即退出循环
                except Exception as e:
                    print(f"Error processing point cloud: {str(e)}")
                    break

    bag.close()

    if saved:
        print(f"Successfully saved point cloud from frame 6.")
    else:
        print("No point clouds were processed.")


if __name__ == '__main__':
    extract_and_merge_pointclouds_from_bag()
