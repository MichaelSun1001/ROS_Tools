import os
import cv2
import rosbag
from cv_bridge import CvBridge

# 定义 bag 文件夹路径
bag_folder = "/media/sax/新加卷1/bag"

# 定义固定的输出路径
output_base_dir = "/media/sax/新加卷1/bag/test"  # 替换为您指定的路径

# 动态读取文件夹中的所有 .bag 文件
bag_files = [
    os.path.join(bag_folder, f) for f in os.listdir(bag_folder) if f.endswith(".bag")
]

# 定义图像话题
image_topics = [
    "/usb_cam/image_raw",
    "/usb_cam/image_raw/compressed",
    "/hik_cam_node/hik_camera",
    "/rtsp_camera_relay/image",
]

# 实例化 CvBridge
bridge = CvBridge()


def process_bag_file(bag_file):
    # 检查文件是否存在
    if not os.path.isfile(bag_file):
        print(f"Error: Bag file '{bag_file}' not found.")
        return

    # 从 bag 文件名生成输出目录
    bag_base_name = os.path.splitext(os.path.basename(bag_file))[0]
    image_output_dir_base = os.path.join(output_base_dir, bag_base_name)

    # 初始化每个话题的 frame_id 字典
    frame_id_dict = {topic: 0 for topic in image_topics}

    print(f"开始处理 bag 文件 '{bag_file}' 中的图像消息...")

    with rosbag.Bag(bag_file, "r") as bag:
        # 处理每个图像消息
        for topic, msg, t in bag.read_messages(topics=image_topics):
            # 获取时间戳的秒和纳秒部分
            secs = msg.header.stamp.secs
            nsecs = msg.header.stamp.nsecs
            timestamp = f"{secs}.{nsecs}"  # 完整时间戳

            frame_id = frame_id_dict[topic]  # 获取当前话题的帧序号

            # 为每个图像话题创建独立的目录
            image_topic_dir = os.path.join(
                image_output_dir_base, topic.strip("/").replace("/", "_")
            )
            if not os.path.exists(image_topic_dir):
                os.makedirs(image_topic_dir, exist_ok=True)

            # 根据话题类型保存图像
            if msg._type == "sensor_msgs/Image":
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    image_file = os.path.join(
                        image_topic_dir, f"Image_{frame_id}_{secs}_{nsecs:09d}.png"
                    )
                    cv2.imwrite(image_file, cv_image)
                    print(f"保存图像文件: {image_file}")
                except Exception as e:
                    print(f"Error processing Image message: {e}")

            elif msg._type == "sensor_msgs/CompressedImage":
                try:
                    cv_image = bridge.compressed_imgmsg_to_cv2(
                        msg, desired_encoding="bgr8"
                    )
                    compressed_image_file = os.path.join(
                        image_topic_dir,
                        f"CompressedImage_{frame_id}_{secs}_{nsecs:09d}.png",
                    )
                    cv2.imwrite(compressed_image_file, cv_image)
                    print(f"保存压缩图像文件: {compressed_image_file}")
                except Exception as e:
                    print(f"Error processing CompressedImage message: {e}")

            frame_id_dict[topic] += 1  # 更新当前话题的帧序号


# 处理多个 bag 文件
for bag_file in bag_files:
    process_bag_file(bag_file)
