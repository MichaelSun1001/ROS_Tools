import os
import cv2
import rosbag
from cv_bridge import CvBridge

# 定义多个 bag 文件路径
bag_files = [
    '/media/sax/新加卷/数据集1105/2024-11-05-16-46-36.bag',
    '/media/sax/新加卷/待测548数据集/jf_test2.bag',
    '/media/sax/新加卷/待测548数据集/jf_test3.bag'
]

# 定义图像话题
image_topics = ['/usb_cam/image_raw', '/usb_cam/image_raw/compressed']

# 实例化CvBridge
bridge = CvBridge()


def process_bag_file(bag_file):
    # 检查文件是否存在
    if not os.path.isfile(bag_file):
        print(f"Error: Bag file '{bag_file}' not found.")
        return

    # 从bag文件名生成输出目录
    bag_base_name = os.path.splitext(os.path.basename(bag_file))[0]
    image_output_dir_base = os.path.join(bag_base_name, 'images')

    # 初始化每个话题的frame_id字典
    frame_id_dict = {topic: 0 for topic in image_topics}

    print(f"开始处理bag文件 '{bag_file}' 中的图像消息...")

    with rosbag.Bag(bag_file, 'r') as bag:
        # 处理每个图像消息
        for topic, msg, t in bag.read_messages(topics=image_topics):
            timestamp = msg.header.stamp.to_sec()  # 获取消息的时间戳
            frame_id = frame_id_dict[topic]  # 获取当前话题的帧序号

            # 为每个图像话题创建独立的目录
            image_topic_dir = os.path.join(
                image_output_dir_base, topic.strip('/').replace('/', '_'))
            if not os.path.exists(image_topic_dir):
                os.makedirs(image_topic_dir, exist_ok=True)

            # 根据话题类型保存图像
            if msg._type == 'sensor_msgs/Image':
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                image_file = os.path.join(
                    image_topic_dir, f'image_raw_{frame_id}_{int(timestamp)}.jpg')
                cv2.imwrite(image_file, cv_image)
                print(f"保存图像文件: {image_file}")

            elif msg._type == 'sensor_msgs/CompressedImage':
                cv_image = bridge.compressed_imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
                compressed_image_file = os.path.join(
                    image_topic_dir, f'image_raw_compressed_{frame_id}_{int(timestamp)}.jpg')
                cv2.imwrite(compressed_image_file, cv_image)
                print(f"保存压缩图像文件: {compressed_image_file}")

            frame_id_dict[topic] += 1  # 更新当前话题的帧序号


# 处理多个 bag 文件
for bag_file in bag_files:
    process_bag_file(bag_file)
