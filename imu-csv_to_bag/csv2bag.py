import csv
import rospy
from sensor_msgs.msg import Imu
import rosbag

# 初始化ROS节点
rospy.init_node('imu_data_publisher')

# 创建rosbag文件
bag = rosbag.Bag('imu_data.bag', 'w')

# 获取当前时间作为基准
current_time = rospy.Time.now()

# 打开CSV文件并读取数据
with open('imu_data.csv', 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        # 创建IMU消息
        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu_frame'

        # 将timestamp_ms转换为ROS时间戳（毫秒转为秒）
        timestamp_ms = int(row['timestamp_ms'])
        imu_msg.header.stamp = current_time + \
            rospy.Duration(timestamp_ms / 1000.0)

        # 填充IMU数据
        imu_msg.linear_acceleration.x = float(row['acc_x'])
        imu_msg.linear_acceleration.y = float(row['acc_y'])
        imu_msg.linear_acceleration.z = float(row['acc_z'])
        imu_msg.angular_velocity.x = float(row['gyr_x'])
        imu_msg.angular_velocity.y = float(row['gyr_y'])
        imu_msg.angular_velocity.z = float(row['gyr_z'])

        # 将IMU消息写入bag文件
        bag.write('/imu/data', imu_msg)

# 关闭bag文件
bag.close()

print("IMU数据已成功写入imu_data.bag文件！")
