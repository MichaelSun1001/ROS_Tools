# import rospy
# import time


# def ros_time_to_date_time(ros_time):
#     # 将ROS时间戳转换为秒和纳秒
#     secs = ros_time.secs
#     nsecs = ros_time.nsecs

#     # 将秒转换为时间结构体
#     time_struct = time.localtime(secs)

#     # 构建日期时间字符串
#     date_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time_struct)

#     # 添加毫秒部分
#     milliseconds = nsecs / 1e6
#     date_time_str += ".{:03d}".format(int(milliseconds))

#     return date_time_str


# if __name__ == "__main__":
#     # 初始化ROS节点
#     rospy.init_node('ros_time_converter')

#     # 获取当前ROS时间
#     # current_ros_time = rospy.Time.now()
#     current_ros_time =

#     # 将ROS时间转换为日期时间字符串
#     date_time_str = ros_time_to_date_time(current_ros_time)

#     # 打印转换后的日期时间字符串
#     print("ROS时间戳转换结果:", date_time_str)


import time


def ros_time_to_date_time(ros_time):
    # 将ROS时间戳转换为秒和纳秒
    secs = ros_time // 1e9
    nsecs = ros_time % 1e9

    # 将秒转换为时间结构体
    time_struct = time.localtime(secs)

    # 构建日期时间字符串
    date_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time_struct)

    # 添加纳秒部分
    date_time_str += ".{:09d}".format(int(nsecs))

    return date_time_str


if __name__ == "__main__":
    # 假设ROS时间戳为纳秒
    ros_time = 1714403567077270000

    # 将ROS时间转换为日期时间字符串
    date_time_str = ros_time_to_date_time(ros_time)

    # 打印转换后的日期时间字符串
    print("ROS时间戳转换结果:", date_time_str)
