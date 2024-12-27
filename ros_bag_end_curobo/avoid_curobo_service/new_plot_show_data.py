#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
from nav_msgs.msg import Path

"""
    /cumotion_effector_path -- 不带插值 默认为32个点
    /end_effector_path -- 带插值 默认为140个点
"""
def extract_and_compare(bag_file):
    cumotion_effector_path = None
    end_effector_path = None

    # 打开 rosbag 文件
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/cumotion_effector_path', '/end_effector_path']):
            if topic == '/cumotion_effector_path' and cumotion_effector_path is None:
                cumotion_effector_path = msg
            elif topic == '/end_effector_path' and end_effector_path is None:
                end_effector_path = msg
            
            # 如果两个主题都已接收，退出循环
            if cumotion_effector_path and end_effector_path:
                break

    # 检查是否获取了数据
    if not cumotion_effector_path or not end_effector_path:
        print("Error: Failed to retrieve messages from bag file.")
        return

    # 提取数据
    cumotion_x = [pose.pose.position.x for pose in cumotion_effector_path.poses]
    cumotion_y = [pose.pose.position.y for pose in cumotion_effector_path.poses]
    cumotion_z = [pose.pose.position.z for pose in cumotion_effector_path.poses]

    end_x = [pose.pose.position.x for pose in end_effector_path.poses]
    end_y = [pose.pose.position.y for pose in end_effector_path.poses]
    end_z = [pose.pose.position.z for pose in end_effector_path.poses]

    # 打印长度
    print(f"cumotion_effector_path length: {len(cumotion_x)}")
    print(f"end_effector_path length: {len(end_x)}")
    cumotion_indices = range(len(cumotion_x))
    end_indices = range(len(end_x))

    # 可视化比较
    plt.figure(figsize=(10, 9))

    for i, (cumotion_data, cumotion_idx, end_data, end_idx, label) in enumerate([
        (cumotion_x, cumotion_indices, end_x, end_indices, "X"),
        (cumotion_y, cumotion_indices, end_y, end_indices, "Y"),
        (cumotion_z, cumotion_indices, end_z, end_indices, "Z"),
    ]):
        # 创建子图
        plt.subplot(3, 1, i + 1)
        plt.plot(cumotion_idx, cumotion_data, label="cumotion_effector_path", color="blue", linestyle="-")
        plt.plot(end_idx, end_data, label="end_effector_path", color="orange", linestyle="--")
        plt.title(f"Comparison of {label} values")
        plt.xlabel("Index")
        plt.ylabel(f"{label} Value")
        plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 修改为你的 rosbag 文件路径
    bag_file = "./end_effector_with_time.bag"
    
    extract_and_compare(bag_file)
