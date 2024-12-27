#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
from nav_msgs.msg import Path

"""
    /cumotion_effector_path -- 不带插值 默认为32个点
    /end_effector_path -- 带插值 默认为140个点

    控制流/插值之后 和 插值之前进行对比
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

    # 计算采样间隔
    total_points = len(end_x)         # 140
    target_points = len(cumotion_x)   # 32 

    # 从 end_effector_path 中均匀采样
    sampled_end_x = []
    sampled_end_y = []
    sampled_end_z = []
    for i in range(target_points):
        index = i * (total_points // target_points) + min(i, total_points % target_points)
        sampled_end_x.append(end_x[index])
        sampled_end_y.append(end_y[index])
        sampled_end_z.append(end_z[index])

    # 打印采样后的长度
    print(f"Sampled end_effector_path length: {len(sampled_end_x)}")

    # 可视化比较
    plt.figure(figsize=(10, 9))

    for i, (cumotion_data, sampled_end_data, label) in enumerate([
        (cumotion_x, sampled_end_x, "X"),
        (cumotion_y, sampled_end_y, "Y"),
        (cumotion_z, sampled_end_z, "Z"),
    ]):
        # 创建子图
        plt.subplot(3, 1, i + 1)
        plt.plot(cumotion_data, label="cumotion_effector_path", color="blue", linestyle="-")
        plt.plot(sampled_end_data, label="Sampled end_effector_path", color="orange", linestyle="--")
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
