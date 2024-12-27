#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

# 初始化 ROS 节点
rospy.init_node('image_saver')

# 创建 CvBridge 对象
bridge = CvBridge()

# 定义全局变量来存储图像
rgb_image = None
depth_image = None

def rgb_callback(msg):
    global rgb_image
    rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # rospy.loginfo("RGB image received")

def depth_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    # rospy.loginfo("Depth image received")


# 订阅 RGB 和深度图像
rospy.Subscriber('/head_camera/color/image_raw', Image, rgb_callback)
rospy.Subscriber('/head_camera/aligned_depth_to_color/image_raw', Image, depth_callback)

# 设置图像保存的路径
rgb_image_path = './rgb.png'
depth_image_path = './depth.png'

def save_images():
    print("Saving images...")
    global rgb_image, depth_image
    if rgb_image is not None and depth_image is not None:
        # 保存 RGB 图像
        cv2.imwrite(rgb_image_path, rgb_image)
        # 保存深度图像
        depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 适当缩放
        cv2.imwrite(depth_image_path, depth_image_8bit)
        rospy.loginfo("Images saved to: %s and %s", rgb_image_path, depth_image_path)

# 主循环，监听键盘输入
def key_listener():
    print("Press 'p' to save images, 'q' to quit.")
    while not rospy.is_shutdown():
        key = input()  # 在终端监听输入
        if key == 'p':
            save_images()
        elif key == 'q':
            print("Quitting...")
            break

if __name__ == '__main__':
    try:
        key_listener()
    except rospy.ROSInterruptException:
        pass
