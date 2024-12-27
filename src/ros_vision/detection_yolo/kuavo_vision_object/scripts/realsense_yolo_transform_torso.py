#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_conjugate
from std_msgs.msg import Header
import math
from geometry_msgs.msg import PoseStamped
import geometry_msgs
import numpy as np

# 是否使用Gen6D输出姿态信息
USE_GEN_6DOF_FLAG = False
WHO_USE_DOF_INFO_FLAG = True
gen6d_pose = PoseStamped()
gen6d_pose.pose.orientation.x = 0.0 
gen6d_pose.pose.orientation.y = 0.0 
gen6d_pose.pose.orientation.z = 0.0 
gen6d_pose.pose.orientation.w = 1.0

# 根据需求创建Gen6D姿态订阅器
if USE_GEN_6DOF_FLAG:
    def gen6d_pose_callback(msg):
        """
            订阅回调函数
        """
        global gen6d_pose
        gen6d_pose = msg
        # rospy.loginfo(f"Received Gen6D pose: {gen6d_pose}")
    rospy.Subscriber('/gen6d/pose', geometry_msgs.msg.PoseStamped, gen6d_pose_callback)

class YoloTransform:
    def __init__(self):
        # 初始化节点
        rospy.init_node('yolo_transform', anonymous=True)

        # 订阅检测结果话题
        self.detection_sub = rospy.Subscriber('/object_yolo_segment_result', Detection2DArray, self.detection_callback)

        # 发布转换后的结果话题
        self.transformed_pub = rospy.Publisher('/object_yolo_tf2_torso_result', Detection2DArray, queue_size=10)

        # TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def normalize_quaternion(self, quat):
        length = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        quat.x /= length
        quat.y /= length
        quat.z /= length
        quat.w /= length
        return quat

    def detection_callback(self, msg):
        global USE_GEN_6DOF_FLAG
        global gen6d_pose

        try:
            # 查找坐标变换
            transform = self.tf_buffer.lookup_transform('torso', 'camera_color_optical_frame', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('TF lookup failed')
            return

        # 创建新的Detection2DArray消息
        transformed_msg = Detection2DArray()
        transformed_msg.header = msg.header
        transformed_msg.header.frame_id = 'torso'

        # 遍历所有检测结果并转换坐标
        for detection in msg.detections:
            if detection.results[0].pose.pose.position.x == 0 and \
               detection.results[0].pose.pose.position.y == 0 and \
               detection.results[0].pose.pose.position.z == 0:
                continue
            
            transformed_detection = Detection2D()
            transformed_detection.header = detection.header
            transformed_detection.results = detection.results
            transformed_detection.bbox = detection.bbox
            transformed_detection.source_img = detection.source_img

            # 变换位置坐标
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = detection.results[0].pose.pose

            # 使用TF变换坐标 -- 设置抓取位姿初始化，位置发送变换，姿态保持不变
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            transformed_detection.results[0].pose.pose = transformed_pose.pose  
            if USE_GEN_6DOF_FLAG:
                if WHO_USE_DOF_INFO_FLAG: # 判断谁主要使用Gen6D信息(True为相机坐标系用姿态信息，False为机器人基坐标系用姿态信息)
                    pass
                else:
                    transformed_detection.results[0].pose.pose.orientation.x = gen6d_pose.pose.orientation.x  
                    transformed_detection.results[0].pose.pose.orientation.y = gen6d_pose.pose.orientation.y  
                    transformed_detection.results[0].pose.pose.orientation.z = gen6d_pose.pose.orientation.z  
                    transformed_detection.results[0].pose.pose.orientation.w = gen6d_pose.pose.orientation.w  
            else:
                # 标准姿态
                transformed_detection.results[0].pose.pose.orientation.x = 0.0
                transformed_detection.results[0].pose.pose.orientation.y = 0.0
                transformed_detection.results[0].pose.pose.orientation.z = 0.0
                transformed_detection.results[0].pose.pose.orientation.w = 1.0 
                """
                    修改姿态
                """
                # 当前姿态的四元数
                current_orientation = [
                    transformed_detection.results[0].pose.pose.orientation.x,
                    transformed_detection.results[0].pose.pose.orientation.y,
                    transformed_detection.results[0].pose.pose.orientation.z,
                    transformed_detection.results[0].pose.pose.orientation.w,
                ]
                # 计算绕yaw轴旋转110度的四元数
                yaw_angle = 160 * np.pi / 180  # 将度数转换为弧度
                #### yaw_angle = -140 * np.pi / 180  # 将度数转换为弧度
                # yaw_angle = 0
                rotation_quaternion = quaternion_from_euler(0, 0, yaw_angle)

                # 计算新的四元数
                new_orientation = quaternion_multiply(current_orientation, rotation_quaternion)

                # # 对四元数进行规范化
                # new_orientation = new_orientation / np.linalg.norm(new_orientation)

                # 更新姿态
                transformed_detection.results[0].pose.pose.orientation.x = new_orientation[0]
                transformed_detection.results[0].pose.pose.orientation.y = new_orientation[1]
                transformed_detection.results[0].pose.pose.orientation.z = new_orientation[2]
                transformed_detection.results[0].pose.pose.orientation.w = new_orientation[3]

            # 创建TransformStamped消息用于TF广播
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = 'torso'
            transform_stamped.child_frame_id = f"torso_object_{detection.results[0].id}"
            transform_stamped.transform.translation.x = transformed_pose.pose.position.x
            transform_stamped.transform.translation.y = transformed_pose.pose.position.y
            transform_stamped.transform.translation.z = transformed_pose.pose.position.z
            if USE_GEN_6DOF_FLAG:
                transform_stamped.transform.rotation.x = transformed_pose.pose.orientation.x
                transform_stamped.transform.rotation.y = transformed_pose.pose.orientation.y
                transform_stamped.transform.rotation.z = transformed_pose.pose.orientation.z
                transform_stamped.transform.rotation.w = transformed_pose.pose.orientation.w
            else:
                transform_stamped.transform.rotation.w = 1  # 单位四元数
            # transform_stamped.transform.rotation = self.normalize_quaternion(transformed_pose.pose.orientation)

            # 广播转换
            self.tf_broadcaster.sendTransform(transform_stamped)

            transformed_msg.detections.append(transformed_detection)

        # 发布转换后的检测结果
        self.transformed_pub.publish(transformed_msg)

if __name__ == '__main__':
    try:
        yolo_transform = YoloTransform()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
