#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import tf

def publish_pose():
    rospy.init_node('object_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/object_yolo_tf2_torso_result', Detection2DArray, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        # 构造 Detection2DArray 消息
        detection_array = Detection2DArray()
        detection_array.header.stamp = rospy.Time.now()
        detection_array.header.frame_id = 'torso'

        # 构造 Detection2D 消息
        detection = Detection2D()

        # 构造 ObjectHypothesisWithPose 消息
        hypothesis = ObjectHypothesisWithPose()

        # 设置位置
        hypothesis.pose.pose.position.x = 0.4
        hypothesis.pose.pose.position.y = 0.2
        hypothesis.pose.pose.position.z = 0.1

        # 设置四元数为单位四元数
        hypothesis.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))

        # 将 ObjectHypothesisWithPose 添加到 Detection2D 的 results 中
        detection.results.append(hypothesis)

        # 将 Detection2D 添加到数组中
        detection_array.detections.append(detection)

        # 发布消息
        pub.publish(detection_array)

        # 休眠以维持100Hz的频率
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
