#!/usr/bin/env python
import rospy
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
import tf.transformations as tft
import os

USE_NVIDIA_FOUNDATION_FLAG = True  # 默认值

class ObjectPoseVisualizer:
    def __init__(self):
        self.marker_pub = rospy.Publisher("object_visualization_marker", Marker, queue_size=10)
        if USE_NVIDIA_FOUNDATION_FLAG:
            self.model_path = "/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/src/ros_robot_model/contactdb/Mac_and_cheese_0_1.obj"
        else:
            self.model_path = "/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/src/ros_robot_model/contactdb/water_bottle/water_bottle.stl"
        rospy.Subscriber("/object_yolo_tf2_torso_result", Detection2DArray, self.callback)

    def callback(self, data):
        # receive data
        for detection in data.detections:
            pose = detection.results[0].pose.pose

            position = [pose.position.x, pose.position.y, pose.position.z]
            orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

            marker = self.construct_marker(position, orientation, r=1.0, g=0.5, b=0.0)
            self.marker_pub.publish(marker)

    def construct_marker(self, position, orientation, r, g, b):
        marker = Marker()
        marker.header.frame_id = "torso"
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "file://" + self.model_path

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.header.stamp = rospy.Time.now()

        return marker

if __name__ == "__main__":
    rospy.init_node("object_pose_visualizer_node")

    # 获取参数 use_nvidia_foundation，默认为 True
    USE_NVIDIA_FOUNDATION_FLAG = rospy.get_param("~use_nvidia_foundation", True)

    rospy.loginfo(f"USE_NVIDIA_FOUNDATION_FLAG set to: {USE_NVIDIA_FOUNDATION_FLAG}")

    visualizer = ObjectPoseVisualizer()
    rospy.spin()
