#!/usr/bin/env python

import rospy
import subprocess
import yaml
import os

CONFIG_FILE = '/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/src/ros_utils/ros_bridge_python/config/config.yaml'

class KuavoBridgeNode:
    def __init__(self):
        # 初始化ROS1节点
        rospy.init_node('kuavo_bridge_node')
        
        # 从配置文件加载配置
        with open(CONFIG_FILE, 'r') as f:
            config = yaml.safe_load(f)
        
        # 从配置中获取参数
        ros_master_uri = config.get('ros_master_uri', 'http://192.168.0.140:11311')
        ros_bridge_ws_path = os.path.expanduser(config.get('ros_bridge_ws_path', '~/ros_bridge_ws'))

        # log info 
        rospy.loginfo(f"ROS Master URI: {ros_master_uri}")
        rospy.loginfo(f"ROS Bridge WS Path: {ros_bridge_ws_path}")

        # 构建命令
        command = f"""
        cd {ros_bridge_ws_path} &&
        source /opt/ros/noetic/setup.bash &&
        source /opt/ros/foxy/setup.bash &&
        source /home/lab/GenDexGrasp/curobot_ros_ws/bridge_ws/ros1_msgs_ws/devel_isolated/setup.bash &&
        source /home/lab/GenDexGrasp/curobot_ros_ws/bridge_ws/ros2_msgs_ws/install/setup.bash &&
        source /home/lab/GenDexGrasp/curobot_ros_ws/bridge_ws/ros2_bridge_ws/install/setup.bash &&
        rosparam load bridge.yaml 
        ros2 run ros1_bridge parameter_bridge
        """

        # 执行命令
        subprocess.run(command, shell=True, executable="/bin/bash")
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = KuavoBridgeNode()
    node.spin()
