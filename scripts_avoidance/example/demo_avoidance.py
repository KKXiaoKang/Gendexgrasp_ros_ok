#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import time
import math

def callback(data):
    # 遍历接收到的 JointTrajectory 消息中的所有点
    for point in data.points:
        # 为每个点创建一个 JointState 消息对象
        joint_state_msg = JointState()

        # 提取 JointTrajectory 点中的位置信息并进行弧度到角度转换
        if len(point.positions) >= 7:
            # 仅取前7个关节的位置数据并转换为角度
            joint_state_msg.position = [pos * 180.0 / math.pi for pos in point.positions[:7]]

            # 填充剩余位置为零，确保消息长度为14
            joint_state_msg.position += [0.0] * (14 - len(joint_state_msg.position))

            # 发布消息
            pub.publish(joint_state_msg)
            rospy.loginfo(f"Published joint positions (degrees): {joint_state_msg.position}")

            # 增加延时以便依次处理每个点
            time.sleep(0.1)  # 根据需要调整该延时
        else:
            rospy.logwarn("Received trajectory point does not contain at least 7 joint positions.")

def demo_avoidance():
    rospy.init_node('demo_avoidance', anonymous=True)

    # 订阅 /joint_trajectory_topic 话题
    rospy.Subscriber('/joint_trajectory_topic', JointTrajectory, callback)

    # 发布到 /kuavo_arm_traj 话题
    global pub
    pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        demo_avoidance()
    except rospy.ROSInterruptException:
        pass
