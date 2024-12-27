#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import math

AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = False
PUBLISH_COUNT = 0  # 发布计数器

def callback(data):
    global AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG
    global PUBLISH_COUNT

    PUBLISH_COUNT = 0
    AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = False  # 转发中

    start_time = rospy.get_time()  # 获取当前时间
    points = data.points

    if len(points) < 2:
        rospy.logwarn("Trajectory contains less than 2 points. Skipping.")
        return

    # 遍历轨迹点
    for i in range(len(points) - 1):
        current_point = points[i]
        next_point = points[i + 1]

        # 创建 JointState 消息对象
        joint_state_msg = JointState()

        # 设置消息头的时间戳
        joint_state_msg.header.stamp = rospy.Time.now()

        # 提取 `next_point` 的位置信息并转换为角度
        if len(next_point.positions) >= 7:
            joint_state_msg.position = [
                pos * 180.0 / math.pi for pos in next_point.positions[:7]
            ]
            joint_state_msg.position += [0.0] * (14 - len(joint_state_msg.position))

            # 发布下一个点的位置信息 | 控制下发下一个点的位置信息
            pub.publish(joint_state_msg)

            # 计算等待时间 (next_point 的时间戳 - 当前时间戳)
            time_to_wait = start_time + next_point.time_from_start.to_sec() - rospy.get_time()
            if time_to_wait > 0:
                rospy.sleep(time_to_wait)  # 等待时间到达 next_point 的时间戳

            # TODO:发布当前实际位置应该到达信息
            current_msg = JointState()
            current_msg.header.stamp = rospy.Time.now()
            current_msg.position = list(current_point.positions[:7])  # 转换为列表
            current_msg.position += [0.0] * (14 - len(current_msg.position))  # 补齐剩余右手位置
            current_pub.publish(current_msg)

            # 计数器
            PUBLISH_COUNT += 1
            rospy.loginfo(f"Published next point positions (degrees): {joint_state_msg.position}")
        else:
            rospy.logwarn("Trajectory point does not contain at least 7 joint positions.")

    # 标记轨迹转发完成
    AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = True
    rospy.loginfo(f"Published PUBLISH_COUNT: {PUBLISH_COUNT}")


def publish_flag(event):
    global AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG
    flag_msg = Bool()
    flag_msg.data = AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG
    flag_pub.publish(flag_msg)

def demo_avoidance():
    rospy.init_node('avoidance_traj_bridge', anonymous=True)

    # 订阅 /joint_trajectory_topic 话题
    rospy.Subscriber('/joint_trajectory_topic', JointTrajectory, callback)

    # 发布到 /kuavo_arm_traj 话题
    global pub
    pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)

    # 发布当前点到达位置的发布者
    global current_pub
    current_pub = rospy.Publisher('/current_joint_position', JointState, queue_size=10)

    # 发布标志状态的定时器
    global flag_pub
    flag_pub = rospy.Publisher('/avoidance_traj_bridge_flag', Bool, queue_size=10)
    rospy.Timer(rospy.Duration(0.1), publish_flag)  # 10 Hz 定时器

    # 节点循环
    rospy.spin()

if __name__ == '__main__':
    try:
        demo_avoidance()
    except rospy.ROSInterruptException:
        pass
