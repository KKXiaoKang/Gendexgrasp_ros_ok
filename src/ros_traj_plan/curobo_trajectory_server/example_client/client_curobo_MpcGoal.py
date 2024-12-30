#!/usr/bin/env python
import rospy
from curobo_trajectory_server.srv import cuRoMpcSetting, cuRoMpcSettingRequest
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

def call_mpc_goal_service(pose):
    """
    调用'/cumotion/mpc_set_goal'服务，传递目标pose并获取响应
    """
    # 初始化ROS节点
    rospy.init_node('mpc_goal_client')

    # 创建 Header
    header = Header()
    header.seq = 0
    header.stamp = rospy.Time.now()  # 使用当前时间戳
    header.frame_id = "base_link"  # 可以根据需要更改为其他 frame_id

    # 创建 PoseStamped 消息
    pose_msg = PoseStamped()
    pose_msg.header = header
    pose_msg.pose.position.x = pose.position.x  # 传入目标位置x
    pose_msg.pose.position.y = pose.position.y  # 传入目标位置y
    pose_msg.pose.position.z = pose.position.z  # 传入目标位置z
    pose_msg.pose.orientation.x = pose.orientation.x  # 传入目标姿态x
    pose_msg.pose.orientation.y = pose.orientation.y  # 传入目标姿态y
    pose_msg.pose.orientation.z = pose.orientation.z  # 传入目标姿态z
    pose_msg.pose.orientation.w = pose.orientation.w  # 传入目标姿态w

    # 创建服务请求
    request = cuRoMpcSettingRequest()
    request.pose = pose_msg  # 将Pose数据传递到请求

    # 等待服务启动
    try:
        rospy.wait_for_service('/cumotion/mpc_set_goal')
        mpc_goal_service = rospy.ServiceProxy('/cumotion/mpc_set_goal', cuRoMpcSetting)

        # 调用服务
        response = mpc_goal_service(request)

        # 根据返回的结果打印信息
        if response.result:
            rospy.loginfo("MPC goal set successfully!")
        else:
            rospy.logwarn("Failed to set MPC goal.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    """
    运行客户端，设置目标Pose并调用服务
    """
    try:
        # 创建目标 Pose 消息
        pose = Pose()
        pose.position.x = 0.3 # 设置目标位置x
        pose.position.y = 0.1  # 设置目标位置y
        pose.position.z = 0.2  # 设置目标位置z
        pose.orientation.x = 0.0  # 设置目标姿态x
        pose.orientation.y = 0.0  # 设置目标姿态y
        pose.orientation.z = 0.0  # 设置目标姿态z
        pose.orientation.w = 1.0  # 设置目标姿态w

        # 调用服务
        call_mpc_goal_service(pose)
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred")

if __name__ == "__main__":
    main()
