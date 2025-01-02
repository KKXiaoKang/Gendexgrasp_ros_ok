#!/usr/bin/env python
import rospy
from curobo_trajectory_server.srv import cuRoMpcSetting, cuRoMpcSettingRequest
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

def call_mpc_goal_service(pose):
    """
    调用'/cumotion/mpc_set_goal'服务，传递目标pose并获取响应
    """
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
    运行客户端，设置目标Pose并调用服务，频率为50Hz，position.y和position.z线性增减
    """
    # 初始化ROS节点
    rospy.init_node('mpc_goal_client')

    # 创建目标 Pose 消息
    pose = Pose()
    pose.position.x = 0.4  # 固定目标位置x
    pose.orientation.x = 0.0  # 固定目标姿态x
    pose.orientation.y = -0.70710677  # 固定目标姿态y
    pose.orientation.z = 0.0  # 固定目标姿态z
    pose.orientation.w = 0.70710677  # 固定目标姿态w

    # 初始化 position.y 和 position.z
    pose.position.y = 0.0
    pose.position.z = 0.0

    # 定义增量和方向
    increment = 0.05
    direction_y = 1  # 1 表示增加，-1 表示减少
    direction_z = 1

    rate = rospy.Rate(5)  # 设置频率为5Hz

    try:
        while not rospy.is_shutdown():
            # 根据方向线性改变 position.y
            pose.position.y += direction_y * increment
            if pose.position.y >= 0.5 or pose.position.y <= 0.0:
                direction_y *= -1  # 到达边界时反向

            # 根据方向线性改变 position.z
            pose.position.z += direction_z * increment
            if pose.position.z >= 0.5 or pose.position.z <= 0.0:
                direction_z *= -1  # 到达边界时反向

            # 调用服务
            call_mpc_goal_service(pose)

            # 按照50Hz休眠
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred")

if __name__ == "__main__":
    main()
