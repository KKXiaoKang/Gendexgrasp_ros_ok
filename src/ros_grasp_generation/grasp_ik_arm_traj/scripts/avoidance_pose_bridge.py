#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from grasp_ik_arm_traj.srv import excuteCumotionTraj
import time
import math
from dynamic_biped.msg import armTargetPoses

AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = False
PUBLISH_COUNT = 0  # 发布计数器
IF_USE_ARM_TARGET_POSES = True  # 是否使用 armTargetPoses 话题发布轨迹

TIME_DATA = []   # 时间序列
TRAJ_ARRAY = []  # 关节序列
GO_TRAJ_POINTS = []  # go 状态下的轨迹点
BACK_TRAJ_POINTS = []  # back 状态下的轨迹点
TRAJ_COUNTER = 1  # 轨迹点接收计数器

def parse_csv_data(points, start_time):
    """
        解析成/kuavo_arm_target_poses的格式发布
    """
    global TIME_DATA
    global TRAJ_ARRAY

    # 清空全局变量
    TIME_DATA = []
    TRAJ_ARRAY = []

    # 遍历 points 数据
    for point in points:
        # 提取时间信息，计算相对于起始时间的偏移
        time_offset = point.time_from_start.to_sec()
        TIME_DATA.append(time_offset)

        # 提取位置数据，扩充为14维度，剩余部分填充为0
        if len(point.positions) >= 7:
            joint_positions = [
                pos * 180.0 / math.pi for pos in point.positions[:7]  # 转换为角度值
            ]
            joint_positions += [0.0] * (14 - len(joint_positions))    # 补齐剩余的维度
            TRAJ_ARRAY.extend(joint_positions)  # 添加到全局关节轨迹数组
        else:
            rospy.logwarn("Trajectory point does not contain at least 7 joint positions.")
    
    # 发布处理后的时间和轨迹数据
    pub_kuavo_arm_with_time(TIME_DATA, TRAJ_ARRAY)
    rospy.loginfo(f"Published /kuavo_arm_target_poses TIME_DATA  :  {TIME_DATA}  ")
    # rospy.loginfo(f"Published /kuavo_arm_target_poses TRAJ_ARRAY :  {TRAJ_ARRAY} ")
    rospy.loginfo(f"Published /kuavo_arm_target_poses arm trajectory with {len(points)} points.")

    # 等待 TIME_DATA 最后一个时间值
    if TIME_DATA:
        wait_time = TIME_DATA[-1]
        rospy.loginfo(f"Waiting for {wait_time:.2f} seconds to synchronize trajectory.")
        rospy.sleep(wait_time)
       
def parse_kuavo_arm_traj(points, start_time):
    """
        解析成/kuavo_arm_traj的格式发布
    """
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

# TODO: 将这个轨迹的存放callback从一个/joint_trajectory_topic 变换为 /cumotion_go_joint_trajectory_topic 和 /cumotion_back_joint_trajectory_topic
# def callback(data):
#     """
#         这个话题适用于 /joint_trajectory_topic 根据结果单次发布的话题
#     """
#     global AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG
#     global PUBLISH_COUNT
#     global IF_USE_ARM_TARGET_POSES
#     global GO_TRAJ_POINTS
#     global BACK_TRAJ_POINTS
#     global TRAJ_COUNTER

#     PUBLISH_COUNT = 0
#     AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = False  # 转发中

#     points = data.points

#     if len(points) < 2:
#         rospy.logwarn("Trajectory contains less than 2 points. Skipping.")
#         return

#     # 存储轨迹点
#     if TRAJ_COUNTER % 3 == 0:
#         GO_TRAJ_POINTS.clear()
#         BACK_TRAJ_POINTS.clear()
#         TRAJ_COUNTER = 1 # 重新置1
#         rospy.loginfo(f" -------- delete point -------------")
#     if TRAJ_COUNTER % 2 == 1:
#         GO_TRAJ_POINTS.extend(points)  # 存储 go 状态轨迹点
#         rospy.loginfo(f"Received go trajectory point {len(GO_TRAJ_POINTS)}")
#     elif TRAJ_COUNTER % 2 == 0:
#         BACK_TRAJ_POINTS.extend(points)  # 存储 back 状态轨迹点
#         rospy.loginfo(f"Received back trajectory point {len(BACK_TRAJ_POINTS)}")
#     TRAJ_COUNTER += 1

#     # 标记状态
#     rospy.loginfo(f"Received trajectory: {'go' if TRAJ_COUNTER % 2 == 0 else 'back'}")

def go_traj_callback(data):
    global GO_TRAJ_POINTS

    points = data.points

    if len(points) < 2:
        # rospy.logwarn("go_traj Trajectory contains less than 2 points. Skipping.")
        return
    
    # GO_TRAJ_POINTS.extend(points)
    GO_TRAJ_POINTS = list(points)

def back_traj_callback(data):
    global BACK_TRAJ_POINTS

    points = data.points

    if len(points) < 2:
        # rospy.logwarn("back_traj Trajectory contains less than 2 points. Skipping.")
        return
    
    # BACK_TRAJ_POINTS.extend(points)
    BACK_TRAJ_POINTS = list(points)

def publish_flag(event):
    global AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG
    flag_msg = Bool()
    flag_msg.data = AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG
    flag_pub.publish(flag_msg)

def pub_kuavo_arm_with_time(time, traj_jointstate):
    """
        发布时间和轨迹
    """
    global arm_traj_with_time_pub
    arm_traj_msg = armTargetPoses()
    arm_traj_msg.times = time
    arm_traj_msg.values = traj_jointstate
    arm_traj_with_time_pub.publish(arm_traj_msg) 
    rospy.loginfo(f"Published /kuavo_arm_target_poses arm trajectory with {len(time)} points.")

def execute_plan_traj_cumotion(req):
    global GO_TRAJ_POINTS
    global BACK_TRAJ_POINTS
    global TRAJ_COUNTER
    global AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG
    rospy.loginfo(f"Executing trajectory plan with id: {req.id}")

    flag = False
    AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = False  # 转发中

    # 根据服务 ID 选择 go 或 back 轨迹
    if req.id == 1:  # go 轨迹  
        rospy.loginfo("Executing go trajectory.")
        parse_csv_data(GO_TRAJ_POINTS, rospy.get_time())
        if len(GO_TRAJ_POINTS) == 0:
            rospy.logwarn("Go trajectory is empty. Skipping.")
            flag = False
        else:
            flag = True
    elif req.id == 2:  # back 轨迹
        rospy.loginfo("Executing back trajectory.")
        parse_csv_data(BACK_TRAJ_POINTS, rospy.get_time())
        if len(BACK_TRAJ_POINTS) == 0:
            rospy.logwarn("Back trajectory is empty. Skipping.")
            flag = False
        else:
            flag = True
    else:
        rospy.logwarn("Invalid id received, expected 1(Go) or 2(Back).")
        flag = False

    # 标记轨迹转发完成
    AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = True

    return flag

def demo_avoidance():
    rospy.init_node('avoidance_traj_bridge', anonymous=True)

    # 订阅 /joint_trajectory_topic 话题
    # rospy.Subscriber('/joint_trajectory_topic', JointTrajectory, callback)

    # 订阅 /cumotion_go_joint_trajectory_topic 和 /cumotion_back_joint_trajectory_topic 状态进行回调
    rospy.Subscriber('/cumotion_go_joint_trajectory_topic', JointTrajectory, go_traj_callback)
    rospy.Subscriber('/cumotion_back_joint_trajectory_topic', JointTrajectory, back_traj_callback)

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

    # 发布话题
    global arm_traj_with_time_pub
    arm_traj_with_time_pub = rospy.Publisher("/kuavo_arm_target_poses", armTargetPoses, queue_size=10)

    # 服务
    rospy.Service('/excute_plan_traj_cumotion', excuteCumotionTraj, execute_plan_traj_cumotion)

    # 节点循环
    rospy.spin()

if __name__ == '__main__':
    try:
        demo_avoidance()
    except rospy.ROSInterruptException:
        pass
