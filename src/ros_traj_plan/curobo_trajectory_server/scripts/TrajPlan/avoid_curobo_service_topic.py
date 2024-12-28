import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import interp1d
import numpy as np
import rospkg
import os

DATA_VISUALIZATION = True

JOINT_NAME_LIST = [ "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link", "zarm_l5_link", "zarm_l6_link", "zarm_l7_link",
                    "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link", "zarm_r5_link", "zarm_r6_link", "zarm_r7_link"]

last_position_first_data = None

# 加载配置文件
rospack = rospkg.RosPack()
config_path = os.path.join(rospack.get_path('curobo_trajectory_server'), 'config/biped_s42_v4_left_arm.urdf')

if DATA_VISUALIZATION:
    import ikpy
    from ikpy import chain
    # 从URDF加载机器人链条
    ik_chain = chain.Chain.from_urdf_file(f"{config_path}")
    # 给定关节角度
    joint_angles = [0.0, 0.332926481962204, 0.8579579591751099, 0.5656951069831848, -1.215211033821106, -0.3651212453842163, -0.535624623298645, -0.014952611178159714]  # 示例关节angle
    # 计算正向运动学，返回末端执行器位姿
    end_effector_frame = ik_chain.forward_kinematics(joint_angles)
    print("End effector position: ", end_effector_frame[:3, 3])  # 末端位置
    print("End effector orientation: ", end_effector_frame[:3, :3])  # 末端姿态

def radians_to_degrees(radians):
    # 将弧度转换为角度
    return radians * (180.0 / np.pi)

def interpolate_position(joint_trajectory, target_frequency=100):
    # 获取时间和目标关节位置数据
    times = np.array([point.time_from_start.to_sec() for point in joint_trajectory.points])
    positions = np.array([point.positions for point in joint_trajectory.points])

    # 计算插值的时间点（按100Hz频率）
    new_times = np.linspace(times[0], times[-1], int((times[-1] - times[0]) * target_frequency) + 1)

    # 对位置进行线性插值
    interpolator_position = interp1d(times, positions, kind='linear', axis=0)

    interpolated_positions = interpolator_position(new_times)

    return new_times, interpolated_positions

def publish_joint_state(new_times, interpolated_positions):
    global JOINT_NAME_LIST
    joint_state_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    joint_state_msg = JointState()

    rate = rospy.Rate(100)  # 设置发布频率为100Hz
    joint_state_msg.name = JOINT_NAME_LIST
    
    # 按照100Hz发布JointState消息
    for t, position in zip(new_times, interpolated_positions):
        joint_state_msg.header.stamp = rospy.Time.from_sec(t)
        
        # 将弧度值转换为角度值，并构造一个14维的position数组，前7个维度设置为转换后的角度，后7个维度设置为0
        position_in_degrees = radians_to_degrees(position)
        joint_state_msg.position = np.concatenate([position_in_degrees, [0.0]*7])

        joint_state_pub.publish(joint_state_msg)
        # rospy.loginfo(f"Published joint_state_msg with position (in degrees): {joint_state_msg.position}")
        
        rate.sleep()  # 按照100Hz的频率发布消息

def calculate_end_effector_pose(joint_angles):
    # 确保 joint_angles 是列表形式
    joint_angles_with_base = [0.0] + list(joint_angles)  # 将元组转换为列表并添加 0.0
    # 计算正向运动学，返回末端执行器位姿
    end_effector_frame = ik_chain.forward_kinematics(joint_angles_with_base)
    return end_effector_frame[:3, 3]  # 返回末端位置

def publish_end_effector_path(new_times, interpolated_positions):
    path_pub = rospy.Publisher('/end_effector_path', Path, queue_size=10)
    path_msg = Path()

    # 设置路径的坐标系
    path_msg.header.frame_id = "base_link"

    for t, position in zip(new_times, interpolated_positions):
        # 通过关节角度插值结果计算末端执行器的位姿
        end_effector_position = calculate_end_effector_pose(position)

        # 创建 PoseStamped 消息来表示每个时间点的末端执行器位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"

        # 设置位置
        pose_stamped.pose.position.x = end_effector_position[0]
        pose_stamped.pose.position.y = end_effector_position[1]
        pose_stamped.pose.position.z = end_effector_position[2]

        # 设置一个默认的朝向（这里只设定姿态为单位矩阵，即没有旋转）
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        path_msg.poses.append(pose_stamped)

    # 发布路径消息
    path_pub.publish(path_msg)

def publish_effector_path(joint_trajectory):
    """
    根据 JointTrajectory 消息生成一个 Path 消息，并发布到 /cumotion_effector_path 话题
    """
    path_pub = rospy.Publisher('/cumotion_effector_path', Path, queue_size=10)
    path_msg = Path()
    path_msg.header.frame_id = "base_link"

    # 遍历 JointTrajectory 的点，生成每个点的位姿信息
    for point in joint_trajectory.points:
        joint_angles = point.positions
        end_effector_position = calculate_end_effector_pose(joint_angles)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"

        # 设置位置信息
        pose_stamped.pose.position.x = end_effector_position[0]
        pose_stamped.pose.position.y = end_effector_position[1]
        pose_stamped.pose.position.z = end_effector_position[2]

        # 设置一个默认的姿态 (单位四元数，无旋转)
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        path_msg.poses.append(pose_stamped)

    # 发布路径消息
    path_pub.publish(path_msg)
    # rospy.loginfo("Published effector path with %d poses." % len(path_msg.poses))

# ROS回调函数
def joint_trajectory_callback(msg):
    global last_position_first_data
    # 打印接收到的JointTrajectory消息信息
    # rospy.loginfo(f"Received JointTrajectory with {len(msg.points)} points.")

    # 发布轨迹路径到 /cumotion_effector_path
    publish_effector_path(msg)

    # 进行位置插值
    new_times, interpolated_positions = interpolate_position(msg)

    # debug
    # rospy.loginfo(f"Interpolated {len(new_times)} points at 100Hz.")

    # 发布路径数据
    publish_end_effector_path(new_times, interpolated_positions)

    # 检查是否相同，如相同停止重新控制命令
    if last_position_first_data == msg.points[31].positions[0]:
        rospy.loginfo("Received same data, ignore.")
        return
    last_position_first_data = msg.points[31].positions[0]

    # 发布JointState消息
    publish_joint_state(new_times, interpolated_positions)
    rospy.loginfo(" Published Robot Control command.")


if __name__ == '__main__':
    rospy.init_node('rosbag_interpolator')

    # 加载URDF文件 
    rospy.loginfo(f"Loading URDF file from {config_path}")

    # 订阅/cumotion_go_joint_trajectory_topic话题
    rospy.Subscriber('/cumotion_go_joint_trajectory_topic', JointTrajectory, joint_trajectory_callback)

    # 保持节点活跃，直到手动关闭
    rospy.spin()
