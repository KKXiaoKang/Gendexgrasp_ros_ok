import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from scipy.interpolate import interp1d

DATA_VISUALIZATION = True

if DATA_VISUALIZATION:
    import ikpy
    from ikpy import chain
    from ikpy import urdf
    # 从URDF加载机器人链条
    ik_chain = chain.Chain.from_urdf_file("/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/scripts_avoidance/example/biped_s42_v4_left_arm.urdf")
    # 给定关节角度
    joint_angles = [0.0, 0.332926481962204, 0.8579579591751099, 0.5656951069831848, -1.215211033821106, -0.3651212453842163, -0.535624623298645, -0.014952611178159714]  # 示例关节angle
    # 计算正向运动学，返回末端执行器位姿
    end_effector_frame = ik_chain.forward_kinematics(joint_angles)
    print("End effector position: ", end_effector_frame[:3, 3])  # 末端位置
    print("End effector orientation: ", end_effector_frame[:3, :3])  # 末端姿态

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
    joint_state_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    joint_state_msg = JointState()

    rate = rospy.Rate(100)  # 设置发布频率为100Hz

    # 按照100Hz发布JointState消息
    for t, position in zip(new_times, interpolated_positions):
        joint_state_msg.header.stamp = rospy.Time.from_sec(t)
        joint_state_msg.position = position
        joint_state_pub.publish(joint_state_msg)
        rate.sleep()  # 按照100Hz的频率发布消息

def calculate_end_effector_pose(joint_angles):
    # 确保 joint_angles 是列表形式
    joint_angles_with_base = [0.0] + list(joint_angles)  # 将元组转换为列表并添加 0.0
    # 计算正向运动学，返回末端执行器位姿
    end_effector_frame = ik_chain.forward_kinematics(joint_angles_with_base)
    return end_effector_frame[:3, 3]  # 返回末端位置

# ROS回调函数
def joint_trajectory_callback(msg):
    # 打印接收到的JointTrajectory消息信息
    rospy.loginfo(f"Received JointTrajectory with {len(msg.points)} points.")

    # 进行位置插值
    new_times, interpolated_positions = interpolate_position(msg)

    # debug
    rospy.loginfo(f"Interpolated {len(new_times)} points at 100Hz.")

    # 发布JointState消息
    publish_joint_state(new_times, interpolated_positions)


if __name__ == '__main__':
    rospy.init_node('rosbag_interpolator')

    # 订阅/cumotion_go_joint_trajectory_topic话题
    rospy.Subscriber('/cumotion_go_joint_trajectory_topic', JointTrajectory, joint_trajectory_callback)

    # 保持节点活跃，直到手动关闭
    rospy.spin()
