"""
robot_arm_action.py
 - 这个模块提供了执行不同手臂动作的函数，可以通过传入不同的动作名称执行对应的动作。
 - 它包含了发布机器人手臂轨迹、执行作揖、打招呼、靠近传感器和送花等功能。

 - 打招呼 robot_arm_action(robot_instance, "say_hello")
 - 作揖   robot_arm_action(robot_instance, "bowing")
 - 送花   robot_arm_action(robot_instance, "sending_flowers")

 设计轨迹过程当中，优先设计左手手臂轨迹，然后通过l_to_r函数映射到右手轨迹上
"""
import rospy
import sensor_msgs.msg
import time

from kuavoRobotSDK import kuavo

from utils import rad_to_angle, l_to_r, load_traj

ROS_PUBLIHSER_HZ = 30

# kuavo_human_traj | figure_human_traj
INPUT_FILE_DIR = "kuavo_human_traj"

""" -------- moveit plan function ----------- """
def publish_l_arm_traj(publisher, traj) -> None:
    """发布左手轨迹
    """
    joint_state = sensor_msgs.msg.JointState()
    positions  = [0 for _ in range(14)]
    velocities = [0 for _ in range(14)]
    rate = rospy.Rate(ROS_PUBLIHSER_HZ)
    
    for point in traj.joint_trajectory.points:
        if rospy.is_shutdown():
            rospy.logerr("用户终止程序")
            exit(0)
        positions[0:7] = rad_to_angle(point.positions)
        velocities[0:7] = point.velocities
        joint_state.position = positions
        joint_state.velocity = velocities
        
        publisher.publish(joint_state)
        rate.sleep()

def publish_r_arm_traj(publisher, traj) -> None:
    """发布右手轨迹
    """
    joint_state = sensor_msgs.msg.JointState()
    positions  = [0 for _ in range(14)]
    velocities = [0 for _ in range(14)]
    rate = rospy.Rate(ROS_PUBLIHSER_HZ)
    
    for point in traj.joint_trajectory.points:
        if rospy.is_shutdown():
            rospy.logerr("用户终止程序")
            exit(0)
        positions[7:14] = rad_to_angle(point.positions)
        velocities[7:14] = point.velocities
        joint_state.position = positions
        joint_state.velocity = velocities
        
        publisher.publish(joint_state)
        rate.sleep()

def publish_lr_arm_traj(publisher, l_traj, r_traj) -> None:
    """发布左右手轨迹
    """
    joint_state = sensor_msgs.msg.JointState()
    positions  = [0 for _ in range(14)]
    velocities = [0 for _ in range(14)]
    rate = rospy.Rate(ROS_PUBLIHSER_HZ)
    
    if len(l_traj.joint_trajectory.points) > len(r_traj.joint_trajectory.points):
        for i in range(len(r_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
            
        for i in range(len(r_traj.joint_trajectory.points) , len(l_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = [0 for _ in range(7)]
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = [0 for _ in range(7)]
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
        
    else:
        for i in range(len(l_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
        for i in range(len(l_traj.joint_trajectory.points) , len(r_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = [0 for _ in range(7)]
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = [0 for _ in range(7)]
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()

# 定义轨迹发布函数
def publish_trajectory(publisher, l_traj, arm='left'):
    if arm == 0:   # left
        publish_l_arm_traj(publisher, l_traj)
    elif arm == 1: # right
        r_traj = l_to_r(l_traj)
        publish_r_arm_traj(publisher, r_traj)
    elif arm == 2: # both
        r_traj = l_to_r(l_traj)
        publish_lr_arm_traj(publisher, l_traj, r_traj)

""" -------- kuavo arm action function ----------- """
def set_robot_arm_zero_go_to_prepare(kuavo_robot, control_index, publisher):
    """
        从 zero 到 prepare 位置
    """
    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/zero_go_to_prepare/zero_go_to_prepare_1.json")
    publish_trajectory(publisher, l_traj, control_index)

    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/zero_go_to_prepare/zero_go_to_prepare_2.json")
    publish_trajectory(publisher, l_traj, control_index)

    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/zero_go_to_prepare/zero_go_to_prepare_3.json")
    publish_trajectory(publisher, l_traj, control_index)

    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/zero_go_to_prepare/zero_go_to_prepare_4.json")
    publish_trajectory(publisher, l_traj, control_index)

    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/zero_go_to_prepare/zero_go_to_prepare_5.json")
    publish_trajectory(publisher, l_traj, control_index)

def set_robot_arm_prepare_go_to_zero(kuavo_robot, control_index, publisher):
    """
        从 prepare 到 zero 位置
    """
    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/prepare_go_to_zero/prepare_go_to_zero_1.json")
    publish_trajectory(publisher, l_traj, control_index)

    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/prepare_go_to_zero/prepare_go_to_zero_2.json")
    publish_trajectory(publisher, l_traj, control_index)

    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/prepare_go_to_zero/prepare_go_to_zero_3.json")
    publish_trajectory(publisher, l_traj, control_index)

    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/prepare_go_to_zero/prepare_go_to_zero_4.json")
    publish_trajectory(publisher, l_traj, control_index)

    l_traj = load_traj(f"traj/{INPUT_FILE_DIR}/prepare_go_to_zero/prepare_go_to_zero_5.json")
    publish_trajectory(publisher, l_traj, control_index)


def robot_arm_action(kuavo_robot, control_index:int, action_name:str)->bool:
    """
        根据输入的动作名字读取对应的特定的动作发布给Kuavo手臂
    """
    robot_arm_publisher = rospy.Publisher(
        "/kuavo_arm_traj",
        sensor_msgs.msg.JointState,
        queue_size=1
    )

    # 执行动作
    if action_name == "zero_go_to_prepare":
        # 执行作揖动作
        set_robot_arm_zero_go_to_prepare(kuavo_robot, control_index, robot_arm_publisher)
        return True
    elif action_name == "prepare_go_to_zero":
        # 执行初始位置
        set_robot_arm_prepare_go_to_zero(kuavo_robot, control_index, robot_arm_publisher)
        return True
    else:
        # 未知动作
        print(f"未知的动作: {action_name}"+"......please check you action again")
        return False

if __name__ == '__main__':  
    # 初始化节点
    rospy.init_node('kuavo_arm_action_node') 

    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")

    # 等待2s
    time.sleep(2)

    # 机器人进入到手臂规划模式
    robot_instance.set_robot_arm_ctl_mode(True)

    # 机器人执行 动作 打招呼
    robot_arm_action(robot_instance, "zero_go_to_prepare")

    # 等待2s 
    time.sleep(2)

    # 手臂归中
    robot_instance.set_robot_arm_recenter() 

    # 等待1s
    time.sleep(1)

    # 关闭手臂控制
    robot_instance.set_robot_arm_ctl_mode(False)