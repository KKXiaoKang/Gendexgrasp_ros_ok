#!/usr/bin/env python
import rospy
from dynamic_biped.srv import planCumotionJoint, planCumotionJointRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool
import time

rospy.init_node("plan_Box_grasp_node")
CUMOTION_FINSHED_FLAG = False  # cumotion避障轨迹是否完成

# cumotion避障客户端
service_name = '/moveit2_plan_cumotion_joint_srv'
rospy.wait_for_service(service_name)
plan_cumotion_joint = rospy.ServiceProxy(
    service_name, planCumotionJoint
)  # 创建服务代理

# 全局机器人实时状态 joint space | 7维度 | 左手
global_robot_real_init_state_joint = JointState(
    header=Header(stamp=rospy.Time(0), frame_id='torso'),
    name=[
        'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'
    ],
    position=[
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
    velocity=[],
    effort=[]
)

# 全局机器人目标状态 joint space | 靠近的目标姿态 | 7维度
global_robot_real_target_state_joint = JointState(
    header=Header(stamp=rospy.Time(0), frame_id='torso'),
    name=[
        'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'
    ],
    position=[
        0.3490658503988659, 1.8726646259971648, 0.55453292519943295, -1.2217304763960306, -0.3490658503988659, -0.5235987755982988, 0.0  # 初始准备姿态 | 请确保周围无障碍物
    ],
    velocity=[],
    effort=[]
)

# 创建发布器，用于发布 CUMOTION_FINSHED_FLAG
flag_pub = rospy.Publisher('/grab_box/curobo_finsh_flag', Bool, queue_size=10)

def call_cumotion_service(start_flag, joint_sapce_init, joint_sapce_target):
    """
        调用 cumotion 避障服务，生成避障轨迹。
        :param joint_sapce_init: 初始关节空间
        :param joint_sapce_target: 目标关节空间
        :return: 成功返回 True，失败返回 False
    """
    global plan_cumotion_joint

    # 创建服务请求
    req = planCumotionJointRequest()

    req.start_flag = start_flag
    req.init_robot_joint_space = joint_sapce_init
    req.target_robot_joint_space = joint_sapce_target

    try:
        # 调用服务并获取响应
        rospy.loginfo("Sending service request...")
        response = plan_cumotion_joint(req)
        rospy.loginfo(f"Response: {response.result}")
        return response.result
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def update_finish_flag():
    """
    更新 CUMOTION_FINSHED_FLAG，并发布到 /grab_box/curobo_finsh_flag
    """
    global CUMOTION_FINSHED_FLAG
    flag_msg = Bool()
    flag_msg.data = CUMOTION_FINSHED_FLAG
    flag_pub.publish(flag_msg)
    rospy.loginfo(f"Published CUMOTION_FINSHED_FLAG: {CUMOTION_FINSHED_FLAG}")

if __name__ == '__main__':
    """
        服务端接收请求之后需要做的事情
    """
    # TODO :（1）根据AprilTag世界坐标系下的位置进行ik求解

    # （2）调取curobo服务
    call_cumotion_service(True, global_robot_real_init_state_joint, global_robot_real_target_state_joint)
    time.sleep(0.5)
    CUMOTION_FINSHED_FLAG = False  # 规划未完成
    # 发布初始完成状态
    update_finish_flag()
    
    # （3）查询服务状态
    while not CUMOTION_FINSHED_FLAG:
        time.sleep(0.5)
        rospy.loginfo(f"查询cumotion轨迹规划完成状态: {CUMOTION_FINSHED_FLAG}")
        CUMOTION_FINSHED_FLAG = call_cumotion_service(False, global_robot_real_init_state_joint, global_robot_real_target_state_joint)  # 查询规划状态
        # 更新并发布完成状态
        update_finish_flag()

    # 确保规划完成后也发布一次完成状态
    update_finish_flag()
