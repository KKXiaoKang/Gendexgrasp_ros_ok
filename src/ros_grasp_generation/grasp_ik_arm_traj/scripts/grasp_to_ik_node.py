"""

    （1）订阅最佳抓取姿态 | 可视化
    （2）发布IK是否成功
    （3）与灵巧手实物控制服务器通信
    （4）可视化当前机器人手臂状态
"""
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from motion_capture_ik.msg import twoArmHandPose, armHandPose
from visualization_msgs.msg import Marker
import numpy as np
import math
from dynamic_biped.msg import robotHandPosition, robotHeadMotionData
from hand_sdk_control.srv import handPosService, handPosServiceResponse, handPosServiceRequest
from grasp_ik_arm_traj.srv import ikMonitorService, ikMonitorServiceRequest, ikMonitorServiceResponse
import tf.transformations as tft
import time
from dynamic_biped.srv import planBezierJoint, planBezierJointRequest, planBezierJointResponse # 贝塞尔曲线轨迹规划模块 planBezierJoint
from dynamic_biped.msg import armTargetPoses

GLOBAL_IK_SUCCESS = False
last_seq = None

IF_OPEN_MONITOR = False # True开始监听是否ik成功，False则关闭监听ik状态发布
IF_USE_ROBOT_HEAD = True # 是否开启头部

USE_CUMOTION_AVOID = True #  True 是否使用带避障的抓取轨迹发布flag

IF_USE_ROBOT_ARM_TARGET_POSE = True # 是否使用ARM Target pose控制模式控制手臂

def quaternion_to_rotation_matrix(q):
    """Convert a quaternion into a rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])

class GraspToIK:
    def __init__(self):
        # 订阅最佳抓取姿态
        self.grasp_sub = rospy.Subscriber('/best_grasp_pose', PoseStamped, self.grasp_callback)
        
        # 订阅机器人轨迹 | ik之后的轨迹
        self.traj_sub = rospy.Subscriber('/kuavo_arm_traj', JointState, self.traj_callback)
        self.robot_arm_traj = JointState()

        self.now_pub_traj_sub = rospy.Subscriber('/robot_ik_arm_traj', JointState, self.robot_traj_callback)
        self.now_pub_traj_traj = JointState()
        self.pub_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
        self.pub_avoid_target_pub = rospy.Publisher('/kuavo_cumotion_avoid_joint_target', JointState, queue_size=10)

        # 订阅手部状态
        self.hand_pose_sub = rospy.Subscriber('/best_hand_pos', JointState, self.hand_pose_callback)
        self.hand_pose = JointState()

        # 发布 IK 命令
        self.ik_pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd_curobo', twoArmHandPose, queue_size=10)
        
        # 发布ik是否成功的全局流标志位
        self.ik_status_pub = rospy.Publisher('/ik_solver_status', Int32, queue_size=10)

        # TODO:增加灵巧手实物控制服务
        rospy.wait_for_service('/hand_sdk_control_service')
        self.hand_control_client = rospy.ServiceProxy('/hand_sdk_control_service', handPosService)       

        # ik监听状态服务器
        self.ik_status_monitor_server = rospy.Service('/ik_solver_status_monitor', ikMonitorService, self.handle_ik_monitor_request)

        # 全局ik状态设置服务器
        self.ik_global_ik_success_server = rospy.Service('/set_global_ik_success', ikMonitorService, self.handle_ik_global_ik_success_request)

        # 发布机器人的关节状态
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 发布 Marker
        self.marker_pub = rospy.Publisher('/ik_before_visualization_marker', Marker, queue_size=10)

        # 模型路径
        self.model_path = "/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/src/ros_robot_model/biped_s4/meshes/l_hand_roll.STL"

        # 增加真实的物体的姿态
        self.object_real_sub = rospy.Subscriber("object_visualization_marker", Marker, self.object_real_callback)
        self.object_real_msg_info = Marker()
        self.object_real_msg_info.pose.orientation.x = 0.0
        self.object_real_msg_info.pose.orientation.y = 0.0
        self.object_real_msg_info.pose.orientation.z = 0.0
        self.object_real_msg_info.pose.orientation.w = 1.0

        # 初始化joint_state变量
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.define_joint_names()
        self.joint_state_msg.position = [0.0] * len(self.joint_state_msg.name)

        # 50Hz频率发布JointState
        rospy.Timer(rospy.Duration(0.02), self.publish_joint_states)

        # 头部电机数据实时获取
        self.head_motor_data = robotHeadMotionData()
        self.head_motor_data.joint_data.append(0.0) # yaw轴
        self.head_motor_data.joint_data.append(0.0) # pitch轴
        self.head_motor_sub = rospy.Subscriber('/robot_head_motor_position', robotHeadMotionData, self.head_motor_callback) # 底层sdk读出来

        # 创建贝塞尔关节空间曲线服务
        self.generate_plan_arm_trajectory_service_server = rospy.Service('/generate_plan_arm_trajectory_service', planBezierJoint, self.generate_plan_arm_trajectory_service_callback)

    """ -------------------------------------- 贝塞尔插值模块（必须为14维度） ------------------------------------ """
    def publish_joint_state(self, positions):
        """ 发布JointState到 /kuavo_arm_traj 话题 """
        joint_state_msg = JointState()

        # 关节名称
        joint_state_msg.name = [
            "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link", "zarm_l5_link", "zarm_l6_link", "zarm_l7_link",
            "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link", "zarm_r5_link", "zarm_r6_link", "zarm_r7_link"
        ]
        
        # 14维度的关节位置
        if len(positions) == 14:
            joint_state_msg.position = positions
        else:
            rospy.logerr("Position list must be of length 14.")
            return

        # 发布JointState消息
        joint_state_msg.header.stamp = rospy.Time.now()
        self.pub_traj_pub.publish(joint_state_msg) 
        # rospy.loginfo("发布了JointState到/kuavo_arm_traj")

    def bezier_curve(self, P0, P1, P2, P3, t):
        return (1 - t)**3 * np.array(P0) + \
               3 * (1 - t)**2 * t * np.array(P1) + \
               3 * (1 - t) * t**2 * np.array(P2) + \
               t**3 * np.array(P3)

    def pub_kuavo_arm_with_time(self, time, traj_jointstate):
        """
            发布时间和轨迹
        """
        global arm_traj_with_time_pub
        arm_traj_msg = armTargetPoses()
        arm_traj_msg.times = time
        arm_traj_msg.values = traj_jointstate
        arm_traj_with_time_pub.publish(arm_traj_msg) 
        rospy.loginfo(f"Published /kuavo_arm_target_poses arm trajectory with {len(time)} points.")

    def generate_plan_arm_trajectory_service(self, start_positions, target_positions, start_time, end_time, alpha=0.3, offset=1.0):
        # 返回值
        flag = False

        # 初始化一个存储所有关节贝塞尔曲线的列表
        all_bezier_points = []

        # 对每个关节遍历并生成贝塞尔曲线
        for i in range(14):  # 假设有14个关节
            # 构造单关节p0和p3
            P0 = [start_time, start_positions[i]]
            P3 = [end_time, target_positions[i]]
            # print(f"PO (Joint {i}) : {P0} | P3 : {P3}")

            direction = np.array([P3[0] - P0[0], P3[1] - P0[1]])
            # print(f"Direction (Joint {i}) : {direction}")

            perpendicular = np.array([-direction[1], direction[0]])
            perpendicular = perpendicular / np.linalg.norm(perpendicular)  # 归一化

            # 生成控制点 P1 和 P2，加入垂直方向的偏移
            P1 = P0 + alpha * direction + offset * perpendicular
            P2 = P3 - alpha * direction + offset * perpendicular
            # print(f"P1 (Joint {i}): [{P1[0]:.6f}, {P1[1]:.6f}], P2: [{P2[0]:.6f}, {P2[1]:.6f}]")

            # 生成 t 的取值
            t_values = np.linspace(0, 1, 100)

            # 计算贝塞尔曲线的点
            bezier_points = np.array([self.bezier_curve(P0, P1, P2, P3, t) for t in t_values])
            np.set_printoptions(suppress=True, precision=6, formatter={'float_kind': '{:f}'.format})
            # print(f"bezier_points (Joint {i}): {bezier_points}")
            # print(f"bezier_points length (Joint {i}): {len(bezier_points)}")

            # 将当前关节的贝塞尔曲线添加到总列表中
            all_bezier_points.append(bezier_points)

        # 转换为 NumPy 数组，方便操作
        all_bezier_points = np.array(all_bezier_points)

        # 打印和返回所有关节的贝塞尔曲线
        # print(f"All Bezier Points Shape: {all_bezier_points.shape}")

        # 遍历时间步长并发布关节状态
        for t_index in range(len(t_values)):
            # 初始化要发布的关节位置列表
            current_joint_positions = []

            # 提取所有关节在当前时刻的贝塞尔曲线位置
            for i in range(14):
                joint_position_at_t = all_bezier_points[i][t_index][1]  # 获取每个关节的当前时刻的位置
                current_joint_positions.append(joint_position_at_t)

            # 发布当前时刻的所有关节位置
            # print(f"current_joint_positions : {current_joint_positions}")
            time.sleep(0.02)
            self.publish_joint_state(current_joint_positions) # 发布/kuavo_arm_traj话题

        flag = True
        return flag
    
    def generate_arm_target_trajectory_service(self, start_positions, target_positions, start_time, end_time, alpha=0.3, offset=1.0):
        # 初始化一个存储所有关节贝塞尔曲线的列表
        all_bezier_points = []
    
        # 对每个关节遍历并生成贝塞尔曲线
        for i in range(14):  # 假设有14个关节
            # 构造单关节p0和p3
            P0 = [start_time, start_positions[i]]
            P3 = [end_time, target_positions[i]]
    
            direction = np.array([P3[0] - P0[0], P3[1] - P0[1]])
            perpendicular = np.array([-direction[1], direction[0]])
            perpendicular = perpendicular / np.linalg.norm(perpendicular)  # 归一化
    
            # 生成控制点 P1 和 P2，加入垂直方向的偏移
            P1 = P0 + alpha * direction + offset * perpendicular
            P2 = P3 - alpha * direction + offset * perpendicular
    
            # 生成 t 的取值
            t_values = np.linspace(0, 1, 100)
    
            # 计算贝塞尔曲线的点
            bezier_points = np.array([self.bezier_curve(P0, P1, P2, P3, t) for t in t_values])
    
            # 将当前关节的贝塞尔曲线添加到总列表中
            all_bezier_points.append(bezier_points)
    
        # 转换为 NumPy 数组，方便操作
        all_bezier_points = np.array(all_bezier_points)
    
        # 时间序列
        TIME_DATA = [start_time + (end_time - start_time) * t for t in t_values]
    
        # 初始化轨迹数组
        TRAJ_ARRAY = []
    
        # 遍历时间步长，生成关节位置序列
        for t_index in range(len(t_values)):
            # 初始化当前时间点的关节位置列表
            current_joint_positions = []

            # 提取所有关节在当前时刻的贝塞尔曲线位置
            for i in range(14):
                joint_position_at_t = all_bezier_points[i][t_index][1]  # 获取每个关节的当前时刻的位置
                current_joint_positions.append(joint_position_at_t)

            # 直接扩展 current_joint_positions 到 TRAJ_ARRAY（因为当前值已经是角度）
            TRAJ_ARRAY.extend(current_joint_positions)

        # 确保 TRAJ_ARRAY 中所有值为 float
        TRAJ_ARRAY = [float(value) for value in TRAJ_ARRAY]
    
        # 发布到 /kuavo_arm_target_poses
        self.pub_kuavo_arm_with_time(TIME_DATA, TRAJ_ARRAY)
        # rospy.loginfo(f"Published arm target trajectory to TIME_DATA:{TIME_DATA}")
        # rospy.loginfo(f"Published arm target trajectory to TRAJ_ARRAY:{TRAJ_ARRAY}")
        rospy.loginfo("Published arm target trajectory to /kuavo_arm_target_poses.")
        return True

    def generate_plan_arm_trajectory_service_callback(self, request):
        """
            贝塞尔插值服务回调函数 14个关节的手臂回调服务
            统一格式
            传入的都为弧度值
        """
        global IF_USE_ROBOT_ARM_TARGET_POSE
        # data
        start_positions_radians = request.init_robot_joint_space.position
        target_positions_radians = request.target_robot_joint_space.position
        start_flag = request.start_flag

        # 转换弧度为角度
        start_positions_degrees = [math.degrees(pos) for pos in start_positions_radians]
        target_positions_degrees = [math.degrees(pos) for pos in target_positions_radians]

        # 仅调试时查看转换结果
        rospy.loginfo(f"Start positions (degrees): {start_positions_degrees}")
        rospy.loginfo(f"Target positions (degrees): {target_positions_degrees}")

        # 设置开始和结束的时间
        start_time = rospy.Time.now().to_sec()
        end_time = start_time + 2.0  # 假设轨迹时长为2秒

        # 生成贝塞尔曲线服务 | 同时发布出去 | 传入角度数值
        if IF_USE_ROBOT_ARM_TARGET_POSE:
            start_time = 0.0
            end_time = 2.0
            success = self.generate_arm_target_trajectory_service(start_positions_degrees, target_positions_degrees, start_time, end_time, alpha=0.3, offset=1.0)
        else:
            success = self.generate_plan_arm_trajectory_service(start_positions_degrees, target_positions_degrees, start_time, end_time, alpha=0.3, offset=1.0)
        
        # 创建响应 
        response = planBezierJointResponse()

        if success:
            response.result = True
            rospy.loginfo("成功生成 | 发布贝塞尔轨迹！")
        else:
            response.result = False
            rospy.logerr("贝塞尔轨迹发布失败。")

        # 返回响应
        return response

    """ -------------------------------------- 贝塞尔插值模块 ------------------------------------ """

    def head_motor_callback(self, msg):
        """
            默认传入的为角度，需要转换为弧度
        """
        # 头部赋值数据
        self.head_motor_data = msg
        
        # 检查 joint_data 是否有足够的数据
        if len(self.head_motor_data.joint_data) >= 2:
            # 更新 joint 状态，转换为弧度
            self.joint_state_msg.position[-2] = -math.radians(self.head_motor_data.joint_data[0])  # yaw
            self.joint_state_msg.position[-1] = math.radians(self.head_motor_data.joint_data[1])  # pitch
        else:
            rospy.logwarn("头部数据不完整，joint_data 需要至少两个元素。")

        # rospy.loginfo(f"head_motor_data: {self.head_motor_data}")

    def handle_ik_global_ik_success_request(self, request):
        global GLOBAL_IK_SUCCESS
        # 根据请求数据设置全局变量
        if request.data == 1:
            GLOBAL_IK_SUCCESS = True
        else:
            GLOBAL_IK_SUCCESS = False
            rospy.loginfo(f"GLOBAL_IK_SUCCESS is set to {GLOBAL_IK_SUCCESS}")
        # 返回操作成功的响应
        response = ikMonitorServiceResponse()
        response.success = True
        return response
    
    def handle_ik_monitor_request(self, request):
        global IF_OPEN_MONITOR
        # 根据请求数据设置全局变量
        if request.data == 1:
            IF_OPEN_MONITOR = True
        else:
            IF_OPEN_MONITOR = False
        rospy.loginfo(f"IF_OPEN_MONITOR is set to {IF_OPEN_MONITOR}")
        # 返回操作成功的响应
        response = ikMonitorServiceResponse()
        response.success = True
        return response
    
    def send_hand_control_service(self, hand_positions):
        try:
            """
                客户端传入手指的弧度值，发送给服务端，服务端控制会自动做角度的转换，并且控制灵巧手
            """
            # 创建服务请求对象
            request = handPosServiceRequest()

            # 构建 Header
            request.header = rospy.Header()
            request.header.stamp = rospy.Time.now()

            # 将手指弧度值转换为角度
            hand_positions_degrees = [math.degrees(pos) for pos in hand_positions]
            
            # 选择 hand_positions 中的第1, 2, 3, 5, 7, 9 作为控制维度
            selected_positions = [int(hand_positions_degrees[i]) for i in [0, 1, 2, 4, 6, 8]]

            # 将 selected_positions 中的值限制在 0 到 255 之间
            selected_positions = [max(0, min(255, pos)) for pos in selected_positions]
            print(" selected_positions : ", selected_positions)

            # 设置 left_hand_position 和 right_hand_position
            request.left_hand_position = selected_positions
            request.right_hand_position = [0] * 6  # 假设右手没有动作

            # 调用服务并处理响应
            response = self.hand_control_client(request)
            if response.result:
                pass
                # rospy.loginfo("Hand control command successfully sent to the real hand controller.")
            else:
                rospy.logwarn("Failed to send hand control command.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))

    def define_joint_names(self):
        return [
            # 左臂和左手
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", 
            "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", 
            "l_hand_roll",
            # 左手手指
            "l_thumb_proximal_yaw", "l_thumb_distal_pitch", 
            "l_index_proximal_finger", "l_index_distal_finger", 
            "l_middle_proximal_finger", "l_middle_distal_finger", 
            "l_ring_proximal_finger", "l_ring_distal_finger", 
            "l_pinky_proximal_finger", "l_pinky_distal_finger",
            # 右臂和右手
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", 
            "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", 
            "r_hand_roll",
            # 剩余关节名称
            "r_thumb_proximal_yaw", "r_thumb_distal_pitch",
            "r_index_proximal_finger", "r_index_distal_finger",
            "r_middle_proximal_finger", "r_middle_distal_finger",
            "r_ring_proximal_finger", "r_ring_distal_finger",
            "r_pinky_proximal_finger", "r_pinky_distal_finger",
            "l_leg_roll", "l_leg_yaw", "l_leg_pitch", "l_knee",
            "l_foot_pitch", "l_foot_roll", "l_l_bar", "l_l_tendon",
            "l_r_bar", "l_r_tendon", "r_leg_roll", "r_leg_yaw",
            "r_leg_pitch", "r_knee", "r_foot_pitch", "r_foot_roll",
            "r_l_bar", "r_l_tendon", "r_r_bar", "r_r_tendon",
            "head_yaw", "head_pitch"
        ]
    
    def hand_pose_callback(self, msg):
        # 接收到手部状态
        # rospy.loginfo("Received a new hand 灵巧手 pose")
        self.hand_pose = msg

    def object_real_callback(self, msg):
        # 接收到物体的实际位置
        self.object_real_msg_info.pose = msg.pose
        self.object_real_msg_info.header = msg.header

    def robot_traj_callback(self, msg):
        """
            用于监听ik节点发过来的/robot_ik_arm_traj。如果ik监听器打开的情况下才会处理这个话题
        """
        global GLOBAL_IK_SUCCESS, last_seq
        global IF_OPEN_MONITOR
        global USE_CUMOTION_AVOID

        # 比较当前seq与上次的seq
        # 接收到机器人轨迹 | 代表IK逆解成功
        if IF_OPEN_MONITOR:  # 如果为True，开始监听ik状态
            GLOBAL_IK_SUCCESS = True
            # TODO:将轨迹话题转发给/kuavo_arm_traj
            if USE_CUMOTION_AVOID:
                self.pub_avoid_target_pub.publish(msg)  # Avoid msg robot state
            else:
                self.pub_traj_pub.publish(msg)          # Target msg robot state
        else:
            GLOBAL_IK_SUCCESS = False

    def traj_callback(self, msg):
        """
            brief: 用于发布服务灵巧手服务实物可以进行控制 | 重写joint_state将手臂和灵巧手的可视化结果发布出来
            param msg: 机器人轨迹/kuavo_arm_traj
            主要用于可视化
        """
        # 提取左臂和右臂的关节角度
        if len(msg.position) >= 14:
            left_arm_positions = list(msg.position[:7])
            right_arm_positions = list(msg.position[7:14])

            # 角度转弧度
            left_arm_positions = [math.radians(angle) for angle in left_arm_positions]
            right_arm_positions = [math.radians(angle) for angle in right_arm_positions]
        else:
            rospy.logerr("Received joint trajectory does not have enough positions. Expected at least 14, got %d", len(msg.position))
            return

        # 如果 hand_pose 没有接收到，或者 hand_pose.position 数组为空，就将手指的关节弧度置为 0
        if self.hand_pose is None or len(self.hand_pose.position) < 10:
            # rospy.logwarn("Hand pose not received yet or hand pose position array is incomplete. Visualizing only arm positions.")
            left_finger_positions = [0.0] * 10  # 假设手指关节数量为 10
        else:
            # 提取左手手指的关节弧度
            left_finger_positions = list(self.hand_pose.position[:10])

        # 更新 joint_state_msg 的位置
        positions = left_arm_positions + left_finger_positions + right_arm_positions

        # 填充剩余关节
        remaining_joints = len(self.joint_state_msg.name) - len(positions)
        positions.extend([0.0] * remaining_joints)

        # 添加头部的yaw和pitch数据
        positions[-2] = math.radians(self.head_motor_data.joint_data[0])  # yaw
        positions[-1] = math.radians(self.head_motor_data.joint_data[1])  # pitch

        # rospy.loginfo(f"head_motor_data: {self.head_motor_data}")
        # rospy.loginfo(f"positions: {positions}")
        
        self.joint_state_msg.position = positions


    def check_and_call_service(self, event):
        global GLOBAL_IK_SUCCESS, last_seq
        if GLOBAL_IK_SUCCESS:
            # ik成功，告知灵巧手控制节点实物控制灵巧手
            # rospy.loginfo("Calling hand control service with new finger positions.")
            pass # 不需要由ik决定是否发灵巧手
            # left_finger_positions = list(self.hand_pose.position[:10])
            # self.send_hand_control_service(left_finger_positions)

    def publish_joint_states(self, event):
        # 发布 JointState 消息
        # rospy.loginfo("Publishing joint state at 50Hz")
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(self.joint_state_msg)

    def pub_ik_success_service(self, event):
        global GLOBAL_IK_SUCCESS
        if GLOBAL_IK_SUCCESS:
            ik_flag = 1 
        else:
            ik_flag = 0
        # 发布话题
        ik_status_msg = Int32()
        ik_status_msg.data = ik_flag
        self.ik_status_pub.publish(ik_status_msg)
        # rospy.loginfo(f"Published IK success flag: {GLOBAL_IK_SUCCESS}")

    def grasp_callback(self, msg):
        global GLOBAL_IK_SUCCESS, last_seq

        # 接收到机器人grasp | 默认都是显示ik失败
        GLOBAL_IK_SUCCESS = False
    
        # 归一化四元数
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        norm = np.linalg.norm(quat)
        if norm > 0:
            quat /= norm
        else:
            rospy.logerr("四元数的模为零！")

        # 初始化 IK 命令消息
        ik_msg = twoArmHandPose()

        object_position = np.array([
            self.object_real_msg_info.pose.position.x,
            self.object_real_msg_info.pose.position.y,
            self.object_real_msg_info.pose.position.z
        ])
        object_orientation = np.array([
            self.object_real_msg_info.pose.orientation.x,
            self.object_real_msg_info.pose.orientation.y,
            self.object_real_msg_info.pose.orientation.z,
            self.object_real_msg_info.pose.orientation.w
        ])
        grasp_position_offset = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z 
        ])        
        """
            设计左手最终抓取位置
        """
        # 假设姿态是针对右手的
        # final_grasp_pose_x = self.object_real_msg_info.pose.position.x + msg.pose.position.x
        # final_grasp_pose_y = self.object_real_msg_info.pose.position.y + msg.pose.position.y
        # final_grasp_pose_z = self.object_real_msg_info.pose.position.z + msg.pose.position.z
        rotation_matrix = quaternion_to_rotation_matrix(object_orientation)
        rotated_grasp_offset = np.dot(rotation_matrix, grasp_position_offset)
        final_grasp_position = object_position + rotated_grasp_offset
        
        # TODO: 添加offset
        rospy.loginfo(f"final_grasp_position: {final_grasp_position}")

        """ 打开虎口的时候 | kuavo预备姿态"""
        # offset_x = 0.02    # X 轴偏移量
        # offset_y = -0.02   # Y 轴偏移量      
        # offset_z = 0.01  # Z 轴偏移量 0.01

        offset_x = 0.005    # X 轴偏移量
        offset_y = -0.015   # Y 轴偏移量      
        offset_z = 0.0      # Z 轴偏移量 0.0

        final_grasp_position[0] += offset_x  # X 轴
        final_grasp_position[1] += offset_y  # Y 轴
        final_grasp_position[2] += offset_z  # Z 轴

        rospy.loginfo(f"final_grasp_position with offset: {final_grasp_position}")
        ik_msg.left_pose.pos_xyz = final_grasp_position.tolist()
        """
            设计左手最终抓取姿态
        """
        # 合并抓取姿态和物体姿态
        ## object_orientation为基底。quat为旋转
        combined_quat = tft.quaternion_multiply(object_orientation, quat)
        rospy.loginfo(f"final_grasp_ combined_quat: {combined_quat}")
        ik_msg.left_pose.quat_xyzw = combined_quat.tolist()
        """
            左手的其他设置
        """
        # 简单起见，将肘部位置和关节角度设置为零
        ik_msg.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        ik_msg.left_pose.joint_angles = [0.0] * 7

        # 如果需要，可以选择填充 right_pose
        ik_msg.right_pose.pos_xyz = [0.0, 0.0, 0.0]
        ik_msg.right_pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        ik_msg.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        ik_msg.right_pose.joint_angles = [0.0] * 7

        # 发布 IK 命令
        # rospy.loginfo("基于最佳抓取姿态发布 IK 命令")
        self.ik_pub.publish(ik_msg)

        # 创建并发布 Marker
        marker = Marker()
        marker.header.frame_id = "torso"
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "file://" + self.model_path

        marker.pose.position.x = ik_msg.left_pose.pos_xyz[0] 
        marker.pose.position.y = ik_msg.left_pose.pos_xyz[1] 
        marker.pose.position.z = ik_msg.left_pose.pos_xyz[2] 

        marker.pose.orientation.x = ik_msg.left_pose.quat_xyzw[0]
        marker.pose.orientation.y = ik_msg.left_pose.quat_xyzw[1]
        marker.pose.orientation.z = ik_msg.left_pose.quat_xyzw[2]
        marker.pose.orientation.w = ik_msg.left_pose.quat_xyzw[3]

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5

        # rospy.loginfo("发布 IK 前的 Marker")
        self.marker_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('grasp_to_ik_node', anonymous=True)

    # armTargetPose全局
    global arm_traj_with_time_pub
    arm_traj_with_time_pub = rospy.Publisher("/kuavo_arm_target_poses", armTargetPoses, queue_size=10)
    
    # ik节点实例化
    node = GraspToIK()

    # 定时器用于检查并调用灵巧手抓取服务
    rospy.Timer(rospy.Duration(0.01), node.check_and_call_service)
    
    # TODO:增加一个定时器用于逆解成功后 | 服务调用：通知gendexgrasp可以停止生成姿态了
    rospy.Timer(rospy.Duration(0.01), node.pub_ik_success_service)

    rospy.spin()