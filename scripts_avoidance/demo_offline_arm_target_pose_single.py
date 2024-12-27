"""
    离线姿态 - 生成框架
"""
import rospy
import rich
import questionary
from rich.console import Console
from questionary import Separator, Choice
from kuavoRobotSDK import kuavo

from std_msgs.msg import Int32
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from ros_gendexgrasp.srv import dex_contract_srv, dex_contract_srvResponse, dex_contract_srvRequest
from ros_gendexgrasp.srv import dex_gengrasp_srv, dex_gengrasp_srvResponse, dex_gengrasp_srvRequest
from ros_gendexgrasp.srv import dex_grasp_index, dex_grasp_indexResponse, dex_grasp_indexRequest

from grasp_ik_arm_traj.srv import ikMonitorService, ikMonitorServiceResponse, ikMonitorServiceRequest # ik状态监听器
from grasp_filter_gendex.srv import offlineGraspButton, offlineGraspButtonResponse, offlineGraspButtonRequest # 离线姿态筛选/发布器开关

from arm_specific_actions import robot_arm_action # 预设计轨迹
from dynamic_biped.srv import changeArmCtrlMode
from dynamic_biped.msg import robotHeadMotionData
import time

from std_srvs.srv import Trigger, TriggerRequest # 头部采集环境服务
from dynamic_biped.srv import planCumotionJoint, planCumotionJointRequest # cumotion避障轨迹调用
from dynamic_biped.srv import planBezierJoint, planBezierJointRequest, planBezierJointResponse # 贝塞尔曲线轨迹规划模块 planBezierJoint
from dynamic_biped.msg import cuRobotArmStatus #  机器人手臂规划状态 | Go 状态 | Back状态
import math
from std_msgs.msg import Bool

rospy.init_node("kuavo_grasp_demo_node")

arm_mode = False
exit_menu = False
IK_SUCCESS_FLAG = False
IF_USE_PRE_POSE_FLAG = False # True
CUMOTION_FINSHED_FLAG = False # cumotion避障轨迹是否完成
USE_SIM_FLAG = False # 是否仿真测试 True
PLAN_ROBOT_ARM_STATUS_FLAG = True # 手臂规划状态 | True 为go状态 | False 为back状态

console = Console()
robot_instance = kuavo("4_2_kuavo")

# 机器人PLAN_ARM_ROBOT_ARM_STATUS 发布器
robot_arm_status_publisher = rospy.Publisher(
    "/robot_arm_plan_status",
    cuRobotArmStatus,
    queue_size=10
)
def robot_arm_status_timer_callback(event):
    """Callback function for robot arm status timer"""
    global PLAN_ROBOT_ARM_STATUS_FLAG
    msg = cuRobotArmStatus()
    msg.data = PLAN_ROBOT_ARM_STATUS_FLAG
    robot_arm_status_publisher.publish(msg)

robot_arm_publish_rate = 5  # 5次每秒
robot_arm_timer_period = 1.0 / robot_arm_publish_rate
robot_arm_status_timer = rospy.Timer(rospy.Duration(robot_arm_timer_period), robot_arm_status_timer_callback)

# 全局机器人初始状态 joint space | 7维度 | 左手
global_robot_real_zero_state_joint = JointState( 
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

def robot_real_init_joint_states_callback(msg):
    """Callback function for /joint_states topic"""
    global global_robot_real_init_state_joint

    if len(msg.position) >= 7:  # 确保 position 至少有 7 个值
        # 提取前 7 个关节（左手）的位置值 
        global_robot_real_init_state_joint.position = msg.position[:7]
        # rospy.loginfo("Received /joint_states message with position data.")
    else:
        rospy.logwarn("Received /joint_states message with insufficient position data.")

rospy.Subscriber('/joint_states', JointState, robot_real_init_joint_states_callback)

# 全局机器人目标状态 joint space | 靠近的目标姿态 | 7维度 2 
global_robot_new_real_target_state_joint = JointState(
    header=Header(stamp=rospy.Time(0), frame_id='torso'),
    name=[
        'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'
    ],
    position=[
        0.3490658503988659, 0.8726646259971648, 0.55453292519943295, -1.2217304763960306, -0.3490658503988659, -0.5235987755982988, 0.0 # 初始准备姿态 | 请确保周围无障碍物
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
        0.3490658503988659, 0.8726646259971648, 0.55453292519943295, -1.2217304763960306, -0.3490658503988659, -0.5235987755982988, 0.0 # 初始准备姿态 | 请确保周围无障碍物
    ],
    velocity=[],
    effort=[]
)

# 最终的姿态 | 14维度
global_robot_real_current_state_joint = JointState(
    header=Header(stamp=rospy.Time(0), frame_id='torso'),
    name=[
        'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint',
        'zarm_r1_joint', 'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint', 'zarm_r5_joint', 'zarm_r6_joint', 'zarm_r7_joint'
    ],
    position=[
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 
    ],
    velocity=[],
    effort=[]
)

# 全局发布者如果需要
global_robot_arm_publisher = rospy.Publisher(
    "/kuavo_arm_traj",
    JointState,
    queue_size=1
)

def robot_real_target_joint_states_callback(msg):
    """Callback function for /kuavo_cumotion_avoid_joint_target"""
    global global_robot_real_target_state_joint
    global global_robot_real_current_state_joint
    global global_robot_new_real_target_state_joint
    if len(msg.position) >= 7:  # 确保 position 至少有 7 个值
        # 提取前 7 个关节的位置值
        joint_positions = msg.position[:7]

        # 打印数值 
        rospy.loginfo(f"最终目标赋值 joint_positions : {joint_positions}")

        # 转换为弧度 | 发送cumotion请求
        joint_positions_rad = [math.radians(pos) for pos in joint_positions]

        # 最终目标赋值 | 前7个目标值
        global_robot_real_current_state_joint.position[:7] = joint_positions_rad
        global_robot_real_current_state_joint.position[0] += 0.1 # 大臂pitch低一点
        global_robot_real_current_state_joint.position[6] -= 0.3 # 最终抓取位置往中间靠

        # 默认第三个关节的弧度减去0.3 | 发送cumotion请求
        joint_positions_rad[1] += 0.2  # 大臂roll 往稍微拉伸一下 0.3
        joint_positions_rad[3] -= 0.4  # 大臂pitch 往上抬
        joint_positions_rad[5] -= 0.2  # 小臂pitch 往上抬

        # 更新为global_robot_real_target_state_joint | 发送cumotion请求
        global_robot_real_target_state_joint.position = joint_positions_rad
        global_robot_new_real_target_state_joint.position = joint_positions_rad
        
        # 打印数值 
        rospy.loginfo(f" 发送cumotion请求 joint_positions_rad : {global_robot_real_target_state_joint.position}")

        # 打印获取到目标位置 | 因为这是ik之后的结果 频率较低
        rospy.loginfo("Received /kuavo_cumotion_avoid_joint_target message with position data.")
    else:
        rospy.logwarn("Received /kuavo_cumotion_avoid_joint_target message with insufficient position data.")

rospy.Subscriber('/kuavo_cumotion_avoid_joint_target', JointState, robot_real_target_joint_states_callback)

# 头部采集环境客户端
rospy.loginfo('Waiting for /ros1_collect_the_environment service...')
rospy.wait_for_service('/ros1_collect_the_environment')
collect_service = rospy.ServiceProxy('/ros1_collect_the_environment', Trigger)
rospy.loginfo('Service is available!')

def call_collect_service():
    global collect_service
    """调用服务"""
    try:
        rospy.loginfo('Calling /ros1_collect_the_environment service...')
        response = collect_service(TriggerRequest())
        rospy.loginfo(f'Service response: success={response.success}, message="{response.message}"')
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')

# cumotion避障客户端
service_name = '/moveit2_plan_cumotion_joint_srv'
rospy.wait_for_service(service_name)
plan_cumotion_joint = rospy.ServiceProxy(
    service_name, planCumotionJoint
) # 创建服务代理

# 贝塞尔客户端
bezier_service_name = "/generate_plan_arm_trajectory_service"
rospy.wait_for_service(bezier_service_name)
bezier_client = rospy.ServiceProxy(
    bezier_service_name,  planBezierJoint
)

# 避障轨迹是否转发完毕的状态标志
AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = False
def avoidance_traj_bridge_finshed_flag_callback(msg):
    global AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG
    AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG = msg.data

rospy.Subscriber('/avoidance_traj_bridge_flag', Bool, avoidance_traj_bridge_finshed_flag_callback)

### -------------------------------------------- 避障引入 -------------------------------------------------------- ### 
"""
    Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
    Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
    Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
    Point_3 = angle_to_rad([-15, 70, 0, -50, 45, -40, 0])
    Point_4 = angle_to_rad([-50, 50, 0, -30,  0, -50, 0])
    Point_5 = angle_to_rad([-10,  0, 0, -70,-90,   0, 0])
"""
# 张开虎口
hand_traj_data = [0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0]
zero_traj_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
catch_traj_data = [100, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0]

# grasp工厂 | 生成抓取状态服务
gendexgrasp_factory_status_client = rospy.ServiceProxy(
    "/gendex_grasp_service", dex_gengrasp_srv
)

gendexgrasp_factory_set_index_client = rospy.ServiceProxy(
    "/set_grasp_index", dex_grasp_index
)

gendexgrasp_factory_get_index_client = rospy.ServiceProxy(
    "/get_grasp_index", dex_grasp_index
)

# 根据随机种子生成不同的接触图
gendexgrasp_contact_factory_client = rospy.ServiceProxy(
    "/gendex_contact_service", dex_contract_srv
)

# IK监听器打开 | 关闭
ik_monitor_client = rospy.ServiceProxy(
    "/ik_solver_status_monitor", ikMonitorService
)

# 全局ik设置器开关
ik_global_ik_success_request_client = rospy.ServiceProxy(
    "/set_global_ik_success", ikMonitorService
)


# 离线服务器开关
offline_grasp_button_client = rospy.ServiceProxy(
    "/offline_grasp_service", offlineGraspButton
)

# 头部控制话题开关
robot_head_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)

def IK_status_callback(data):
    global IK_SUCCESS_FLAG
    if data.data == 1:
        IK_SUCCESS_FLAG = True
    else:
        IK_SUCCESS_FLAG = False

ik_status_sub= rospy.Subscriber("/ik_solver_status", Int32, IK_status_callback)

## ------------------------------------------ 功能函数 -------------------------------------------------- ## 
def print_dividing_line():
    console.print(
        "[violet]---------------------------------------------------------------------------------"
    )

## ------------------------------------------ ROS服务函数 -------------------------------------------------- ## 
def call_bizer_plan_service(start_flag, joint_sapce_init, joint_sapce_target):
    """
        调用 贝塞尔关节插值 服务，生成贝塞尔轨迹。
        请注意贝塞尔提供14关节自由度的插值，保证维度为14

        :param joint_sapce_init: 初始关节空间    // 7自由度  | 左手 or 右手
        :param joint_sapce_target: 目标关节空间  // 14自由度 | 包含左手and右手
        :return: 成功返回 True，失败返回 False
    """
    global bezier_client

    # 创建服务请求
    req = planBezierJointRequest()
    
    # 设置开始状态
    req.start_flag = start_flag

    # 目标关节空间位置
    req.target_robot_joint_space = joint_sapce_target

    # 初始位置
    bizer_plan_service_state_joint = JointState(
        header=Header(stamp=rospy.Time(0), frame_id='torso'),
        name=[
            'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint',
            'zarm_r1_joint', 'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint', 'zarm_r5_joint', 'zarm_r6_joint', 'zarm_r7_joint'
        ],
        position=[
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 
        ],
        velocity=[],
        effort=[]
    )
    bizer_plan_service_state_joint.position[:7] = joint_sapce_init.position # 左手位置赋值 | 初始位置赋值必须为7维度 | 
    req.init_robot_joint_space = bizer_plan_service_state_joint

    try:
        # 调用服务并获取响应
        rospy.loginfo("Sending bezier_client service request...")
        response = bezier_client(req)
        rospy.loginfo(f"Response: {response.result}")
        return response.result
    except rospy.ServiceException as e:
        rospy.logerr(f"Service bezier_client call failed: {e}")


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

def handleOfflineGraspButton(flag):
    """
    调用 offline_grasp_service 服务，根据 flag 的值开启或关闭抓取话题发布。

    :param flag: int 值，1 表示开启抓取话题发布，0 表示关闭抓取话题发布。
    :return: None
    """
    try:
        # 创建请求对象并设置 data 为传入的 flag
        request = offlineGraspButtonRequest()
        request.data = flag

        # 调用服务并接收响应
        response = offline_grasp_button_client(request)

        # 根据响应结果输出日志
        if response.success:
            if flag == 1:
                rospy.loginfo("Service call succeeded, grasp topic publishing started.")
            else:
                rospy.loginfo("Service call succeeded, grasp topic publishing stopped.")
        else:
            rospy.logwarn("Service call failed.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def ik_global_setting_service(status:int):
    """
        按钮映射到ik监听器服务
        0 设置全局ik的状态标志为 0
        1 设置全局ik的状态标志为 1
    """
    try:
        request = ikMonitorServiceRequest() 
        
        request.data = status

        response = ik_global_ik_success_request_client(request)

        if response.success:
           rospy.loginfo("ik_global_ik_success_request_client command successfully sent.")
        else:
            rospy.logerr("Failed to send ik_global_ik_success_request_client command.")
    except rospy.ServiceException as e:
        rospy.logerr("ik_global_ik_success_request_client Service call failed: %s", str(e))


def button_to_ik_monitor_service(status:int):
    """
        按钮映射到ik监听器服务
        0 关闭ik监听器
        1 打开ik监听器
    """
    try:
        request = ikMonitorServiceRequest()
        
        request.data = status

        response = ik_monitor_client(request)

        if response.success:
           rospy.loginfo("ik_monitor_service command successfully sent.")
        else:
            rospy.logerr("Failed to send ik_monitor_service command.")
    except rospy.ServiceException as e:
        rospy.logerr("ik_monitor_service Service call failed: %s", str(e))

def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    global arm_mode
    result = True
    service_name = "change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=0.5)
        change_arm_ctrl_mode = rospy.ServiceProxy(
            "change_arm_ctrl_mode", changeArmCtrlMode
        )
        change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo("Service call successful")
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
        result = False
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available")
        result = False
    finally:
        if result is False:
            arm_mode = not arm_mode
            console.print(
                f"[red]Failed to change arm control mode to {arm_ctrl_mode}[/red]"
            )
        else:
            console.print(
                f"[green]Changed arm control mode to {arm_ctrl_mode} successfully[/green]"
            )
        return result

def handle_gendexgrasp_factory_status(status, seed_num=42):
    """
        客户端传入status状态，
        1 | 开始生成抓取姿态
        0 | 停止生成抓取姿态
    """
    try:
        request = dex_gengrasp_srvRequest()
        
        request.header = Header()
        request.header.stamp = rospy.Time.now()  # 获取当前时间戳
        request.header.frame_id = "torso"  # 可以根据需要设置帧 ID
        request.status = status
        request.seed_num  = seed_num  
        
        response = gendexgrasp_factory_status_client(request)

        if response.result:
            rospy.loginfo("handle_gendexgrasp_factory_status command successfully gendexgrasp_factory.")
        else:
            rospy.logwarn("Failed to send gendexgrasp_factory command.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

def handle_gendexgrasp_set_index(set_grasp_index):
    """
        客户端传入抓取姿态序号，
        设置抓取姿态序号
    """
    try:
        # 创建服务请求对象
        request = dex_grasp_indexRequest()
        
        # 设置 Header
        request.header = Header()
        request.header.stamp = rospy.Time.now()  # 当前时间戳
        request.header.frame_id = "world"  # 可根据需要修改
        
        # 设置抓取姿态序号
        request.set_grasp_index = set_grasp_index
        
        # 调用服务，获取响应
        response = gendexgrasp_factory_get_index_client(request)

        if response.result:
            rospy.loginfo(f"Grasp index successfully set: {response.set_grasp_index}")
        else:
            rospy.logwarn("Failed to retrieve grasp index.")
        
        return response.result

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return None

def handle_gendexgrasp_get_index():
    """
        客户端获取当前的抓取姿态序号
    """
    try:
        # 创建服务请求对象
        request = dex_grasp_indexRequest()
        
        # 设置 Header
        request.header = Header()
        request.header.stamp = rospy.Time.now()  # 当前时间戳
        request.header.frame_id = "world"  # 可根据需要修改
        request.set_grasp_index = 0
        
        # 调用服务，获取响应
        response = gendexgrasp_factory_get_index_client(request)

        if response.result:
            rospy.loginfo(f"Grasp index successfully get: {response.get_grasp_index}")
        else:
            rospy.logwarn("Failed to retrieve grasp index.")
        
        return response.get_grasp_index

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return None

## ------------------------------------------ 逻辑函数 -------------------------------------------------- ## 
def gendexgrasp():
    global robot_instance
    global arm_mode
    global hand_traj_data
    global catch_traj_data
    global USE_SIM_FLAG
    global global_robot_real_init_state_joint
    global global_robot_real_target_state_joint
    global global_robot_real_current_state_joint
    global CUMOTION_FINSHED_FLAG
    global PLAN_ROBOT_ARM_STATUS_FLAG
    global global_robot_real_zero_state_joint
    global global_robot_new_real_target_state_joint
    """
        （0） 先让机器人扫描周围环境| 然后回到低头状态 | 调用服务端代码
        （1） 设置手臂初始位置 | 调用/kuavo_arm_target_poses
        （2） 调用/gendex_grasp_service 开始生成姿态
        （3） 监听/ik_solver_status 查看是否成功
        （4） 根据监听/kuavo_cumotion_avoid_joint_target', JointState的结果 | 和手臂此时状态的结果 | 构建cumotion避障轨迹客户端 |
        （5） 如果成功调用/gendex_grasp_service 停止生成姿态 | 并且打印ik序号 | 并且按时用户是否继续
        （6） 调用灵巧手进行抓取
        （7） 完成抓取姿态 | 关闭ik监听
    """
    # （0） 观察周围环境
    # call_collect_service()
    arm_mode = False
    call_change_arm_ctrl_mode_service(arm_mode) # 关闭手臂规划模式 | 使用ArmTargetPose控制手臂
    robot_instance.srv_controlEndHand(zero_traj_data) # 手指放平
    PLAN_ROBOT_ARM_STATUS_FLAG = True # 手臂状态为Go状态
    # time.sleep(3)
    
    # （1） 设置手臂初始位置(目前demo展示单手先去到抓取位置)
    if IF_USE_PRE_POSE_FLAG: # 如果要使用预定准备的姿态，则需要来到这个位置
        # robot_arm_action(robot_instance, 0, "zero_go_to_prepare") # 初始位置有所改变，需要更新global_robot_real_init_state_joint
        call_cumotion_service(True, global_robot_real_init_state_joint, global_robot_real_target_state_joint)

    # input( " 请等待机器人robot arm 去到prepare位置 ---------完成后Enter键继续")
    
    # （2） 调用/gendex_grasp_service 开始生成姿态 | 默认使用42的随机种子 | 
    ik_global_setting_service(0) # 全局ik状态为0
    button_to_ik_monitor_service(1) # 打开IK监听
    handleOfflineGraspButton(1) # 开启生成姿态
    time.sleep(1)

    # （3）等待IK_SUCCESS_FLAG为真
    while not IK_SUCCESS_FLAG:
        time.sleep(0.1)

    """
        cuMotion避障轨迹规划 | go 状态
    """
    # （4） 根据监听/kuavo_cumotion_avoid_joint_target', JointState的结果 | 和手臂此时状态的结果 | 构建cumotion避障轨迹客户端 | 
    time.sleep(1)
    call_cumotion_service(True, global_robot_real_init_state_joint, global_robot_new_real_target_state_joint) # 第一次调用规划状态
    rospy.loginfo(f"global_robot_real_init_state_joint.position: {global_robot_real_init_state_joint.position}")
    rospy.loginfo(f"global_robot_new_real_target_state_joint.position: {global_robot_new_real_target_state_joint.position}")

    # （5） 如果成功调用/gendex_grasp_service 停止生成姿态 | 并且打印ik序号 | 并且按时用户是否继续 
    time.sleep(0.5)
    CUMOTION_FINSHED_FLAG = False # 规划未完成
    while not CUMOTION_FINSHED_FLAG:
        time.sleep(0.5) # 查询时间快一些
        rospy.loginfo(f"查询cumotion轨迹规划完成状态: {CUMOTION_FINSHED_FLAG}")
        CUMOTION_FINSHED_FLAG = call_cumotion_service(False, global_robot_real_init_state_joint, global_robot_real_target_state_joint) # 查询规划状态

    """
        贝塞尔插值规划
    """
    # TODO:因为目标为障碍物 所以把最终贴近目标的物体进行追踪
    rospy.loginfo(f"查询cumotion轨迹规划完成状态: {CUMOTION_FINSHED_FLAG}")

    # TODO: 这里要等避障轨迹发完之后才可以进行下一步，
    time.sleep(1)
    while not AVOIDANCE_TRAJ_BRIDGE_FINISHED_FLAG:
        time.sleep(0.001)
    
    # input( " cumotion轨迹规划完已经完成 ---------完成后Enter键继续")
    
    if not USE_SIM_FLAG:
        # 贝塞尔插值到最终目标位置
        call_bizer_plan_service(True, global_robot_real_target_state_joint, global_robot_real_current_state_joint) 
        rospy.loginfo(f"global_robot_real_target_state_joint.position: {global_robot_real_target_state_joint.position}")
        rospy.loginfo(f"global_robot_real_current_state_joint.position: {global_robot_real_current_state_joint.position}")

    time.sleep(2)
    robot_instance.srv_controlEndHand(hand_traj_data) # 打开虎口
    time.sleep(2)

    # 最终位置已到达
    # input( " 最终位置已到达 ---------完成后Enter键继续")

    """
        最终抓取 | 灵巧手已经调用抓取
    """
    # （6） 调用灵巧手进行抓取
    ik_global_setting_service(0) # 全局ik状态为0
    handleOfflineGraspButton(0) # 停止生成姿态
    # TODO:调用/ik_solver_status_monitor 关闭ik监听
    button_to_ik_monitor_service(0) # 关闭ik监听
    
    robot_instance.srv_controlEndHand(catch_traj_data) # 抓住瓶子
    time.sleep(2)
    print_dividing_line()
    
    # input( " 灵巧手已经调用抓取 ---------完成后Enter键继续")

    """
        递水 | 往前递到指定位置
    """
    # （7）完成抓取姿态 | 关闭ik监听
    GoAndcatchStatemsg = global_robot_real_target_state_joint # 确保初始位置是7维度数据
    GoAndcatchStatemsg.position = global_robot_real_current_state_joint.position[:7] # 最终目标位置
    rospy.loginfo(f"global_robot_real_target_state_joint.position: {global_robot_real_target_state_joint.position}")
    rospy.loginfo(f"GoAndcatchStatemsg: {GoAndcatchStatemsg.position}")

    SendJoinStatemsg = global_robot_real_current_state_joint  # 14维度数据 | 递水位置
    SendJoinStatemsg.position[0] -= 0.7 # 大臂pitch 抬高 | 弧度制
    SendJoinStatemsg.position[3] += 0.4 # 小臂pitch 降低 | 弧度制
    rospy.loginfo(f"SendJoinStatemsg: {SendJoinStatemsg.position}")

    call_bizer_plan_service(True, GoAndcatchStatemsg, SendJoinStatemsg) # 从最终目标位置到递水位置
    time.sleep(2)
    robot_instance.srv_controlEndHand(zero_traj_data) # 回到零位
    time.sleep(1)
    # input( "  递水已经完成 ---------完成后Enter键继续")

    """
        从递水位置 | 回到避障的到达的位置
    """
    SendWaterStatemsg = global_robot_real_target_state_joint # 7维度数据
    SendWaterStatemsg.position = SendJoinStatemsg.position[:7] # 从递水位置开始
    rospy.loginfo(f"SendWaterStatemsg: {SendWaterStatemsg.position}")

    BackJoinStatemsg = global_robot_real_current_state_joint  # 14维度数据
    BackJoinStatemsg.position[:7] = global_robot_new_real_target_state_joint.position # 回到原来的目标位置
    rospy.loginfo(f"global_robot_new_real_target_state_joint.position: {global_robot_new_real_target_state_joint.position}")
    rospy.loginfo(f"BackJoinStatemsg: {BackJoinStatemsg.position}")

    call_bizer_plan_service(True, SendWaterStatemsg, BackJoinStatemsg) 
    time.sleep(2)

    # input( " 从递水位置 | 回到避障的到达的位置 ---------完成后Enter键即可结束")

    """
        cuMotion避障轨迹规划 | back 状态
        避障的到达的位置 | 回到初始位置
    """
    PLAN_ROBOT_ARM_STATUS_FLAG = False # 手臂状态为back状态
    time.sleep(1)
    call_cumotion_service(True, global_robot_new_real_target_state_joint, global_robot_real_zero_state_joint)
    rospy.loginfo(f"global_robot_new_real_target_state_joint.position: {global_robot_new_real_target_state_joint.position}")
    rospy.loginfo(f"global_robot_real_target_state_joint.position: {global_robot_real_target_state_joint.position}")
    rospy.loginfo(f"global_robot_real_zero_state_joint.position: {global_robot_real_zero_state_joint.position}")

    time.sleep(0.5)
    CUMOTION_FINSHED_FLAG = False # 规划未完成
    while not CUMOTION_FINSHED_FLAG:
        time.sleep(0.5)
        rospy.loginfo(f"查询cumotion轨迹规划完成状态: {CUMOTION_FINSHED_FLAG}")
        CUMOTION_FINSHED_FLAG = call_cumotion_service(False, global_robot_real_target_state_joint, global_robot_real_zero_state_joint) # 查询规划状态
    
    # input( " 请等待机器人robot arm 回到零位 ---------完成后Enter键即可结束")


def product_contact_map(seed_num=42, contact_num=10):
    """
        根据输入的随机种子和接触点数量，产生接触图
    """
    try:
        # 创建服务请求对象
        request = dex_contract_srvRequest()
        
        # 设置 Header
        request.header = Header()
        request.header.stamp = rospy.Time.now()  # 当前时间戳
        request.header.frame_id = "world"  # 可根据需要修改
        
        # 设置随机种子和接触点数量
        request.seed_num = seed_num
        request.contact_num = contact_num
        
        # 调用服务，获取响应
        response = gendexgrasp_contact_factory_client(request)

        if response.result:
            rospy.loginfo("Contact map generated successfully.")
        else:
            rospy.logwarn("Failed to generate contact map.")
        
        return response.result

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return None

class Menu:
    def __init__(
        self,
        title,
    ):
        self.title = title

    def ask(self):
        question = self.get_question()
        return question.ask()

    def get_question(self):
        return questionary.select(
            self.title,
            choices=[
                "开启手臂规划" if not arm_mode else "关闭手臂规划",
                "采集环境数据", 
                "开始进行Gendexgrasp抓取 ",
                "生成Gendexgrasp接触图", 
                "单独调用姿态生成 | 开始",
                "单独调用姿态生成 | 停止",
                "控制头部head | yaw | Pitch",
                "打开虎口 | 灵巧手",
                "归为零位置 | 灵巧手",
                "复位 - 机械臂回到初始位置",
                "测试 | 灵巧手",
                Separator(),
                "退出",
            ],
        )

    def hand_option(self, option):
        global exit_menu, arm_mode
        global robot_instance
        global hand_traj_data
        if option == "开启手臂规划":
            arm_mode = True
            call_change_arm_ctrl_mode_service(arm_mode)
        elif option == "关闭手臂规划":
            arm_mode = False
            call_change_arm_ctrl_mode_service(arm_mode)
        elif option == "采集环境数据":
            # TODO: call environment_data_collection_service
            call_collect_service()
        elif option == "开始进行Gendexgrasp抓取 ":
            # TODO: call gendexgrasp service
            gendexgrasp()
        elif option == "生成Gendexgrasp接触图":
            # TODO: call gendexgrasp factory service
            seed_num_key = int(input("请输入随机种子的数量 --- ："))
            map_num_key = int(input("请输入接触图的数量 --- ："))
            product_contact_map(seed_num_key, map_num_key)
        elif option == "单独调用姿态生成 | 开始":
            handle_gendexgrasp_factory_status(status=1, seed_num=42)
        elif option == "单独调用姿态生成 | 停止":
            handle_gendexgrasp_factory_status(status=0)
        elif option == "控制头部head | yaw | Pitch":
            # TODO: call head_control service
            msg = robotHeadMotionData()
            # msg.joint_data = [25, -20]  # 向右 | 向下
            # msg.joint_data = [0, -20] # 保持不动 | 向下
            msg.joint_data = [0, 0]
            print("msg.joint_data: ", msg.joint_data)
            robot_head_pub.publish(msg)
            time.sleep(0.1)
        elif option == "打开虎口 | 灵巧手":
            # TODO: call head_control service
            robot_instance.srv_controlEndHand(hand_traj_data) 
        elif option == "归为零位置 | 灵巧手":
            # TODO: call head_control service
            robot_instance.srv_controlEndHand(zero_traj_data) 
        elif option == "复位 - 机械臂回到初始位置":
            # TODO: call reset_arm_service
            robot_arm_action(robot_instance, 0, "prepare_go_to_zero")
        elif option == "测试 | 灵巧手":
            # test
            test_traj_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            robot_instance.srv_controlEndHand(test_traj_data) 
            time.sleep(1)
            test_traj_data = [0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            robot_instance.srv_controlEndHand(test_traj_data) 
        elif option == "退出" or option is None:
            exit_menu = True
            print_dividing_line()


if __name__ == '__main__':
    main_menu = Menu("Main Menu")

    while exit_menu is False :
        option = main_menu.ask()
        main_menu.hand_option(option)
