"""
    在线姿态 - 生成框架
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

from arm_specific_actions import robot_arm_action # 预设计轨迹
from dynamic_biped.srv import changeArmCtrlMode
import time

arm_mode = False
exit_menu = False
IK_SUCCESS_FLAG = False

console = Console()
robot_instance = kuavo("4_2_kuavo")

"""
    Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
    Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
    Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
    Point_3 = angle_to_rad([-15, 70, 0, -50, 45, -40, 0])
    Point_4 = angle_to_rad([-50, 50, 0, -30,  0, -50, 0])
    Point_5 = angle_to_rad([-10,  0, 0, -70,-90,   0, 0])
"""
csv_data = {
    'init': [
        "hand_pose0 0.5  20  50 0   0  10   0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
        "hand_pose1 1  30  90 0 -50  90 -30 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
        "hand_pose2 1.5 -15  70 0 -50  45 -40 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
        "hand_pose3 2 -50  50 0 -30   0 -50 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
        "hand_pose4 3 -10   0 0 -70 -90   0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    ]
}

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

def IK_status_callback(data):
    global IK_SUCCESS_FLAG
    if data.data == 1:
        IK_SUCCESS_FLAG = True
    else:
        IK_SUCCESS_FLAG = False

ik_status_sub= rospy.Subscriber("/ik_solver_status", Int32, IK_status_callback)

## ------------------------------------------ 功能函数 -------------------------------------------------- ## 
def parse_csv_data(data):
    time_data = []
    traj_array = []
    for row in data:
        columns = row.split()
        time_data.append(float(columns[1]))
        traj_array.extend([float(a) for a in columns[2:16]])
    return time_data, traj_array

def test_arm_init(robot_instance):
    """
        kuavo -- 手部 /kuavo_arm_target_poses 控制
        从armTargetPose 转换为 moveit固定轨迹规划
    """
    data = csv_data['init']
    time_data, traj_array = parse_csv_data(data)
    robot_instance.pub_kuavo_arm_with_time(time_data, traj_array)
    print("robot arm traj publisher finish")
    time.sleep(1)

def print_dividing_line():
    console.print(
        "[violet]---------------------------------------------------------------------------------"
    )

## ------------------------------------------ ROS服务函数 -------------------------------------------------- ## 
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
    """
        （1） 设置手臂初始位置 | 调用/kuavo_arm_target_poses
        （2） 调用/gendex_grasp_service 开始生成姿态
        （3） 监听/ik_solver_status 查看是否成功
        （4） 如果成功调用/gendex_grasp_service 停止生成姿态 | 并且打印ik序号 | 并且按时用户是否继续
    """
    arm_mode = True
    call_change_arm_ctrl_mode_service(arm_mode)

    # （1） 设置手臂初始位置(目前demo展示单手先去到抓取位置)
    robot_arm_action(robot_instance, 0, "zero_go_to_prepare")
    input( " 请等待机器人robot arm 去到prepare位置 ---------完成后Enter键继续")

    # （2） 调用/gendex_grasp_service 开始生成姿态 | 默认使用42的随机种子 | 打开IK监听
    # TODO:调用/ik_solver_status_monitor 打开ik监听
    button_to_ik_monitor_service(1)
    time.sleep(2)
    handle_gendexgrasp_factory_status(status=1, seed_num=42)

    # （3）等待IK_SUCCESS_FLAG为真
    while not IK_SUCCESS_FLAG:
        time.sleep(0.1)

    # （4） 如果成功调用/gendex_grasp_service 停止生成姿态 | 并且打印ik序号 | 并且按时用户是否继续 
    handle_gendexgrasp_factory_status(status=0)
    ik_success_index_grasp = handle_gendexgrasp_get_index()
    print_dividing_line()
    console.print(f"[green]IK_SUCCESS_FLAG is True, IK_SUCCESS_INDEX_GRASP is {ik_success_index_grasp}[/green]")
    input( " IK求解成功，姿态解算已经完成 ---------完成后Enter键继续")

    # （5）完成抓取姿态 | 关闭ik监听
    # TODO:调用/ik_solver_status_monitor 关闭ik监听
    button_to_ik_monitor_service(0)
    robot_arm_action(robot_instance, 0, "prepare_go_to_zero")
    input( " 请等待机器人robot arm 回到零位 ---------完成后Enter键即可结束")

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
                "开始进行Gendexgrasp抓取 ",
                "生成Gendexgrasp接触图", 
                "单独调用姿态生成 | 开始",
                "单独调用姿态生成 | 停止",
                Separator(),
                "退出",
            ],
        )

    def hand_option(self, option):
        global exit_menu, arm_mode
        if option == "开启手臂规划":
            arm_mode = True
            call_change_arm_ctrl_mode_service(arm_mode)
        elif option == "关闭手臂规划":
            arm_mode = False
            call_change_arm_ctrl_mode_service(arm_mode)
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
        elif option == "退出" or option is None:
            exit_menu = True
            print_dividing_line()


if __name__ == '__main__':
    rospy.init_node("kuavo_grasp_demo_node")
    main_menu = Menu("Main Menu")

    while exit_menu is False :
        option = main_menu.ask()
        main_menu.hand_option(option)
