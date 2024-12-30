from os import path
import time
import rospy
import math
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid
from curobo.geom.types import Cylinder
from curobo.geom.types import Mesh
from curobo.geom.types import Sphere
from curobo.geom.types import VoxelGrid as CuVoxelGrid
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState as CuJointState # 防止和ros自带的JointState重名
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import get_robot_configs_path
from curobo.util_file import join_path
from curobo.util_file import load_yaml
from curobo.rollout.rollout_base import Goal
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import RobotTrajectory
from nvblox_msgs.srv import EsdfAndGradients, EsdfAndGradientsRequest, EsdfAndGradientsResponse
import numpy as np
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from std_msgs.msg import Bool
import torch
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import MultiDOFJointTrajectory
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from curobo_trajectory_server.srv import cuRoMoveGroup, cuRoMoveGroupResponse, cuRoMoveGroupRequest
from curobo_trajectory_server.srv import cuRoMpcSetting, cuRoMpcSettingResponse, cuRoMpcSettingRequest
from kuavo_msgs.msg import sensorsData
import threading

# from curobo.types.robot import JointState
# from isaac_ros_cumotion.xrdf_utils import convert_xrdf_to_curobo
# from moveit_msgs.action import MoveGroup
# from ament_index_python.packages import get_package_share_directory
# from kuavo_robot_interfaces.msg import CuRobotArmStatus
# from nvblox_msgs.srv import EsdfAndGradients

global_esdf_grid = None # 全局esdf地图 | 用于存放当前规划所用的图
origin_esdf_grid = None # 实时后台更新的esdf地图
tsdf_response = None # tsdf_response结果
"""
    # 机器人状态 | True 为 go状态 | False 为 back状态
"""
global_robot_arm_plan_status = True 

"""
    # 全局是否在处理 True 空闲 为 | False 为 处理中
"""
global_if_process_action_flag = True 

"""
    # 机器人轨迹状态 | 1 Go状态下的轨迹发布器 | 0 Back状态下的轨迹发布器
    JointTrajectory
"""
TRAJ_COUNTER = 1

"""
    # 机器人轨迹存放全局
"""
JOINT_TRAJ_GO = None
JOINT_TRAJ_BACK = None

"""
    # 是否进入处理图的过程
"""
IF_PROCESS_VOXEL_MAP_SATUS = False

DEBUG_MODE_FLAG = False

JOINT_NAME_LIST = [ "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link", "zarm_l5_link", "zarm_l6_link", "zarm_l7_link",
                    "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link", "zarm_r5_link", "zarm_r6_link", "zarm_r7_link"]
class CumotionActionServer:
    def __init__(self):
        rospy.init_node('trajectory_server_node', anonymous=True)

        # Declare and initialize parameters
        self.tensor_args = TensorDeviceType()
        self.robot_config = rospy.get_param('~robot', '/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/kuavo_assets/biped_s40/urdf/biped_s40.yml')
        self.time_dilation_factor = rospy.get_param('~time_dilation_factor', 1.0)
        self.collision_cache_mesh = rospy.get_param('~collision_cache_mesh', 20)
        self.collision_cache_cuboid = rospy.get_param('~collision_cache_cuboid', 20)
        self.interpolation_dt = rospy.get_param('~interpolation_dt', 0.02)
        
        self.__voxel_dims = rospy.get_param('~voxel_dims', [2.0, 2.0, 2.0])
        self.__voxel_size = rospy.get_param('~voxel_size', 0.02)
        # rospy.loginfo(f"Voxel dimensions: {self.__voxel_dims}, Voxel size: {self.__voxel_size}")

        self.publish_voxel_size = rospy.get_param('~publish_voxel_size', 0.01)
        self.max_publish_voxels = rospy.get_param('~max_publish_voxels', 50000)
        self.joint_states_topic = rospy.get_param('~joint_states_topic', '/joint_states')
        self.tool_frame = rospy.get_param('~tool_frame', None)
        self.grid_position = rospy.get_param('~grid_position', [0.0, 0.0, 0.0])
        self.esdf_service_name = rospy.get_param('~esdf_service_name', '/nvblox_node/get_esdf_and_gradient')
        self.urdf_path = rospy.get_param('~urdf_path', '/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/kuavo_assets/biped_s40/urdf/biped_s40_left_arm.urdf')
        self.enable_debug_mode = rospy.get_param('~enable_curobo_debug_mode', False)
        self.add_ground_plane = rospy.get_param('~add_ground_plane', False)
        self.override_moveit_scaling_factors = rospy.get_param('~override_moveit_scaling_factors', True)
        self.read_esdf_world = rospy.get_param('~read_esdf_world', True)
        self.publish_curobo_world_as_voxels = rospy.get_param('~publish_curobo_world_as_voxels', True)

        # Configure logging
        if self.enable_debug_mode:
            setup_curobo_logger('info')
        else:
            setup_curobo_logger('warning')

        # 目标物体 |目标mpc全局goal
        self.goal = None
        self.cube_pose = PoseStamped()
        self.cube_pose.header.frame_id = 'base_link'
        self.cube_pose.pose.position.x = 0.4
        self.cube_pose.pose.position.y = 0.3
        self.cube_pose.pose.position.z = 0.3
        self.cube_pose.pose.orientation.x = 0.0
        self.cube_pose.pose.orientation.y = 0.0
        self.cube_pose.pose.orientation.z = 0.0 
        self.cube_pose.pose.orientation.w = 1.0
        rospy.Timer(rospy.Duration(0.5), self.publish_marker)
        self.mpc_l_arm_marker_pub = rospy.Publisher('/mpc_l_arm_goal/marker', Marker, queue_size=10)
        
        # 记录上一次末端追踪的pose
        self.past_pose = None

        # Publishers
        self.voxel_pub = rospy.Publisher('/curobo/voxels', Marker, queue_size=10)
        self.go_joint_traj_pub = rospy.Publisher('/cumotion_go_joint_trajectory_topic', JointTrajectory, queue_size=10)
        self.back_joint_traj_pub = rospy.Publisher('/cumotion_back_joint_trajectory_topic', JointTrajectory, queue_size=10)
        self.global_if_process_flag_pub = rospy.Publisher('/global_if_process_action_flag_topic', Bool, queue_size=10)
        self.flag_publisher_ = rospy.Publisher('/global_success_action_flag_topic', Bool, queue_size=10)

        # Timers for periodic publishing
        rospy.Timer(rospy.Duration(0.2), self.timer_callback)
        self.arm_status_sub = rospy.Subscriber('/robot_arm_plan_status', Bool, self.arm_status_callback)

        # 监听关节数据
        self.current_curobo_joint_space = None # 用于将机器人实时的关节状态转换为CuJointState状态Cuda数据
        self._joint_names = ['zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint']
        self.current_robot_joint_space = JointState(
            header=Header(stamp=rospy.Time(0), frame_id='torso'),
            name=self._joint_names,
            position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            effort=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.ocs2_robot_arm_status_sub = rospy.Subscriber('/sensors_data_raw', sensorsData, self.ocs2_robot_arm_status_callback)
        
        # 发送关节空间控制命令
        self.robot_arm_joint_cmd_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)

        # Server服务端
        self._action_server = rospy.Service('/cumotion/move_group', cuRoMoveGroup, self.execute_callback)
        self._mpc_action_server = rospy.Service('/cumotion/mpc_set_goal', cuRoMpcSetting, self.mpc_goal_callback)
        
        # TF2 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.source_frame = 'camera_link'
        self.target_frame = 'base_link'

        # Initialize additional attributes
        self.__esdf_client = None
        self.__esdf_req = None
        self.__read_esdf_grid = self.read_esdf_world        
        # warm up
        self.warmup()
        self.__query_count = 0
        self.__tensor_args = self.mpc.tensor_args
        self.subscription = rospy.Subscriber(self.joint_states_topic, JointState, self.js_callback)
        self.__js_buffer = None
        # 创建客户端
        if self.__read_esdf_grid:
            # 等待
            rospy.loginfo(f"Waiting for {self.esdf_service_name} service...")
            rospy.wait_for_service(self.esdf_service_name)
            # 赋值
            self.__esdf_req = EsdfAndGradientsRequest()
            # 创建esdf客户端
            self.__esdf_client = rospy.ServiceProxy(self.esdf_service_name, EsdfAndGradients)
            rospy.loginfo(f"Connected to {self.esdf_service_name} service")

    def mpc_goal_callback(self, request):
        try:
            # 提取Pose数据并赋值给self.cube_pose
            self.cube_pose = request.pose

            # 输出接收到的目标pose信息
            rospy.loginfo("Received new goal pose:")
            rospy.loginfo(f"Position: {self.cube_pose.pose.position}")
            rospy.loginfo(f"Orientation: {self.cube_pose.pose.orientation}")

            # 返回成功结果
            return cuRoMpcSettingResponse(result=True)

        except Exception as e:
            rospy.logerr(f"Error in mpc_goal_callback: {e}")
            return cuRoMpcSettingResponse(result=False)

    def control_robot_arm_cmd(self, interpolated_positions):
        joint_state_msg = JointState()
        joint_state_msg.name = JOINT_NAME_LIST
        joint_state_msg.position = list(interpolated_positions)  # 填充左臂的7个位置
        joint_state_msg.position += [0.0] * 7  # 为右臂的7个关节位置填充零
        joint_state_msg.position = [angle * (180 / math.pi) for angle in joint_state_msg.position]
        self.robot_arm_joint_cmd_pub.publish(joint_state_msg)

    def ocs2_robot_arm_status_callback(self, msg):
        """处理机器人手臂状态的回调函数"""
        # sensors_msg/JointState 维护
        if len(msg.joint_data.joint_q) >= 19:  # 确保数据长度足够
            self.current_robot_joint_space.position = msg.joint_data.joint_q[12:19]
            # rospy.loginfo(f"Received /sensors_data_raw message: {self.current_robot_joint_space.position}")
        else:
            rospy.logwarn("Received joint_data.joint_q does not contain enough elements!")

        curobo_joint_state = torch.tensor([self.current_robot_joint_space.position[0], 
                                           self.current_robot_joint_space.position[1],
                                           self.current_robot_joint_space.position[2],
                                           self.current_robot_joint_space.position[3], 
                                           self.current_robot_joint_space.position[4], 
                                           self.current_robot_joint_space.position[5], 
                                           self.current_robot_joint_space.position[6]
                                           ])
        self.current_curobo_joint_space = CuJointState.from_position(position=curobo_joint_state,
                                                                     joint_names=self._joint_names) 

    def publish_marker(self, event):
        """定时器回调函数，用于发布 Marker"""
        # 创建 Marker 消息
        marker = Marker()

        # 设置 Marker 的基本属性
        marker.header.frame_id = "base_link"  # 设置父坐标系
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cube_namespace"  # 命名空间
        marker.id = 0  # Marker 的 ID
        marker.type = Marker.CUBE  # 设置 Marker 的形状为 CUBE
        marker.action = Marker.ADD  # 添加 Marker

        # 设置 Marker 的位置和方向
        marker.pose.position.x = self.cube_pose.pose.position.x
        marker.pose.position.y = self.cube_pose.pose.position.y
        marker.pose.position.z = self.cube_pose.pose.position.z
        marker.pose.orientation.x = self.cube_pose.pose.orientation.x
        marker.pose.orientation.y = self.cube_pose.pose.orientation.y
        marker.pose.orientation.z = self.cube_pose.pose.orientation.z
        marker.pose.orientation.w = self.cube_pose.pose.orientation.w

        # 设置 Marker 的尺寸（例如，立方体的边长）
        marker.scale.x = 0.05  # 长度
        marker.scale.y = 0.05  # 宽度
        marker.scale.z = 0.05  # 高度

        # 设置 Marker 的颜色（例如，红色）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # alpha 透明度

        # 发布 Marker 消息
        self.mpc_l_arm_marker_pub.publish(marker)
        
    def timer_callback(self, event):
        try:
            global global_if_process_action_flag
            msg = Bool()
            msg.data = global_if_process_action_flag
            self.global_if_process_flag_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Exception in timer_callback: {e}")


    def arm_status_callback(self, msg):
        """处理机器人手臂规划状态的回调函数"""
        global global_robot_arm_plan_status  # 声明使用全局变量
        global_robot_arm_plan_status = msg.data  # 更新全局变量

    def js_callback(self, msg):
        self.__js_buffer = {
            'joint_names': msg.name,
            'position': msg.position,
            'velocity': msg.velocity,
        }

    def warmup(self):
        global DEBUG_MODE_FLAG
        # Load robot config
        self.robot_config_path = join_path(get_robot_configs_path(), self.robot_config)

        # Retrieve parameters from ROS parameter server
        collision_cache_cuboid = rospy.get_param('~collision_cache_cuboid', 20)
        collision_cache_mesh = rospy.get_param('~collision_cache_mesh', 20)
        interpolation_dt = rospy.get_param('~interpolation_dt', 0.02)

        # Define world configuration
        world_file = WorldConfig.from_dict(
            {
                'cuboid': {
                    'table': {
                        'pose': [0, 0, -0.05, 1, 0, 0, 0],
                        'dims': [2.0, 2.0, 0.1],
                    }
                },
                'voxel': {
                    'world_voxel': {
                        'dims': self.__voxel_dims,
                        'pose': [0, 0, 0, 1, 0, 0, 0],
                        'voxel_size': self.__voxel_size,
                        'feature_dtype': torch.bfloat16,
                    },
                },
            }
        )

        # Load the robot configuration
        robot_dict = load_yaml(self.robot_config_path)
        rospy.loginfo(f"Loaded robot config: {self.robot_config_path}")

        # Update robot configuration for URDF path if needed
        if self.urdf_path:
            robot_dict['robot_cfg']['kinematics']['urdf_path'] = self.urdf_path

        robot_dict = robot_dict['robot_cfg']

        # MPC - Setting
        mpc_config = MpcSolverConfig.load_from_robot_config(
            robot_dict,
            world_file,
            tensor_args=self.tensor_args,
            collision_cache={
                'obb': collision_cache_cuboid,
                'mesh': collision_cache_mesh,
            },
            collision_checker_type=CollisionCheckerType.VOXEL,
            use_cuda_graph=True,
            use_cuda_graph_metrics=True,
            use_cuda_graph_full_step=False,
            self_collision_check=True,
            use_mppi=True,
            use_lbfgs=False,
            use_es=False,
            store_rollouts=True,
            step_dt=0.02,
        )

        mpc = MpcSolver(mpc_config)
        self.mpc = mpc
        self.__robot_base_frame = mpc.kinematics.base_link
        rospy.loginfo(f"Robot base frame: {self.__robot_base_frame}")

        # 初次求解 | 要等/sensors_data_raw话题完成
        rospy.loginfo("Waiting for /sensors_data_raw topic to publish...")
        sensors_data = rospy.wait_for_message('/sensors_data_raw', sensorsData)  # 替换 SensorMessageType 为实际消息类型
        rospy.loginfo(f"Received /sensors_data_raw message: {sensors_data.joint_data.joint_q}")

        retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
        joint_names = mpc.rollout_fn.joint_names
        state = mpc.rollout_fn.compute_kinematics(
            CuJointState.from_position(retract_cfg, joint_names=joint_names)
        )
        rospy.loginfo(f"MPC warmup state retract_cfg: {retract_cfg}") # 获取初始位置

        current_state = CuJointState.from_position(retract_cfg, joint_names=joint_names)
        retract_pose = Pose(state.ee_pos_seq, quaternion=state.ee_quat_seq)
        goal = Goal(
            current_state=current_state,
            goal_state=CuJointState.from_position(retract_cfg, joint_names=joint_names),
            goal_pose=retract_pose,
        )
        self.goal_buffer = mpc.setup_solve_single(goal, 1)
        mpc.update_goal(self.goal_buffer)
        mpc_result = mpc.step(current_state, max_attempts=2)
        rospy.loginfo(f"MPC warmup mpc_result: { mpc_result.js_action.position.cpu().numpy()}") # 获取position位置

        # 碰撞世界检查声明
        self.__world_collision = mpc.world_coll_checker
        rospy.loginfo('cuMotion_MPC_Server is ready for planning queries!')

    def send_request(self, aabb_min_m, aabb_size_m):
        self.__esdf_req.aabb_min_m = aabb_min_m
        self.__esdf_req.aabb_size_m = aabb_size_m
        
        # esdf_future = self.__esdf_client.call_async(self.__esdf_req) # 异步客户端调用服务端
        # return esdf_future
        try:
            # 使用同步调用而不是异步调用
            esdf_response = self.__esdf_client(self.__esdf_req)
            return esdf_response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
        
    def execute_callback(self, goal_handle):
        """
            TODO: 用于后续更新mpc追踪的ik_goal(具体更新cube_pose的PoseStamped数据类型即可)
        """
        rospy.logwarn("You are using cuMotion_MPC_Server, this service is for cuMotion_Motion_Server . it's not recommended to use it for planning service")

    def update_mpc_goal(self):
        """
        更新目标状态 (例如更新目标位姿或目标位置)
        """
        # 这里根据需要更新目标
        if self.cube_pose:
            # 更新全局目标
            cube_pose_position = torch.tensor([self.cube_pose.pose.position.x, 
                                               self.cube_pose.pose.position.y,
                                               self.cube_pose.pose.position.z]).to('cuda')  # 转移到 CUDA

            cube_pose_orientation = torch.tensor([self.cube_pose.pose.orientation.x, 
                                                  self.cube_pose.pose.orientation.y,
                                                  self.cube_pose.pose.orientation.z,
                                                  self.cube_pose.pose.orientation.w]).to('cuda')  # 转移到 CUDA
            
            goal_mpc_pose = Pose(cube_pose_position, cube_pose_orientation)
            self.goal = Goal(
                current_state=self.current_curobo_joint_space,
                goal_pose=goal_mpc_pose,
            )
            # 更新MPC的目标缓冲区
            self.goal_buffer = self.mpc.setup_solve_single(self.goal, 1)
            self.mpc.update_goal(self.goal_buffer)
        
            # self.goal_buffer.goal_pose.copy_(self.goal)
            # self.mpc.update_goal(self.goal_buffer)
            # 更新
            # rospy.loginfo(f"MPC goal updated: {self.goal_buffer.goal_pose.position.cpu().numpy()}")
            
    def run(self):
        rate = rospy.Rate(100)  # 设置运行频率为 100Hz
        rospy.loginfo("cuMotion_MPC_Server Now is running")
        while not rospy.is_shutdown():
            # TODO:更新目标
            self.update_mpc_goal()

            # TODO:更新下一步状态
            mpc_result = self.mpc.step(self.current_curobo_joint_space, max_attempts=2)

            # TODO:发送CMD命令
            if mpc_result and hasattr(mpc_result, 'js_action') and mpc_result.js_action:
                joint_positions = mpc_result.js_action.position
                # Convert to list or numpy array for publishing
                joint_positions_list = joint_positions.squeeze().cpu().numpy().tolist()  # Remove singleton dimensions and move to CPU
                # rospy.loginfo(f"Control command joint positions: {joint_positions_list}")
                # control-CMD
                self.control_robot_arm_cmd(joint_positions_list)
            # 等待Hz
            rate.sleep()

def map_update_thread(cumotion_action_server):
    """线程中运行的地图更新逻辑"""
    global IF_PROCESS_VOXEL_MAP_SATUS
    global origin_esdf_grid
    global tsdf_response

    origin_voxel_dims = [2.0, 2.0, 2.0]
    while not rospy.is_shutdown():
        if not IF_PROCESS_VOXEL_MAP_SATUS:
            # This is half of x,y and z dims
            aabb_min = Point()
            aabb_min.x = -1 * origin_voxel_dims[0] / 2
            aabb_min.y = -1 * origin_voxel_dims[1] / 2
            aabb_min.z = -1 * origin_voxel_dims[2] / 2
            # This is a voxel size.
            voxel_dims = Vector3()
            voxel_dims.x = origin_voxel_dims[0]
            voxel_dims.y = origin_voxel_dims[1]
            voxel_dims.z = origin_voxel_dims[2]
            # 发送服务请求
            esdf_response = cumotion_action_server.send_request(aabb_min, voxel_dims)
            
            if esdf_response  is not None:
                # rospy.loginfo("ESDF map updated successfully")
                tsdf_response = esdf_response 
            else:
                rospy.logwarn("Failed to update ESDF map")
        else:
            rospy.loginfo("Voxel map processing status is active, skipping update")

        rospy.sleep(1.0)  # 控制线程更新频率，避免过于频繁调用

if __name__ == '__main__':
    curoBoServer = CumotionActionServer()
    planning_thread = threading.Thread(target=map_update_thread, args=(curoBoServer,))
    planning_thread.start()
    
    try:
        curoBoServer.run()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
        rospy.shutdown()