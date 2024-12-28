from os import path
import time
import rospy

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
from curobo.wrap.reacher.motion_gen import MotionGen
from curobo.wrap.reacher.motion_gen import MotionGenConfig
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
from curobo.wrap.reacher.motion_gen import MotionGenStatus
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import RobotTrajectory
from nvblox_msgs.srv import EsdfAndGradients, EsdfAndGradientsRequest, EsdfAndGradientsResponse

import numpy as np
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
import torch
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import MultiDOFJointTrajectory
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from std_msgs.msg import Bool
from curobo_trajectory_server.srv import cuRoMoveGroup, cuRoMoveGroupResponse, cuRoMoveGroupRequest
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

def DEBUG_IK_MODEL(motion_gen):
    goal_pose = Pose.from_list([0.12595975554344743,  # x
                                0.2546565360634217,   # y
                                0.03443182480649596,  # z
                                0.9077303036242385,   # qw
                                -0.07733529679630219, # qx
                                -0.4109196668427433,  # qy
                                -0.03449601648776949  # qz
                                ]) 
    retract_config = torch.tensor([[0.34, 0.87, 0.17, -1.22, -0.34, -0.52, 0.00]]).cuda()
    start_state = CuJointState.from_position(
        retract_config, 
        joint_names=[
            "zarm_l1_joint",
            "zarm_l2_joint",
            "zarm_l3_joint",
            "zarm_l4_joint",
            "zarm_l5_joint",
            "zarm_l6_joint",
            "zarm_l7_joint"
        ],
    )
    result = motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=1))
    traj = result.get_interpolated_plan()  # result.interpolation_dt has the dt between timesteps
    print("Trajectory Generated: ", result.success)
    print(" ---------------- ")
    print("Plan Result: ", result)
    print(" ---------------- ")
    print("Status: ", result.status)
    print(" ---------------- ")
    print("Solve Time: ", result.solve_time)
    print(" ---------------- ")

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

        # Publishers
        self.voxel_pub = rospy.Publisher('/curobo/voxels', Marker, queue_size=10)
        self.go_joint_traj_pub = rospy.Publisher('/cumotion_go_joint_trajectory_topic', JointTrajectory, queue_size=10)
        self.back_joint_traj_pub = rospy.Publisher('/cumotion_back_joint_trajectory_topic', JointTrajectory, queue_size=10)
        self.global_if_process_flag_pub = rospy.Publisher('/global_if_process_action_flag_topic', Bool, queue_size=10)
        self.flag_publisher_ = rospy.Publisher('/global_success_action_flag_topic', Bool, queue_size=10)

        # # Timers for periodic publishing
        rospy.Timer(rospy.Duration(0.2), self.timer_callback)
        self.arm_status_sub = rospy.Subscriber('/robot_arm_plan_status', Bool, self.arm_status_callback)

        # Server服务端
        self._action_server = rospy.Service('/cumotion/move_group', cuRoMoveGroup, self.execute_callback)

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
        self.__tensor_args = self.motion_gen.tensor_args
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

        # Initialize motion generation config
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_dict,
            world_file,
            self.tensor_args,
            interpolation_dt=interpolation_dt,
            collision_cache={
                'obb': collision_cache_cuboid,
                'mesh': collision_cache_mesh,
            },
            collision_checker_type=CollisionCheckerType.VOXEL,
            ee_link_name=self.tool_frame,
        )

        # Initialize the motion generator
        motion_gen = MotionGen(motion_gen_config)
        self.motion_gen = motion_gen
        self.__robot_base_frame = motion_gen.kinematics.base_link
        rospy.loginfo(f"Robot base frame: {self.__robot_base_frame}")
        motion_gen.warmup(enable_graph=True)

        self.__world_collision = motion_gen.world_coll_checker
        if not self.add_ground_plane:
            motion_gen.clear_world_cache()

        rospy.loginfo('cuMotion is ready for planning queries!')
        
        # DEBUG MODE
        if DEBUG_MODE_FLAG:
            DEBUG_IK_MODEL(motion_gen)
            