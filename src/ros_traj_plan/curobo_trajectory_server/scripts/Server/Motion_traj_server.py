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

    def update_voxel_grid(self):
        global global_robot_arm_plan_status
        global global_esdf_grid
        global origin_esdf_grid
        global IF_PROCESS_VOXEL_MAP_SATUS
        global tsdf_response

        # 获取ESDF地图并更新
        origin_esdf_grid = self.get_esdf_voxel_grid(tsdf_response)

        IF_PROCESS_VOXEL_MAP_SATUS = True  # 正在进行体素图处理
        rospy.loginfo('Calling ESDF service')

        # 判断机器人手臂的状态
        if global_robot_arm_plan_status:
            rospy.loginfo('Updated go状态 ESDF grid')
            esdf_grid = origin_esdf_grid  # 使用当前的origin实时更新的地图
        else:
            rospy.loginfo('Updated back状态 ESDF grid')
            esdf_grid = global_esdf_grid  # 使用上一次的全局地图

        # ESDF地图描述
        if torch.max(esdf_grid.feature_tensor) <= (-1000.0 + 0.5 * self.__voxel_size + 1e-5):
            rospy.logerr('ESDF data is empty, try again after few seconds.')
            return False

        # 更新体素地图，假设__world_collision是处理碰撞检测的类
        self.__world_collision.update_voxel_data(esdf_grid)
        rospy.loginfo('Updated ESDF grid')

        # 体素地图更新完毕
        IF_PROCESS_VOXEL_MAP_SATUS = False
        return True
    
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

    def get_esdf_voxel_grid(self, esdf_data):
        esdf_array = esdf_data.esdf_and_gradients # 提取esdf网格和梯度信息
        array_shape = [ # 从 esdf_array 的布局信息中读取体素网格的三维形状 [x, y, z]，表示体素在每个维度上的数量
            esdf_array.layout.dim[0].size,
            esdf_array.layout.dim[1].size,
            esdf_array.layout.dim[2].size,
        ]

        # TODO:添加
        array_data = np.array(esdf_array.data) # 将 esdf_array.data 转换为 NumPy 数组 array_data，方便后续数据处理

        # 将 array_data 转换到目标设备上（如 GPU），确保数据存储在合适的计算设备上加速处理
        array_data = self.__tensor_args.to_device(array_data) 

        # Array data is reshaped to x y z channels
        array_data = array_data.view(array_shape[0], array_shape[1], array_shape[2]).contiguous() # 调整 array_data 的形状，使其匹配体素网格的三维形状 [x, y, z]，并保持内存连续性以优化性

        # Array is squeezed to 1 dimension
        array_data = array_data.reshape(-1, 1) # 将 array_data 压缩为二维数组（[-1, 1]），即每一行表示一个体素的值，用于后续处理

        # nvblox uses negative distance inside obstacles, cuRobo needs the opposite:
        array_data = -1 * array_data # 将距离值取反，将内部障碍物的负距离值转化为正距离值，以匹配 cuRobo 的要求

        # nvblox assigns a value of -1000.0 for unobserved voxels, making
        array_data[array_data >= 1000.0] = -1000.0  # 将未观测到的体素（值为 1000.0 或更大）重设为 -1000.0，以表示未观测的区域符合 cuRobo 规范

        # nvblox distance are at origin of each voxel, cuRobo's esdf needs it to be at faces
        array_data = array_data + 0.5 * self.__voxel_size # 将体素中心的距离值转换为体素表面距离，使距离值表示体素的表面位置

        """
            使用生成的数据创建 CuVoxelGrid 对象 esdf_grid
            设置网格的名称、维度、位姿、体素大小、数据类型和特征数据。位置 pose 包括位置坐标 [x, y, z] 和姿态的四元数表示 [qw, qx, qy, qz]
        """
        esdf_grid = CuVoxelGrid(
            name='world_voxel',
            dims=self.__voxel_dims,
            pose=[
                self.grid_position[0],
                self.grid_position[1],
                self.grid_position[2],
                1,
                0.0,
                0.0,
                0,
            ],  # x, y, z, qw, qx, qy, qz
            voxel_size=self.__voxel_size,
            feature_dtype=torch.float32,
            feature_tensor=array_data,
        )

        return esdf_grid # 返回生成的 CuVoxelGrid 对象 esdf_grid，用于描述 cuRobo 的碰撞检测环境

    def get_cumotion_collision_object(self, mv_object: CollisionObject):
        objs = []
        pose = mv_object.pose

        world_pose = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]
        world_pose = Pose.from_list(world_pose)
        supported_objects = True
        if len(mv_object.primitives) > 0:
            for k in range(len(mv_object.primitives)):
                pose = mv_object.primitive_poses[k]
                primitive_pose = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                ]
                object_pose = world_pose.multiply(Pose.from_list(primitive_pose)).tolist()

                if mv_object.primitives[k].type == SolidPrimitive.BOX:
                    # cuboid:
                    dims = mv_object.primitives[k].dimensions
                    obj = Cuboid(
                        name=str(mv_object.id) + '_' + str(k) + '_cuboid',
                        pose=object_pose,
                        dims=dims,
                    )
                    objs.append(obj)
                elif mv_object.primitives[k].type == SolidPrimitive.SPHERE:
                    # sphere:
                    radius = mv_object.primitives[k].dimensions[
                        mv_object.primitives[k].SPHERE_RADIUS
                    ]
                    obj = Sphere(
                        name=str(mv_object.id) + '_' + str(k) + '_sphere',
                        pose=object_pose,
                        radius=radius,
                    )
                    objs.append(obj)
                elif mv_object.primitives[k].type == SolidPrimitive.CYLINDER:
                    # cylinder:
                    cyl_height = mv_object.primitives[k].dimensions[
                        mv_object.primitives[k].CYLINDER_HEIGHT
                    ]
                    cyl_radius = mv_object.primitives[k].dimensions[
                        mv_object.primitives[k].CYLINDER_RADIUS
                    ]
                    obj = Cylinder(
                        name=str(mv_object.id) + '_' + str(k) + '_cylinder',
                        pose=object_pose,
                        height=cyl_height,
                        radius=cyl_radius,
                    )
                    objs.append(obj)
                elif mv_object.primitives[k].type == SolidPrimitive.CONE:
                    rospy.logerr('Cone primitive is not supported')
                    supported_objects = False
                else:
                    rospy.logerr('Unknown primitive type')
                    supported_objects = False
        if len(mv_object.meshes) > 0:
            for k in range(len(mv_object.meshes)):
                pose = mv_object.mesh_poses[k]
                mesh_pose = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                ]
                object_pose = world_pose.multiply(Pose.from_list(mesh_pose)).tolist()
                verts = mv_object.meshes[k].vertices
                verts = [[v.x, v.y, v.z] for v in verts]
                tris = [
                    [v.vertex_indices[0], v.vertex_indices[1], v.vertex_indices[2]]
                    for v in mv_object.meshes[k].triangles
                ]

                obj = Mesh(
                    name=str(mv_object.id) + '_' + str(len(objs)) + '_mesh',
                    pose=object_pose,
                    vertices=verts,
                    faces=tris,
                )
                objs.append(obj)
        return objs, supported_objects

    def get_joint_trajectory(self, js: CuJointState, dt: float):
        traj = RobotTrajectory()
        cmd_traj = JointTrajectory()
        q_traj = js.position.cpu().view(-1, js.position.shape[-1]).numpy()
        vel = js.velocity.cpu().view(-1, js.position.shape[-1]).numpy()
        acc = js.acceleration.view(-1, js.position.shape[-1]).cpu().numpy()
        for i in range(len(q_traj)):
            traj_pt = JointTrajectoryPoint()
            traj_pt.positions = q_traj[i].tolist()
            if js is not None and i < len(vel):
                traj_pt.velocities = vel[i].tolist()
            if js is not None and i < len(acc):
                traj_pt.accelerations = acc[i].tolist()
            time_d = rospy.Duration(i * dt)
            traj_pt.time_from_start = time_d
            cmd_traj.points.append(traj_pt)
        cmd_traj.joint_names = js.joint_names
        cmd_traj.header.stamp = rospy.Time.now()
        traj.joint_trajectory = cmd_traj
        return traj

    def update_world_objects(self, moveit_objects):
        rospy.loginfo("Calling update_world_objects function.")
        world_update_status = True
        if len(moveit_objects) > 0:
            cuboid_list = []
            sphere_list = []
            cylinder_list = []
            mesh_list = []
            for i, obj in enumerate(moveit_objects):
                cumotion_objects, world_update_status = self.get_cumotion_collision_object(obj)
                for cumotion_object in cumotion_objects:
                    if isinstance(cumotion_object, Cuboid):
                        cuboid_list.append(cumotion_object)
                    elif isinstance(cumotion_object, Cylinder):
                        cylinder_list.append(cumotion_object)
                    elif isinstance(cumotion_object, Sphere):
                        sphere_list.append(cumotion_object)
                    elif isinstance(cumotion_object, Mesh):
                        mesh_list.append(cumotion_object)

            world_model = WorldConfig(
                cuboid=cuboid_list,
                cylinder=cylinder_list,
                sphere=sphere_list,
                mesh=mesh_list,
            ).get_collision_check_world()
            rospy.loginfo("Calling motion_gen.update_world(world_model)  function.") # 开启esdf服务查询时不会调取该服务
            self.motion_gen.update_world(world_model)
        if self.__read_esdf_grid: # 从esdf当中读取当前世界碰撞距离
            rospy.loginfo("Calling 更新体素世界 world_update_status = self.update_voxel_grid() function.")
            world_update_status = self.update_voxel_grid() # 根据全局状态是go的状态还是back的状态 | 判断是否拿上一张图进行避障返回
        if self.publish_curobo_world_as_voxels: # 是否发布体素世界
            rospy.loginfo("Calling 发布体素世界 world_update_status = self.publish_voxels(xyzr_tensor) function.")
            voxels = self.__world_collision.get_esdf_in_bounding_box(
                Cuboid(
                    name='test',
                    pose=[0.0, 0.0, 0.0, 1, 0, 0, 0],  # x, y, z, qw, qx, qy, qz
                    dims=self.__voxel_dims,
                ),
                voxel_size=self.publish_voxel_size,
            )
            xyzr_tensor = voxels.xyzr_tensor.clone()
            xyzr_tensor[..., 3] = voxels.feature_tensor
            self.publish_voxels(xyzr_tensor)
        return world_update_status

    def execute_callback(self, goal_handle):
        global global_if_process_action_flag # cumotion是否正在进行处理
        global TRAJ_COUNTER  #  轨迹发布状态计数器
        global global_robot_arm_plan_status # 机器人手臂的状态是go状态 还是 back状态

        global JOINT_TRAJ_GO 
        global JOINT_TRAJ_BACK 

        global global_esdf_grid  # 全局地图（成功的Go地图 | 作为back状态的参考）
        global origin_esdf_grid  # 实时更新的体素地图

        # 正在处理中
        global_if_process_action_flag = False 
        # 发布失败action调用失败
        not_process_msg = Bool()
        not_process_msg.data = False
        self.flag_publisher_.publish(not_process_msg)

        rospy.loginfo('Executing goal...')
        # check moveit scaling factors:
        min_scaling_factor = min(goal_handle.request.max_velocity_scaling_factor,
                                 goal_handle.request.max_acceleration_scaling_factor)
        time_dilation_factor = min(1.0, min_scaling_factor)

        if time_dilation_factor <= 0.0 or self.override_moveit_scaling_factors:
            time_dilation_factor = self.time_dilation_factor
        rospy.loginfo('Planning with time_dilation_factor: ' + str(time_dilation_factor))
        plan_req = goal_handle.request
        scene = goal_handle.planning_options.planning_scene_diff
        # goal_handle.succeed()
        world_objects = scene.world.collision_objects
        world_update_status = self.update_world_objects(world_objects) # 更新世界描述 | 根据规划的状态 go 还是 back 状态 进行地图获取调整 | 
        
        # 创建返回结果
        # result = MoveGroup.Result() 
        result = cuRoMoveGroupResponse() 
        if not world_update_status:
            result.error_code.val = MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE
            rospy.logerr('World update failed.')
            return result
        start_state = None
        if len(plan_req.start_state.joint_state.position) > 0:
            start_state = self.motion_gen.get_active_js(
                CuJointState.from_position(
                    position=self.tensor_args.to_device(
                        plan_req.start_state.joint_state.position
                    ).unsqueeze(0),
                    joint_names=plan_req.start_state.joint_state.name,
                )
            )
        else:
            rospy.loginfo(
                'PlanRequest start state was empty, reading current joint state'
            )
        if start_state is None or plan_req.start_state.is_diff:
            if self.__js_buffer is None:
                rospy.logerr(
                    'joint_state was not received from ' + self.__joint_states_topic
                )
                return result

            # read joint state:
            state = CuJointState.from_position(
                position=self.tensor_args.to_device(self.__js_buffer['position']).unsqueeze(0),
                joint_names=self.__js_buffer['joint_names'],
            )
            state.velocity = self.tensor_args.to_device(self.__js_buffer['velocity']).unsqueeze(0)
            current_joint_state = self.motion_gen.get_active_js(state)
            if start_state is not None and plan_req.start_state.is_diff:
                start_state.position += current_joint_state.position
                start_state.velocity += current_joint_state.velocity
            else:
                start_state = current_joint_state

        if len(plan_req.goal_constraints[0].joint_constraints) > 0:
            rospy.loginfo('Calculating goal pose from Joint target')
            goal_config = [
                plan_req.goal_constraints[0].joint_constraints[x].position
                for x in range(len(plan_req.goal_constraints[0].joint_constraints))
            ]
            goal_jnames = [
                plan_req.goal_constraints[0].joint_constraints[x].joint_name
                for x in range(len(plan_req.goal_constraints[0].joint_constraints))
            ]

            goal_state = self.motion_gen.get_active_js(
                CuJointState.from_position(
                    position=self.tensor_args.to_device(goal_config).view(1, -1),
                    joint_names=goal_jnames,
                )
            )
            goal_pose = self.motion_gen.compute_kinematics(goal_state).ee_pose.clone()
        elif (
            len(plan_req.goal_constraints[0].position_constraints) > 0
            and len(plan_req.goal_constraints[0].orientation_constraints) > 0
        ):
            rospy.loginfo('Using goal from Pose')

            position = (
                plan_req.goal_constraints[0]
                .position_constraints[0]
                .constraint_region.primitive_poses[0]
                .position
            )
            position = [position.x, position.y, position.z]
            orientation = plan_req.goal_constraints[0].orientation_constraints[0].orientation
            orientation = [orientation.w, orientation.x, orientation.y, orientation.z]
            pose_list = position + orientation
            goal_pose = Pose.from_list(pose_list, tensor_args=self.tensor_args)

            # Check if link names match:
            position_link_name = plan_req.goal_constraints[0].position_constraints[0].link_name
            orientation_link_name = (
                plan_req.goal_constraints[0].orientation_constraints[0].link_name
            )
            plan_link_name = self.motion_gen.kinematics.ee_link
            if position_link_name != orientation_link_name:
                rospy.logerr(
                    'Link name for Target Position "'
                    + position_link_name
                    + '" and Target Orientation "'
                    + orientation_link_name
                    + '" do not match'
                )
                result.error_code.val = MoveItErrorCodes.INVALID_LINK_NAME
                return result
            if position_link_name != plan_link_name:
                rospy.logerr(
                    'Link name for Target Pose "'
                    + position_link_name
                    + '" and Planning frame "'
                    + plan_link_name
                    + '" do not match, relaunch node with tool_frame = '
                    + position_link_name
                )
                result.error_code.val = MoveItErrorCodes.INVALID_LINK_NAME
                return result
        else:
            rospy.logerr('Goal constraints not supported')
        
        # 设置MotionGen开始进行规划 | start_state 开始状态读取joint_states | goal_pose 为全局target位置
        self.motion_gen.reset(reset_seed=False)
        motion_gen_result = self.motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(max_attempts=5, enable_graph_attempt=1,
                                time_dilation_factor=time_dilation_factor),
        )

        # result = MoveGroup.Result()
        result = cuRoMoveGroupResponse() 
        if motion_gen_result.success.item():
            result.error_code.val = MoveItErrorCodes.SUCCESS
            result.trajectory_start = plan_req.start_state
            traj = self.get_joint_trajectory(
                motion_gen_result.optimized_plan, motion_gen_result.optimized_dt.item()
            )
            result.planning_time = motion_gen_result.total_time
            result.planned_trajectory = traj

            # 在这里记录全局地图，因为作为origin_esdf_grid成功了
            global_esdf_grid = origin_esdf_grid

            # 记录发布的轨迹内容 | 发布轨迹的内容            
            if global_robot_arm_plan_status:
                JOINT_TRAJ_GO = traj.joint_trajectory
                self.go_joint_traj_pub.publish(JOINT_TRAJ_GO)
            else:
                JOINT_TRAJ_BACK = traj.joint_trajectory
                self.back_joint_traj_pub.publish(JOINT_TRAJ_BACK)

            # 发布成功action调用成功
            success_msg = Bool()
            success_msg.data = True
            self.flag_publisher_.publish(success_msg)

            # 打印
            rospy.loginfo('motion_gen_result.success.item')

        elif not motion_gen_result.valid_query:
            # 发布失败action调用失败
            fail_msg = Bool()
            fail_msg.data = False
            self.flag_publisher_.publish(fail_msg)

            rospy.logerr(
                f'Invalid planning query: {motion_gen_result.status}'
            )
            if motion_gen_result.status == MotionGenStatus.INVALID_START_STATE_JOINT_LIMITS:
                result.error_code.val = MoveItErrorCodes.START_STATE_INVALID
            if motion_gen_result.status in [
                    MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION,
                    MotionGenStatus.INVALID_START_STATE_SELF_COLLISION,
                    ]:

                result.error_code.val = MoveItErrorCodes.START_STATE_IN_COLLISION
        else:
            # 发布失败action调用失败
            Motion_msg = Bool()
            Motion_msg.data = False
            self.flag_publisher_.publish(Motion_msg)

            rospy.logerr(
                f'Motion planning failed wih status: {motion_gen_result.status}'
            )
            if motion_gen_result.status == MotionGenStatus.IK_FAIL:
                result.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION

        # 发布状态
        rospy.loginfo(
            'returned planning result (query, success, failure_status): '
            + str(self.__query_count)
            + ' '
            + str(motion_gen_result.success.item())
            + ' '
            + str(motion_gen_result.status)
        )

        # 发布全局运行代表excute一次callback完成
        global_if_process_action_flag = True 

        # 计算队列数
        self.__query_count += 1
        return result

    def publish_voxels(self, voxels):
        vox_size = self.publish_voxel_size

        # create marker:
        marker = Marker()
        marker.header.frame_id = self.__robot_base_frame
        marker.id = 0
        marker.type = 6  # cube list
        marker.ns = 'curobo_world'
        marker.action = 0
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(1000.0)  # 设置持续时间
        marker.frame_locked = False
        marker.scale.x = vox_size
        marker.scale.y = vox_size
        marker.scale.z = vox_size

        # get only voxels that are inside surfaces:

        voxels = voxels[voxels[:, 3] >= 0.0]
        vox = voxels.view(-1, 4).cpu().numpy()
        marker.points = []

        for i in range(min(len(vox), self.max_publish_voxels)):

            pt = Point()
            pt.x = float(vox[i, 0])
            pt.y = float(vox[i, 1])
            pt.z = float(vox[i, 2])
            color = ColorRGBA()
            d = vox[i, 3]

            rgba = [min(1.0, 1.0 - float(d)), 0.0, 0.0, 1.0]

            color.r = rgba[0]
            color.g = rgba[1]
            color.b = rgba[2]
            color.a = rgba[3]
            marker.colors.append(color)
            marker.points.append(pt)
        # publish voxels:
        marker.header.stamp = rospy.Time.now() 

        self.voxel_pub.publish(marker)

    def run(self):
        rospy.spin()


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