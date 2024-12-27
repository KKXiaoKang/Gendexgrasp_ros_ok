import sys
import os
import time
import math

import numpy as np
import pydrake

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    PiecewisePolynomial,
    FixedOffsetFrame,
    RigidTransform,
    InverseKinematics,
    Solve,
    RotationMatrix,
    RollPitchYaw,
    SnoptSolver,
    InitializeAutoDiff,
    JacobianWrtVariable,
    ExtractGradient,
    ExtractValue,
    ComPositionConstraint,
    CentroidalMomentumConstraint,
)
from pydrake.all import StartMeshcat, AddMultibodyPlantSceneGraph, MeshcatVisualizer

# from scipy.spatial.transform import Rotation as R
import numpy as np
from functools import partial

current_dir = os.path.dirname(os.path.abspath(__file__))
tools_dir = os.path.abspath(os.path.join(current_dir, '..', 'tools'))
print(tools_dir)
sys.path.append(tools_dir)
from utils import get_package_path, ArmIdx, IkTypeIdx
from drake_trans import quaternion_to_RPY, rpy_to_matrix


class TorsoIK:
    def __init__(
        self,
        plant,
        frame_name_list,
        constraint_tol=1.0e-8,
        solver_tol=1.0e-6,
        iterations_limit=1000,
        ctrl_arm_idx=ArmIdx.LEFT,
        as_mc_ik=True,
        # elbow_frame_name_list=None,
    ):
        self.__plant = plant
        self.__plant_context = self.__plant.CreateDefaultContext()
        self.__constraint_tol = constraint_tol
        self.__iterations_limit = iterations_limit
        self.__solver_tol = solver_tol
        self.__frames = [self.__plant.GetFrameByName(name) for name in frame_name_list]
        self.__ctrl_arm_idx = ctrl_arm_idx
        self.__as_mc_ik = as_mc_ik
        # self.__elbow_frames = [self.__plant.GetFrameByName(name) for name in elbow_frame_name_list] if elbow_frame_name_list is not None else None

    def set_as_mc_ik(self, as_mc_ik):
        print(f"[TorsoIK] set_as_mc_ik: {as_mc_ik}")
        self.__as_mc_ik = as_mc_ik

    def solve(self, pose_list, q0=[], left_shoulder_rpy=None, right_shoulder_rpy=None, last_solution=None):
        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=True)
        snopt = SnoptSolver().solver_id()
        self.__IK.prog().SetSolverOption(
            snopt, "Major Optimality Tolerance", self.__solver_tol
        )
        self.__IK.prog().SetSolverOption(
            snopt, "Major Iterations Limit", self.__iterations_limit
        )
        for i, frame in enumerate(self.__frames):
            if not pose_list[i][0] is None:
                # print(f"ori: {i}")
                self.__IK.AddOrientationConstraint(
                    self.__plant.world_frame(),
                    RotationMatrix(RollPitchYaw(pose_list[i][0])),
                    frame,
                    RotationMatrix(RollPitchYaw(0, 0, 0)),
                    self.__constraint_tol,
                )
            if not pose_list[i][1] is None:
                # print(f"pos: {i}")
                if self.__as_mc_ik and i != 0:
                    cost_matrix = 10 * np.eye(3)
                    if i > 2: # 3,4 -> elbow
                        cost_matrix = 1 * np.eye(3)
                    self.__IK.AddPositionCost(
                        frameA=self.__plant.world_frame(),
                        p_AP=pose_list[i][1],
                        frameB=frame,
                        p_BQ=np.zeros(3),
                        C=cost_matrix,
                    )
                if(not self.__as_mc_ik and i<=2) or (self.__as_mc_ik and i==0): # torso
                    self.__IK.AddPositionConstraint(frameB=frame,
                                                    p_BQ=np.zeros(3),
                                                    frameA=self.__plant.world_frame(),
                                                    p_AQ_lower=np.array(pose_list[i][1])-self.__constraint_tol,
                                                    p_AQ_upper=np.array(pose_list[i][1])+self.__constraint_tol)

        q_normal = np.array([-0.3928, 0.2608, 0.0143, -0.3927, 0.2970, -0.8364, 0.0589])
        q_normal_right = np.array(
            [0.3928, 0.2608, 0.0143, -0.3927, 0.2970, -0.8364, 0.0589]
        )
        W = np.diag([0.1])
        bias = 5.0 * np.pi / 180.0
        # if left_shoulder_rpy is not None: # left shoulder
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[left_shoulder_rpy[1]]]), np.array([[self.__IK.q()[7+0]]])) # joint-1
        #     # self.__IK.prog().AddLinearConstraint(self.__IK.q()[7+0], lb=left_shoulder_rpy[1]-bias, ub=left_shoulder_rpy[1]+bias) # joint-1
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[left_shoulder_rpy[0]]]), np.array([[self.__IK.q()[7+1]]])) # joint-2
        #     # self.__IK.prog().AddLinearConstraint(self.__IK.q()[7+1], lb=left_shoulder_rpy[0]-bias, ub=left_shoulder_rpy[0]+bias) # joint-2
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[left_shoulder_rpy[2]]]), np.array([[self.__IK.q()[7+2]]])) # joint-3
        #     self.__IK.prog().AddLinearConstraint(self.__IK.q()[7+2], lb=left_shoulder_rpy[2]-bias, ub=left_shoulder_rpy[2]+bias) # joint-3
        # if right_shoulder_rpy is not None:
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[right_shoulder_rpy[1]]]), np.array([[self.__IK.q()[14+0]]]))
        #     # self.__IK.prog().AddLinearConstraint(self.__IK.q()[14+0], lb=right_shoulder_rpy[1]-bias, ub=right_shoulder_rpy[1]+bias)
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[right_shoulder_rpy[0]]]), np.array([[self.__IK.q()[14+1]]]))
        #     # self.__IK.prog().AddLinearConstraint(self.__IK.q()[14+1], lb=right_shoulder_rpy[0]-bias, ub=right_shoulder_rpy[0]+bias)
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[right_shoulder_rpy[2]]]), np.array([[self.__IK.q()[14+2]]]))
        #     self.__IK.prog().AddLinearConstraint(self.__IK.q()[14+2], lb=right_shoulder_rpy[2]-bias, ub=right_shoulder_rpy[2]+bias)

        if last_solution is not None:
            W_vec = np.array([0.1] * self.__plant.num_positions())
            # W_vec[7:10] *= 10 # shoulder joints
            # W_vec[14:17] *= 10
            W_prev_solution = np.diag(W_vec)
            self.__IK.prog().AddQuadraticErrorCost(W_prev_solution, last_solution, self.__IK.q())

        # if self.__ctrl_arm_idx == ArmIdx.LEFT or self.__ctrl_arm_idx == ArmIdx.BOTH:
        #     self.__IK.prog().AddQuadraticErrorCost(W, q_normal, self.__IK.q()[7:14])
        # if self.__ctrl_arm_idx == ArmIdx.RIGHT or self.__ctrl_arm_idx == ArmIdx.BOTH:
        #     self.__IK.prog().AddQuadraticErrorCost(W, q_normal_right, self.__IK.q()[-7:])
        # self.__IK.prog().AddQuadraticErrorCost(np.diag([0.01]*14), q0, self.__IK.q())
        result = Solve(self.__IK.prog(), q0)
        if result.is_success():
            return [True, result.GetSolution()]
        return [False, []]


class ArmIk:
    """
    eef_z_bias: float, default 0.0(-> arm length=0.58m), the z-axis offset of the end-effector frame
        i.e. if u want to make length to 0.6m, set eef_z_bias to 0.02(0.58+0.02=0.6)
    """

    def __init__(
        self,
        model_file,
        end_frames_name,
        meshcat,
        constraint_tol=1e-4,
        solver_tol=1.0e-4,
        iterations_limit=1000,
        eef_z_bias=0.0,
        shoulder_frame_names=["l_arm_pitch", "r_arm_pitch"],
        ctrl_arm_idx=ArmIdx.LEFT,
        as_mc_ik=True,
    ):
        builder = DiagramBuilder()
        self.__plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
        parser = Parser(self.__plant)
        robot = parser.AddModelFromFile(model_file)
        # custom added
        eef_frame_name_list = ["frame_eef_left", "frame_eef_right"]
        self.shoulder_frame_names = shoulder_frame_names
        eef_frame_left = self.__plant.GetFrameByName(end_frames_name[1])
        eef_frame_right = self.__plant.GetFrameByName(end_frames_name[2])
        p = np.array([0, 0, -(0.2254 + eef_z_bias)])
        self.frame_eef_left_custom = self.__plant.AddFrame(
            FixedOffsetFrame(eef_frame_name_list[0], eef_frame_left, RigidTransform(p))
        )
        self.frame_eef_right_custom = self.__plant.AddFrame(
            FixedOffsetFrame(eef_frame_name_list[1], eef_frame_right, RigidTransform(p))
        )
        # 假设躯干坐标系的名称为 "torso_frame"
        torso_frame = self.__plant.GetFrameByName("torso")

        # 焊接躯干坐标系到世界坐标系
        self.__plant.WeldFrames(self.__plant.world_frame(), torso_frame)
        self.__plant.Finalize()
        self.__meshcat = meshcat

        if self.__meshcat is not None:
            self.__visualizer = MeshcatVisualizer.AddToBuilder(
                builder, scene_graph, meshcat
            )
        self.__diagram = builder.Build()
        self.__diagram_context = self.__diagram.CreateDefaultContext()

        self.__plant_context = self.__plant.GetMyContextFromRoot(self.__diagram_context)
        self.__q0 = self.__plant.GetPositions(self.__plant_context)
        self.__v0 = self.__plant.GetVelocities(self.__plant_context)
        self.__r0 = self.__plant.CalcCenterOfMassPositionInWorld(self.__plant_context)

        self.__base_link_name = end_frames_name[0]
        self.__left_eef_name = end_frames_name[1]
        self.__right_eef_name = end_frames_name[2]
        # end_frames_name[1] = self.__left_eef_name
        # end_frames_name[2] = self.__right_eef_name

        self.__IK = TorsoIK(
            self.__plant,
            end_frames_name,
            constraint_tol,
            solver_tol,
            iterations_limit=iterations_limit,
            ctrl_arm_idx=ctrl_arm_idx,
            as_mc_ik=as_mc_ik,
        )
        # 添加记录上一次求得的逆解的属性，初始化为全零向量
        self.__last_solution = np.zeros(self.__plant.num_positions())
        print("initializing arm ik")

    def set_as_mc_ik(self, as_mc_ik):
        self.__IK.set_as_mc_ik(as_mc_ik)
        # print(f"[ArmIk] set as_mc_ik to {as_mc_ik}")

    def type(self):
        return IkTypeIdx.TorsoIK

    def q0(self):
        return self.__q0

    def init_state(self, torso_yaw_deg, torso_height):
        self.__torso_yaw_rad = math.radians(torso_yaw_deg)
        self.__torso_height = torso_height
        self.__q0[6] = torso_height

    def computeIK(self, q0, l_hand_pose, r_hand_pose, l_hand_RPY=None, r_hand_RPY=None, l_elbow_pos=None, r_elbow_pos=None, left_shoulder_rpy=None, right_shoulder_rpy=None):
        torsoR = [0.0, self.__torso_yaw_rad, 0.0]
        r = [0.0, 0.0, self.__torso_height]
        # print(f"l_elbow_pos: {l_elbow_pos}, r_elbow_pos: {r_elbow_pos}")
        pose_list = [
            [torsoR, r],
            [l_hand_RPY, l_hand_pose],
            [r_hand_RPY, r_hand_pose],
            [None, l_elbow_pos],
            [None, r_elbow_pos],
        ]
        # print("IK com!!!!!!!!!!!!!!!!!!!!")
        is_success, q = self.__IK.solve(pose_list, q0=q0, left_shoulder_rpy=left_shoulder_rpy, right_shoulder_rpy=right_shoulder_rpy, last_solution=self.__last_solution)
        if not is_success:
            # print(f"pose: {pose_list[0][0]}, {pose_list[0][1]}")
            # print(f"lhand: {pose_list[1][0]}, {pose_list[1][1]}")
            # print(f"rhand: {pose_list[2][0]}, {pose_list[2][1]}")
            # raise RuntimeError("Failed to IK0!")
            return None
        else:
            self.__last_solution = q
            return q

    def start_recording(self):
        if self.__meshcat is None:
            return
        self.__visualizer.StartRecording()

    def stop_andpublish_recording(self):
        if self.__meshcat is None:
            return
        self.__visualizer.StopRecording()
        self.__visualizer.PublishRecording()

    def visualize_animation(self, q_list, start_time=0.0, duration=1.1):
        t_sol = np.arange(start_time, start_time + duration, 1)
        q_sol = np.array(q_list).T
        # print(f"q_sol: {q_sol.shape}, t_sol: {t_sol.shape}")
        q_pp = PiecewisePolynomial.FirstOrderHold(t_sol, q_sol)
        t0 = t_sol[0]
        tf = t_sol[-1]
        t = t0
        # self.__visualizer.StartRecording()
        while t < tf:
            q = q_pp.value(t)
            self.__plant.SetPositions(self.__plant_context, q)
            self.__diagram_context.SetTime(t)
            self.__diagram.ForcedPublish(self.__diagram_context)
            t += 0.01
        # self.__visualizer.StopRecording()
        # self.__visualizer.PublishRecording()
        # while True:
        time.sleep(0.1)

    def left_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context,
            JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__left_eef_name),
            [0, 0, 0],
            self.__plant.world_frame(),
            self.__plant.world_frame(),
        )
        return J_hand_in_world[:, 6:13]

    def right_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context,
            JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__right_eef_name),
            [0, 0, 0],
            self.__plant.world_frame(),
            self.__plant.world_frame(),
        )
        return J_hand_in_world[:, -7:]

    def left_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        l_hand_in_base = self.__plant.GetFrameByName(self.__left_eef_name).CalcPose(
            self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name)
        )
        # print("left hand position in base:", l_hand_in_base.translation())
        return (l_hand_in_base.translation(), l_hand_in_base.rotation().ToRollPitchYaw().vector())

    def right_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        r_hand_in_base = self.__plant.GetFrameByName(self.__right_eef_name).CalcPose(
            self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name)
        )
        # print("right hand position in base:", r_hand_in_base.translation())
        return (r_hand_in_base.translation(), r_hand_in_base.rotation().ToRollPitchYaw().vector())

    def get_arm_length(self):
        shoulder_frame_left = self.__plant.GetFrameByName(self.shoulder_frame_names[0])
        X_shoulder_left = shoulder_frame_left.CalcPoseInWorld(self.__plant_context)
        X_eef_left = self.__plant.GetFrameByName(self.__left_eef_name).CalcPoseInWorld(
            self.__plant_context
        )
        dis = X_shoulder_left.translation() - X_eef_left.translation()
        # print(f"left dis: {dis}")
        length_left = np.linalg.norm(dis)

        shoulder_frame_right = self.__plant.GetFrameByName(self.shoulder_frame_names[1])
        X_shoulder_right = shoulder_frame_right.CalcPoseInWorld(self.__plant_context)
        X_eef_right = self.__plant.GetFrameByName(
            self.__right_eef_name
        ).CalcPoseInWorld(self.__plant_context)
        dis = X_shoulder_right.translation() - X_eef_right.translation()
        # print(f"right dis: {dis}")
        length_right = np.linalg.norm(dis)

        return length_left, length_right

    def get_two_frame_dis(self, frame_a_name, frame_b_name):
        frame_a = self.__plant.GetFrameByName(frame_a_name)
        X_a = frame_a.CalcPoseInWorld(self.__plant_context)
        X_b = self.__plant.GetFrameByName(frame_b_name).CalcPoseInWorld(
            self.__plant_context
        )
        dis = X_a.translation() - X_b.translation()
        # print(f"left dis: {dis}")
        length = np.linalg.norm(dis)

        return length

if __name__ == "__main__":
    # test = np.load("./rosbag_joint.npy")  # drake RPY版本
    # test = np.load("./rosbag_s.npy") # 四元数版本（x,y,z,w）
    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    meshcat = StartMeshcat()
    version = 4
    eef_z_bias = -0.0
    kuavo_pkg_path = get_package_path("motion_capture_ik")
    print(f"kuavo_pkg_path: {kuavo_pkg_path}")

    model_file = kuavo_pkg_path + "/models/biped_gen4.0/urdf/biped_v3_arm.urdf"
    end_frames_name = ["torso", "l_hand_roll", "r_hand_roll", "l_forearm_pitch", "r_forearm_pitch"]
    if version == 3:
        model_file = kuavo_pkg_path + "/models/biped_gen3.4/urdf/biped_v3_arm.urdf"
        eef_z_bias = -0.098
        end_frames_name = ["torso", "l_hand_pitch", "r_hand_pitch"]

    arm_ik = ArmIk(model_file, end_frames_name, meshcat, 1e-3, 1e-3, 1000, eef_z_bias=eef_z_bias)
    # end_frames_name = [base_link_name, "l_hand_eef", "r_hand_eef"]

    # arm_ik = ArmIk(model_file, end_frames_name, meshcat, 1e-3, 1e-3, 1000, -0.098)
    torso_yaw_deg = 0.0
    torso_height = 0.0
    arm_ik.init_state(torso_yaw_deg, torso_height)
    arm_length_l, arm_length_r = arm_ik.get_arm_length()
    print(f"Arm length left: {arm_length_l:.3f} m, right: {arm_length_r:.3f} m")
    q0 = arm_ik.q0()
    q_list = [q0]
    last_q = q0
    arm_ik.start_recording()
    t = 0.0
    l_pose = arm_ik.left_hand_pose(q0)
    print(
        f"left_hand_pose: {l_pose[0]}, {l_pose[1]}"
    )

    for i in range(5):
        l_hand_pose = np.array([0.15, 0.21, -0.121])
        l_hand_RPY = np.array([0.4092, -0.3443, 0.5193])
        l_elbow_pos = np.array([0.15, 0.21, 0.121])
        r_elbow_pos = np.array([0.15, -0.21, 0.121])
        l_hand_RPY = None

        r_hand_RPY = None
        r_hand_pose = None  # [x, y, z]
        time_0 = time.time()
        q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY, l_elbow_pos, r_elbow_pos)
        time_cost = time.time() - time_0
        print(f"i: {i}, time cost: {1e3*time_cost:.3f} ms")
        if q is not None:
            q_list.append(q)
            q0 = q
            # animate trajectory
            arm_ik.visualize_animation([last_q, q], t)
            last_q = q
            t = t + 1.0
        else:
            print(f"Failed to IK in step {i}!")
        time.sleep(0.001)
    pos = arm_ik.left_hand_pose(last_q)
    print(
        f"end left_hand_pose: {pos[0]}, {pos[1]}"
    )
    arm_ik.stop_andpublish_recording()
    print("Program end, Press Ctrl + C to exit.")
    while True:
        time.sleep(0.01)
