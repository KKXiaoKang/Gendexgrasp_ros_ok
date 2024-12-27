from drake_trans import *
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList, JoySticks
import rospy
from std_msgs.msg import Float32MultiArray


# to dev
hand_thumb_pub = rospy.Publisher('/hand_thumb', Float32MultiArray, queue_size=10)

# Array of bone names
bone_names = [
    "LeftArmUpper",
    "LeftArmLower",
    "RightArmUpper",
    "RightArmLower",
    "LeftHandPalm",
    "RightHandPalm",
    "LeftHandThumbMetacarpal",
    "LeftHandThumbProximal",
    "LeftHandThumbDistal",
    "LeftHandThumbTip",
    "LeftHandIndexTip",
    "LeftHandMiddleTip",
    "LeftHandRingTip",
    "LeftHandLittleTip",
    "RightHandThumbMetacarpal",
    "RightHandThumbProximal",
    "RightHandThumbDistal",
    "RightHandThumbTip",
    "RightHandIndexTip",
    "RightHandMiddleTip",
    "RightHandRingTip",
    "RightHandLittleTip",
    "Root",  # Added new bone
    "Chest"  # Added new bone
]


# Create bone_name_to_index dictionary
bone_name_to_index = {name: index for index, name in enumerate(bone_names)}

# List of bone IDs in order
fullBodyBoneIds_leju_arms = [bone_name_to_index[name] for name in bone_names]

# Create a reverse mapping from index to bone name
index_to_bone_name = {index: bone_names[index] for index in range(len(bone_names))}

bias_chest_to_base_link = [-0.1, 0, 0.3]

class Quest3ArmInfoTransformer:
    def __init__(self, model_path, vis_pub=True):
        self.model_path = model_path
        self.vis_pub = vis_pub
        self.left_finger_joints = None # down dom to 6 dof
        self.right_finger_joints = None
        self.left_hand_pose = None # pos + quaternion
        self.right_hand_pose = None
        self.left_elbow_pos = None
        self.right_elbow_pos = None
        self.pose_info_list = None # PoseInfoList message
        self.timestamp_ms = 0.0
        self.is_high_confidence = True
        self.is_hand_tracking = False
        self.ok_gesture_counts = 0
        self.shot_gesture_counts = 0
        self.is_runing = False        
        self.init_R_wC = rpy_to_matrix([np.pi/2, 0, np.pi/2]) # rotation matrix from world to chest
        self.init_R_wLS = None # rotation matrix from world to left shoulder
        self.init_R_wRS = None # rotation matrix from world to right shoulder
        self.left_shoulder_rpy_in_robot = [0.0, 0.0, 0.0] # expressed in robot frame
        self.right_shoulder_rpy_in_robot = [0.0, 0.0, 0.0]
        self.chest_axis_agl = [0.0, 0.0, 0.0]  # in robot frame
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker_pub_right = rospy.Publisher("visualization_marker_right", Marker, queue_size=10)
        self.marker_pub_elbow = rospy.Publisher("visualization_marker/elbow", Marker, queue_size=10)
        self.marker_pub_elbow_right = rospy.Publisher("visualization_marker_right/elbow", Marker, queue_size=10)
        self.marker_pub_shoulder = rospy.Publisher("visualization_marker/shoulder", Marker, queue_size=10)
        self.marker_pub_shoulder_right = rospy.Publisher("visualization_marker_right/shoulder", Marker, queue_size=10)
        self.marker_pub_chest = rospy.Publisher("visualization_marker_chest", Marker, queue_size=10)
        self.joint_state_puber = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.shoulder_angle_puber = rospy.Publisher('/quest3_debug/shoulder_angle', Float32MultiArray, queue_size=10)
        self.chest_axis_puber = rospy.Publisher('/quest3_debug/chest_axis', Float32MultiArray, queue_size=10)
        self.left_joystick = None
        self.right_joystick = None

    def read_joySticks_msg(self, msg):
        self.left_joystick = [msg.left_trigger, msg.left_grip]
        self.right_joystick = [msg.right_trigger, msg.right_grip]

    def read_msg(self, msg):
        """
        Read the PoseInfoList message and extract the left and right finger joint angles.
        """
        self.pose_info_list = msg.poses
        self.timestamp_ms = msg.timestamp_ms
        self.is_high_confidence = msg.is_high_confidence
        self.is_hand_tracking = msg.is_hand_tracking
        # print("is_high_confidence: {}, is_hand_tracking: {}".format(self.is_high_confidence, self.is_hand_tracking))
        if self.is_hand_tracking:
            self.compute_finger_joints("Left")
            self.compute_finger_joints("Right")
        else:
            self.compute_finger_joints_joy("Left")
            self.compute_finger_joints_joy("Right")
        self.compute_hand_pose("Left")
        self.compute_hand_pose("Right")
        # gesture counts
        max_counts = 50
        if(self.is_runing_gesture()):
            self.ok_gesture_counts += 1
            self.shot_gesture_counts = 0
            if(self.ok_gesture_counts >= max_counts):
                self.is_runing = True
        elif(self.is_stop_gesture()):
            self.shot_gesture_counts += 1
            self.ok_gesture_counts = 0
            if(self.shot_gesture_counts >= max_counts):
                self.is_runing = False

    def compute_finger_joints(self, side):
        """
        y轴从手腕指向中指指尖, x轴从拇指位置指向无名指方向
        """
        if self.pose_info_list is None:
            return None
        T_hand = self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandPalm"]])
        T_finger_thumb_tip = self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandThumbTip"]])
        finger_index_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandIndexTip"]].orientation
        finger_middle_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandMiddleTip"]].orientation
        finger_ring_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandRingTip"]].orientation
        finger_little_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandLittleTip"]].orientation
        R0_inv = T_hand[:3, :3].T
        finger_index_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_index_tip_ori))
        finger_middle_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_middle_tip_ori))
        finger_ring_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_ring_tip_ori))
        finger_little_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_little_tip_ori))
        # print(f"index: {finger_index_tip_rpy[2]}, middle: {finger_middle_tip_rpy[2]}")
        # finger_joints = [finger_thumb_metacarpal_rpy[2], finger_thumb_proximal_rpy[2], finger_thumb_distal_rpy[2], finger_thumb_tip_rpy[2], 

        p_hf_w = T_finger_thumb_tip[:3, 3] - T_hand[:3, 3]
        p_hf_h = R0_inv @ p_hf_w
        x, z = 0.0, 0.0
        if side == "Left":
            # x:[-0.075, 0]->[open, close]->[0, 100]
            # z:[0.044, 0.08]->[close, open]->[100, 0]
            x_min, x_max = -0.075, 0.0
            z_min, z_max = 0.044, 0.08
            x = min(x_max, max(x_min, p_hf_h[0]))
            z = min(z_max, max(z_min, p_hf_h[2]))
            x = 100.0 * (x - x_min) / (x_max - x_min)
            z = 100.0 - 100.0 * (z - z_min) / (z_max - z_min)
        if side == "Right":
            # x:[0, 0.075]->[close, open]->[100, 0]
            # z:[-0.08, -0.044]->[open, close]
            x_min, x_max = 0.0, 0.075
            z_min, z_max = -0.08, 0.044
            x = min(x_max, max(x_min, p_hf_h[0]))
            z = min(z_max, max(z_min, p_hf_h[2]))
            x = 100.0 - 100.0 * (x - x_min) / (x_max - x_min)
            z = 100.0 * (z - z_min) / (z_max - z_min)

        finger_joints = [1.7*z/100.0, 1.7*x/100.0, # TODO: fix thumb finger joints
                finger_index_tip_rpy[0], finger_middle_tip_rpy[0], finger_ring_tip_rpy[0], finger_little_tip_rpy[0]]
        for i in range(len(finger_joints)):
            finger_joints[i] = self.limit_finger_angle(finger_joints[i], back_agl=np.pi/2.0)
        if side == "Left":
            self.left_finger_joints = finger_joints        
            thumb_msg = Float32MultiArray()
            thumb_msg.data = p_hf_h
            hand_thumb_pub.publish(thumb_msg)
        elif side == "Right":
            self.right_finger_joints = finger_joints
        return finger_joints
 
    def compute_finger_joints_joy(self, side):
        """
        手柄版本的计算手指关节角度
        """
        if self.pose_info_list is None:
            return None
        hand_ori = self.pose_info_list[bone_name_to_index[side + "HandPalm"]].orientation
        finger_thumb_metacarpal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbMetacarpal"]].orientation
        finger_thumb_proximal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbProximal"]].orientation
        finger_thumb_distal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbDistal"]].orientation
        finger_thumb_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbTip"]].orientation
        finger_index_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandIndexTip"]].orientation
        finger_middle_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandMiddleTip"]].orientation
        finger_ring_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandRingTip"]].orientation
        finger_little_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandLittleTip"]].orientation
        R0_inv = self.quaternion_msg_to_matrix(hand_ori).T
        finger_thumb_metacarpal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_metacarpal_ori))
        finger_thumb_proximal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_proximal_ori))
        finger_thumb_distal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_distal_ori))
        finger_thumb_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_tip_ori))
        finger_index_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_index_tip_ori))
        finger_middle_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_middle_tip_ori))
        finger_ring_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_ring_tip_ori))
        finger_little_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_little_tip_ori))
        # print(f"index: {finger_index_tip_rpy[2]}, middle: {finger_middle_tip_rpy[2]}")
        # finger_joints = [finger_thumb_metacarpal_rpy[2], finger_thumb_proximal_rpy[2], finger_thumb_distal_rpy[2], finger_thumb_tip_rpy[2], 

        finger_joints = [0.0 for _ in range(6)]
        finger_joints[0] = self.limit_finger_angle(finger_index_tip_rpy[0])
        finger_joints[1] = 1.7
        finger_joints[2] = self.limit_finger_angle(finger_index_tip_rpy[0])
        finger_joints[3] = 0.0 if finger_middle_tip_rpy[0] > 0.0 else 1.7
        finger_joints[4] = 0.0 if finger_ring_tip_rpy[0] > 0.0 else 1.7
        finger_joints[5] = 0.0 if finger_little_tip_rpy[0] > 0.0 else 1.7

        if side == "Left":
            self.left_finger_joints = finger_joints
        elif side == "Right":
            self.right_finger_joints = finger_joints
        return finger_joints

    def get_finger_joints(self, side):
        if side == "Left":
            return self.left_finger_joints
        elif side == "Right":
            return self.right_finger_joints
        else:
            print("Invalid side: {}".format(side))
            return None

    @staticmethod
    def limit_finger_angle(angle, min_angle=0.0, max_angle=1.7, back_agl=1/4*np.pi):
        """
        finger joint angle might in the range of [-3.14, 3.14]
        Limit the finger joint angles to within the range [0.0, 1.7].
        back_agl: VR中手指可能存在反向的角度, 这里设置一个阈值, 超过这个阈值, 才认为是反向的, 否则认为是0
        """
        res = angle
        if res < 0.0:
            res += 2.0 * np.pi
        res = np.fmod(res, 2.0 * np.pi)
        if res < min_angle:
            res = min_angle
        if res > max_angle:
            if res < 2*np.pi - back_agl:
                res = max_angle
            else:  # res > 3/2*np.pi and res <= 2*np.pi
                res = 0.0
        return res

    def is_runing_gesture(self):
        """
        ok-ok:->Runing
        """
        is_runing = True
        is_runing &= self.ok_gesture_check(self.left_finger_joints)
        is_runing &= self.ok_gesture_check(self.right_finger_joints)

        return is_runing
 
    def is_stop_gesture(self):
        """
        shot-shot:->Stop
        """
        is_stop = True
        is_stop &= self.shot_gesture_check(self.left_finger_joints)
        is_stop &= self.shot_gesture_check(self.right_finger_joints)

        # 兼容手柄的情况
        is_joyStop = True
        is_joyStop &= self.joy_shot_gesture_check(self.left_joystick)
        is_joyStop &= self.joy_shot_gesture_check(self.right_joystick)

        return is_stop or is_joyStop

    @staticmethod
    def ok_gesture_check(finger_pos, min_agl=0.5, max_agl=1.0):
        """
        finger_pos: list of 6 float, [thumb1, thumb2, index, middle, ring, pinky]
        """
        is_ok = True
        is_ok &= (finger_pos[2] > max_agl)
        is_ok &= (finger_pos[3] < min_agl)
        is_ok &= (finger_pos[4] < min_agl)
        return is_ok    

    @staticmethod
    def shot_gesture_check(finger_pos, min_agl=0.5, max_agl=1.0):
        """
        finger_pos: list of 6 float, [thumb1, thumb2, index, middle, ring, pinky]
        """
        is_shot = True
        is_shot &= (finger_pos[2] < min_agl)
        is_shot &= (finger_pos[3] > max_agl)
        is_shot &= (finger_pos[4] > max_agl)
        return is_shot
    
    @staticmethod
    def joy_shot_gesture_check(stick_pos, min_agl=0.5, max_agl=0.8):
        """
        stick_pos : [trigger, grip]
        """
        is_shot = True
        is_shot &= (stick_pos[0] < min_agl)
        is_shot &= (stick_pos[1] > max_agl)
        return is_shot


    @staticmethod
    def quaternion_msg_to_matrix(ori):
        """
        Convert a quaternion to roll-pitch-yaw angles.
        """
        quat = [ori.x, ori.y, ori.z, ori.w]
        mat = quaternion_to_matrix(quat)
        return mat

    @staticmethod
    def quaternion_msg_to_RPY(ori):
        """
        Convert a quaternion to roll-pitch-yaw angles.
        """
        quat = [ori.x, ori.y, ori.z, ori.w]
        rpy = quaternion_to_RPY(quat)
        return rpy

    def get_hand_pose(self, side):
        if side == "Left":
            return self.left_hand_pose, self.left_elbow_pos
        elif side == "Right":
            return self.right_hand_pose, self.right_elbow_pos
        else:
            print("Invalid side: {}".format(side))
            return None, None

    def compute_shoudler_pose(self, R_wS, side):
        """
        R_wS: 3x3 rotation matrix from world to <side> shoulder
        """
        if side == "Left":
            R_01 = self.init_R_wLS.T @ R_wS
            # align to urdf model, the shoulder angle is negative
            self.left_shoulder_rpy_in_robot[0] = matrix_to_rpy(R_01)[2]
            self.left_shoulder_rpy_in_robot[1] = -matrix_to_rpy(R_01)[0]
            self.left_shoulder_rpy_in_robot[2] = -matrix_to_rpy(R_01)[1]
        elif side == "Right":
            R_01 = self.init_R_wRS.T @ R_wS
            # align to urdf model, the shoulder angle is positive
            self.right_shoulder_rpy_in_robot[0] = -matrix_to_rpy(R_01)[2]
            self.right_shoulder_rpy_in_robot[1] = -matrix_to_rpy(R_01)[0]
            self.right_shoulder_rpy_in_robot[2] = matrix_to_rpy(R_01)[1]
        msg = Float32MultiArray()
        msg.data = np.append(self.left_shoulder_rpy_in_robot, self.right_shoulder_rpy_in_robot)
        self.shoulder_angle_puber.publish(msg)

    def compute_hand_pose(self, side):
        chest_pose = self.pose_info_list[bone_name_to_index["Chest"]]
        elbow_pose = self.pose_info_list[bone_name_to_index[side + "ArmLower"]]
        shoulder_pose = self.pose_info_list[bone_name_to_index[side + "ArmUpper"]]
        # print(f"{side} elbow pos: {elbow_pose.position.x}, {elbow_pose.position.y}, {elbow_pose.position.z}")
        hand_pose = self.pose_info_list[bone_name_to_index[side + "HandPalm"]]

        vr_quat = [hand_pose.orientation.x, hand_pose.orientation.y, hand_pose.orientation.z, hand_pose.orientation.w]
        # print(f"{side} hand quat: {vr_quat}")
        # if use joystick, the hand_tracking_offset is 20.0 deg
        hand_quat_in_w = vr_quat2robot_quat(vr_quat, side, 15*np.pi/180.0 if not self.is_hand_tracking else 0.0) # [x, y, z, w]
        # hand_rpy = quaternion_to_RPY(hand_quat)

        T_wChest = self.pose_info2_transform(chest_pose)
        T_wElbow = self.pose_info2_transform(elbow_pose)
        T_wHand = self.pose_info2_transform(hand_pose)
        # if self.init_R_wC is None:
        #     self.init_R_wC = T_wChest[:3, :3]
        axis, angle = matrix_to_axis_angle(self.init_R_wC.T @ T_wChest[:3, :3])
        self.chest_axis_agl = [0, 0, axis[1]]

        # hand_rpy_cH = hand_rpy - chest_rpy
        hand_mat_cH = axis_angle_to_matrix(self.chest_axis_agl).T @ quaternion_to_matrix(hand_quat_in_w)
        hand_quat = matrix_to_quaternion(hand_mat_cH)
        chest_axis_msg = Float32MultiArray()
        chest_axis_msg.data = axis
        self.chest_axis_puber.publish(chest_axis_msg)
        T_wS = self.pose_info2_transform(shoulder_pose)
        if side == "Left" and self.init_R_wLS is None:
            self.init_R_wLS = T_wS[:3, :3]
        if side == "Right" and self.init_R_wRS is None:
            self.init_R_wRS = T_wS[:3, :3]
        self.compute_shoudler_pose(T_wS[:3, :3], side)
        # rpy_chest_robot = [np.pi, 0, np.pi/2.0]
        # # rpy_chest_robot = [0.0, 0, 0.0]
        # T_chest_robot = pos_rpy_to_transform([0, 0, 0], rpy_chest_robot)

        T_chestW = transform_inverse(T_wChest)
        # T_elbow_chest = T_elbow_W @ T_W_chest
        # T_hand_chest = T_hand_W @ T_W_chest
        T_ChestElbow = T_wElbow
        T_ChestHand = T_wHand

        hand_pos, _ = transform_to_pos_rpy(T_ChestHand)
        hand_pos[0] -= chest_pose.position.x
        hand_pos[1] -= chest_pose.position.y
        hand_pos[2] -= chest_pose.position.z
        hand_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ hand_pos
        hand_pos[0] += bias_chest_to_base_link[0]
        hand_pos[1] += bias_chest_to_base_link[1]
        hand_pos[2] += bias_chest_to_base_link[2]
        hand_pos[0] = 0.1 if hand_pos[0] < 0.1 else hand_pos[0]
        
        elbow_pos, _ = transform_to_pos_rpy(T_ChestElbow)
        elbow_pos[0] -= chest_pose.position.x
        elbow_pos[1] -= chest_pose.position.y
        elbow_pos[2] -= chest_pose.position.z
        elbow_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ elbow_pos
        elbow_pos[0] += bias_chest_to_base_link[0] # TODO: adjust elbow position
        elbow_pos[2] += bias_chest_to_base_link[2]

        shoulder_pos, _ = transform_to_pos_rpy(T_wS)
        shoulder_pos[0] -= chest_pose.position.x
        shoulder_pos[1] -= chest_pose.position.y
        shoulder_pos[2] -= chest_pose.position.z
        shoulder_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ shoulder_pos
        shoulder_pos[0] += bias_chest_to_base_link[0]
        shoulder_pos[2] += bias_chest_to_base_link[2]

        # chest_pos = [chest_pose.position.x, chest_pose.position.y, chest_pose.position.z]
        # radi0 = 26/14 # 机器人肩（measurement）/人体肩
        radi1 = 17.0/32 # 机器人大臂/人体大臂
        radi2 = 19.0/26 # 机器人小臂/人体小臂
        # shoulder_pos[1] = radi0 * shoulder_pos[1]
        if (side == "Right"):
            shoulder_pos[1] -= 0.22
        elif (side == "Left"):
            shoulder_pos[1] += 0.22
        for i in range(3):
            elbow_pos[i] = shoulder_pos[i] + radi1 * (elbow_pos[i] - shoulder_pos[i])
            # hand_pos[i] = elbow_pos[i] + (hand_pos[i] - (shoulder_pos[i] + (elbow_pos[i] - shoulder_pos[i]) / radi1))

        # print(f"{side} elbow pos: {elbow_pos}")

        # print("{} hand pos: {}, elbow pos: {}".format(side, hand_pos, elbow_pos))
        if self.vis_pub:
            marker = self.construct_marker(hand_pos, hand_quat, 1.0 if side == "Left" else 0.0, 0.0 if side == "Left" else 1.0, 0.0, side)
            elbow_marker = self.construct_point_marker(elbow_pos, 0.1)
            shoulder_marker = self.construct_point_marker(shoulder_pos, 0.1)
            if side == "Left":
                self.marker_pub.publish(marker)
                self.marker_pub_elbow.publish(elbow_marker)
                self.marker_pub_shoulder.publish(shoulder_marker)
            else:
                self.marker_pub_right.publish(marker)
                self.marker_pub_elbow_right.publish(elbow_marker)
                self.marker_pub_shoulder_right.publish(shoulder_marker)
            chest_pos = [chest_pose.position.x, chest_pose.position.y, chest_pose.position.z]
            chest_quat = [chest_pose.orientation.w, chest_pose.orientation.x, chest_pose.orientation.y, chest_pose.orientation.z]
            # chest_marker = self.construct_marker(chest_pos, chest_quat, 0, 0, 1, "Torso")
            chest_marker = self.construct_point_marker(chest_pos, 0.1, 0.8)
            self.marker_pub_chest.publish(chest_marker)
        if side == "Left":
            self.left_hand_pose = (hand_pos, hand_quat)
            self.left_elbow_pos = elbow_pos
        else:
            self.right_hand_pose = (hand_pos, hand_quat)
            self.right_elbow_pos = elbow_pos

    def check_if_vr_error(self):
        """
        Check if the VR system is error.
        """
        left_hand_pos = self.left_hand_pose[0]
        right_hand_pos = self.right_hand_pose[0]
        error = True
        error &= (0.09 < left_hand_pos[0] < 0.11)
        error &= (-0.03 < left_hand_pos[1] < 0.03)
        error &= (0.32 < left_hand_pos[2] < 0.34)
        error &= (0.09 < right_hand_pos[0] < 0.11)
        error &= (-0.03 < right_hand_pos[1] < 0.03)
        error &= (0.32 < right_hand_pos[2] < 0.34)
        return error

    @staticmethod
    def pose_info2_transform(pose_info):
        """
        Convert a PoseInfo message to a 4x4 transformation matrix.
        """
        pos = [pose_info.position.x, pose_info.position.y, pose_info.position.z]
        quat = [pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z, pose_info.orientation.w]
        rpy = quaternion_to_RPY(quat)
        T = pos_rpy_to_transform(pos, rpy)
        return T
    
    def construct_marker(self, arm_pose_p, arm_pose_q, r, g, b, side):
        if len(arm_pose_q) != 4 or len(arm_pose_p)!= 3:
            print("Invalid arm pose, cannot construct marker")
            return None
        marker = Marker()
        # marker.id = id
        marker.header.frame_id = (
            "base_link"  # set frame_id according to the actual situation
        )
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        if side == "Left":
            marker.mesh_resource = (
                "file://"
                + self.model_path
                + "/models/biped_gen4.0/meshes/l_hand_roll.obj"
            )
        elif side == "Right":
            marker.mesh_resource = (
                "file://"
                + self.model_path
                + "/models/biped_gen4.0/meshes/r_hand_roll.obj"
            )
        elif side == "Torso":
            marker.mesh_resource = (
                "file://"
                + self.model_path
                + "/models/biped_gen4.0/meshes/base_link.obj"
            )
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 0.3
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = arm_pose_p[0]
        marker.pose.position.y = arm_pose_p[1]
        marker.pose.position.z = arm_pose_p[2]
        marker.pose.orientation = Quaternion(*arm_pose_q)
        return marker

    def construct_point_marker(self, point, scale=0.05, alpha=0.3):
        if len(point) != 3:
            print("Invalid pos, cannot construct marker")
            return None
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = alpha
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.w = 1.0
        return marker 

    def pub_whole_body_joint_state_msg(self, arm_joint_angles, map_finger=True):
        """
        Build a JointState message for the whole body joints.
        """
        if len(arm_joint_angles) != 14:
            print("Invalid arm joint angles, cannot construct joint state message")
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [
            "leg_l1_joint",
            "leg_l2_joint",
            "leg_l3_joint",
            "leg_l4_joint",
            "leg_l5_joint",
            "leg_l6_joint",
            "leg_r1_joint",
            "leg_r2_joint",
            "leg_r3_joint",
            "leg_r4_joint",
            "leg_r5_joint",
            "leg_r6_joint",
            "zarm_l1_joint",
            "zarm_l2_joint",
            "zarm_l3_joint",
            "zarm_l4_joint",
            "zarm_l5_joint",
            "zarm_l6_joint",
            "zarm_l7_joint",
            "zarm_r1_joint",
            "zarm_r2_joint",
            "zarm_r3_joint",
            "zarm_r4_joint",
            "zarm_r5_joint",
            "zarm_r6_joint",
            "zarm_r7_joint",
            "l_thumb_proximal_yaw_joint",
            "l_thumb_distal_pitch_joint",
            "l_index_proximal_finger_joint",
            "l_index_distal_finger_joint",
            "l_middle_proximal_finger_joint",
            "l_middle_distal_finger_joint",
            "l_ring_proximal_finger_joint",
            "l_ring_distal_finger_joint",
            "l_pinky_proximal_finger_joint",
            "l_pinky_distal_finger_joint",
            "r_thumb_proximal_yaw_joint",
            "r_thumb_distal_pitch_joint",
            "r_index_proximal_finger_joint",
            "r_index_distal_finger_joint",
            "r_middle_proximal_finger_joint",
            "r_middle_distal_finger_joint",
            "r_ring_proximal_finger_joint",
            "r_ring_distal_finger_joint",
            "r_pinky_proximal_finger_joint",
            "r_pinky_distal_finger_joint"
            ]
        # print(f"Dim: {len(msg.name)}")
        for i in range(len(msg.name)):
            msg.position.append(0.0)
        for i in range(14):  # arm
            msg.position[12 + i] = arm_joint_angles[i]
        map = [0, 1, 2, 4, 6, 8]  # 6
        if map_finger:
            for i in range(6):  # left hand finger
                msg.position[26 + map[i]] = self.left_finger_joints[i]
            for i in range(6):  # right hand finger
                msg.position[36 + map[i]] = self.right_finger_joints[i]
        # deal right hand thumb
        msg.position[36 + 1] *= -1
        self.joint_state_puber.publish(msg)
       