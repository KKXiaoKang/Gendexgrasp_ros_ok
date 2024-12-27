import socket
import sys
import os
import json
import time
import signal
import sys
from pprint import pprint
import tf
import rospy
from kuavo_ros_interfaces.msg import robotHeadMotionData
from noitom_hi5_hand_udp_python.msg import PoseInfoList, PoseInfo, JoySticks
from geometry_msgs.msg import Point, Quaternion

# Add the parent directory to the system path to allow relative imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

# Import the hand_pose_pb2 module
import protos.hand_pose_pb2 as event_pb2
import sys

# Check if server address and port are provided as a single command-line argument
if len(sys.argv) != 2:
    print("Usage: python test_unity.py <server_address[:port]> ")
    sys.exit(1)

# Parse the server address and port
try:
    if ':' in sys.argv[1]:
        server_address, port = sys.argv[1].split(':')
        port = int(port)
    else:
        server_address = sys.argv[1]
        port = 10019
except ValueError:
    print("Argument must be in the format <server_address[:port]> and port must be an integer")
    sys.exit(1)


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
    "Chest",  # Added new bone
    "Neck",
    "Head"
]

current_file_dir = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(current_file_dir, "config.json")
calibrated_head_quat_matrix = None
head_motion_range = None

def load_config():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            return json.load(f)
    return {}

def save_config(config):
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=4)


def get_head_motion_range():
    config = load_config()
    return config.get("head_motion_range", None)


import numpy as np

if get_head_motion_range() is not None:
    head_motion_range = get_head_motion_range()
# Create bone_name_to_index dictionary
bone_name_to_index = {name: index for index, name in enumerate(bone_names)}

# List of bone IDs in order
fullBodyBoneIds_leju_arms = [bone_name_to_index[name] for name in bone_names]

# Create a reverse mapping from index to bone name
index_to_bone_name = {index: bone_names[index] for index in range(len(bone_names))}

# # Usage example: get index of 'FullBody_LeftShoulder'
# index_of_left_shoulder = bone_name_to_index["FullBody_LeftShoulder"]
# print(index_of_left_shoulder)  # Output: 0


def signal_handler(sig, frame):
    print('Exiting gracefully...')
    sock.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


rospy.init_node('Quest3_bone_frame_publisher', anonymous=True)
rate = rospy.Rate(1000.0)  # Set the rate of transform update to 100 Hz

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1)  # Set timeout to 1 second

# Define the server address and port
server_address = (server_address, port)

# Create an Event message
event = event_pb2.LejuHandPoseEvent()

# Send "hi" to the server first with retry logic
message = b'hi'
max_retries = 200
for attempt in range(max_retries):
    try:
        sock.sendto(message, server_address)
        # print(f"Message sent successfully on attempt {attempt + 1}")
        # Wait for the acknowledgment from the server
        sock.recvfrom(1024)  # Buffer size of 1024 bytes
        print(f"Acknowledgment received on attempt {attempt + 1}, start to receiving data...")
        break
    except socket.timeout:
        print(f"Attempt {attempt + 1} timed out. Retrying...")
    except KeyboardInterrupt:
        print("Force quit by Ctrl-c.")
        signal_handler(signal.SIGINT, None)
else:
    print("Failed to send message after 100 attempts.")
    signal_handler(signal.SIGINT, None)

# Function to convert position from left-hand to right-hand coordinate system
def convert_position_to_right_hand(left_hand_position):
    right_hand_position = {
        "x": 0 - left_hand_position["z"],  # Forward in left-hand becomes x in right-hand
        "y": 0 - left_hand_position["x"],  # Left in left-hand becomes y in right-hand
        "z": left_hand_position["y"]       # Upward in left-hand becomes z in right-hand
    }
    return right_hand_position


# Function to convert quaternion from left-hand to right-hand coordinate system
def convert_quaternion_to_right_hand(left_hand_quat):
    # Assuming the quaternion is in the form (x, y, z, w)
    right_hand_quat = (
        0 - left_hand_quat[2],  # z in left-hand becomes x in right-hand
        0 - left_hand_quat[0], # x in left-hand becomes -y in right-hand (flipped)
        left_hand_quat[1],  # y in left-hand becomes z in right-hand
        left_hand_quat[3]   # w remains the same
    )
    return right_hand_quat


def updateAFrame(frame_name, frame_position, frame_rotation_quat, time_now):
    # print("Frame Rotation Quaternion:", frame_rotation_quat)  # Added print statement as per instruction
    br.sendTransform((frame_position["x"], frame_position["y"], frame_position["z"]), frame_rotation_quat, time_now, frame_name, "torso")

br = tf.TransformBroadcaster()

# Initialize ROS publisher
pose_pub = rospy.Publisher('/leju_quest_bone_poses', PoseInfoList, queue_size=10)
head_data_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
joysticks_pub = rospy.Publisher('quest_joystick_data', JoySticks, queue_size=10)
print("Start receving data...")


def normalize_degree_in_180(degree):
    if degree > 180:
        degree -= 180
    elif degree < -180:
        degree += 180
    return degree

def pub_head_motion_data(cur_quat):
    # Convert quaternion to roll, pitch, yaw (RPY)
    global calibrated_head_quat_matrix
    if calibrated_head_quat_matrix is None:
        calibrated_head_quat_matrix = tf.transformations.quaternion_matrix(cur_quat)
    else:
        current_quat_matrix = tf.transformations.quaternion_matrix(cur_quat)
        relative_quat_matrix = tf.transformations.concatenate_matrices(tf.transformations.inverse_matrix(calibrated_head_quat_matrix), current_quat_matrix)
        relative_quat = tf.transformations.quaternion_from_matrix(relative_quat_matrix)
        import math
        rpy = tf.transformations.euler_from_quaternion(relative_quat)
        rpy_deg = [r * 180 / math.pi for r in rpy]
        pitch = max(min(normalize_degree_in_180(round(rpy_deg[0], 2)), head_motion_range["pitch"][1]), head_motion_range["pitch"][0])
        yaw = max(min(normalize_degree_in_180(round(rpy_deg[1], 2)), head_motion_range["yaw"][1]), head_motion_range["yaw"][0])
        msg = robotHeadMotionData()
        msg.joint_data = [yaw , pitch]
        head_data_pub.publish(msg)


while not rospy.is_shutdown():
    try:
        # Wait for data
        data, _ = sock.recvfrom(4096)
        # Deserialize the message from the received data
        event.ParseFromString(data)

        # Print left joystick values
        print("Left Joystick:")
        print(f"  X: {event.left_joystick.x}")
        print(f"  Y: {event.left_joystick.y}")
        print(f"  Trigger: {event.left_joystick.trigger}")
        print(f"  Grip: {event.left_joystick.grip}")
        print(f"  First Button Pressed: {event.left_joystick.firstButtonPressed}")
        print(f"  Second Button Pressed: {event.left_joystick.secondButtonPressed}")
        print(f"  First Button Touched: {event.left_joystick.firstButtonTouched}")
        print(f"  Second Button Touched: {event.left_joystick.secondButtonTouched}")

        # Print right joystick values
        print("Right Joystick:")
        print(f"  X: {event.right_joystick.x}")
        print(f"  Y: {event.right_joystick.y}")
        print(f"  Trigger: {event.right_joystick.trigger}")
        print(f"  Grip: {event.right_joystick.grip}")
        print(f"  First Button Pressed: {event.right_joystick.firstButtonPressed}")
        print(f"  Second Button Pressed: {event.right_joystick.secondButtonPressed}")
        print(f"  First Button Touched: {event.right_joystick.firstButtonTouched}")
        print(f"  Second Button Touched: {event.right_joystick.secondButtonTouched}")

        # Print the timestamp
        # print(f'Received timestamp: {event.timestamp}')
        # pprint(event.poses)
        time_now = rospy.Time.now()

        # Create PoseInfoList message
        pose_info_list = PoseInfoList()

        joysticks_msg = JoySticks()
        
        joysticks_msg.left_x = event.left_joystick.x
        joysticks_msg.left_y = event.left_joystick.y
        joysticks_msg.left_trigger = event.left_joystick.trigger
        joysticks_msg.left_grip = event.left_joystick.grip
        joysticks_msg.left_first_button_pressed = event.left_joystick.firstButtonPressed
        joysticks_msg.left_second_button_pressed = event.left_joystick.secondButtonPressed
        joysticks_msg.left_first_button_touched = event.left_joystick.firstButtonTouched
        joysticks_msg.left_second_button_touched = event.left_joystick.secondButtonTouched
        joysticks_msg.right_x = event.right_joystick.x
        joysticks_msg.right_y = event.right_joystick.y
        joysticks_msg.right_trigger = event.right_joystick.trigger
        joysticks_msg.right_grip = event.right_joystick.grip
        joysticks_msg.right_first_button_pressed = event.right_joystick.firstButtonPressed
        joysticks_msg.right_second_button_pressed = event.right_joystick.secondButtonPressed
        joysticks_msg.right_first_button_touched = event.right_joystick.firstButtonTouched
        joysticks_msg.right_second_button_touched = event.right_joystick.secondButtonTouched

        rospy.loginfo(joysticks_msg)
        joysticks_pub.publish(joysticks_msg)

        # Iterate over event.poses and print each pose
        for i, pose in enumerate(event.poses):
            bone_name = index_to_bone_name[i]
            # print(f'Bone Name: {bone_name}')  # Added line to print the bone name

            # Extract position and quaternion from the pose
            frame_position = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            }
            frame_rotation_quat = (
                pose.quaternion.x,
                pose.quaternion.y,
                pose.quaternion.z,
                pose.quaternion.w
            )

            # Convert to right-hand coordinate system
            right_hand_position = convert_position_to_right_hand(frame_position)
            right_hand_quat = convert_quaternion_to_right_hand(frame_rotation_quat)

            # Create PoseInfo message
            pose_info = PoseInfo()
            pose_info.position = Point(x=right_hand_position["x"], y=right_hand_position["y"], z=right_hand_position["z"])
            pose_info.orientation = Quaternion(x=right_hand_quat[0], y=right_hand_quat[1], z=right_hand_quat[2], w=right_hand_quat[3])

            # Append PoseInfo to PoseInfoList
            pose_info_list.poses.append(pose_info)


            scale_factor_x = 3.0
            scale_factor_y = 3.0
            scale_factor_z = 3.0


            # Scale up x, y, z coordinates
            right_hand_position["x"] *= scale_factor_x
            right_hand_position["y"] *= scale_factor_y
            right_hand_position["z"] *= scale_factor_z

            # Update the frame using the extracted data
            updateAFrame(bone_name, right_hand_position, right_hand_quat, time_now)

        pose_info_list.timestamp_ms = event.timestamp
        pose_info_list.is_high_confidence = event.IsDataHighConfidence
        pose_info_list.is_hand_tracking = event.IsHandTracking

        # Publish PoseInfoList
        pose_pub.publish(pose_info_list)

        if bone_name == "Head":
            pub_head_motion_data(right_hand_quat)

        rate.sleep()
        # time.sleep(0.0005)  # Delay of 10ms
    except socket.timeout:
        print('Timeout occurred, no data received. Exiting...')
        signal_handler(signal.SIGINT, None)
    except Exception as e:
        print(f'An error occurred: {e}')

print('Closing socket')
sock.close()
