import rospy
from std_msgs.msg import Float32MultiArray
from motion_capture_ik.msg import twoArmHandPose

import numpy as np

if __name__ == "__main__":
    rospy.init_node("sim_ik_cmd", anonymous=True)
    pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPose, queue_size=10) # motion_capture_ik末端topic求解接口
    record_data = np.load("../../data/rosbag_s.npy") # quat版本， from huawei
    print(f"data size: {len(record_data)}")
    rate = rospy.Rate(20) # 1/5=0.2s maximum value
    idx = 0
    forward_direction = True
    # 循环读取数据并发布
    while not rospy.is_shutdown():# and idx <= 10:
        eef_pose_msg = twoArmHandPose()
        eef_pose_msg.left_pose.pos_xyz = record_data[idx, :3]
        eef_pose_msg.left_pose.quat_xyzw = record_data[idx, -4:]
        eef_pose_msg.left_pose.elbow_pos_xyz = np.zeros(3)
        eef_pose_msg.right_pose.pos_xyz = record_data[idx, :3]
        # eef_pose_msg.right_pose.pos_xyz[1] *= -1
        eef_pose_msg.right_pose.quat_xyzw = [0, 0, 0, 1]
        eef_pose_msg.right_pose.elbow_pos_xyz = np.zeros(3)
        pub.publish(eef_pose_msg)
        rate.sleep()
        idx = idx + 1 if forward_direction else idx - 1
        if idx == len(record_data) - 1:
            forward_direction = False
        elif idx == 0:
            forward_direction = True
        
        print(f"eef_pose_msg[{idx}]:\n {eef_pose_msg}")
