source /opt/ros/noetic/setup.bash
source /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/devel/setup.bash

rosbag record -o pick_avoidance_traj_demo_data  \
                /current_joint_position \
	            /robot_arm_q_v_tau \
                /kuavo_arm_traj \
                --bz2 
