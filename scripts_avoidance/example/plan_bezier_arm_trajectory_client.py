#!/usr/bin/env python
import rospy
from dynamic_biped.srv import planBezierJoint, planBezierJointRequest, planBezierJointResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def main():
    rospy.init_node('plan_bezier_arm_trajectory_client', anonymous=True)

    # 等待服务启动
    service_name = '/generate_plan_arm_trajectory_service'
    rospy.wait_for_service(service_name)

    try:
        # 创建服务代理
        plan_bezier_joint_client = rospy.ServiceProxy(service_name, planBezierJoint)

        # 设置服务请求
        req = planBezierJointRequest()

        # 设置调用
        req.start_flag = True

        # 设置 init_robot_joint_space
        req.init_robot_joint_space = JointState(
            header=Header(stamp=rospy.Time(0), frame_id='torso'),
            name=[
                'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint',
                'zarm_r1_joint', 'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint', 'zarm_r5_joint', 'zarm_r6_joint', 'zarm_r7_joint'
            ],
            position=[
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ],
            velocity=[],
            effort=[]
        )

        # 设置 target_robot_joint_space
        req.target_robot_joint_space = JointState(
            header=Header(stamp=rospy.Time(0), frame_id='torso'),
            name=[
                'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint',
                'zarm_r1_joint', 'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint', 'zarm_r5_joint', 'zarm_r6_joint', 'zarm_r7_joint'
            ],
            position=[
                -0.5043101656131275, 1.2043101656131275, -0.643101656131275, -0.643101656131275, -0.643101656131275, -0.643101656131275, -0.643101656131275,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ],
            velocity=[],
            effort=[]
        )

        # 调用服务并获取响应
        rospy.loginfo("Sending service request...")
        response = plan_bezier_joint_client(req)
        rospy.loginfo(f"Response: {response.result}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == '__main__':
    main()
