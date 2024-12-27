#!/usr/bin/env python
import rospy
from dynamic_biped.srv import planCumotionJoint, planCumotionJointRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def main():
    rospy.init_node('moveit2_plan_srv_cumotion_client', anonymous=True)

    # 等待服务启动
    service_name = '/moveit2_plan_cumotion_joint_srv'
    rospy.wait_for_service(service_name)

    try:
        # 创建服务代理
        plan_cumotion_joint = rospy.ServiceProxy(service_name, planCumotionJoint)

        # 设置服务请求
        req = planCumotionJointRequest()

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
                # -0.5043101656131275, 0.5043101656131275, 0.5043101656131275, -0.78543101656131275, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                # -0.9043101656131275, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                # 0.3490658503988659, 0.8726646259971648, 0.55453292519943295, -1.2217304763960306, -0.3490658503988659, -0.5235987755982988, 0.0
            ],
            velocity=[],
            effort=[]
        )

        # 设置 target_robot_joint_space
        req.target_robot_joint_space = JointState(
            header=Header(stamp=rospy.Time(0), frame_id='torso'),
            name=[
                'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'
            ],
            position=[
                0.3490658503988659, 0.8726646259971648, 0.55453292519943295, -1.2217304763960306, -0.3490658503988659, -0.5235987755982988, 0.0
            ],
            velocity=[],
            effort=[]
        )

        # 调用服务并获取响应
        rospy.loginfo("Sending service request...")
        response = plan_cumotion_joint(req)
        rospy.loginfo(f"Response: {response.result}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == '__main__':
    main()
