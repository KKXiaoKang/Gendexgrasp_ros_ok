#!/usr/bin/env python
import rospy
from curobo_trajectory_server.srv import cuRoMoveGroup, cuRoMoveGroupRequest, cuRoMoveGroupResponse
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# 初始化 JointState 变量时直接赋值
init_robot_joint_space = JointState(
    header=Header(stamp=rospy.Time(0), frame_id='torso'),
    name=['zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'],
    position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    velocity=[],
    effort=[]
)

target_robot_joint_space = JointState(
    header=Header(stamp=rospy.Time(0), frame_id='torso'),
    name=['zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'],
    position=[0.3490658503988659, 0.8726646259971648, 0.55453292519943295, -1.2217304763960306, -0.3490658503988659, -0.5235987755982988, 0.0],
    velocity=[],
    effort=[]
)

def main():
    global init_robot_joint_space
    rospy.init_node('moveit2_plan_srv_cumotion_client', anonymous=True)

    # 等待服务启动
    service_name = '/cumotion/move_group'
    rospy.wait_for_service(service_name)
    # 创建服务代理
    plan_cumotion_client = rospy.ServiceProxy(service_name, cuRoMoveGroup)

    try:
        # 创建请求对象
        request_msg = cuRoMoveGroupRequest()

        # moveit_msgs/MotionPlanRequest -- 参数配置
        request_msg.request.workspace_parameters.header.frame_id = "torso"
        request_msg.request.workspace_parameters.min_corner.x = -1.0
        request_msg.request.workspace_parameters.min_corner.y = -1.0
        request_msg.request.workspace_parameters.min_corner.z = -1.0
        request_msg.request.workspace_parameters.max_corner.x = 1.0
        request_msg.request.workspace_parameters.max_corner.y = 1.0
        request_msg.request.workspace_parameters.max_corner.z = 1.0
        request_msg.request.pipeline_id = "isaac_ros_cumotion"
        request_msg.request.planner_id = "cuMotion"
        request_msg.request.group_name = "l_arm_group"
        request_msg.request.num_planning_attempts = 10
        request_msg.request.allowed_planning_time = 5.0
        request_msg.request.max_velocity_scaling_factor = 0.1
        request_msg.request.max_acceleration_scaling_factor = 0.1

        # 创建关节初始状态
        request_msg.request.start_state.joint_state = init_robot_joint_space
        request_msg.request.start_state.is_diff = False
        
        # 创建目标关节状态
        goal_constraints = Constraints()
        for i in range(len(target_robot_joint_space.name)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = target_robot_joint_space.name[i]
            joint_constraint.position = target_robot_joint_space.position[i]
            joint_constraint.tolerance_above = 0.01  # 自定义容忍度
            joint_constraint.tolerance_below = 0.01  # 自定义容忍度
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        request_msg.request.goal_constraints.append(goal_constraints)
        
        # 调用
        print(f"request_msg: {request_msg}")
        response = plan_cumotion_client(request_msg)
        print(f"response: {response}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    main()

