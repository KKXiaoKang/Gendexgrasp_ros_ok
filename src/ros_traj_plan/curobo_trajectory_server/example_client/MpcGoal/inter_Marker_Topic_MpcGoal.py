#!/usr/bin/env python
import rospy
from curobo_trajectory_server.srv import cuRoMpcSetting, cuRoMpcSettingRequest
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
import tf

pose_pub = rospy.Publisher('/mpc_cmd_result', PoseStamped, queue_size=10)

def normalize_quaternion(orientation):
    """
    确保四元数规范化
    """
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    normalized_quaternion = tf.transformations.unit_vector(quaternion)
    orientation.x, orientation.y, orientation.z, orientation.w = normalized_quaternion
    return orientation


def create_interactive_marker(server):
    """
    创建一个 Interactive Marker，用于动态调整目标 Pose
    """
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"  # 定义坐标系
    int_marker.name = "mpc_goal_marker"
    int_marker.description = "Drag to set MPC Goal"
    int_marker.scale = 0.5

    # 初始位置
    int_marker.pose.position.x = 0.3
    int_marker.pose.position.y = 0.3
    int_marker.pose.position.z = 0.2
    # 四元数需要标准化
    int_marker.pose.orientation.x = 0.0
    int_marker.pose.orientation.y = -0.70710677
    int_marker.pose.orientation.z = 0.0
    int_marker.pose.orientation.w = 0.70710677

    # 标准化四元数
    int_marker.pose.orientation = normalize_quaternion(int_marker.pose.orientation)

    # 控制：平移和旋转
    controls = []

    # 平移 X
    control = InteractiveMarkerControl()
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation.w = 1.0
    control.orientation.x = 1.0  # 控制 X 轴
    controls.append(control)

    # 平移 Y
    control = InteractiveMarkerControl()
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation.w = 1.0
    control.orientation.y = 1.0  # 控制 Y 轴
    controls.append(control)

    # 平移 Z
    control = InteractiveMarkerControl()
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation.w = 1.0
    control.orientation.z = 1.0  # 控制 Z 轴
    controls.append(control)

    # 旋转 Z
    control = InteractiveMarkerControl()
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation.w = 1.0
    control.orientation.z = 1.0  # 旋转 Z 轴
    controls.append(control)

    # 添加控制到 Marker
    int_marker.controls.extend(controls)

    # 插入 Marker 到服务器
    server.insert(int_marker, update_pose_callback)
    server.applyChanges()

    return int_marker


def update_pose_callback(feedback):
    """
    更新全局的目标 Pose
    """
    global current_pose
    current_pose = feedback.pose  # 更新全局的 Pose
    rospy.loginfo(f"Updated Pose: {current_pose}")


def call_mpc_goal_service(pose):
    """
    调用 '/cumotion/mpc_set_goal' 服务，传递目标 pose 并获取响应
    """
    global pose_pub

    # 创建 Header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"

    # 创建 PoseStamped 消息
    pose_msg = PoseStamped()
    pose_msg.header = header
    pose_msg.pose = pose

    # 创建服务请求
    request = cuRoMpcSettingRequest()
    request.pose = pose_msg

    # 发布话题
    pose_pub.publish(pose_msg)

def main():
    """
    初始化节点，设置 Interactive Marker 和服务调用循环
    """
    global current_pose
    rospy.init_node("interactive_marker_mpc_goal")

    # 初始化 Interactive Marker Server
    server = InteractiveMarkerServer("mpc_goal_marker_server")

    # 创建 Interactive Marker
    create_interactive_marker(server)

    # 设置初始目标 Pose
    current_pose = Pose()
    current_pose.position.x = 0.3
    current_pose.position.y = 0.3
    current_pose.position.z = 0.2
    current_pose.orientation.x = 0.0
    current_pose.orientation.y = -0.70710677
    current_pose.orientation.z = 0.0
    current_pose.orientation.w = 0.70710677

    # 标准化四元数
    current_pose.orientation = normalize_quaternion(current_pose.orientation)

    # 等待服务启动
    rate = rospy.Rate(100)  # 10Hz 频率

    while not rospy.is_shutdown():
        try:
            # 调用服务发送目标 Pose
            call_mpc_goal_service(current_pose)

            # 控制频率
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("ROS Interrupt Exception occurred")
            break


if __name__ == "__main__":
    main()
