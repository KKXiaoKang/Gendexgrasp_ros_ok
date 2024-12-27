import rospy
from std_msgs.msg import Header
from dynamic_biped.msg import robotHandPosition
from hand_sdk_control.srv import handPosService, handPosServiceResponse
import time
import numpy as np  # 引入numpy进行线性插值

class HandSDKControlServer:
    def __init__(self):
        rospy.init_node('hand_sdk_control_server')
        
        self.control_robot_hand_position_pub = rospy.Publisher(
            "/control_robot_hand_position", robotHandPosition, queue_size=10
        )
        
        self.service = rospy.Service(
            "/hand_sdk_control_service", handPosService, self.handle_hand_position_request
        )
        rospy.loginfo("Hand SDK Control Service Ready.")
    
    def handle_hand_position_request(self, req):
        # 等待
        time.sleep(6)

        # 将uint8[]转换为list，以避免它们被误解为bytes
        left_hand_position = list(req.left_hand_position)
        right_hand_position = list(req.right_hand_position)

        # 打印接收到的请求
        rospy.loginfo("Received Hand Position Request: Left Hand Position: {}, Right Hand Position: {}".format(
            left_hand_position, right_hand_position
        ))

        # 初始位置从0开始进行
        # 目标位置，假设最终手的位置是[100, 100, 100, 100, 100, 100]
        # 打开虎口 [0, 100, 0, 0, 0, 0]
        zero_left_hand_position = [0, 100, 0, 0, 0, 0]
        # zero_left_hand_position = [0, 0, 0, 0, 0, 0]

        # target_left_hand_position = [100, 100, 100, 100, 100, 100]
        target_left_hand_position = left_hand_position
        target_left_hand_position[0], target_left_hand_position[1] = target_left_hand_position[1], target_left_hand_position[0] # 左手灵巧手维度交换
        # target_left_hand_position[5] = 0

        # 发布手部姿态，线性插值10次，间隔0.5秒
        num_steps = 2
        step_duration = 0.5  # 每次发布之间的时间间隔

        # 使用numpy进行线性插值，生成从初始到目标的过渡数组
        left_hand_position_steps = np.linspace(zero_left_hand_position, target_left_hand_position, num_steps)
        
        for i in range(num_steps):
            # 构建并发布消息
            hand_position_msg = robotHandPosition()
            hand_position_msg.header = req.header
            hand_position_msg.left_hand_position = left_hand_position_steps[i].astype(int).tolist()  # 转换为整数并发布
            hand_position_msg.right_hand_position = right_hand_position  # 右手位置保持不变
            
            self.control_robot_hand_position_pub.publish(hand_position_msg)
            
            # 打印每次发布的左手姿态
            rospy.loginfo(f"Publishing Step {i + 1}/{num_steps}: Left Hand Position: {hand_position_msg.left_hand_position}")
            
            # 等待0.5秒
            time.sleep(step_duration)

        return handPosServiceResponse(result=True)
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    server = HandSDKControlServer()
    server.run()
