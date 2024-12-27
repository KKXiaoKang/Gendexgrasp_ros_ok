#!/usr/bin/env python

import rospy
from dynamic_biped.msg import robotHeadMotionData
from std_srvs.srv import Trigger,TriggerResponse,TriggerRequest  # ROS 1 的 Trigger 服务类型
import time

MAX_RIGHT_JOINT_ANGLE = [5.0, -5.0]  #  10.0, -5.0
MAX_LEFT_JOINT_ANGLE = [-23.0, -5.0]  # -30.0, -5.0
PRE_JOINT_AGNGLE = [-15.0, -12.0] # -6.0, -16.0
CYCLE_COUNT = 1 # 周期数

class RobotCameraWatchAround:
    def __init__(self):
        # 初始化 ROS 1 节点
        rospy.init_node('robot_camera_watch_around')

        # 发布器，发布到 /robot_head_motion_data 话题
        self.publisher = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
        
        # 数据切换标志，控制每次发布的数据
        self.switch = True
        # 周期计数器
        self.cycle_count = 0
        # 每个周期结束时需要回到的初始位置
        self.initial_position = PRE_JOINT_AGNGLE

        # 创建服务，名字是 /ros1_collect_the_environment，类型是 Trigger
        self.srv = rospy.Service('/ros1_collect_the_environment', Trigger, self.collect_the_environment_callback)

        # 启动定时器（ROS 1 没有定时器像 ROS 2 那样方便，这里我们直接在回调中控制周期）
        self.timer = None

    def collect_the_environment_callback(self, request):
        # 每次服务调用时开始运动
        rospy.loginfo('Starting the head motion for 3 cycles.')

        # 初始化周期计数器
        self.cycle_count = 0
        self.switch = True  # 确保开始时是第一组数据

        # 启动定时器，每 3 秒钟执行一次
        self.timer = rospy.Timer(rospy.Duration(1.5), self.publish_joint_data)

        # 返回响应
        return TriggerResponse(success=True, message='Head motion started for 3 cycles.')

    def publish_joint_data(self, event):
        global CYCLE_COUNT
        global PRE_JOINT_AGNGLE
        global MAX_RIGHT_JOINT_ANGLE
        global MAX_LEFT_JOINT_ANGLE
        
        # 如果已经完成 1 个周期，返回初始位置并停止定时器
        if self.cycle_count >= CYCLE_COUNT:
            joint_data = self.initial_position
            rospy.loginfo(f'Cycle complete, returning to initial position: {joint_data}')
            # 创建消息
            msg = robotHeadMotionData()
            msg.joint_data = joint_data
            # 发布消息
            self.publisher.publish(msg)
            # 在此返回初始位置后停止定时器
            self.timer.shutdown()  # 停止定时器
            return  # 停止执行后续代码

        # 切换 joint_data 的内容
        if self.switch:
            joint_data = MAX_LEFT_JOINT_ANGLE  # 第一组数据
        else:
            joint_data = MAX_RIGHT_JOINT_ANGLE # 第二组数据
        
        # 每完成一个周期，增加周期计数
        if joint_data == MAX_RIGHT_JOINT_ANGLE:
            self.cycle_count += 1

        rospy.loginfo(f'Publishing joint_data: {joint_data}')
        rospy.loginfo(f'cycle_count: {self.cycle_count}')

        # 创建消息
        msg = robotHeadMotionData()
        msg.joint_data = joint_data

        # 发布消息
        self.publisher.publish(msg)
        self.switch = not self.switch

if __name__ == '__main__':
    try:
        robot_camera_watch = RobotCameraWatchAround()
        rospy.spin()  # 进入循环，等待服务和话题的请求
    except rospy.ROSInterruptException:
        pass
