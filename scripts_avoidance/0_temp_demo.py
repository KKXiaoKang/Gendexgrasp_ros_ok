import threading
import time

def cuMotion_planning_thread():
    """
    这是执行cumotion轨迹规划的线程函数
    """
    global CUMOTION_FINSHED_FLAG
    CUMOTION_FINSHED_FLAG = False  # 规划未完成
    call_cumotion_service(True, global_robot_new_real_target_state_joint, global_robot_real_zero_state_joint)
    
    while not CUMOTION_FINSHED_FLAG:
        time.sleep(0.5)
        CUMOTION_FINSHED_FLAG = call_cumotion_service(False, global_robot_real_target_state_joint, global_robot_real_zero_state_joint)  # 查询规划状态

def gendexgrasp():
    # 其他流程...
    
    # （0） 观察周围环境
    call_collect_service()
    arm_mode = False
    call_change_arm_ctrl_mode_service(arm_mode)  # 关闭手臂规划模式 | 使用ArmTargetPose控制手臂
    robot_instance.srv_controlEndHand(zero_traj_data)  # 手指放平
    PLAN_ROBOT_ARM_STATUS_FLAG = True  # 手臂状态为Go状态
    time.sleep(3)
    
    # （1） 设置手臂初始位置(目前demo展示单手先去到抓取位置)
    if IF_USE_PRE_POSE_FLAG:  # 如果要使用预定准备的姿态，则需要来到这个位置
        # robot_arm_action(robot_instance, 0, "zero_go_to_prepare") # 初始位置有所改变，需要更新global_robot_real_init_state_joint
        call_cumotion_service(True, global_robot_real_init_state_joint, global_robot_real_target_state_joint)
    
    # input( " 请等待机器人robot arm 去到prepare位置 ---------完成后Enter键继续")
    time.sleep(2)

    # （2） 调用/gendex_grasp_service 开始生成姿态 | 默认使用42的随机种子 | 
    ik_global_setting_service(0)  # 全局ik状态为0
    button_to_ik_monitor_service(1)  # 打开IK监听
    handleOfflineGraspButton(1)  # 开启生成姿态
    time.sleep(1)

    # （3）等待IK_SUCCESS_FLAG为真
    while not IK_SUCCESS_FLAG:
        time.sleep(0.1)

    # 开启cumotion轨迹规划线程
    planning_thread = threading.Thread(target=cuMotion_planning_thread)
    planning_thread.start()

    # 继续执行其他任务（例如其他步骤）
    # 可以在此处执行其他步骤的逻辑，而不用等待轨迹规划完成
    
    # （5） 如果成功调用/gendex_grasp_service 停止生成姿态 | 并且打印ik序号 | 并且按时用户是否继续 
    time.sleep(0.5)
    CUMOTION_FINSHED_FLAG = False  # 规划未完成
    while not CUMOTION_FINSHED_FLAG:
        time.sleep(0.5)  # 查询时间快一些
        rospy.loginfo(f"查询cumotion轨迹规划完成状态: {CUMOTION_FINSHED_FLAG}")
        CUMOTION_FINSHED_FLAG = call_cumotion_service(False, global_robot_real_init_state_joint, global_robot_real_target_state_joint)  # 查询规划状态

    # 等待线程完成
    planning_thread.join()  # 确保规划线程完成后再继续执行其他步骤
    # 继续后续流程...
