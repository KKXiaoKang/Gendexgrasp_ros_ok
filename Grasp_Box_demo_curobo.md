# 搬箱子 - curobo启动指南
```bash
# cumotion 服务端 - ros2环境
ros2 launch biped_s42_cumotion nvblox_moveit_server.launch.py use_foundation_pose:=false
ros2 launch biped_s42_cumotion kuavo_cuMotion.launch.py

# cumotion 服务端 - ros1环境
cd /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/ros_bag_end_curobo
python3 avoid_curobo_service_topic.py  # 根据轨迹的话题进行转发 + kuavo_arm_traj控制
roslaunch grasp_ik_arm_traj robot_arm_catch_demo.launch # 桥接轨迹话题
 
roslaunch motion_capture_ik visualize.launch visualize:=false robot_version:=4 control_hand_side:=0 send_srv:=0 # 用于ik求解 可用

# 机器人客户端
调用cumotion接口
```

# 搬箱子 - curoboROS服务器启动指南
```bash
# (ROS2主机)打开nvblox-ROS2服务
ros2 launch biped_s42_cumotion nvblox_moveit_server.launch.py use_foundation_pose:=false

# 启动桥接
roslaunch grasp_ik_arm_traj robot_arm_catch_demo.launch 

# 启动避障curobo服务器
rosrun curobo_trajectory_server trajectory_server.py 

# 启动插值模块
rosrun curobo_trajectory_server avoid_curobo_service_topic.py 

# 运行一个简单的调取案例
cd /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/src/ros_traj_plan/curobo_trajectory_server/example_client
python3 client_curobo.py
```