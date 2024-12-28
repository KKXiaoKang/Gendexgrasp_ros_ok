# Gendexgrasp_ros
* 上肢泛化抓取仓库

# Gendexgrasp_ros
a simpe demo for ros workspace Gendexgrasp_ros

# 项目架构
* ![gendexgrasp数据流解析](./IMG/gendexgrasp数据流解析.png)

# quick start(Gen6D启动方式)
```bash
# 抓取姿态可视化Marker节点 | 目标位置可视化Marker节点 | 抓取ik逆解前姿态可视化 | 发布姿态到ik节点 | 创建灵巧手调用服务端
roslaunch grasp_ik_arm_traj robot_grasp_one_start.launch

# 启动ik节点
roslaunch motion_capture_ik visualize.launch visualize:=true robot_version:=4 control_hand_side:=0 send_srv:=0
roslaunch motion_capture_ik visualize.launch control_hand_side:=2 send_srv:=0 eef_z_bias:=-0.15 visualize:=1 enable_quest3:=0 use_cxx:=1 # 新节点

# 发布真实物体的坐标（启动相机 | 启动yolo-onnxruntime）
roslaunch grasp_ik_arm_traj sensor_robot_enable.launch

# 在线生成服务端 -- 发布ros_gendexgrasp服务端
roslaunch ros_gendexgrasp gendexgrasp_ros_service.launch

# 离线生成服务端
rosrun grasp_filter_gendex grasp_filter_node.py

# 发布物体姿态四元数(Gen6D)
cd /home/lab/GenDexGrasp/Gendexgrasp_ros/ros_vision/6DOF_Gen_ros
python3 predict_realsense.py --cfg configs/gen6d_pretrain.yaml --database custom/bottle --output data/custom/bottle/test

# 运行演示demo
cd /home/lab/GenDexGrasp/Gendexgrasp_ros/scripts
python3 demo_offline.py
```
## 编译
```bash
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
# 请注意（运行环境下的numpy环境的不同）
* 在运行drake-visualizer之前，numpy所需环境为1.20.0
* 但是在运行gendexgrasp的时候，numpy所需的版本为1.24.4

## for water_bottle_grasp
```bash
# 旧：生成抓取图（根据指定的面片索引进行接触图的生成）
python inf_cvae.py --pre_process sharp_lift --s_model PointNetCVAE_SqrtFullRobots --num_per_object 2 --comment leju

# 旧：根据抓取图不断生成抓取姿态ros信息
python run_grasp_gen_ros.py --robot_name lejuhand --max_iter 100 --num_particles 32 --learning_rate 5e-3 --init_rand_scale 0.5 --object_name contactdb+water_bottle --cmap_dir logs_inf_cvae/PointNetCVAE_SqrtFullRobots/sharp_lift/leju

# 新：统一发布ros_gendexgrasp服务端
roslaunch ros_gendexgrasp gendexgrasp_ros_service.launch
```

## 最新启动
```bash
# 引入固定物体的抓取姿态 
source /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/devel/setup.bash

# 启动桥接
roslaunch ros_bridge_python start_ros1_bridge.launch

# 启动ik
roslaunch motion_capture_ik visualize.launch visualize:=true robot_version:=4 control_hand_side:=0 send_srv:=0
roslaunch motion_capture_ik visualize.launch visualize:=false robot_version:=4 control_hand_side:=0 send_srv:=0

# 启动相机发布tf节点树（适用于ros2 isaac_ros_避障轨迹）第一步先打开
roslaunch grasp_ik_arm_traj view_d435_model.launch  # 源码有改动关闭rviz显示 
roslaunch realsense2_description view_d435_model.launch

# 启动姿态框架
roslaunch grasp_ik_arm_traj all_in_one.launch

# 启动演示脚本
cd /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/scripts
python3 demo_offline.py
```

```bash
contactdb+water_bottle_11_best_q
contactdb+water_bottle_23_best_q
contactdb+water_bottle_47_best_q
contactdb+water_bottle_126_best_q
contactdb+water_bottle_170_best_q
contactdb+water_bottle_197_best_q
```

## docker - CHANGELOG
```bash
# 环境配置
v1.0 : 
    1. 基础ubuntu20.04

# 环境配置
v2.0 :
    1. ros1 noeitc/ ros2 foxy
    2. vim
    3. x11 host / rviz 映射外部

# 环境配置
v3.0 :
    ## for kuavo_opensource 
    1. ros noetic : apriltag-ros | moveit | trac-ik
    2. apt-get install python3-pip
    3. pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
    4. pip install rospy-message-converter
    5. apt install git
    6. cassie_alip_mpc : https://github.com/UMich-BipedLab/cassie_alip_mpc.git
    7. sudo apt-get install liblcm-dev libgflags-dev libgoogle-glog-dev liblmdb-dev
    8. drake : https://drake.mit.edu/apt.html | sudo apt install drake-dev=1.19.0-1
    9. sudo apt-get install libncurses5-dev libncursesw5-dev
    10. sudo apt-get install libprotobuf-c-dev

# 环境配置
v4.0 : （在不使用姿态估计和Gendexgrasp生成的姿态下，已经基本按特定的姿态跑通demo）
    ## for ros-noetic-binary install
    1. vision_msgs | apt-get install ros-noetic-vision*
    2. rviz_visual_tools | apt-get install ros-noetic-rviz*
    3. realsense2_camera | apt-get install ros-noetic-realsense2*
    4. tf2_ros | apt-get install ros-noetic-tf2*

    ## for ros-foxy-binary install | rs-enumerate-devices
    1. realsense2 | sudo apt-get install ros-foxy-realsense*

    ## for torch - gpu
    1. pip3 install networkx
    2. pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

    ## for onnxruntime-gpu | yolov5
    1. pip install onnxruntime-gpu

    ## for yolov5
    1. pip3 install -r requirements.txt

    ## for motion_capture_ik
    1. pip3 install numpy-quaternion

    ## for demo_offline.py
    1. pip3 install rich

# 环境配置
v5.0 :
    ## 配置anaconda
    # default python path 
    export PATH=/usr/bin:$PATH
    1. bash Anaconda3-2024.10-1-Linux-x86_64.sh -b -p /opt/anaconda 
    2. export LC_ALL=C.UTF-8 | export LANG=C.UTF-8
    /opt/anaconda/bin/conda init
    conda activate

    ## for Gendexgrasp
    ### 快速 copy anaconda 环境
    1. conda env export --name gendexgrasp > gendexgrasp.yml
    ### 快速在另外一台机器上运行
    2. conda env create -f gendexgrasp.yml
    3. conda activate gendexgrasp
    4. pip3 install trimesh
    5. pip3 install plotly 
    6. pip3 install scipy
    7. pip3 install matplotlib
    8. pip3 install pytorch-kinematics
    9. pip3 install transforms3d
    10. pip3 install open3d
    11. pip3 install tensorboard
    12. pip3 install rospkg

    ## for Gen-6D-Pose-Estimation
    ### need to download VGG11_BN_Weights.DEFAULT into default torch
    1. python3 predict_realsense.py --cfg configs/gen6d_pretrain.yaml --database custom/bottle --output data/custom/bottle/test
    
```
```bash
https://drive.google.com/file/d/1wGzpZIwcARBeAmClYDmG3UqRw9YbxwUd/view?usp=sharing
```

## 带避障轨迹的启动
### 编译ros1_bridge桥接
```bash
# 编译桥接功能包
sros1
sros2
source /home/lab/GenDexGrasp/curobot_ros_ws/bridge_ws/ros1_msgs_ws/devel_isolated/setup.bash
source /home/lab/GenDexGrasp/curobot_ros_ws/bridge_ws/ros2_msgs_ws/install/setup.bash
colcon build --packages-select ros1_bridge --cmake-force-configure
ros2 run ros1_bridge dynamic_bridge --print-pairs # 打印查看成功桥接的自定义类型

# 运行桥接功能包
sros1
sros2
source ~/GenDexGrasp/curobot_ros_ws/bridge_ws/ros1_msgs_ws/devel_isolated/setup.bash
source ~/GenDexGrasp/curobot_ros_ws/bridge_ws/ros2_msgs_ws/install/setup.bash
export ROS_MASTER_URI # （选择性引入，如果bashrc有则不进行测试）
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics            # 桥接双向所有话题，有概率会和原始的tf树进行冲突
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics   # 将ros2的所有话题桥接至ros1

# 运行特定自己话题的yaml的话题
rosparam load bridge.yaml  # 规则文件 
ros2 run ros1_bridge parameter_bridge  # 运行特定桥接

rosparam delete /topics && rosparam delete /services_1_to_2 && rosparam delete /services_2_to_1 # 删除之前加载的bridge.yaml规则
```

### 启动
```bash
# 每次关机之后启动docker都要更新 + 编译 isaac_ros_nvblox + 如果添加了isaac_ros_nitros_bridge应该更新一下其依赖环境
sudo apt-get update 
rosdep update 
rosdep install -i -r --from-paths /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/ --rosdistro humble -y
colcon build --symlink-install

# 目标检测 | 姿态估计 | 模块
rosdep install --from-paths /workspaces/isaac_ros-dev/src/isaac_ros_pose_estimation/isaac_ros_foundationpose --ignore-src -y
rosdep install --from-paths /workspaces/isaac_ros-dev/src/isaac_ros_object_detection --ignore-src -y
sudo apt install ros-humble-isaac-ros-nitros-* # 更新issac-ros-nitros依赖环境

# 其他依赖库更新
sudo apt install ros-humble-rqt-tf-tree tree
colcon build --symlink-install --packages-up-to isaac_ros_nvblox
colcon build --symlink-install

# 编译功能包
colcon build --packages-skip isaac_ros_dope # 除了isaac_ros_dope都进行编译
colcon build --packages-select gxf_isaac_utils
colcon build --packages-skip gxf_isaac_utils

# isaac_ros_foundationpose(选做) -- 将模型从 转换.etlt为 TensorRT 引擎计划
## 转换优化模型：
/opt/nvidia/tao/tao-converter -k foundationpose -t fp16 -e ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan -p input1,1x160x160x6,1x160x160x6,252x160x160x6 -p input2,1x160x160x6,1x160x160x6,252x160x160x6 -o output1,output2 ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_model.etlt
## 转换评分模型：
/opt/nvidia/tao/tao-converter -k foundationpose -t fp16 -e ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan -p input1,1x160x160x6,1x160x160x6,252x160x160x6 -p input2,1x160x160x6,1x160x160x6,252x160x160x6 -o output1 ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_model.etlt
## 更新
sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-realsense
## 启动foundationPose
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_mono_rect_depth,foundationpose mesh_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Mac_and_cheese_0_1/Mac_and_cheese_0_1.obj texture_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Mac_and_cheese_0_1/materials/textures/baked_mesh_tex0.png score_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan refine_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan rt_detr_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan
## 启动Rviz进行输出
rviz2 -d  $(ros2 pkg prefix isaac_ros_foundationpose --share)/rviz/foundationpose_realsense.rviz
## 转发姿态节点
ros2 run biped_s42_foundation_tp detection3d_listener
## 临时发布头部关节角度节点
ros2 topic pub /robot_head_motion_data kuavo_robot_interfaces/msg/RobotHeadMotionData "joint_data: [0, -30]"  

# 避障项目必开：避障项目启动顺序(docker内部) - biped-S42pro版本
ros2 launch biped_head_sensor_pub head_state_publisher_s42.launch.py
ros2 launch nvblox_examples_bringup realsense_example.launch.py 
ros2 launch biped_s42_moveit_config biped_s42_moveit.launch.py
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args -p urdf_path:=/workspaces/isaac_ros-dev/src/biped_s42/urdf/biped_s42_v4_left_arm.urdf -p robot:=/workspaces/isaac_ros-dev/src/biped_s42/urdf/biped_s42.xrdf

# 避障项目必开：避障项目启动顺序(docker内部) - biped-S40版本
ros2 launch biped_head_sensor_pub head_state_publisher_s40.launch.py
ros2 launch nvblox_examples_bringup realsense_example.launch.py 
ros2 launch biped_s40_moveit_config biped_s40_moveit.launch.py 
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args -p urdf_path:=/workspaces/isaac_ros-dev/src/biped_s40/urdf/biped_s40_left_arm.urdf -p robot:=/workspaces/isaac_ros-dev/src/biped_s40/urdf/biped_s40.yml

# 避障项目必开：ros1_bridge规定桥接触发（宿主机）
cd /home/lab/GenDexGrasp/Gendexgrasp_ros_ok
source devel/setup.bash
roslaunch ros_bridge_python start_ros1_bridge.launch

# 触发机器人头部旋转触发周围环境采集的demo（宿主机）
cd /home/lab/GenDexGrasp/Gendexgrasp_ros_ok
source devel/setup.bash
python3 scripts_avoidance/example/demo_avoidance.py 
rosservice call /ros1_collect_the_environment "{}"

# ros1的机器轨迹中转发送给机器人（宿主机）
cd /home/lab/GenDexGrasp/Gendexgrasp_ros_ok
source devel/setup.bash
python3 scripts_avoidance/demo_avoidance.py 
```

## 行为树简化流程
* 行为树更新
```bash
# 安装
wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-linux-installer.run
sudo chmod +x Groot2-v1.6.1-linux-installer.run
./Groot2-v1.6.1-linux-installer.run

# 更新路径（一键启动打开）
vim ~/.bashrc
alias Groot="./Groot2/bin/groot2"
```