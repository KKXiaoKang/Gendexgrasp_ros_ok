# ros_reuleaux_ws
a simple demo for use ros_reuleaux_ws to show robot worksapce | IK Fast 

# how to build
## 编译使用
catkin build map_creator biped_s4 workspace_visualization

## 生成dae文件
rosrun collada_urdf urdf_to_collada biped_s4_left_arm.urdf biped_s4_left_arm.dae
rosrun moveit_kinematics round_collada_numbers.py biped_s4_left_arm.dae biped_s4_left_arm.dae 5

## ik-fast 文件生成 | 借助已有的镜像
### 进入镜像
xhost + && sudo docker run  -it --rm  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/snd -e DISPLAY=unix$DISPLAY  -v `pwd`:`pwd`  -w `pwd` fishros2/openrave

### 先查看joint和link的信息
openrave-robot.py biped_s4_left_arm.dae --info links
openrave-robot.py biped_s4_left_arm.dae --info joints

### 生成可达图
* 当以l_hand_end_virtual为末端时，指定index为9进行可达图生成，可以看到如下结果
    -![末端为l_hand_end_virtual的可达图](./pics/reachMap_1_9_6.png)
```bash
## 4biped机器人
### 1 9 8 # 失败
### 1 9 7 # 失败
### 1 9 6 测试中 | 成功有效果
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=biped_s4_left_arm.dae --iktype=transform6d --baselink=1 --eelink=9 --freeindex=6 --savefile=$(pwd)/ikfastBiped.cpp
```
* 当以l_hand_roll为末端时，指定index为8进行可达图生成，可以看到如下结果
    - ![末端为l_hand_roll的可达图](./pics/reachMap_1_8_6.png)
```bash
### 1 8 6 测试中 | 感觉末端ik的结果不是很对
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=biped_s4_left_arm.dae --iktype=transform6d --baselink=1 --eelink=8 --freeindex=6 --savefile=$(pwd)/ikfastBiped3.cpp

## 4biped_pro机器人
### 1 9 6 
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=biped_s42_pro_left.dae --iktype=transform6d --baselink=1 --eelink=9 --freeindex=6 --savefile=$(pwd)/ikfastBiped.cpp
```

## 有了ikfast，开始生成可达图
- rosrun map_creator create_reachability_map 0.05
- rosrun map_creator create_reachability_map 0.08

### 创建全局可达
- rosrun map_creator create_reachability_map_service 0.05 biped_s4_pro_all_196_0.05.h5 

- rosrun map_creator create_reachability_map_service 0.05 biped_s4_all_196_0.05.h5 
- rosrun map_creator create_reachability_map_service 0.08 biped_s4_all_196_0.08.h5 

### 创建特殊可达
- rosrun map_creator create_reachability_map 0.05 biped_s4_spe_196_0.05.h5 
- rosrun map_creator create_reachability_map 0.08 biped_s4_spe_196_0.08.h5 

## 查看可达图
- h5dump /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_186_0.08_2.h5

## 加载/发布 可达图
- rosrun map_creator load_reachability_map /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_196_0.05.h5
- rosrun map_creator load_reachability_map /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_spe_196_0.05.h5

* 终端log link 和 joint信息
```bash
root@9c2c477c1ff8:/home/lab/ros_reuleaux_ws/src/biped_s4/urdf# openrave-robot.py biped_s4_left_arm.dae --info links
name               index parents        
----------------------------------------
world_link         0                    
base_link          1     world_link     
l_arm_pitch        2     base_link      
l_arm_roll         3     l_arm_pitch    
l_arm_yaw          4     l_arm_roll     
l_forearm_pitch    5     l_arm_yaw      
l_hand_yaw         6     l_forearm_pitch
l_hand_pitch       7     l_hand_yaw     
l_hand_roll        8     l_hand_pitch   
l_hand_end_virtual 9     l_hand_roll    
----------------------------------------
name               index parents  
```

* 4pro机器人
```bash
root@bd0a63d62c17:/home/lab/ros_reuleaux_ws/src/biped_s42/urdf# openrave-robot.py biped_s42_pro_left.dae --info links
name                 index parents     
---------------------------------------
world_link           0                 
base_link            1     world_link  
zarm_l1_link         2     base_link   
zarm_l2_link         3     zarm_l1_link
zarm_l3_link         4     zarm_l2_link
zarm_l4_link         5     zarm_l3_link
zarm_l5_link         6     zarm_l4_link
zarm_l6_link         7     zarm_l5_link
zarm_l7_link         8     zarm_l6_link
zarm_l7_end_effector 9     zarm_l7_link
---------------------------------------
name                 index parents    


root@bd0a63d62c17:/home/lab/ros_reuleaux_ws/src/biped_s42/urdf# openrave-robot.py biped_s42_pro_left.dae --info joints
name                       joint_index dof_index parent_link  child_link           mimic
----------------------------------------------------------------------------------------
zarm_l1_joint              0           0         base_link    zarm_l1_link              
zarm_l2_joint              1           1         zarm_l1_link zarm_l2_link              
zarm_l3_joint              2           2         zarm_l2_link zarm_l3_link              
zarm_l4_joint              3           3         zarm_l3_link zarm_l4_link              
zarm_l5_joint              4           4         zarm_l4_link zarm_l5_link              
zarm_l6_joint              5           5         zarm_l5_link zarm_l6_link              
zarm_l7_joint              6           6         zarm_l6_link zarm_l7_link              
world_joint                -1          -1        world_link   base_link                 
zarm_l7_joint_end_effector -1          -1        zarm_l7_link zarm_l7_end_effector      
----------------------------------------------------------------------------------------
name                       joint_index dof_index parent_link  child_link           mimic
```