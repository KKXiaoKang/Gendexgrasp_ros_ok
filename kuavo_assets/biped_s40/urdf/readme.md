```bash
biped_s4.urdf  # 带灵巧手 + 头 的 从torso描述的全身

biped_s4_left_arm.urdf # 不带灵巧手，只有躯干+左手的描述（用于cumotion)
biped_s4_left_arm.usd
biped_s4.xrdf # 不带灵巧手，只有躯干+左手，添加球体碰撞描述（用于cumotion)

biped_s4_all_arm.urdf # 用于Moveit2描述的，只有躯干+双手

HEAD_URDF_new.urdf # 用于nvblox描述从相机坐标系下torso位置的，只有头+躯干，从头描述的躯干的urdf文件(biped_head_sensor_pub)
```