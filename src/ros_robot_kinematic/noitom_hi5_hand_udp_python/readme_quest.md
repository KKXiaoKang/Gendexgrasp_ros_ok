# 相关资源

md5 1c643226cd5ffb064243d62a5e881099 https://kuavo.lejurobot.com/Quest_apks/leju_kuavo_hand-0.0.1-40-g8025c10.apk

# 使用方法 

1. 在 Quest3 上安装 leju_kuavo_hand.apk
2. 查看 Quest3 设备的 IP
3. 运行 ROS 节点的机器需要和 Quest3 在同一个局域网内, 需要安装的依赖在 requirements.txt
4. 启动监听脚本，比如： python ./scripts/monitor_quest3.py 10.10.20.120 这个 IP 需要根据实际情况更新
5. 带上 Quest3 点击安装好的程序，启动。然后就会有一个 topic: `/leju_quest_bone_poses` 发布所有关节的信息。这些信息是一个序列，顺序请参考: bone_name_to_index

# Motion Capture IK 使用方法

闭源：
[使用方法](https://www.lejuhub.com/highlydynamic/motion_capture_ik/-/blob/develop/README.md)

已经打包好：
[使用方法](https://www.lejuhub.com/highlydynamic/motion_capture_ik_packaged/-/blob/develop/README.md)



# 更新记录

1. 添加头和脖子的 Bone 的数据
2. 添加了两个手柄的数据
