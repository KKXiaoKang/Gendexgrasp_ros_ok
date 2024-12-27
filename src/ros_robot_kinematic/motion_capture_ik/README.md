# 说明
## 遥操作
请参考[kuavo_ros1_workspace仓库](https://www.lejuhub.com/highlydynamic/kuavo_ros1_workspace)的readme文件以使用遥操作。

## 调用ik求解
- 启动`motion_capture_ik`节点:
```bash
cd <path-to-kuavo-ws>
source devel/setup.bash
roslaunch motion_capture_ik visualize.launch visualize:=false robot_version:=4 control_hand_side:=2 send_srv:=1
```
> Note: 其中：visualize 置为 false 关闭可视化; robot_version 指代手臂版本，3代表 3.4 版本的手臂，4代表 4.0 版本的手臂；control_hand_side 指定控制的手，0表示左手，1表示右手，2表示双手；send_srv 置为 0 表示跳过ros服务调用（该服务用于启动机器人手臂控制），1表示调用ros服务，如果只想查看rviz可视化内容，请置为 0，否则程序会卡在等待ros服务response的地方。

- 参考`./scripts/test/sim_ik_cmd.py`文件发布末端位姿命令
