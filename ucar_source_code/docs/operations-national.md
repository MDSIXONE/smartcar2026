# 国赛主流程操作文档：ucar_2026_national

这是国赛地图和新版巡线参数对应的主流程。新电脑、WSL、网络、防火墙、bridge 和
COPY MODE 只需按 [部署文档](deployment.md) 配置一次；启动顺序仍是仿真 Master →
Gazebo/RViz → bridge → 小车安全检查 → mission。

## 1. 国赛流程的区别

- 入口包：ucar_2026_national。
- 地图：iflysse_field_walls_national.yaml。
- 终点前往交接区域仍由主流程导航完成；交接后常驻 lane_proto 直接接管终点巡线和地图坐标定点停车，启动时不直接驱动巡线。
- 当前国赛交接启用 is_fork=yolo、板检测和绕板：board_in_lane=true、
  go_around=true、board_stop_dist=0.321、go_around_keepout=0.08、
  board_arc_lat_scale=0.3。
- 当前巡线速度参数为 linear_speed=0.2、gain=1.2、rate=20、goal_pause=1.0；
  `use_lidar=true` 保持终点地图定点闭环，物理左支路到点 120，物理右支路到点 111，
  中间支路到点 111，左右航向 `-90°`、中间航向 `180°`。

## 2. 仿真端

三步启动已合并为一个 WSL 终端命令。脚本会显式使用仿真 Master
`127.0.0.1:11312`，并在 `/map` 就绪后启动 bridge；GUI 启动前后自动执行 COPY MODE 预检：

~~~bash
cd ~/smartcar2026/simulation
bash scripts/start_simulation_stack.sh
~~~

无界面联调：

~~~bash
bash scripts/start_simulation_stack.sh --headless
~~~

必须看到终端单独输出 `OK`，同时 bridge 出现 SIMULATION_BRIDGE_READY 和 state=waiting，并能读到：

~~~bash
rostopic echo -n 1 /map
curl -sS http://<PC_LAN_IP>:11313/status
~~~

## 3. 国赛车端安全检查

先检查电脑 bridge 和车端 Python2/ROS 环境：

~~~bash
bash ~/ucar_ws/src/ucar_2026_national/scripts/start_2026.sh <PC_LAN_IP> check
~~~

启动无自动目标检查模式：

~~~bash
bash ~/ucar_ws/src/ucar_2026_national/scripts/start_2026.sh <PC_LAN_IP> manual
~~~

另开车端终端设置本次车端 Master：

~~~bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=<VEHICLE_IP>
export ROS_MASTER_URI="http://$ROS_IP:11311"
rostopic echo -n 3 /odom_raw
timeout 5 rosrun tf tf_echo odom base_link
timeout 5 rosrun tf tf_echo map base_link
rostopic hz /scan
~~~

只有 /odom_raw 有限、两个 TF 正常、/scan 稳定、无 TF_NAN_INPUT、head_len 或
sensor not active，且车辆已回到起点，才能停止 manual 并进入 mission。单帧
`check crc16 faild(imu)` 由任务记录告警，不单独作为中止条件；发现 NaN
或 TF 错误时先零速和重启底盘/定位链路，不得继续测试。

## 4. 启动国赛主流程

确认 manual 已完全退出后执行：

~~~bash
bash ~/ucar_ws/src/ucar_2026_national/scripts/start_2026.sh <PC_LAN_IP> mission
~~~

起点确认提示只在车辆真实摆放正确时输入 yes。随后按语音提示说“小飞小飞”和
两个不同的物品类别。主流程按二维码、生产 OCR、停靠、仿真联动和终点交接顺序运行。
终点交接时观察：

~~~bash
rostopic echo /lane_proto/state
rostopic echo /lane_proto/result
~~~

首次相位应进入黄线对齐/起跑序列，而不是直接把起跑黄线识别成终点并 STOPPED。正常终点应观察到
`FOLLOW -> CORNER_ADJUST -> STOPPED` 且 `/lane_proto/result=GOAL`；只有 `GOAL` 才会让主流程发布
`SUCCEEDED`，随后播报“任务完成”。雷达数据不可用或角落闭环失败时，节点保持零速并按
`CONFIG/ABORT` 处理，不得继续任务。
板检测或绕板失败时立即停车，保留终端日志，不要反复重新激活巡线节点。

### 任务完成后给裁判展示仿真场景

国赛主流程完成并播报任务结束后，如需重新随机生成仿真物块和锥桶供裁判查看，
保持电脑 WSL 中的仿真一键启动脚本和 Gazebo 继续运行，在电脑上新开一个 WSL
终端执行。以下命令不能在小车终端执行，也不能只复制最后一行；小车使用
`11311`，仿真使用独立 Master `11312`：

~~~bash
cd ~/smartcar2026/simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
rostopic list
~~~

确认 `rostopic list` 能列出仿真话题后，再执行：

~~~bash
rosrun car3 spawn_cubes.py
~~~

看到物块坐标、`锥桶: 10/10 个` 和 `完成` 后，即可让裁判查看 Gazebo 窗口中的
新场景。该命令会先删除旧的 `cube_*` 和 `cone_*`，再生成一组新的随机场景，
不会复位小车或重新执行国赛任务。若提示 `Unable to register with master node`
并指向 `11311` 或 `11312`，先回到仿真一键启动终端确认 ROS Master 仍在运行，
不要反复重试 `rosrun`。

## 5. 停止和清理

~~~bash
bash ~/ucar_ws/src/ucar_2026_national/scripts/stop_2026_task.sh
~~~

然后在一键启动脚本所在终端按 Ctrl-C，脚本会依次停止 bridge、Gazebo/RViz 和仿真 Master。
不要同时保留标准、国赛、额外三个主流程；它们会争用底盘、相机和 /cmd_vel。

## 6. 国赛流程故障处理

| 现象 | 处理 |
| --- | --- |
| WARN:COPY MODE / WORN COPY MODE | 关闭 GUI，检查 weston use_gfxredir = 0 计数必须为 0、/mnt/shared_memory 必须为 tmpfs；先 wsl --terminate Ubuntu-20.04，仍失败再 wsl --shutdown。 |
| 11313 不通 | PC_LAN_IP 必须是 Windows 局域网 IPv4；bridge 必须监听 0.0.0.0:11313；检查 LocalSubnet 防火墙和 NAT portproxy。 |
| bridge 报 `Address already in use` | 先停止原仿真一键启动终端，确认旧 bridge 已退出，再只启动一份脚本；确认 `/status` 返回 `state=waiting` 后才能启动车端 mission。 |
| /map 没有数据或 bridge 不为 waiting | 一键脚本的 ROS_MASTER_URI 必须是 127.0.0.1:11312；先修复仿真，不要启动车端 mission。 |
| /odom_raw 为 NaN、TF 报 TF_NAN_INPUT | 立即零速并停止；重启底盘/定位链路，确认 odom、odom -> base_link、map -> base_link 均恢复有限值后再开始。 |
| 日志有 `check crc16 faild(imu)` | 任务记录限频告警并继续；若伴随 `imu sensor not active`、非有限 odom/TF 或其他底盘链路故障，停止流程检查 CP2102/USB Hub/串口和供电。 |
| 其他 crc16、head_len、sensor not active | 停止流程，检查 CP2102/USB Hub/串口和供电；国赛巡线不能掩盖底盘链路故障。 |
| 交接后立即 STOPPED | 检查车端 lane_proto 和国赛 2026.launch 是否为当前版本，重点核对 is_fork=yolo、board_in_lane、go_around 参数；不在现场临时手改参数后继续跑。 |
| 巡线 Python2 logging 或 cv_bridge 导入错误 | 只能用 ROS Melodic Python2 启动器；不要直接用 Python3 执行 lane_follow.py。 |
| `production_task_2026` 立即 exit code 1 | 用 `run_melodic_python2.sh` 做入口脚本导入检查；若报 `cannot import name shortest_yaw_delta`，说明任务脚本与 `production_task_geometry.py` 版本不一致。同步成套文件后必须重启整套国赛 launch。 |
| `post_turn_recenter_trigger` 参数校验失败 | 当前 `arrival_tolerance=0.12`、`post_turn_recenter_trigger=0.06`，后者必须保持为正且小于前者。修改后必须重启 launch，运行中的参数不会热更新。 |
| `move_base` 显示 `goal reached`，随后出现 `stopped ... from target` | 确认运行的是最新任务脚本并已重启正式 launch。当前版本会对 action 成功但位姿复核超限重发 3 次，仍超限记录 `PRODUCTION_TASK_ARRIVAL_CONTINUE` 后继续；若仍出现旧的 `PRODUCTION_TASK_ABORTED`，说明车上还是旧进程或旧文件。 |
