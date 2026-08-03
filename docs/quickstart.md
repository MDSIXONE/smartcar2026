# 快速操作（环境已配置）

本页只保留日常启动命令。唯一 ROS Master 为本机 WSL Ubuntu 20.04；当前地址与
小车端配置以 [rosmaster/NETWORK_CONFIGURATION.md](../rosmaster/NETWORK_CONFIGURATION.md)
为准，小车端不得执行 `roscore`。

> 当前进行**真机测试**时，只执行下面“真机 2026 测试”章节的命令。不要同时启动
> Gazebo、仿真 RViz 或 `task3_prepare.launch`。

> `simulation_real` 是仅包含真机源码的发布分支。根目录的 `simulation/` 工作区
> 已被忽略且不在该分支中；需要仿真时使用独立的本机仿真工作区，不能将其加入本分支。

## 真机 2026 测试（当前使用）

### 1. WSL：启动唯一 ROS Master

在 WSL Ubuntu 20.04 的第一个终端**完整复制下面两行**：

```bash
unset ROS_IP ROS_HOSTNAME ROS_MASTER_URI
~/start_ros_master.sh
```

启动后必须查看第一行，例如：

```text
Starting ROS Master at http://192.168.8.199:11311 (ROS_IP=192.168.8.199)
```

记住中间的地址；本例是 `192.168.8.199`。如果显示 `localhost` 或 `127.0.0.1`，
立即按 `Ctrl-C` 停止，不能继续启动小车。

### 2. 小车：一条命令启动无自动目标导航

确认旧的 `2026.launch` 已停止后，在**小车终端只复制下面这一行**：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh
```

看到提示：

```text
请输入 WSL Master 地址（例如 192.168.8.199）:
```

只输入第 1 步记下的地址并按回车。不要把提示文字、引号或其他命令一起粘贴。
脚本会自动加载 ROS、计算小车 IP 并检查 Master，不需要手动输入 `source`、`export`
或 `awk`。它还会先用 Python 2 导入 Melodic 的 `tf`；若检测到 Python 3 构建产物
污染，会直接报错停止，不会让 `navigation_scan_relay` 无限重启。正常输出应类似：

```text
网络检查结果：
  WSL Master = http://192.168.8.199:11311
  小车 ROS_IP = 192.168.8.231

Master 连接成功。正在启动无自动目标的导航任务……
```

该入口不会启动二维码、语音、完整任务或默认导航目标。小车只连接 WSL Master，
脚本绝不会在小车启动 `roscore`。

启动后不需要发送任何目标。`navigation_scan_relay` 若遇到一次瞬态启动失败会在 2 秒后
自动重试；它恢复 `/scan` 后，`lidar_loc` 才能发布 `map -> odom`。看到启动初期的
`base_link to map` 等待警告时，先等待 relay、激光和定位就绪，不要发送 RViz 目标。

### 3. WSL：打开真机 RViz

在 WSL Ubuntu 20.04 的第二个终端输入：

```bash
unset ROS_IP ROS_HOSTNAME ROS_MASTER_URI
RVIZ_CONFIG=/mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/ucar_source_code/ucar_ws/src/ucar_2026/rviz/navigation_2026.rviz \
  ~/start_rviz.sh
```

必须显式设置 `RVIZ_CONFIG`，防止 `start_rviz.sh` 的历史默认值打开其他工作区。
该 `navigation_2026.rviz` 用于显示真机的 `/map`、激光、
全局/局部代价地图、车体朝向、尺寸匹配的蓝色简化车体模型，以及 CymPlanner
原始 CV Map/Plan 和 `/usb_cam/image_raw` 相机画面；它不是仿真 RViz。

### 4. 真机安全检查

在小车终端确认以下数据正常后，才在 WSL 的 RViz 使用 **2D Nav Goal**：

```bash
rostopic echo -n 1 /odom_raw
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map base_link
```

如果 `/odom_raw` 出现 `NaN` 或 TF 报 `TF_NAN_INPUT`，不要发送目标；先发布零速度并
重启导航/里程计链路。

相机画面由 `usb_cam` 单独发布，不参与控制或导航目标。RViz 的 **UCar Camera
(/usb_cam/image_raw)** 面板应显示画面；若为空，先确认：

```bash
rostopic info /usb_cam/image_raw
```

预期 publisher 为 `/usb_cam`。相机节点会在设备瞬态失败后每 2 秒自动重试。

CymPlanner 默认使用原始“前视路径点”碰撞检查。需要把同一批前视点扩大为完整
车体面积时，在小车终端复制：

```bash
rostopic pub -1 /ucar/navigation_mode std_msgs/String "data: 'body_projection'"
```

恢复原始路径点模式：

```bash
rostopic pub -1 /ucar/navigation_mode std_msgs/String "data: 'point'"
```

这个切换只改变碰撞检查面积，不会切换巡线控制器或参数。车体投影读取
`/move_base/global_costmap/costmap`：全局代价 253 的膨胀区不会当成车体已经接触，
只有 254 致命障碍格会判为实际碰撞；未知地图区域仍会安全停车。RViz 中的
**CymPlanner CV Map** 和 **CymPlanner CV Plan** 只有在 move_base 收到路径、开始调用
局部规划器后才会刷新。

当前真机导航使用 `iflysse_field_walls_without_middle_vertices.yaml`（中间没有顶点）地图；
当前配置档名为 `testnav20260721`；全局代价地图膨胀半径为 `0.205 m`，局部滚动窗口为
`2.0 m × 2.0 m`。

若等待 15 秒后仍没有 `map -> base_link`，保持不发送目标，在小车终端执行以下只读检查并
保留 relay 退出行：

```bash
rosnode list | grep -E '^/(navigation_scan_relay|lidar_loc|move_base)$'
rostopic info /scan
timeout 5 rosrun tf tf_echo map base_link
```

正常停止真机导航：在运行 `2026.launch` 的小车终端按 `Ctrl-C`。停止 RViz：在运行
`start_rviz.sh` 的 WSL 终端按 `Ctrl-C`。

### 5. 最快的正式任务启动

先用默认 `manual` 模式完成第 4 节全部安全检查，并确认底盘日志没有 `NaN`、
`sensor not active`、`head_len error`、`crc16` 或 `No such device`。还要确认：

```bash
test -e /dev/ttyUSB0 -o -e /dev/ucar_controller_serial_port && echo 底盘串口正常
```

停止 manual launch 后，执行：

```bash
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh "$MASTER_IP" mission
```

`mission` 模式不启动 RViz，执行 `52 → 扫描 262/232/295 → 释放 ROS usb_cam →
原生相机 OCR → 车体投影 → 3/2/1/11/21/31/32/33/34/4/5/6/7/8/9/10/20/30/40/39/38/37`。
每个生产点都沿下一状态点方向拍照；OCR 框通过纯旋转对齐画面水平中心后，用新鲜前向
雷达和扫描时刻 TF 匹配 JSON 墙点。历史 yolo2025 流程仍保留为 `full` 模式，但不属于
这条正式任务。每次观察后若离当前中心点超过 `0.06 m`，
任务会自动用 `move_base` 回中，最多尝试 2 次；回中失败或仍超过 `0.10 m`
只在终端显示警告，任务继续前往后续点位。NaN、TF 失效和传感器掉线仍属于必须停车的
硬安全故障。

运行时只需要看两个话题：

```bash
rostopic echo /ucar_2026/task_state
rostopic echo /ucar_2026/task_result
```

成功时最终状态为 `SUCCEEDED`，结果 JSON 中 `success` 为 `true`，并在
`~/.ros/ucar_2026_observations/run_*/observations.json` 保存三个不同墙点及 OCR 内容。
生产阶段 `/usb_cam` 节点消失是为了释放 `/dev/ucar_video` 的预期行为。若搬动小车、拔插
CP2102 或改变起点，必须先停止任务并重新执行 manual 静态安全检查；不能沿用移动前的
定位坐标继续任务。

旧路线在 2026-07-28 的历史实车验收已经跑通：三个二维码依次为 `…/a`、`…/d`、`…/i`，
12/24/16/28/19 均到达并完成 360°，最终 170 到达误差 `0.018 m`，朝向 319
校验通过，任务结果为 `success: true`。该结果不等于本次 OCR 新路线已完成真机验收。

如果误关了启动终端、任务仍在运行，在**任意新的小车终端**先提供当前 WSL Master
地址，再执行停止脚本。它会先发布零速度，再只停止 `ucar_2026 2026.launch` 及其
兼容 wrapper 子节点，**不会**停止 WSL Master：

```bash
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export MASTER_IP
bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
```

若 `rosnode list` 还显示旧节点但 `rosnode info <节点名>` 报 `connection refused`，它们是
失效注册；可在确认列出的都是旧 2026 节点后执行 `rosnode cleanup` 并输入 `y` 清理。

## 本机仿真（不要与真机测试同时运行）

终端 1（WSL Ubuntu 20.04）：

```bash
~/start_ros_master.sh
```

终端 2（WSL Ubuntu 20.04）：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source ~/.config/smartcar/ros_network.sh
roslaunch car3 task3_prepare.launch gui:=true rviz:=true
```

仅检查、不开窗口时，将最后一行改为：

```bash
roslaunch car3 task3_prepare.launch gui:=false rviz:=false
```

停止仿真：在运行 `roslaunch` 的终端按 `Ctrl-C`。

## 快速确认局部代价地图

```bash
rosparam get /move_base/local_costmap/footprint
rosparam get /move_base/local_costmap/width
rosparam get /move_base/local_costmap/height
rosparam get /move_base/local_costmap/inflation_layer/inflation_radius
```

预期足迹为 `[[0.171, -0.128], [0.171, 0.128], [-0.171, 0.128], [-0.171, -0.128]]`，
局部窗口为 `2.0 m × 2.0 m`，膨胀半径为 `0.07`。

更完整的部署、回滚和排障说明见 [operations.md](operations.md)。
