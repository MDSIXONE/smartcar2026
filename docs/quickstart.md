# 快速操作（环境已配置）

本页只保留日常启动命令。唯一 ROS Master 为本机 WSL Ubuntu 20.04 的
`http://192.168.8.197:11311`；不要在小车 `192.168.8.231` 上执行 `roscore`。

> 当前进行**真机测试**时，只执行下面“真机 2026 测试”章节的命令。不要同时启动
> Gazebo、仿真 RViz 或 `task3_prepare.launch`。

## 真机 2026 测试（当前使用）

### 1. WSL：启动唯一 ROS Master

在 WSL Ubuntu 20.04 的第一个终端输入：

```bash
~/start_ros_master.sh
```

### 2. 小车：启动 RViz 定点导航

确认旧的 `2026.launch` 已停止后，在**小车终端**输入。该入口不会启动
二维码、语音、任务脚本或默认导航目标：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch
```

启动后不需要发送任何目标。`navigation_scan_relay` 若遇到一次瞬态启动失败会在 2 秒后
自动重试；它恢复 `/scan` 后，`lidar_loc` 才能发布 `map -> odom`。看到启动初期的
`base_link to map` 等待警告时，先等待 relay、激光和定位就绪，不要发送 RViz 目标。

### 3. WSL：打开真机 RViz

在 WSL Ubuntu 20.04 的第二个终端输入：

```bash
RVIZ_CONFIG=/mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/ucar_source_code/ucar_ws/src/yolo2025/rviz/navigation_2026.rviz \
  ~/start_rviz.sh
```

必须显式设置 `RVIZ_CONFIG`，防止 `start_rviz.sh` 的历史默认值打开其他工作区。
该 `navigation_2026.rviz` 用于显示真机的 `/map`、激光、
全局/局部代价地图、车体朝向、尺寸匹配的蓝色简化车体模型，以及 CymPlanner
前视车体轮廓和 `/usb_cam/image_raw` 相机画面；它不是仿真 RViz。

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

如果误关了启动终端、任务仍在运行，在**任意新的小车终端**执行下面这一条；它会先发布
零速度，再只停止 `yolo2025 2026.launch` 及其子节点，**不会**停止 WSL Master：

```bash
bash ~/ucar_ws/src/yolo2025/scripts/stop_2026_task.sh
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
export ROS_MASTER_URI=http://192.168.8.197:11311
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
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
