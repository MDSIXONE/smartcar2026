# 操作命令

日常启动只需要查看 [quickstart.md](quickstart.md)；本文件保留部署、回滚和故障排查的完整命令。

## WSL ROS Master 与 RViz

> **网络配置以 [rosmaster/NETWORK_CONFIGURATION.md](../rosmaster/NETWORK_CONFIGURATION.md) 为准。** WSL Ubuntu 20.04 是唯一 ROS Master；小车端不得启动 `roscore`。所有 `source` 完成后必须设置当前 `ROS_MASTER_URI`，小车 `ROS_IP` 由到 Master 的路由动态推导或显式配置。

本机 WSL 的 Ubuntu 20.04 使用 ROS Noetic，只作为局域网 ROS Master 和 RViz 客户端。
在 WSL 终端先清除旧网络变量，再由网络脚本选择当前局域网地址并启动 Master：

```bash
unset ROS_IP ROS_HOSTNAME ROS_MASTER_URI
~/start_ros_master.sh
```

输出必须是电脑当前局域网地址；若显示 `localhost` 或 `127.0.0.1`，立即按 `Ctrl-C`
停止。不要让小车连接回环地址。

输出还必须包含 `XML-RPC=HTTP/1.0`。小车使用 Linux 4.9 内核；在当前 WSL 镜像网络
中，Noetic Master 的 HTTP/1.1 XML-RPC 响应会间歇性无法到达小车，使
`roslaunch` 卡在 `load_parameters starting ...`。仓库提供的启动脚本只对 Master
进程树启用 HTTP/1.0 兼容层，不修改系统 ROS，不改变 TCPROS 话题传输。首次部署或
兼容层更新后执行：

```bash
mkdir -p ~/.config/smartcar/python_http10_compat
cp /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/ucar_source_code/rosmaster/start_ros_master.sh \
  ~/start_ros_master.sh
cp /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/ucar_source_code/rosmaster/python_http10_compat/sitecustomize.py \
  ~/.config/smartcar/python_http10_compat/sitecustomize.py
chmod 0755 ~/start_ros_master.sh
```

部署后先停止旧 Master，再重新运行 `~/start_ros_master.sh`。可在小车上执行以下
零运动回归检查；连续 20 次均须成功：

```bash
for attempt in $(seq 1 20); do
  timeout 2 python2 -c \
    "import xmlrpclib; print(xmlrpclib.ServerProxy('${ROS_MASTER_URI}').system.multicall([]))" \
    >/dev/null || exit 1
done
echo "XML-RPC multicall: 20/20 passed"
```

WSL 的 ROS 环境已设置 DISABLE_ROS1_EOL_WARNINGS=1，因此不会再显示 ROS 1
Noetic 生命周期结束的提示窗口。

必须以当前工作区的 `RVIZ_CONFIG` 覆盖 `~/start_rviz.sh` 的历史默认值：

    export RVIZ_CONFIG=/mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/ucar_source_code/ucar_ws/src/ucar_2026/rviz/navigation_2026.rviz

该配置已启用 `/map`、全局/局部代价地图、`/scan` 激光点云、`base_link` 朝向箭头和
`/usb_cam/image_raw` 相机画面。点云轨迹与朝向 TF 的失效时间均为 `0.3 s`，因此不会把
断流后的旧朝向继续显示为实时数据。

首次使用前，必须在 Windows 的管理员 PowerShell 放行当前本地子网的入站 TCPROS：

    Get-NetFirewallRule -DisplayName 'ROS TCPROS from UCar to WSL' | Get-NetFirewallAddressFilter | Set-NetFirewallAddressFilter -RemoteAddress LocalSubnet
    Get-NetFirewallHyperVRule -DisplayName 'ROS TCPROS from UCar to WSL' | Set-NetFirewallHyperVRule -RemoteAddresses LocalSubnet

规则限制在受信任的当前本地子网；ROS 话题使用动态 TCPROS 端口，不能只开放 11311。

另开一个 WSL 终端即可运行：

    RVIZ_CONFIG=/mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/ucar_source_code/ucar_ws/src/ucar_2026/rviz/navigation_2026.rviz ~/start_rviz.sh

若 Windows 任务栏只有 RViz 图标、无法显示主窗口，说明 WSLg 远程应用会话卡住。在
Windows PowerShell 执行以下命令重建图形会话，再运行上述 Master 和 RViz 启动命令：

    wsl --shutdown

小车端推荐只运行以下脚本；它会提示输入 WSL Master 地址，并自动完成所有 `source`、
动态小车 IP 计算和 Master 连通检查：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh
```

不要在小车端执行 `roscore`，初学者不要再手工复制多行 `export`/`awk` 命令。

若只验证小车到 WSL 的数据链路、绝不发送默认导航目标，小车端使用同一个默认
`manual` 模式：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh
```

小车的唯一 ROS Master 是本机 WSL Ubuntu 20.04。因此以上两条 `export` 必须在所有
`source` 命令**之后**执行；同时先 `unset ROS_HOSTNAME`，避免其与 `ROS_IP` 冲突。
不得使用或启动小车本机 `roscore`，也不要只单独执行最后一条 `roslaunch`。

随后在 WSL 终端检查；下列命令只读取数据，不会移动小车：

```bash
rostopic echo -n 1 /map
rostopic echo -n 1 /scan
rostopic echo -n 1 /move_base/global_costmap/costmap
rostopic echo -n 1 /move_base/local_costmap/costmap
rosrun tf tf_echo map base_link
```

## 当前 2026 RViz 定点导航（CymPlanner）

本节是当前唯一的真机导航入口，取代本文后续历史的 TEB、二维码、语音、默认目标和
生产路线说明。正式入口 `ucar_2026/launch/2026.launch` 只启动如下链路：

推荐命令为 `roslaunch ucar_2026 2026.launch`。`roslaunch yolo2025 2026.launch` 仅由
兼容 wrapper 转发，暂时保留给旧自动化，不再作为正式入口或资源位置。

```text
base_driver + YDLidar (/scan_raw)
  -> navigation_scan_relay (/scan, /scan_global_obstacles)
  -> lidar_loc (map -> odom) + lidar_filter_node (/scan_filtered)
  -> move_base (GlobalPlanner + CymPlanner) -> /cmd_vel -> base_driver
usb_cam (/dev/ucar_video) -> /usb_cam/image_raw -> RViz Image panel
```

`navigation_scan_relay` 仅转发激光并从全局障碍物输入中删除已存在于静态地图的墙体回波；
它优先使用激光时间戳的 `map <- laser_frame` TF，若该查询只因数毫秒发布延迟失败，则可使用
不超过 `0.20 s` 的最新公共 TF。地图掩码或这两种 TF 都未就绪时，它发布全 `inf` 的全局扫描，
避免把错误坐标系的点写入全局代价地图。定位始终使用未滤波的 `/scan`，局部避障使用
`/scan_filtered`，全局重规划使用 `/scan_global_obstacles`。

relay 配置为 `respawn=true` 与 `2.0 s` 重试延迟：若其在启动阶段短暂退出，ROS 会先恢复
`/scan_raw -> /scan`，而不是让 `lidar_loc` 长期失去激光输入。重试期间不得发送导航目标。
相机也以相同的 2 秒重试策略运行；它只供 RViz 显示，未连接任何二维码、语音、任务或
`/cmd_vel` 控制逻辑。

真机仍必须使用真实尺寸 `0.342 m × 0.256 m` 的足迹、`map` / `base_link` 坐标系和
`lidar_loc` 的 `map -> odom`。不得照搬仿真的 Gazebo、`base_footprint` 或 `odom`
接口。`testnav20260721` 是当前生效的导航配置档；其
`global_costmap_common.yaml` 与 `local_costmap_common.yaml` 分别承载仿真的
全局/局部层参数：全局使用 `3/5 Hz`、`0.01185 m`、`3.0/4.0 m`、`0.205 m/0.05`；局部
使用 `8/3 Hz`、`2.0 m × 2.0 m`、`0.03 m`、`3.0/4.0 m`、`0.07 m/4.0`。全局仍消费
静态墙体已掩除的 `/scan_global_obstacles`，局部仍消费 `/scan_filtered`，这是为真实
雷达与地图的小对齐误差保留的安全适配。

当前真机默认地图为
`ucar_nav/maps/iflysse_field_walls_without_middle_vertices.yaml`，即场地中间不含顶点的
版本。不要将它与旧的 `iflysse_2026_direct.yaml` 混用。

同步当前代价地图和新的正式 2026 包、但保持任务停止时，在控制电脑工作区执行。将
`<小车当前主机名或IP>` 替换为当前可达地址：

```powershell
$CAR_HOST = '<小车当前主机名或IP>'
ssh "ucar@$CAR_HOST" 'mkdir -p ~/ucar_ws/src/ucar_nav/config/testnav20260721'
scp -r ucar_ws/src/ucar_2026 "ucar@${CAR_HOST}:~/ucar_ws/src/"
scp ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_common.yaml "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"
scp ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"
scp ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_params.yaml "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"
scp ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"
scp ucar_ws/src/ucar_nav/config/testnav20260721/global_planner_params.yaml "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"
scp ucar_ws/src/ucar_nav/config/testnav20260721/move_base_params.yaml "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"
scp ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/launch/"
scp ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.yaml "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/maps/"
scp ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.pgm "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_nav/maps/"
```

`ucar_2026` 是正式入口包，需要构建；其余 YAML/launch/map 文件不需要重新编译。构建后可在
小车端完成静态解析（不会启动节点或移动）：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
roslaunch --nodes ucar_2026 2026.launch
```

在 Ubuntu 18.04 / ROS Melodic 环境重新构建后，所有 `source` 完成后显式恢复唯一的
WSL Master：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
cd ~/ucar_ws
catkin_make --pkg cym_planner ucar_2026
source devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
python2 /opt/ros/melodic/bin/roslaunch ucar_2026 2026.launch
```

默认入口不会发布自动目标。只在 `/odom_raw` 为有限值、
`odom -> base_link` 与 `map -> base_link` 均可用，且 `/cmd_vel` 只有预期的
`/move_base` 发布者时，才可在控制电脑的 RViz 使用 **2D Nav Goal**。

```bash
rostopic echo -n 1 /odom_raw
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map base_link
rostopic info /cmd_vel
rosparam get /move_base/base_local_planner
rosparam get /move_base/local_costmap/inflation_layer/inflation_radius
```

预期本地规划器为 `cym_planner/CymPlanner`，局部膨胀半径为 `0.07`。RViz 的
**CymPlanner CV Map** 和 **CymPlanner CV Plan** 分别订阅
`/move_base/cym_planner/CymPlanner/debug_map` 与
`/move_base/cym_planner/CymPlanner/debug_plan`；只有 move_base 收到目标并开始调用
局部规划器后才会刷新。

若底盘日志出现 `device disconnected`、`Odom sensor not active` 或
`Imu sensor not active`，先发布零速度并停止整个 2026 launch，再检查内核 USB
事件，不得继续读取一帧临时恢复的数据后发送目标：

```bash
dmesg | grep -E \
  'usb 1-2: (USB disconnect|new high-speed USB device)|ttyUSB0|cp210x' | tail -n 80
lsusb -t
cat /sys/bus/usb/devices/1-2/power/control
cat /sys/bus/usb/devices/1-2.2/power/control
```

若 `usb 1-2` Hub 与多个子设备同时消失再枚举，故障不属于单个 ROS 节点。
在断电状态检查 Hub 供电、上游接插件和线缆，恢复后先运行无目标模式并持续观察，
时间必须覆盖此前出现断开的窗口；稳定期间不得出现新的 Hub disconnect、sensor
inactive、CRC 或 TF 停滞。2026-07-28 的隔离回归中，物理拔除新增 USB 扬声器后，
`base_driver` 仍在约 11 秒内使同一 Hub 再次掉线，因此不能把扬声器视为唯一原因，
还必须继续检查 Hub 本体、上游线缆和公共供电。

若把 CP2102 完全绕过外置 Hub、直接接入主机 USB 口，先用下列只读命令确认拓扑，
不要仅凭物理插接位置判断：

```bash
lsusb -t
dmesg | grep -E \
  'usb .*disconnect|new .*USB device|cp210|ttyUSB|over-current|reset .*USB' | tail -n 80
```

直连的 CP2102 应直接显示在主机 `root_hub` 下，而不是 Terminus Hub 的子端口下。
2026-07-28 的直连复测中，CP2102 显示为根 Hub `1-2` 的 12 Mbit/s 设备，内核没有
新的 disconnect/reset，但仍出现 AHRS CRC、帧长错误以及约 3 秒的
`odom -> base_link` 停滞。这说明原 Hub 不是唯一故障点；下一轮必须在断电状态下
依次只替换一个变量：CP2102 模块、短 USB 线、CP2102 到底盘 MCU 的串口线束，
并检查接插件、公共地、控制板供电和实际固件波特率。不得只改单侧波特率。每轮先只
启动 `base_driver` 静态观察至少 10 分钟；任何 CRC、`head_len`、sensor inactive
或 TF 停滞均判定失败并立即整链停止。

### 只测试 mini 集成 IMU

上游 `WXyuany/iFLYtek` 的说明明确区分两种车型：mini 的 IMU 数据与底盘数据复用
`ucar_controller/base_driver` 的 CP2102 串口，晓版本才用
`fdilink_ahrs` 独立读取 IMU。mini 日常验证先只启动底盘驱动，不得同时启动两个节点
抢占同一个串口：

```bash
# 先在 WSL 启动唯一 Master；小车端 source 后重新设置动态网络变量。
roslaunch ucar_controller base_driver.launch

# 另一个已连接同一 Master 的终端只读采样。
rostopic info /imu
timeout 22 rostopic hz -w 200 /imu
rostopic echo -n 3 /imu
rostopic hz -w 100 /odom
```

测试期间不发送运动命令；启动前和停止前均发布零速度。`/imu` 的四元数、角速度和
线加速度必须全部有限，静止时四元数模长应接近 1、加速度模长应接近重力加速度。
观察时间至少覆盖 10 分钟；出现 CRC、`head_len`、NaN 或频率停滞立即判定失败并
停止。

需要把底盘帧解析与 IMU 帧解析分开对照时，先停止 `base_driver` 并确认串口已释放，
然后可临时让仓库已有、与上游源码一致的独立节点读取同一个 CP2102：

```bash
rosrun fdilink_ahrs ahrs_driver \
  _port:=/dev/ttyUSB0 _baud:=921600 _debug:=false \
  _imu_topic:=/imu _imu_frame:=imu _device_type:=1
```

该命令只用于静态诊断，不能与 `base_driver` 并行运行，也不是 mini 的正式启动方式。
2026-07-28 对照中，集成驱动约 4 分钟、独立驱动约 2 分钟均维持约 50 Hz 的有限
IMU 数据，未复现 CRC；测试时 CP2102 已重新位于外置 Hub 的 `1-2.3`，并非先前的
根 Hub 直连，所以结果只证明当次间歇故障没有发生，不能宣告硬件修复。

### 无 RViz 的 yolo2025 自动目标差分

为判断 RViz 的跨机话题流量是否会放大串口问题，必须保持 WSL 为唯一 Master，先用
同一套完整 launch 关闭自动目标完成静态安全门；不得直接让冷启动节点运动：

```bash
roslaunch yolo2025 2026.launch startup_goal_enabled:=false

# 运动前必须全部通过。
rostopic hz -w 100 /odom_raw
rostopic hz -w 200 /imu
rostopic hz -w 100 /scan
rostopic echo -n 10 /odom_raw
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map base_link
rosnode list | grep rviz
```

确认 `/odom_raw` 全部有限、两段 TF 连续可读、无 CRC/USB 新事件且没有 RViz 节点后，
发布零速度并停止无目标 launch，再重启自动目标：

```bash
roslaunch yolo2025 2026.launch startup_goal_enabled:=true
```

2026-07-28 的 GitHub `main` 对照中，启动目标实际发出并到达；采样到线速度约
`0.20～0.32 m/s`、角速度最高 `1.0 rad/s`，随后完成三个二维码朝向停靠，识别到
2/3 个不同二维码，因数量不足而没有进入生产路线。无 RViz 时运动阶段没有 CRC，
但二维码流程结束约 44 秒、里程计已经为零后仍出现一次
`check crc16 faild(ahrs)`。测试没有新增内核 USB disconnect/reset，故 RViz 负载
可能提高复现概率，但不是 CRC 的必要条件。出现 CRC 后必须立即发布零速度并停止
完整 launch，不得因为自动目标已经完成就继续留机观察。

### 部署 GitHub main 的 yolo2025 做静态对照

此流程只替换小车的 `~/ucar_ws/src/yolo2025`，不切换或覆盖本地当前工作树。车端不留
旧包备份；需要恢复时重新从目标 Git 分支部署。先在 Windows PowerShell 从已确认的
`origin/main` 生成归档并上传：

```powershell
$CAR_HOST = '<小车当前主机名或IP>'
$ARCHIVE = Join-Path $env:TEMP 'smartcar-yolo2025-main.tar'
git archive --format=tar --output=$ARCHIVE origin/main ucar_ws/src/yolo2025
scp $ARCHIVE "ucar@${CAR_HOST}:/tmp/smartcar-yolo2025-main.tar"
```

在小车上把归档解到 `/tmp`，完成依赖入口检查后精确替换包目录。Windows 生成的归档
可能带 CRLF，因此部署后必须规范化文本并检查 Python 2 shebang；不得直接启动：

```bash
rm -rf /tmp/smartcar-yolo2025-main-stage
mkdir -p /tmp/smartcar-yolo2025-main-stage
tar -xf /tmp/smartcar-yolo2025-main.tar -C /tmp/smartcar-yolo2025-main-stage
test "$(readlink -f ~/ucar_ws/src/yolo2025)" = \
  "/home/ucar/ucar_ws/src/yolo2025"
rm -rf ~/ucar_ws/src/yolo2025
mv /tmp/smartcar-yolo2025-main-stage/ucar_ws/src/yolo2025 ~/ucar_ws/src/yolo2025
find ~/ucar_ws/src/yolo2025 -type f \
  \( -name '*.py' -o -name '*.sh' -o -name '*.launch' -o -name '*.xml' \
     -o -name '*.json' -o -name '*.rviz' -o -name '*.urdf' \
     -o -name 'CMakeLists.txt' \) \
  -exec sed -i 's/\r$//' {} +
find ~/ucar_ws/src/yolo2025/scripts -maxdepth 1 -type f \
  \( -name '*.py' -o -name '*.sh' \) -exec chmod 0755 {} +
rm -rf /tmp/smartcar-yolo2025-main-stage
rm -f /tmp/smartcar-yolo2025-main.tar
od -An -tx1 -N32 ~/ucar_ws/src/yolo2025/scripts/2026.py
```

重新构建只能在小车 Ubuntu 18.04 上进行。所有 `source` 后仍须恢复当前 WSL Master：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk \
  '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
cd ~/ucar_ws
catkin_make --pkg yolo2025 -DPYTHON_EXECUTABLE=/usr/bin/python2
```

`main` 的 `2026.launch` 默认会发送启动目标，静态对照必须显式关闭：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk \
  '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
~/ucar_ws/src/ucar_2026/scripts/run_melodic_python2.sh \
  /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch \
  startup_goal_enabled:=false
```

2026-07-28 对照中，`main` 已成功启动且确认
`/navigation_2026/startup_goal_enabled=false`，但 45 秒内仍出现 IMU 与 AHRS CRC
错误，因此当前底盘串口问题不由 `simulation_real` 的 yolo2025 改动引起。

### `map` TF 等待与 scan relay 异常退出

若 move_base 重复报告 `Timed out waiting for transform from base_link to map`，不要把它
当作代价地图数值错误。`map_server` 仅发布 `/map`，实际的 `map -> odom` 由 `lidar_loc`
在同时取得 `/scan` 和有效里程计后发布。当前链路中 `/scan` 又完全依赖
`navigation_scan_relay` 将 `/scan_raw` 转发出来。

先保持无导航目标，并在小车端执行以下只读检查；所有 `source` 完成后必须恢复 WSL Master：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
rosnode list | grep -E '^/(navigation_scan_relay|lidar_loc|move_base)$'
rostopic info /scan
rostopic echo -n 1 /odom_raw
timeout 5 rosrun tf tf_echo map base_link
```

2026-07-21 曾发生 relay 在完整 launch 开始约两秒后以 exit code 1 退出：随后没有 `/scan`，
`lidar_loc` 不会发布 `map -> odom`，上述代价地图警告随之出现。该次异常的启动前 stderr
没有写进 ROS 日志；relay 文件本身的 Python 2 编译、ROS 依赖、可执行权限和 LF shebang 均已
验证正常，且它可以单独保持运行。若复现，保留并记录终端中
`[navigation_scan_relay-8] process has died` 前后的完整输出；不要在 `/odom_raw` 为 `NaN`
或 `map -> base_link` 缺失时发送 RViz 目标。若出现 `NaN`，先发布零速度并重启导航/底盘
里程计链路，再继续检查。

本次变更已按用户指示同步到小车进行静态验证；不要创建小车端备份、不要提交或推送 GitHub。

## ROS Melodic 终端恢复

小车的 ROS Melodic 工具必须由 Python 2 运行，而系统 `/usr/bin/python` 当前为 Python 3。`.bashrc` 已将 `roslaunch` 定义为调用 `python2 /opt/ros/melodic/bin/roslaunch` 的 shell 函数，并且每个 ROS overlay 只加载一次。

若终端出现 `Argument list too long`，当前 shell 的环境变量已膨胀，不能继续 source 或启动命令。关闭该终端并新开一个终端，然后验证：

```bash
type roslaunch
```

预期输出应包含 `roslaunch is a function`。如需绕过 shell 函数，可显式使用：

```bash
python2 /opt/ros/melodic/bin/roslaunch ucar_2026 2026.launch
```

### 2026.launch 异常终端关闭后的停止

正常停止始终优先在启动终端按 `Ctrl-C`。若终端已被关闭但真机任务仍在运行，不要使用
宽泛的 `pkill -f roslaunch` 或 `rosnode kill -a`。在小车的新终端提供当前 WSL Master
地址后运行专用脚本：

```bash
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export MASTER_IP
bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
```

脚本会在所有 `source` 后显式恢复 WSL Master，先向 `/cmd_vel` 发布零速度，暂停精确匹配
的 `ucar_2026 2026.launch` 父进程以阻止 `move_base` 重生，关闭其直接子进程，最后清理该
launch 父进程。它不启动或停止 WSL Master，也不匹配其他 launch。

若关停后 Master 仍列出不可达的旧节点，先检查：

```bash
rosnode info /navigation_2026
```

只有显示 `connection refused`，并且 `rosnode cleanup` 提示的节点全是旧 2026 任务节点时，
才执行：

```bash
rosnode cleanup
```

在提示时输入 `y`。新的 `2026.launch` 会重新注册同名节点，无需重启 WSL Master。

### Odometry NaN safety gate

Before sending a navigation goal or any manual rotation, verify that the
wheel odometry is finite and that both transform links exist:

```bash
rostopic echo -n 1 /odom_raw
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map base_link
```

> **历史任务记录（当前不要执行）：** 以下 TEB、默认目标、二维码和生产路线章节记录的是旧
> `yolo2025/scripts/2026.py` 任务链路；该脚本本次未迁移，也不属于正式
> `ucar_2026/launch/2026.launch`。正式 launch 不发送默认目标、不执行二维码或生产路线任务，
> 当前操作请以本文前部“当前 2026 RViz 定点导航（CymPlanner）”为准。

## 2026 TEB 试运行

当前 2026 入口使用
`ucar_nav/launch/teb_move_base_omni_2026.launch`，通过
`teb_move_base_params_2026.yaml` 选择已经安装在小车 Melodic 中的
`teb_local_planner/TebLocalPlannerROS`。TEB 直接发布 `/cmd_vel`，不要套用旧
的 `/teb_cmd_vel` 重映射；当前没有启动对应 relay。

TEB 首次完整路线按已经验证的 CymPlanner 生产参数做保守基线：前进/后退
`0.25 m/s`、横移 `0.10 m/s`、角速度 `1.0 rad/s`、到点距离 `0.05 m`、
最终朝向误差 `0.10 rad`、车体外硬净距 `0.03 m`。为避免窄路口在多个拓扑间
切换，首次基线关闭 homotopy class；不强制绑定每个全局路径 viapoint，避免
局部地图把路径判为不可行。TEB 分支的局部 costmap 膨胀半径也设为 `0.03 m`，
避免与 TEB 自身 `0.03 m` 硬净距重复叠加成过大的通道收缩。TEB 的
`map_frame` 必须是 `map`，与局部 costmap 的 `global_frame` 一致。

本地 WSL Noetic 没有安装 TEB 插件，不能在本地运行时加载该 planner。小车
端先同步以下文件，再进行静态解析和构建检查：

```bash
scp ucar_ws/src/ucar_nav/launch/teb_move_base_omni_2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/launch/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/teb_move_base_params_2026.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/teb_local_planner_params.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
scp ucar_ws/src/yolo2025/launch/2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
scp ucar_ws/src/yolo2025/scripts/2026.py \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/scripts/
```

小车端验证 TEB 已安装、launch 可解析；每次 `source` 后显式恢复 WSL
Master，不启动小车本机 `roscore`：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
rospack find teb_local_planner
# 历史命令，仅记录，当前不要执行。
roslaunch --nodes yolo2025 2026.launch startup_goal_enabled:=false
```

第一次真正启动时先停止旧任务，并保持无自动目标：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
# 历史命令，仅记录，当前不要执行。
bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch \
  startup_goal_enabled:=false
```

启动日志必须显示 TEB 使用 polygon 车体模型，不应再出现
`No robot footprint model specified`。以下两条应分别输出 `polygon` 和 `0.03`：

```bash
rosparam get /move_base/TebLocalPlannerROS/footprint_model/type
rosparam get /move_base/TebLocalPlannerROS/min_obstacle_dist
```

`0.03 m` 是 TEB 在车体 polygon 外额外要求的硬障碍净距，不是把车体缩成 3 cm；
车体仍为 `0.342 m × 0.256 m`。局部 costmap 的 `0.03 m/pix` 是栅格分辨率，含义不同。

确认 `/odom_raw` 为有限值、`odom -> base_link` 和 `map -> base_link` TF 正常、
且 `/cmd_vel` 没有多个非预期发布者后，再由 RViz 发送一个手动目标。回滚
时把 `2026.launch` 的 include 改回
`cym_move_base_omni_2026.launch`，然后按同样的停止/启动流程重启。

## CymPlanner 终点 180° 震荡修复：构建与验证

该修复属于 `cym_planner` C++ 代码变更。仅调整 YAML 参数不能解决
`+pi/-pi` 分界导致的正反角速度翻转；部署后必须重新编译
`cym_planner` 并重启 `move_base/2026.launch`。

本地 WSL Ubuntu 20.04 使用独立临时 catkin 工作区验证，避免仓库中其他
ROS 包的缺失依赖干扰。所有 `source` 后必须立即恢复唯一 ROS Master：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311

build_root=$(mktemp -d /tmp/cym-planner-build.XXXXXX)
mkdir -p "$build_root/src"
ln -s /mnt/d/WORK/ALLCODE/smartcar2026/ucar_ws/src/cym_planner \
  "$build_root/src/cym_planner"
cd "$build_root/src"
catkin_init_workspace
cd "$build_root"
catkin_make -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_cym_planner
catkin_test_results
```

验证完成后只清理上面由 `mktemp` 创建且已确认位于 `/tmp/` 的目录。

同步到小车后，在小车端编译时仍只使用 WSL 的 ROS Master，禁止启动小车
本机 `roscore`：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
cd ~/ucar_ws
catkin_make --pkg cym_planner
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
```

重启导航前先确认旧任务已经停止。启动后先做静态安全检查；只有
`/odom_raw` 全部为有限值，且 `odom -> base_link`、`map -> base_link` TF
均正常时，才允许发送目标验证终点朝向：

```bash
rostopic echo -n 1 /odom_raw
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map base_link
```

### 无 RViz 向正式 ucar_2026 发送单个目标

正式 `ucar_2026/launch/2026.launch` 不包含自动目标节点；无 RViz 测试时，在
静态安全门通过后直接向 move_base 的标准 `/move_base_simple/goal` 输入发布
`PoseStamped`。以下命令发送与 2026 起点对照测试相同的目标
`map (-1.734, 2.305, yaw=90°)`：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk \
  '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'map'}
pose:
  position: {x: -1.734, y: 2.305, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.70710678, w: 0.70710678}"
```

发送前还应确认 `rostopic info /cmd_vel` 只有 `/move_base` 发布、
`/base_driver` 订阅，且 `rosnode list` 中没有 RViz。完成后检查：

```bash
rostopic echo -n 1 /move_base/status
rostopic echo -n 1 /odom_raw
```

成功结果应包含 `status: 3` 与 `Goal reached.`，随后里程计线速度和角速度均为
有限的 `0.0`。若出现 NaN、`TF_NAN_INPUT`、AHRS/IMU CRC、`head_len` 或传感器
掉线，立即取消目标、发布零速度并停止完整导航链路。单次 `NO PATH` 后若目标仍为
active 且重新规划恢复，可继续观察；连续失败或状态转为 aborted 时必须停车。

### ucar_2026 正式生产任务：52 扫码、原生相机 OCR 和墙点匹配

`task_enabled` 默认是 `false`，所以普通
`roslaunch ucar_2026 2026.launch` 永远不会自动移动。正式任务的固定流程是：

1. 使用点模式导航到中心点 52 `(-1.75, 2.25)`。
2. 车辆保持在 52，依次面向墙边观察点
   262 `(-2.50, 2.25)`、232 `(-1.75, 3.00)`、
   295 `(-1.75, 1.50)` 扫码。观察点只决定车头方向，车辆不得导航到墙边。
3. 每个观察方向先静止识别 4 秒。没有新二维码时，以 `0.18 rad/s` 最多正转
   一整圈并持续识别；一圈结束仍没有新结果则停车并终止任务。默认要求三次结果互不重复。
4. 三个二维码全部识别后，任务先发布零速度并关闭扫码，但保持 `/usb_cam` 运行。
   Python 2 任务节点通过 `cv_bridge` 从 `/usb_cam/image_raw` 保存当前帧，随后把图片
   路径交给 Python 3 helper。helper 不再直接持有 `/dev/ucar_video`，避免与完整
   supervisor 自动拉起的 ROS 相机争抢设备。
5. 通过 `/ucar/navigation_mode` 将 CymPlanner 切换为
   `body_projection`；控制律、速度参数和目标点算法不变，只把前视碰撞检查从点扩大为
   车体，并在全局代价地图上只把原始 254 致命格（OccupancyGrid 100）判为实际接触。
   原始 253 内切膨胀格（OccupancyGrid 99）只用于路径引导，不会被车体投影二次挡停；
   未知区仍安全拒绝。
6. 完整编号路线仍严格保持
   `3 → 2 → 1 → 11 → 21 → 31 → 32 → 33 → 34 → 35 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 20 → 30 → 40 → 39 → 38 → 37`，
   但连续共线点自动合并为
   `3→1`、`1→31`、`31→35`、`35→4`、`4→10`、`10→40`、`40→37`
   七个连续识别段。11、21 等中间编号只作为经过点，不再逐点停车。
7. 长线段导航过程中，任务以 `navigation_ocr_poll_period` 周期保存最新 ROS 相机帧，
   由 Python 3 helper 直接复用
   `/home/ucar/ocr3/ppocr_trt/python/live_ppocr.py` 的 `PPOcr` CUDA 引擎和三类
   中文候选词；不使用 Tesseract。候选置信度达到门限后，必须先取消当前 move_base
   目标并得到终态，再连续确认新鲜 `/odom_raw` 三轴速度低于停车阈值；只有停车门禁
   通过后才允许对准、雷达匹配和保存。移动 OCR 在单任务异步 worker 中运行，主状态机
   在推理期间继续检查 action、安全门和超时；迟到候选不会覆盖已经结束的目标。
   对准通过 move_base 发送同位置的小角度朝向目标，继续使用车体轮廓碰撞检查，不由
   任务节点直接发布非零 `/cmd_vel`。处理完成后恢复当前长线段原终点。
   若识别框中心没有落在图像水平中心的 ±18 px 内，任务只发布受限纯角速度，停车后
   重新拍照验证，最多 6 次。
8. 居中后等待一帧新的 `/scan`，取正前方 ±3° 有限距离的中位数，并使用该扫描时间的
   `map ← laser` TF 投影墙面交点。最近点只在 JSON 的
   `wall_reference_point_numbers` 中查询；匹配误差超过 `0.18 m` 不会计入正式结果。
   JSON 保留 1–418 的旧编号，新增 419–445 中间线端点、446–451 左右墙顶点和
   452–459 左右墙边中点。
9. 观察发生在长线段中的实际停车位置，不再强制回到 11、21 等旧中心点。对准和雷达
   采集完成后，任务重新向当前段原终点发送 move_base 目标；若恢复规划或导航失败，
   任务停车中止，不会跳过该段。
10. 任务结束于点 37。每次尝试都写入
    `~/.ros/ucar_2026_observations/run_*/observations.json`；同一墙点只保留置信度
    最高的一次，最终必须得到 3 个不同墙点及其 OCR 内容，否则保存部分结果并将任务标为失败。

从本地仓库同步本次任务文件；不要在小车端创建备份目录或压缩包：

```bash
scp ucar_ws/src/ucar_2026/scripts/production_task_2026.py \
  ucar_ws/src/ucar_2026/scripts/production_task_geometry.py \
  ucar_ws/src/ucar_2026/scripts/production_task_perception.py \
  ucar_ws/src/ucar_2026/scripts/production_camera_ocr.py \
  ucar@<CAR_HOST>:~/ucar_ws/src/ucar_2026/scripts/
scp ucar_ws/src/ucar_2026/config/production_full_grid_all_numbered.json \
  ucar@<CAR_HOST>:~/ucar_ws/src/ucar_2026/config/
scp ucar_ws/src/ucar_2026/launch/2026.launch \
  ucar@<CAR_HOST>:~/ucar_ws/src/ucar_2026/launch/2026.launch
scp ucar_ws/src/ucar_2026/CMakeLists.txt \
  ucar_ws/src/ucar_2026/package.xml \
  ucar@<CAR_HOST>:~/ucar_ws/src/ucar_2026/
scp ucar_ws/src/ucar_2026/test/test_production_task_geometry.py \
  ucar_ws/src/ucar_2026/test/test_production_task_perception.py \
  ucar_ws/src/ucar_2026/test/test_production_camera_ocr.py \
  ucar@<CAR_HOST>:~/ucar_ws/src/ucar_2026/test/
```

部署后只能在小车 Ubuntu 18.04 构建和测试。OCR 使用小车已有的
`live_ppocr.py`、CUDA plan 和 Python 3 OpenCV，不安装 Tesseract：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk \
  '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
cd ~/ucar_ws
test -f /home/ucar/ocr3/ppocr_trt/python/live_ppocr.py
test -f /home/ucar/ocr3/ppocr_trt/out/det.plan
test -f /home/ucar/ocr3/ppocr_trt/out/rec.plan
test -f /home/ucar/ocr3/ppocr_trt/out/keys.txt
python3 -c 'import cv2, numpy; print(cv2.__version__)'
python2 -m py_compile \
  src/ucar_2026/scripts/production_task_geometry.py \
  src/ucar_2026/scripts/production_task_perception.py \
  src/ucar_2026/scripts/production_task_2026.py
python3 -m py_compile src/ucar_2026/scripts/production_camera_ocr.py
python3 src/ucar_2026/test/test_production_camera_ocr.py
python2 -m unittest discover -s src/ucar_2026/test -p 'test_*.py' -v
catkin_make -DCATKIN_ENABLE_TESTING=ON \
  -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware;yolo2025;ucar_2026" \
  -j2
(cd build && ctest -R production_camera_ocr_py3 --output-on-failure)
catkin_make run_tests_cym_planner -j2
catkin_test_results build/test_results/cym_planner
```

第一次实车运行前，先使用默认 `manual` 模式完成静态门禁；停止该 launch 后，再运行：

```bash
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh "$MASTER_IP" mission
```

`lidar_loc` 的启动初值固定为起点 `(-0.25, 2.75, 0)`。若任务中途终止且已经离开起点，
停止完整 launch 后必须先把实车放回起点，才能再次启动 mission。不得在车辆仍位于
52 或生产区其他点时直接重启整条定位链路；固定初值会造成地图位姿与实车位置不一致。

该模式不需要也不启动 RViz。运行时可在连接同一 Master 的诊断终端观察：

```bash
rostopic echo /ucar_2026/task_state
rostopic echo /ucar_2026/task_result
rostopic echo /qr_result
ls -1dt ~/.ros/ucar_2026_observations/run_* | head -n 1
```

任务节点持续检查 `/odom_raw` 有限性和新鲜度、`odom -> base_link`、
`map -> base_link`、move_base action 结果，以及 `crc16`、`head_len`、
`sensor not active`、`TF_NAN_INPUT` 日志。任一安全门失败时会取消目标、关闭扫码并
连续发布零速度。生产 OCR 阶段 `/usb_cam` 应继续存在，图像话题应维持约 30 Hz；
Python 3 helper 只读取任务节点保存的图片，不直接持有 `/dev/ucar_video`。视觉居中期间 `/cmd_vel` 的
预期发布者为 `/move_base` 和 `/production_task_2026`；后者只在 move_base 目标已经
结束或取消后发布纯角速度。
回中阶段状态为 `PRODUCTION_RECENTER_<点号>_<次数>`，此时重新由 `/move_base`
发布速度；日志 `PRODUCTION_TASK_RECENTER_NOT_NEEDED` 表示转后误差未超过
`0.06 m`，无需产生额外运动。`PRODUCTION_TASK_RECENTER_WARNING`、
`PRODUCTION_TASK_NAVIGATION_WARNING` 或 `PRODUCTION_TASK_POSITION_WARNING`
表示回中未达到预期，但任务会继续。NaN、TF 失效、传感器掉线等硬安全故障仍会停车。

以下 2026-07-28 冷启动实车结果属于被本次 OCR 路线替代的历史任务，不代表新路线已验收：

- 52 到达误差 `0.031 m`，识别到三个不同二维码 `…/a`、`…/d`、`…/i`；
- 修正后的 body projection 成功通过原先被局部 253 误挡的位置；
- 12、24、16、28、19 均到达并各完成一圈，转后位置误差最大 `0.055 m`；
- 170 到达误差 `0.018 m`，朝向 319 校验通过；
- 最终 `/ucar_2026/task_state=SUCCEEDED`，结果 `success: true`；
- 全程没有 CRC、`head_len`、NaN、TF_NAN 或串口掉线。少量瞬时
  `NO PATH`/TF 外推警告均自行恢复，move_base action 未中止。

紧急停止时在运行 launch 的终端按 `Ctrl-C`；若终端丢失，先恢复本次 Master 网络变量，
再执行：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
```

## 真机 RViz 车体模型

`navigation_2026.rviz` 的 **UCar 2026 Visual Model** 显示
`ucar_2026/urdf/ucar_2026_visual.urdf` 中的简化蓝色车体。它以已存在的 `base_link`
为根，尺寸与全局、局部代价地图足迹一致（`0.342 m × 0.256 m`）；不发布 TF、不参与碰撞检测，
也不改变底盘控制。红黄足迹 Marker 仍用于观察足迹和 5 cm 安全边界。

部署可视车模或足迹 Marker 修复时，同步 `ucar_2026` 的 URDF 与 launch；不在小车端创建备份：

```bash
scp ucar_ws/src/ucar_2026/urdf/ucar_2026_visual.urdf \
  ucar@<CAR_HOST>:~/ucar_ws/src/ucar_2026/urdf/
scp ucar_ws/src/ucar_2026/launch/2026.launch \
  ucar@<CAR_HOST>:~/ucar_ws/src/ucar_2026/launch/
```

停止旧的 `2026.launch` 后，在小车端使用正式入口重启，使其加载
`/robot_description`；然后在 WSL 重启 `~/start_rviz.sh`：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
python2 /opt/ros/melodic/bin/roslaunch ucar_2026 2026.launch
```

If `/odom_raw` contains `x: nan` or `y: nan`, or the terminal reports
`TF_NAN_INPUT` for `wheelodom`, publish zero velocity and restart the
navigation/odometry chain before continuing.  Do not run a localization or
costmap test against that state:

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
pkill -INT -f 'roslaunch ucar_2026 2026.launch' || true

# In a clean terminal, then start the no-goal launch again.
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
python2 /opt/ros/melodic/bin/roslaunch ucar_2026 2026.launch
```

不要在同一终端重复执行 `source ~/ucar_ws/devel/setup.bash`；新的终端已由 `.bashrc`
加载工作区。`navigation_scan_relay.py` 和历史 `yolo2025/scripts/2026.py` 必须使用
ROS Melodic 的 Python 2。两个 launch 都通过
`ucar_2026/scripts/run_melodic_python2.sh` 剔除 Python 3 模块目录并固定调用
`/usr/bin/python2`。

### Python 2 环境重建

若日志出现以下错误，说明 Python 2 进程错误加载了 Python 3 的 `tf2` 二进制：

```text
ImportError: dynamic module does not define init function (init_tf2)
```

先停止导航并确认目标目录，再只删除工作空间中无效的 Python 3 构建产物；不得删除
`src/`，也不得在小车保留备份目录：

```bash
pkill -INT -f 'roslaunch .*2026.launch' || true
cd /home/ucar/ucar_ws
test "$(readlink -f /home/ucar/ucar_ws/devel/lib/python3)" = \
  "/home/ucar/ucar_ws/devel/lib/python3"
catkin_make clean \
  -DPYTHON_EXECUTABLE=/usr/bin/python2 \
  -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware;yolo2025;ucar_2026"
rm -rf /home/ucar/ucar_ws/devel/lib/python3
catkin_make --force-cmake \
  -DPYTHON_EXECUTABLE=/usr/bin/python2 \
  -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware;yolo2025;ucar_2026"
```

重建后必须通过以下检查，输出路径只能是 `python2.7/dist-packages`：

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
/home/ucar/ucar_ws/src/ucar_2026/scripts/run_melodic_python2.sh -c \
  'import rospy; import tf; from sensor_msgs.msg import LaserScan; print(tf.__file__)'
find /home/ucar/ucar_ws/devel/lib -maxdepth 1 -type d -name 'python3*' -print
```

第二条 `find` 不应输出任何路径。之后只使用
`bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh` 启动。

## CymPlanner 真机构建与启动

在小车上确认没有已有导航 launch 正在运行后执行：

```bash
cd ~/ucar_ws
# 新开终端的 .bashrc 已加载 ROS 和工作区；不要在这里重复 source setup.bash。
# 保留当前仍需构建的历史包，并把正式入口 ucar_2026 加入构建图。
catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware;yolo2025;ucar_2026"
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
roslaunch ucar_2026 2026.launch
```

当 CymPlanner 有变更时，先从本机同步源码、头文件、依赖声明和参数，再在小车端执行
上面的 `catkin_make`。小车地址按当前网络提供，不写死旧 Wi-Fi IP：

```powershell
$CAR_HOST = '<小车当前主机名或IP>'
git ls-files --eol `
  ucar_ws/src/ucar_2026/scripts/navigation_scan_relay.py `
  ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
scp ucar_ws/src/cym_planner/src/cym_planner.cpp \
  "ucar@${CAR_HOST}:~/ucar_ws/src/cym_planner/src/"
scp ucar_ws/src/cym_planner/include/cym_planner.h \
  "ucar@${CAR_HOST}:~/ucar_ws/src/cym_planner/include/"
scp ucar_ws/src/cym_planner/CMakeLists.txt \
  ucar_ws/src/cym_planner/package.xml \
  "ucar@${CAR_HOST}:~/ucar_ws/src/cym_planner/"
scp ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml \
  "ucar@${CAR_HOST}:~/ucar_ws/src/cym_planner/config/"
scp ucar_ws/src/ucar_2026/package.xml \
  "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_2026/"
scp ucar_ws/src/ucar_2026/scripts/navigation_scan_relay.py \
  ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh \
  "ucar@${CAR_HOST}:~/ucar_ws/src/ucar_2026/scripts/"
ssh "ucar@$CAR_HOST" 'chmod 755 \
  ~/ucar_ws/src/ucar_2026/scripts/navigation_scan_relay.py \
  ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh; \
  head -n 1 ~/ucar_ws/src/ucar_2026/scripts/navigation_scan_relay.py | od -An -tx1'
```

从 Windows 文件系统执行 `scp` 后必须显式复核脚本权限；本仓库当前索引中的这两个文件为
普通文件模式，不能假定上传后自动保留 Linux 可执行位。`git ls-files --eol` 的工作树列
必须为 `w/lf`，车端首行十六进制必须以 `0a` 结束且不能出现 `0d 0a`；否则
`#!/usr/bin/env python2` 会被解释成不存在的 `python2\r`。

校验上传文件时，若待校验路径以 `ucar_ws/` 开头，远程命令必须从小车家目录执行：

```powershell
ssh "ucar@$CAR_HOST" 'cd ~ && sha256sum ucar_ws/src/cym_planner/src/cym_planner.cpp ucar_ws/src/ucar_2026/package.xml'
```

不要先 `cd ~/ucar_ws` 后再使用 `ucar_ws/src/...`，否则会错误查询
`~/ucar_ws/ucar_ws/src/...`。从 PowerShell 通过 SSH 执行包含 Bash `$()` 的命令时，应让
远程脚本整体使用单引号，或先取得本次明确的 `ROS_IP` 后直接传值；PowerShell 中的反斜杠
不能阻止 `$()` 被本地解释。

构建只写入新的库文件，不会替换已运行进程内存中的插件。请在当前 launch 终端按 `Ctrl-C` 后，使用以下正式入口手动启动；该入口不会自动发送目标：

```bash
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
roslaunch ucar_2026 2026.launch
```

小车主机名当前只解析到无作用域的 IPv6 link-local 地址；启动前必须设置上述 `ROS_IP`，否则 roslaunch 的 XML-RPC 服务可能无法自检并导致无法启动。

RViz 在控制电脑上启动，通过 ROS Master 连接小车；不要在小车端的 `2026.launch` 中启动 RViz。控制电脑的 RViz 预设文件为 `ucar_ws/src/ucar_2026/rviz/navigation_2026.rviz`。

在运行 RViz 的电脑终端设置 ROS 网络环境后加载预设（将 `<控制电脑IP>` 替换为该电脑在小车同一网段的 IP）：

```bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=<控制电脑IP>
rviz -d /path/to/navigation_2026.rviz
```

也可在已打开的 RViz 中选择 **File → Open Config**，打开上述预设文件。

启动后在 RViz 中将 Fixed Frame 设为 `map`，按以下顺序操作：

1. 等待 `map -> odom` TF 稳定；节点会使用 launch 中配置的初始位姿。
2. 如实际位置与初始位姿有偏差，使用 **2D Pose Estimate** 重新定位。

局部膨胀半径更新为 `0.05 m`、全局膨胀半径更新为 `0.40 m` 后，必须重启小车端 `roslaunch ucar_2026 2026.launch`，使 move_base 重新加载代价地图参数。
3. 使用 **2D Nav Goal** 发送导航目标。

当 `move_base` 返回 `Goal reached.` 后，再使用 `rostopic echo -n 1 /odom` 确认线速度和角速度均为 `0.0`；这表示该目标已经结束且底盘没有继续接收运动速度。

2026 任务使用 `lidar_loc` 发布 `map -> odom`；初始位姿由 `2026.launch` 的
`initial_pose_x/y/a` 控制，默认值为 `(-0.25, 2.75, 0)`。它以 `/scan` 匹配静态地图，
并使用同一帧激光时间戳的 `odom -> base_link` 计算 `map -> odom`，避免旋转时将新里程计
和旧激光位姿混用。RViz 的 **2D Pose Estimate** 可在实际摆放位置存在偏差时重新定位。

部署激光定位与局部 TF 修复时上传源码和两个 launch 文件，在小车端构建 `jie_ware` 后，
再用无自动目标模式重启：

```bash
scp ucar_ws/src/ucar_2026/launch/2026.launch \
  ucar@<CAR_HOST>:~/ucar_ws/src/ucar_2026/launch/
scp ucar_ws/src/jie_ware/src/lidar_loc.cpp \
  ucar@192.168.8.231:~/ucar_ws/src/jie_ware/src/
scp ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/launch/

ssh ucar@192.168.8.231 'source /opt/ros/melodic/setup.bash && cd ~/ucar_ws && catkin_make -DCATKIN_WHITELIST_PACKAGES="jie_ware" --pkg jie_ware'

# 在小车上
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
python2 /opt/ros/melodic/bin/roslaunch ucar_2026 2026.launch
```

建图和导航发布的 `/scan` 仍遵循 ROS 标准 `LaserScan` 约定（`+X` 前、`+Y` 左）。但当前 YDLidar 任务地图与 `lidar_loc` 内部的 OpenCV 图像行坐标相配，`lidar_loc.cpp` 必须保留 `y_laser = -range*sin(angle)` 的内部镜像；不要仅依据通用 ROS 约定把它改成正号。更改该符号前必须以静态墙体重合率验证。

当前 CymPlanner 只有一套原始真车直接巡线控制：沿全局路径选择
`0.20 m` 外的第一个目标点，使用直线 `P=1.5、D=0.5` 和角度 `P=2.5、D=0.4`，
不使用航向余弦降速，进入 `0.05 m` 位置容差后执行终点对正。真车命令上限仍为线速度
`0.5 m/s`、行进和末端角速度 `1.0 rad/s`。默认 `point` 模式沿路径前视 `0.25 m`
检查局部代价地图 `cost >= 253`。

CymPlanner 只加载 `$(find cym_planner)/config/ucar_cym_planner_params.yaml`，参数根键为
`cym_planner/CymPlanner`，运行参数位于
`/move_base/cym_planner/CymPlanner/*`。通过 `/ucar/navigation_mode` 切换为
`body_projection` 后，规划器把同一批前视路径点扩大为真实车体 footprint，并投影到
同一控制周期的 local Costmap2D 快照。原始 253 是内切膨胀区，不算实际接触；
254 是致命障碍，255 未知区继续安全拒绝。footprint 尺寸仍读取 local costmap 的
统一车体配置；local map 同时包含 `/map` StaticLayer 和 `/scan_filtered`
ObstacleLayer。`point` 与 `body_projection` 的控制增益和速度已经拆为
`mode1_point`、`mode2_body_projection` 两组；兼容别名 `main_legacy` 等同
`point`，`laser_avoidance` 等同 `body_projection`。修改 C++、头文件或依赖声明后
必须重新构建插件并重启 launch；只修改 YAML 时无需编译，但同样必须重启。

启动后必须同时核对参数与插件日志，避免 YAML 因命名空间错误而被忽略：

```bash
rosparam get /move_base/cym_planner/CymPlanner/linear_x_kd
rosparam get /move_base/cym_planner/CymPlanner/angular_kd
rosparam get /move_base/cym_planner/CymPlanner/obstacle_lookahead_distance
rostopic info /move_base/cym_planner/CymPlanner/debug_map
rostopic info /move_base/cym_planner/CymPlanner/debug_plan
# 插件日志必须包含：initialized in point mode
# 三个参数预期依次为：0.5、0.4、0.25
```

部署后先运行不启动车轮的构建与测试：

```bash
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
catkin_make -DCATKIN_ENABLE_TESTING=ON \
  -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware;yolo2025;ucar_2026"
catkin_make run_tests_cym_planner
catkin_test_results build/test_results/cym_planner
```

重新启动前设置唯一 ROS Master，并禁用自动目标：

```bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
read -r -p '请输入 WSL Master 当前地址: ' MASTER_IP
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
python2 /opt/ros/melodic/bin/roslaunch ucar_2026 2026.launch
```

确认 `/odom_raw` 全部为有限值、`odom -> base_link` 与 `map -> base_link` TF 可用后，
才允许发送首个低风险 RViz 目标。

当前 2026 导航的局部代价地图与全局代价地图使用同一足迹 `0.342 m × 0.256 m`（`±0.171 m`、`±0.128 m`），使全局路径、局部碰撞检查和 RViz 车体模型一致。局部滚动窗口为 `2.0 m × 2.0 m`、分辨率 `0.03 m`、更新/发布频率 `8/3 Hz`；全局更新/发布频率为 `3/5 Hz`；障碍物/清除范围 `3.0/4.0 m`、`inflation_radius: 0.07 m`、`cost_scaling_factor: 4.0`。真机必须保留 `map` / `base_link` 坐标系和 `/scan_filtered` 局部观测，否则会破坏 `lidar_loc` 定位与离群点过滤。全局代价地图 `inflation_radius` 为 `0.205 m`，全局插件顺序为 `static_layer → obstacle_layer → inflation_layer`。`body_projection` 已改为使用同一控制周期的局部代价地图快照，不再依赖全局代价地图发布；`global_costmap/always_send_full_costmap` 可按本文后续性能优化章节设为 `false`。修改代价地图或 CymPlanner 参数后，必须停止并重新执行上述 `roslaunch` 命令，运行中的 `move_base` 不会自动重新加载 YAML。

当前激光数据经 `/scan_raw → navigation_scan_relay → /scan` 中继，距离保持原值；
`lidar_loc` 始终订阅原始 `/scan`。`jie_ware/lidar_filter_node` 以 `0.10 m` 的近邻差阈值
删除单束离群回波并发布 `/scan_filtered`；局部代价地图订阅该过滤话题，CymPlanner
只读取局部代价地图，不再直接订阅激光。全局代价地图继续使用
`/scan_global_obstacles`，不得把定位或全局静态墙过滤改接为 `/scan_filtered`。

修改该滤波链路时，上传 `2026.launch` 和 `costmap_common_params.yaml` 并重启导航；`lidar_filter_node` 已是 `jie_ware` 的构建目标。若小车端尚未构建过该包，先执行 `catkin_make --pkg jie_ware`。无目标启动后先确认滤波话题正常，再进行导航：

```bash
rostopic hz /scan
rostopic hz /scan_filtered
```

`/scan_filtered` 的频率应与 `/scan` 相近；它只会将孤立回波改为 `inf`，不会改变有效束的角度、时间戳或距离。若需要在 RViz 检查效果，请保留原始 `/scan` 显示并额外添加 `/scan_filtered`，不要用滤波话题替换 `lidar_loc` 输入。
部分 YDLidar 驱动会复用 `LaserScan.header.seq`，即使时间戳不同；验证时不得把该序号当作唯一帧键，应比较话题频率与多帧有效回波统计。

`ucar_bringup.launch` 中 `base_link -> laser_frame` 的静态外参为平移
`(-0.11, 0.0, 0.165) m`、yaw `0.0 rad`。该 yaw 必须反映雷达相对车体的实际安装角；之前的 `-0.07 rad` 会令 RViz 中每帧激光相对静态地图恒定顺时针偏转约 `4°`。修改外参后无需重新编译，但必须同步 launch 并完整重启导航/底盘链路，旧 TF 发布进程不会热更新：

```bash
scp ucar_ws/src/ucar_controller/launch/ucar_bringup.launch \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_controller/launch/

# 停止旧的 2026.launch 后，在小车端以无目标模式重新启动。
source /opt/ros/melodic/setup.bash
cd ~/ucar_ws
source devel/setup.bash
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
python2 /opt/ros/melodic/bin/roslaunch ucar_2026 2026.launch
```

重启后保持车静止，先检查 `base_link -> laser_frame` 的 yaw 为 `0`，再在 RViz 对照 `/scan` 和 `/map`。若仍存在恒定角差，只能按实测安装角重新标定该单个 yaw；不得改回定位内部的 `-sin(angle)` 或通过旋转地图掩盖问题。

当前 `navigation_scan_relay.py` 只负责 `/scan_raw → /scan` 转发和全局静态墙过滤；
`lidar_loc` 由正式 `2026.launch` 直接启动，并且是 `map -> odom` 的唯一发布者。默认目标、
二维码扫描和生产路线属于未迁移的历史 `yolo2025/scripts/2026.py`，不属于正式入口。
RViz 的 **2D Nav Goal** 直接发送给 `move_base`。

历史 `yolo2025/scripts/2026.py` 曾在 move_base 就绪后发送一次启动测试目标：
`map (-1.734, 2.305, yaw 1.570796 rad)`，并继续执行二维码定向扫描。以下仅保留当时的
禁用自动目标命令供追溯，当前兼容 wrapper 已不再承载该任务逻辑，**不要执行**：

```bash
roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

## 取消导航速度中继的部署与验证

以下为历史任务部署记录，当前不要执行。在本机完成备份后，当时会将任务脚本及相关文件
同步到小车；`yolo2025/scripts/2026.py` 未迁移、未删除：

```bash
scp ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/launch/
scp ucar_ws/src/yolo2025/scripts/2026.py \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/scripts/
scp ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/cym_planner/config/
scp ucar_ws/src/yolo2025/scripts/2026.py \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/scripts/
scp ucar_ws/src/yolo2025/launch/2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
scp ucar_ws/src/jie_ware/src/lidar_loc.cpp \
  ucar@192.168.8.231:~/ucar_ws/src/jie_ware/src/
scp ucar_ws/src/ucar_controller/config/driver_params_mini.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_controller/config/
```

在小车端验证 Python 语法和 launch 图，再停止旧导航进程并重启。启动时禁用默认测试目标，待确认话题图正确后再从 RViz 发目标：

```bash
cd ~/ucar_ws
python3 -m py_compile src/yolo2025/scripts/2026.py
roslaunch --nodes yolo2025 2026.launch
pkill -f 'roslaunch yolo2025 2026.launch' || true
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

运行后，`rostopic info /cmd_vel` 应显示发布者为 `/move_base`、订阅者为 `/base_driver`，且不应存在 `/teb_cmd_vel`。以下命令应分别得到 `1.0`、`1.0`、`1.0`、`3.0` 与 `3.14`；同时，启动日志应出现 `cym_planner initialized` 且两个速度均为 `1.00`：

```bash
rosparam get /move_base/cym_planner/CymPlanner/max_vel_x
rosparam get /move_base/cym_planner/CymPlanner/max_vel_theta
rosparam get /move_base/cym_planner/CymPlanner/final_yaw_max_vel
rosparam get /base_driver/linear_speed_max
rosparam get /base_driver/angular_speed_max
```

回滚时，从本机 `back/2026-07-14-speed-x5-gain-x6-before-deploy/` 中已校验的两个配置文件重新执行对应 `scp` 命令，再重启上述 launch；不得在小车端保存备份。

历史任务完成静态检查、确认路径清空且急停可用后，曾使用以下命令重启并发送旧
`2026.py` 的默认目标 `map (-1.734, 2.305, yaw -1.570796)`；当前不要执行：

```bash
pkill -INT -f 'roslaunch yolo2025 2026.launch' || true
cd ~/ucar_ws
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
roslaunch yolo2025 2026.launch startup_goal_enabled:=true
```

测试结束后，使用 `rostopic echo -n 1 /move_base/status` 检查结果中是否出现 `status: 3` 和 `Goal reached.`；再检查 `/odom` 的线速度、角速度均为 `0.0`。

## 备份策略

现有完整备份可用于需要时的回滚；常规修改和部署不再额外创建备份。若未来明确需要新备份，只能存放在本地 `D:\WORK\ALLCODE\smartcar2026\back\`；小车端不创建或保留备份目录、压缩包或副本。若发现小车端历史备份，先迁移并校验到本地 `back/`，再从小车端清理。

## 检查插件是否可发现

```bash
rospack plugins --attrib=plugin nav_core | grep cym_planner
```

## QR Code 扫描

二维码序列正常完成并识别至少 `3` 个不同二维码后，任务在 `0.1 s` 后进入中部生产网格。它保留普通 `move_base` 全局路径规划，且在向每一个格心发送目标前调用 `/move_base/make_plan`；无有效路径则零速度停止，不会直线穿墙。网格路线严格读取 `production_square_centers.json`，当前编号顺序为 `2 → 12 → 22 → 32 → 31 → 21 → 11 → 1`，所有停靠点均为 JSON 中的格心。

每个请求的对角路线会先补成格心折线，再逐个交给 `move_base`：规则为先横向、后纵向，因此 `1 → 26` 自动变为 `1 → 6 → 26`（6 与 1 同行、与 26 同列）。每段平移都由 `move_base` 规划到下一格心，并以该段的**到达方向**结束；到达格心后，任务再以 `linear.x=0`、最大 `0.5 rad/s` 的限时原地对准下一段方向，完成后才发送下一段规划目标。这样转向只发生在已指定的格心；若 `8 s` 内仍不能对准即零速度停止，不会在 31 号点无限转向。CymPlanner 始终使用普通模式，任务速度通过 `task_max_vel` 限为 `0.25 m/s`，不发送横向速度。CymPlanner 实际以 `0.05 m` 完成目标；由于 `lidar_loc` 可在 action 回调前后刷新几毫米，回调复核容差为 `0.08 m`，超过该值才零速度停止。

### Dynamic obstacle propagation

启动、默认导航和二维码扫描期间，局部障碍层消费 `/scan_filtered`，全局障碍层消费 `/scan_global_obstacles`。扫码完成、进入中部生产网格时，`navigation_2026` 仅通过 dynamic-reconfigure 禁用**局部**障碍层；全局障碍层保持启用且不清图，因而原有全局代价始终保留。任务停止向 `/scan_global_obstacles` 发布新的扫描，冻结全局障碍层的新增标记和清除射线；定位 `/scan` 不受影响。阶段完成、故障停车或节点退出时，恢复局部障碍层和全局扫描发布，不调用 `/move_base/clear_costmaps`。

### RViz 碰墙判定

`2026.launch` 现在加载仅用于 RViz 的 `robot_description`，因此预设会显示固定于 `base_link` 的蓝色简化车体模型。`navigation_2026` 仍会在 `/navigation_2026/footprint` 发布两个带单位四元数姿态的 Marker：红框为与全局/局部代价地图一致的足迹 `[[0.171, -0.128], [0.171, 0.128], [-0.171, 0.128], [-0.171, -0.128]]`，黄框为其外扩 `0.05 m` 的安全边界。若 `/scan` 的红色点进入黄框，应视为碰墙风险并停车检查；进入红框则视为已接触或定位/激光异常。模型与 Marker 都只用于可视化，不改变代价地图或碰撞控制。

The global scan filter is fail-closed: if the static-map mask is not ready or the
`map <- laser_frame` transform is unavailable at a scan timestamp, historical `2026.py`
publishes an all-infinite scan instead of forwarding vehicle-frame points into
the map-frame global obstacle layer. `observation_persistence` is `0.0`, so a
stale transformed frame is not retained while the vehicle rotates. The local
costmap continues to consume raw `/scan` for immediate collision avoidance, but
its local observation persistence is also `0.0` and its TF tolerance is `0.30 s`;
this prevents old scan poses from leaving a rotating obstacle trail.

The local `plugins` list must be nested under `local_costmap` and contain only
`ObstacleLayer` plus `InflationLayer`. Do not add the static map to this rolling
window: the global layer already owns static walls, while the local layer must
only reflect live collision observations. Its `global_frame` is `map`, matching
the localization output and RViz fixed frame; do not change it back to `odom`
while `lidar_loc` is responsible for localization.

以下是中部生产网格历史任务部署记录，当前不要执行；该能力来自未迁移的
`yolo2025/scripts/2026.py`，不属于正式 `ucar_2026` launch：

```bash
ssh ucar@192.168.8.231 'mkdir -p ~/ucar_ws/src/yolo2025/config'
scp ucar_ws/src/yolo2025/scripts/2026.py ucar@192.168.8.231:~/ucar_ws/src/yolo2025/scripts/
scp ucar_ws/src/yolo2025/launch/2026.launch ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
scp ucar_ws/src/yolo2025/config/production_square_centers.json ucar@192.168.8.231:~/ucar_ws/src/yolo2025/config/
scp ucar_ws/src/yolo2025/CMakeLists.txt ucar@192.168.8.231:~/ucar_ws/src/yolo2025/
scp ucar_ws/src/yolo2025/package.xml ucar@192.168.8.231:~/ucar_ws/src/yolo2025/

cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
chmod +x src/yolo2025/scripts/2026.py
catkin_make -DCATKIN_WHITELIST_PACKAGES="jie_ware;yolo2025"
source devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
roslaunch yolo2025 2026.launch
```

### Task-map global route check

The current task map has a narrow doorway and a slight laser/static-map offset.
Keep the global plugin order as `static_layer`, `obstacle_layer`, then
`inflation_layer`; the global obstacle source must be `/scan_global_obstacles`,
and the global inflation radius must remain `0.21 m`. After deploying
`global_costmap_params.yaml`, restart the launch and use the no-motion probe
below before running the automatic task:

```bash
# 历史命令，仅记录，当前不要执行。
pkill -INT -f 'roslaunch yolo2025 2026.launch' || true
roslaunch yolo2025 2026.launch startup_goal_enabled:=false
rosservice call /move_base/make_plan "start: {header: {frame_id: map}, pose: {position: {x: -0.25, y: 2.75, z: 0.0}, orientation: {w: 1.0}}} goal: {header: {frame_id: map}, pose: {position: {x: -1.734, y: 2.305, z: 0.0}, orientation: {z: 0.707107, w: 0.707107}}} tolerance: 0.0"
```

The response must contain a nonempty `plan.poses` list. Stop the probe with
`pkill -INT -f 'roslaunch yolo2025 2026.launch'`, then start the historical normal task
(record only; do not execute now):

```bash
roslaunch yolo2025 2026.launch
```

Confirm that the filtered global scan is live before the task is started:

```bash
rostopic hz /scan_global_obstacles
```

It should publish while `/scan` continues to feed `lidar_loc` and the
local costmap. The filtered topic intentionally has fewer valid returns because
mapped static walls are removed from global obstacle marking. Until the static
wall mask is ready at launch, this topic intentionally contains no valid
returns; the global static layer remains active throughout that short interval.

The normal launch waits 15 seconds before it begins the startup readiness
check, then requires three seconds of stable `map -> base_link` translation and
five consecutive nonempty global plans. Do not shorten this delay while the
laser localization is still settling after power-on.

二维码扫描节点使用 `/usb_cam/image_raw`，原始识别文本发布到 `/qr_result`。单独使用摄像头和扫描器：

首次部署或修改二维码脚本后，在小车端构建该包。工作区已有 CymPlanner 构建白名单时，保留原包并加入 `yolo2025`：

```bash
source /opt/ros/melodic/setup.bash
cd ~/ucar_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware;yolo2025" --pkg yolo2025
```

```bash
cd ~/ucar_ws
roslaunch yolo2025 qrcode.launch
rostopic echo /qr_result
```

若摄像头已由其他 launch 启动，避免重复占用设备：

```bash
roslaunch yolo2025 qrcode.launch start_camera:=false
```

二维码内容为 `a` 至 `i`（或 URL 路径最后一段为该字母）时，启用接口查询并查看 JSON 结果：

```bash
roslaunch yolo2025 qrcode.launch api_enabled:=true
rostopic echo /qr_api_result
```

接口查询目标默认为 `http://192.168.8.1:3663`。扫描开关兼容旧任务话题：

```bash
rostopic pub -1 /qrcode_start_flag std_msgs/Int8 "data: 1"
rostopic pub -1 /qrcode_start_flag std_msgs/Int8 "data: 0"
```

历史 `roslaunch yolo2025 2026.launch` 配合 `yolo2025/scripts/2026.py` 时，默认导航目标为顺时针 `270°`（二维码 `d`）。到点后每个朝向静止 `3.5 s` 扫描，再由 move_base/CymPlanner 向当前 `map -> base_link` 位置依次发送 `yaw=π` 扫描 `a`、`yaw=-π/2` 扫描 `i`；最终停在顺时针 `90°`（ROS `yaw=-π/2`）。3.5 秒来自实车扫码器约 `2~3.4 s` 的稳定解码延迟。二维码在整个扫描阶段均计入结果。每个定向目标最多等待 `6 s`，超时后任务取消该旋转并扫描当前朝向，避免持续原地旋转。终端输出 `QR_SCAN_RESULT`、`QR_SCAN_GOAL_TIMEOUT` 与 `QR_SCAN_FINISHED`。此段仅为历史记录，当前不要执行；正式 `ucar_2026` launch 不启动该任务。

当二维码序列正常完成且已识别至少 `3` 个不同二维码时，任务禁用局部动态激光障碍层，并冻结全局障碍扫描输入（保留已有全局代价），仍通过 `move_base` 逐个规划生产格心路线 `2 → 12 → 22 → 32 → 31 → 21 → 11 → 1`。若请求中的相邻点不同行也不同列，先自动加入“起点行 + 终点列”的格心，例如 `1 → 26` 变为 `1 → 6 → 26`，再依序发送这些目标。每次到达格心后先执行有限时的原地朝向对准，再发送下一目标；它始终将 `/move_base/cym_planner/CymPlanner/holonomic_mode` 设为 `false`，并以 `0.25 m/s` 限制任务速度。二维码不足、扫描超时、任一全局计划/目标失败、到点复核失败、朝向 `8 s` 未收敛、局部层开关失败或无法读取 `map -> base_link` 时，立即清零速度并停止。

本地修改 CymPlanner 后，小车部署前需重新构建该包：

```bash
source /opt/ros/melodic/setup.bash
cd ~/ucar_ws
catkin_make --pkg cym_planner
source devel/setup.bash
```

二维码扫描依赖 `odom -> base_link` TF 计量实际旋转角度。若该 TF 连续 5 次不可用，任务会立即发布零速度并以 `QR_SCAN_FINISHED reason=odom transform unavailable` 停止扫描；先排除里程计 `NaN` 后再重试。

预期输出包含 `cym_planner_plugin.xml`。

## 手动建图、键盘遥控与替换任务地图

此模式用于实物赛道与静态任务地图不一致时重建地图。它只启动底盘、激光、
原始激光转发和 `gmapping`，不启动 `map_server`、`lidar_loc`、`move_base`、相机或自动任务；
因此启动前必须先在正在运行 `2026.launch` 的终端按 `Ctrl-C`。

部署新脚本和启动文件后，在小车端构建一次 `yolo2025`：

```bash
source /opt/ros/melodic/setup.bash
cd ~/ucar_ws
catkin_make --pkg yolo2025
source devel/setup.bash
```

第一个终端启动 SLAM 建图（默认移动速度为 `0.12 m/s`、转速为 `0.45 rad/s`）：

```bash
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
roslaunch yolo2025 mapping.launch
```

确认终端显示 `slam_gmapping` 已启动后，在**第二个终端**设置相同 ROS 环境并运行：

```bash
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
rosrun yolo2025 mapping_keyboard.py
```

建图 launch 还应显示 `mapping_scan_relay: /scan_raw -> /scan is ready`；这是为了让
`gmapping` 接收底盘激光驱动固定发布的 `/scan_raw`。若 `s` 后持续显示
`Waiting for the map`，先不要按 `t`，检查第一个终端是否有这条日志。

键盘程序需要直接占用第二个终端：`W`/上箭头前进，`X`/下箭头后退，`A`/左箭头左转，
`D`/右箭头右转，空格立即停车。松开按键约 `0.35 s` 后也会自动发布零速度。
默认线速度为 `0.12 m/s`、转速为 `0.45 rad/s`。运行中按 `+` 或 `=` 每次把线速度加
`0.02 m/s`，按 `-` 每次减 `0.02 m/s`（范围 `0.02~0.50 m/s`）；按 `]` 或 `[` 每次把
转速加或减 `0.05 rad/s`（范围 `0.10~1.50 rad/s`）。按 `v` 显示当前两个值。
按 `s` 将当前 `/map` 暂存到 `/tmp`；按 `t` 仅在本次会话已成功保存后，将该地图直接替换
`ucar_nav/maps/iflysse_2026_direct.pgm` 和 `.yaml`，并删除临时地图，不在小车端保留备份。
按 `q` 停车并退出。替换完成后，在两个终端分别退出建图和键盘程序，再重新启动
`roslaunch ucar_2026 2026.launch`，新地图才会被 `map_server` 和导航代价地图加载。

### 从 `ros_map.zip` 恢复任务地图

`ros_map.zip` 的 `ros_map/iflysse_2026_direct.pgm` 和 `.yaml` 是 5 m × 6 m 的任务地图。
恢复时直接替换本地 `ucar_nav/maps/iflysse_2026_direct.*`，不创建新备份。同步到小车后必须
重启导航，运行中的 `map_server` 不会自动读取替换后的文件：

当前归档的 SHA-256 为
`333760de3ca64de36906833f7cea895a52ae9979694dfeb3b45c6a4e0ec1d01a`；其中主地图
PGM 的 SHA-256 为
`2308ab7d197720ec1e50701727ed5a72a1d9ba551c4bf5371126257d867f9f9b`，YAML 为
`1cdad0e7008f827ee37f246722dc79e9a2336a39faaff2a68f7a94458db627eb`。归档还包含
同名的 metadata 和 preview，供本地核验；小车端的 `map_server` 只读取 PGM 和 YAML。

```bash
scp ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.pgm \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/maps/
scp ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/maps/

# 在小车上；先停止旧 launch
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
python2 /opt/ros/melodic/bin/roslaunch ucar_2026 2026.launch
```

## Git 与 GitHub 私有仓库

首次将本工作区发布到 GitHub 前，先确认忽略规则没有遗漏本地归档、模型权重或设备配置：

```bash
git status --short
git check-ignore -v ucar_ws_source.tar.gz \
  simulation/ \
  ucar_ws/src/ucar_yolo/scripts/yolov4.weights \
  ucar_ws/src/xf_mic_asr_offline/config/appid_params.yaml
```

初始化根仓库并完成首次提交后，使用已登录的 GitHub CLI 创建私有仓库并推送：

```bash
git init -b main
git config core.autocrlf false
git add -A
git commit -m "chore: prepare SmartCar 2026 workspace"
gh repo create smartcar2026 --private --source . --remote origin
git push -u origin main
```

后续同步前应先检查变更和忽略项，再提交并推送：

```bash
git status --short
git add <changed-files>
git commit -m "<summary>"
git push
```

仓库默认私有。不要用 `--public` 发布设备地址、地图、模型、厂商资源或任何本机凭据；模型权重和 `appid_params.yaml` 保持为本地配置。

### `simulation_real` 真机发布分支

`simulation_real` 只发布真机导航所需的源码、地图、配置和文档。根目录
`simulation/` 是本地仿真工作区，不得加入该分支；当前分支通过 `.gitignore` 忽略它，
且 `HEAD` 中没有这个目录。发布或复核前执行：

```bash
git switch simulation_real
git status --short
git check-ignore -v --no-index simulation/
git ls-tree -d --name-only HEAD simulation
git push
```

`git check-ignore` 应显示根 `.gitignore` 的 `/simulation/` 规则；最后一条
`git ls-tree` 不应有输出。不要删除或改写 `size-cymplanner` 等既有远端分支。

## 本机 Simulation 部署（WSL Ubuntu 20.04）

将 Windows 工作区同步到 WSL 的 Linux 文件系统后构建，避免在 `/mnt/d` 上运行
Gazebo/CMake 时出现 9P 文件系统阻塞。以下命令仅用于本机仿真：

```bash
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  g++ dos2unix libopencv-dev \
  ros-noetic-gazebo-ros ros-noetic-gazebo-plugins \
  ros-noetic-gazebo-ros-control ros-noetic-navigation \
  ros-noetic-map-server ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui ros-noetic-joint-state-controller \
  ros-noetic-joint-trajectory-controller ros-noetic-position-controllers \
  ros-noetic-control-toolbox

mkdir -p ~/smartcar2026-simulation
rsync -a --exclude build --exclude devel --exclude install \
  /mnt/d/WORK/ALLCODE/smartcar2026/simulation/ \
  ~/smartcar2026-simulation/
find ~/smartcar2026-simulation/src -type f -name '*.py' -exec dos2unix {} \;
find ~/smartcar2026-simulation/src/car3/scripts -type f -name '*.py' -exec chmod +x {} \;
chmod +x ~/smartcar2026-simulation/start_v3_clean.sh

cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
export DISABLE_ROS1_EOL_WARNINGS=1
catkin_make -j2
```

若 Windows/WSL 当前没有分配 `192.168.8.197`（例如未接入小车局域网），只为
本机仿真可在 WSL 回环接口临时添加该地址。不要在控制电脑已持有该实体地址时重复
添加，也不要把这个回环地址用于小车通信：

```bash
sudo ip address add 192.168.8.197/32 dev lo
~/start_ros_master.sh
```

在另一个已设置相同 ROS 环境的终端中启动准备阶段。它不执行取放任务，也不发送
导航目标；`gui:=false rviz:=false` 适合无界面验证：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
export DISABLE_ROS1_EOL_WARNINGS=1
roslaunch car3 task3_prepare.launch gui:=false rviz:=false
```

使用 `gui:=true rviz:=true` 打开 Gazebo 和 RViz。验证时只读检查 ROS Master、节点、
控制器及核心话题：

```bash
rosparam list
rosnode list
rosservice call /controller_manager/list_controllers
rostopic list
```

### Simulation CymPlanner 前视 footprint 验收

该改动增加了 `visualization_msgs` 依赖并修改了 `cym_planner` C++ 源码；首次
同步后必须重新构建该包。以下命令仅用于本机 WSL 仿真，不会启动小车本机
`roscore`。每个 `source` 之后都显式恢复唯一的 WSL ROS Master：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
catkin_make --pkg cym_planner
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
roslaunch car3 task3_prepare.launch gui:=false rviz:=false
```

在另一个已加载相同 ROS 环境的 WSL 终端中，以下只读检查应显示 `0.3`，并在
规划器收到路径后显示 latched Marker：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
rosparam get /move_base/CymPlanner/obstacle_lookahead_distance
rostopic echo -n 1 /move_base/CymPlanner/lookahead_footprint
```

`task3_prepare.launch` 不发送导航目标。若需要发送会导致运动的目标以验证 Marker
位置，先检查 `/odom_raw` 和 `odom -> base_link`、`map -> base_link` TF。只要出现
`NaN` 或 `TF_NAN_INPUT`，先发布零速度并重启导航/底盘里程计链路，确认恢复有限值
和两个 TF 后才可继续。

### Simulation 物块可见性验收

`task3_prepare.launch` 启动完成后，确认 `/gazebo/get_world_properties`
的 `model_names` 同时包含 `cube_0`、`cube_1`、`cube_2`。若模型实体存在但
Gazebo 中不可见，检查 `src/car3/models/cube/model_*.sdf` 的 visual mesh URI
必须是可移植的 `model://cube/meshes/cube_*.obj`，不得包含特定机器的绝对
`file:///...` 路径。`v3_cym_gazebo.launch` 必须在启动 `empty_world.launch`
（即 `gzserver`）之前设置 `GAZEBO_MODEL_PATH`，否则服务端无法解析该 URI。

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
rosservice call /gazebo/get_world_properties
```

### Simulation 场区文字标识验收

食品、日用品和电子产品场区的墙面文字由 `math.world` 的 `wall_food`、
`wall_daily` 和 `wall_electronics` mesh 提供。它们必须使用
`model://sign/meshes/...` URI；`src/car3/models/sign/` 必须同时包含
`model.config` 和 `model.sdf`，使 Gazebo 能从启动前设置的
`GAZEBO_MODEL_PATH` 中定位资源。启动后在 Gazebo 视图确认三块文字墙可见。

```bash
cd ~/smartcar2026-simulation
grep -n 'model://sign/meshes' src/car3/world/math.world
test -f src/car3/models/sign/model.config
test -f src/car3/models/sign/model.sdf
```

## 2026 局部代价地图与仿真对齐

本次仅将真机任务的局部代价地图几何和数值调参与 Task 3 仿真对齐；`map` /
`base_link` 坐标系和 `/scan_filtered` 输入仍是实车专用接口。不要为了文本一致而改用
仿真的 `odom`、`base_footprint` 或原始 `/scan`，否则会破坏 `lidar_loc` 与离群点
过滤链路。

以下为历史任务脚本与局部代价地图对齐记录，当前不要执行。先在本机检查
`yolo2025/scripts/2026.py` 和配置，再同步当时的四个文件；不要在小车端创建备份：

```bash
python3 -m py_compile ucar_ws/src/yolo2025/scripts/2026.py

scp ucar_ws/src/yolo2025/scripts/2026.py \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/scripts/
scp ucar_ws/src/yolo2025/launch/2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
scp ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/launch/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
```

在小车端先静态验证，再停止旧导航并以无自动目标模式启动。以下均为历史命令，当前不要
执行；所有 `source` 完成后都要显式恢复唯一的 WSL Master：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
cd ~/ucar_ws
python2 -m py_compile src/yolo2025/scripts/2026.py
catkin_make --pkg yolo2025
source devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
roslaunch --nodes yolo2025 2026.launch

# 停止旧 2026.launch 后执行；不允许小车端启动 roscore。
python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

启动后先做静态和零运动检查，只有 `/odom_raw` 为有限值且两个 TF 都可用时，才允许
在 RViz 发送导航目标：

```bash
rosparam get /move_base/local_costmap/footprint
rosparam get /move_base/local_costmap/width
rosparam get /move_base/local_costmap/height
rosparam get /move_base/local_costmap/resolution
rosparam get /move_base/local_costmap/inflation_layer/inflation_radius
rosparam get /move_base/local_costmap/inflation_layer/cost_scaling_factor
rostopic echo -n 1 /odom_raw
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map base_link
```
### 完整自动任务

完整任务会执行默认启动目标、`d → a → i` 二维码方向扫描，随后按
`2 → 12 → 22 → 32 → 31 → 21 → 11 → 1` 运行生产路线。正式导航与 RViz
先以无运动模式启动并完成安全门检查，然后在车端停止该 launch，再用以下命令启用任务：

```bash
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
roslaunch yolo2025 2026.launch full_task_enabled:=true
```

完整任务模式由 `navigation_2026` 独占 `/scan_raw → /scan` 和
`/scan_global_obstacles` 转发，独立 `navigation_scan_relay` 不启动；二维码节点复用
正式入口中的 `/usb_cam`，不会重复打开摄像头。

## 2026 CymPlanner 两模式参数与受限横移

`ucar_cym_planner_params.yaml` 的控制参数分为两套：

- `mode1_point/*`：二维码/普通点碰撞阶段；
- `mode2_body_projection/*`：生产路线真实车体阶段。

修改生产路线速度优先调整模式 2 的 `max_vel_x`、`max_vel_theta` 和
`heading_slowdown_min_scale`。后者为紧转弯时的最低线速度比例；实际比例为
`max(该值, cos(abs(航向误差))²)`。YAML 修改后必须重启 move_base 才会加载。
`command_sweep_time` 和 `command_sweep_step` 控制候选 Twist 的未来车体扫掠，
生产模式默认 `0.40 s`、`0.025 s`，不应为了提速而关闭。模式 2 的
`command_sweep_time` 若配置为 `0` 或负数，规划器会记录错误并强制恢复为
`0.40 s`；`command_sweep_step` 若为 NaN、Inf、`0` 或负数，会强制恢复为
`0.025 s`。若日志出现 `control cycle exceeded 50 ms`，先停止实车运动，再
评估代价地图大小和 `command_sweep_step`，不得通过关闭
`command_sweep_time` 绕过保护。

受限横移当前必须保持 `escape_enabled: false`。首版实现还缺少完整状态保持、
全接触格统计和本地动态障碍检查；当前源码也会拒绝任何 `true` 请求。
模式切换只能在车辆已停止、两个导航目标之间执行，不允许运动中切换。

同步修改时不得在小车端创建备份目录。以下命令只部署、构建和测试，不启动 ROS、
不发布速度：

```bash
export VEHICLE_HOST="ucar@<按 rosmaster/NETWORK_CONFIGURATION.md 发现的当前小车 IP>"
scp ucar_ws/src/cym_planner/CMakeLists.txt \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/"
scp ucar_ws/src/cym_planner/src/cym_planner.cpp \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/src/"
scp ucar_ws/src/cym_planner/include/cym_planner.h \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/include/"
scp ucar_ws/src/cym_planner/include/cym_planner/escape_recovery.h \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/include/cym_planner/"
scp ucar_ws/src/cym_planner/include/cym_planner/planner_tuning.h \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/include/cym_planner/"
scp ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/config/"
scp ucar_ws/src/cym_planner/test/escape_recovery_test.cpp \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/test/"
scp ucar_ws/src/cym_planner/test/planner_tuning_test.cpp \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/test/"

ssh "${VEHICLE_HOST}"
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
catkin_make --pkg cym_planner -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_cym_planner
catkin_test_results --verbose build/test_results/cym_planner
```

实车验证必须等待用户明确确认并把车放回固定起点。启动前仍按本文件的安全门检查
`/odom_raw`、`odom → base_link`、`map → base_link` 和雷达有限值；任一 NaN 或
`TF_NAN_INPUT` 都必须先零速停机并重启导航/里程计链路。启动后先只读确认：

```bash
rosparam get /move_base/cym_planner/CymPlanner/mode1_point
rosparam get /move_base/cym_planner/CymPlanner/mode2_body_projection
rosparam get /move_base/cym_planner/CymPlanner/escape_enabled
```

第三条必须为 `false`。若不是，立即停止任务，不得进入实车运动。

## 2026 相机按需采集与 local footprint 性能优化

`2026.launch` 中的 `/usb_cam` 节点保持存在，但默认以
`camera_start_suspended:=true` 启动，不使用 RViz/相机时不采集。
任务到达 52 且停车后才开启 QR 相机；QR 结束到 OCR 模型加载完成期间再次暂停；
连续 OCR 路线开始前重新开启；成功、异常或 shutdown 时暂停。这里的“暂停”是
`VIDIOC_STREAMOFF`，减少 USB/转换/发布负载但不释放设备 fd。若
`use_ros_camera_for_ocr=false` 需要原生 V4L2，仍必须走任务已有的精确
`rosnode kill /usb_cam` 释放路径。

QR 每个朝向的 `qr_search_timeout`（默认 `4.0 s`）只是尚未识别时的搜索上限；
一旦收到新的、符合去重规则的 QR 回调，任务立刻进入下一个朝向，不会为该参数
额外停留。旧参数 `qr_hold_seconds` 仅作为未迁移旧 launch 的兼容回退。
手工 RViz 确实需要立即图像时，显式使用
`camera_start_suspended:=false`。最终停止服务连续两次失败时，任务会精确停止
`/usb_cam` 节点作为资源释放兜底；任务模式下该节点不 respawn。

CymPlanner 的 `debug_images_enabled` 默认必须保持 `false`。模式 2 使用 local
Costmap2D 的单周期快照，不再订阅 `/move_base/global_costmap/costmap`。局部代价
地图必须同时包含 `/map` StaticLayer 和 `/scan_filtered` ObstacleLayer；
`expected_update_rate: 0.30` 配合 `costmap_ros_->isCurrent()` 令雷达断流失败关闭。
raw 253 不表示真实 footprint 接触，raw 254/255 必须阻挡。

完成依赖移除后，global costmap 可以设置：

```yaml
global_costmap:
  always_send_full_costmap: false
```

GlobalPlanner 读取 move_base 进程内的 global Costmap2D，不依赖完整 OccupancyGrid
发布；不得在旧版仍订阅 full costmap 时单独修改这个参数。

只部署、构建和测试，不启动 ROS：

```bash
export VEHICLE_HOST="ucar@<当前小车 IP>"

scp ucar_ws/src/cym_planner/src/cym_planner.cpp \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/src/"
scp ucar_ws/src/cym_planner/include/cym_planner.h \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/include/"
scp ucar_ws/src/cym_planner/include/cym_planner/global_cost_semantics.h \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/include/cym_planner/"
scp ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/config/"
scp ucar_ws/src/cym_planner/test/global_cost_semantics_test.cpp \
  "${VEHICLE_HOST}:~/ucar_ws/src/cym_planner/test/"

scp ucar_ws/src/ucar_2026/launch/2026.launch \
  "${VEHICLE_HOST}:~/ucar_ws/src/ucar_2026/launch/"
scp ucar_ws/src/ucar_2026/scripts/production_task_2026.py \
  "${VEHICLE_HOST}:~/ucar_ws/src/ucar_2026/scripts/"
scp ucar_ws/src/ucar_2026/test/test_production_task_geometry.py \
  "${VEHICLE_HOST}:~/ucar_ws/src/ucar_2026/test/"
scp ucar_ws/src/ucar_2026/package.xml \
  "${VEHICLE_HOST}:~/ucar_ws/src/ucar_2026/"

scp ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml \
  "${VEHICLE_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"
scp ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml \
  "${VEHICLE_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"
scp ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_params.yaml \
  "${VEHICLE_HOST}:~/ucar_ws/src/ucar_nav/config/testnav20260721/"

ssh "${VEHICLE_HOST}"
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
catkin_make --pkg cym_planner ucar_2026 -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_cym_planner
catkin_make run_tests_ucar_2026
catkin_test_results --verbose build/test_results/cym_planner
catkin_test_results --verbose build/test_results/ucar_2026
```

用户确认静态验证时，必须先启动唯一的 WSL Master，完成所有 `source` 后再设置本次
动态发现的 `ROS_MASTER_URI`/`ROS_IP`。任务保持禁用或把启动延时调长，依次检查：

```bash
rosparam get /move_base/cym_planner/CymPlanner/debug_images_enabled
rosparam get /move_base/global_costmap/always_send_full_costmap
rosparam get /move_base/local_costmap/plugins
rosparam get /move_base/local_costmap/obstacle_layer/laser_scan_sensor/expected_update_rate
rosservice call /usb_cam/start_capture
rostopic echo -n 1 /usb_cam/image_raw/header
rosservice call /usb_cam/stop_capture
```

前两项应为 `false`；local plugins 应包含 static、obstacle、inflation。停止相机后应
不再产生新图，重新开启后必须出现更新后的时间戳。随后只做静态 make_plan，
确认 global planner 仍能生成路径。暂停 `/scan_filtered` 的断流验证必须确认
CymPlanner 报 local costmap not current 并保持零速。任何 NaN、TF_NAN、地图墙
缺失、相机无新帧或重复控制周期超 50 ms 都应立即停止，不得进入完整任务。
