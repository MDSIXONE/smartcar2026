# 操作命令

## WSL ROS Master 与 RViz

本机 WSL 的 Ubuntu 20.04 使用 ROS Noetic，只作为局域网 ROS Master 和 RViz 客户端；
地址固定为控制电脑的 192.168.8.197。在 WSL 终端执行以下命令启动 Master：

    ~/start_ros_master.sh

WSL 的 ROS 环境已设置 DISABLE_ROS1_EOL_WARNINGS=1，因此不会再显示 ROS 1
Noetic 生命周期结束的提示窗口。

首次使用前，必须在 Windows 的管理员 PowerShell 放行仅来自小车的入站 TCPROS：

    $wslVm = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
    New-NetFirewallHyperVRule -DisplayName 'ROS TCPROS from UCar to WSL' -Direction Inbound -VMCreatorId $wslVm -Protocol TCP -LocalPorts Any -RemoteAddresses 192.168.8.231 -Action Allow -Enabled True -Profiles Any
    New-NetFirewallRule -DisplayName 'ROS TCPROS from UCar to WSL' -Direction Inbound -Action Allow -Profile Any -Protocol TCP -RemoteAddress 192.168.8.231

这两条规则只允许小车 IP 访问 WSL 的 TCP 端口；ROS 话题使用动态 TCPROS 端口，不能只开放 11311。

另开一个 WSL 终端即可运行：

    ~/start_rviz.sh

若 Windows 任务栏只有 RViz 图标、无法显示主窗口，说明 WSLg 远程应用会话卡住。在
Windows PowerShell 执行以下命令重建图形会话，再运行上述 Master 和 RViz 启动命令：

    wsl --shutdown

小车端必须停止已有的本机 Master，并在启动 ROS 节点前设置：

    export ROS_IP=192.168.8.231
    export ROS_MASTER_URI=http://192.168.8.197:11311

之后再启动 roslaunch yolo2025 2026.launch。不要在小车端再执行 roscore。

## ROS Melodic 终端恢复

小车的 ROS Melodic 工具必须由 Python 2 运行，而系统 `/usr/bin/python` 当前为 Python 3。`.bashrc` 已将 `roslaunch` 定义为调用 `python2 /opt/ros/melodic/bin/roslaunch` 的 shell 函数，并且每个 ROS overlay 只加载一次。

若终端出现 `Argument list too long`，当前 shell 的环境变量已膨胀，不能继续 source 或启动命令。关闭该终端并新开一个终端，然后验证：

```bash
type roslaunch
```

预期输出应包含 `roslaunch is a function`。如需绕过 shell 函数，可显式使用：

```bash
python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

不要在同一终端重复执行 `source ~/ucar_ws/devel/setup.bash`；新的终端已由 `.bashrc` 加载工作区。

`yolo2025/scripts/2026.py` 必须以 Python 2 运行；其 shebang 已固定为 `#!/usr/bin/env python2`，以便使用 ROS Melodic 的 `tf` 和 `nav_msgs.srv` 模块。不要将其改回 Python 3。

## CymPlanner 真机构建与启动

在小车上确认没有已有导航 launch 正在运行后执行：

```bash
cd ~/ucar_ws
# 新开终端的 .bashrc 已加载 ROS 和工作区；不要在这里重复 source setup.bash。
# 此工作区原先只白名单构建 xf_tts_offline；需要把 CymPlanner 加入构建图。
catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware"
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.231:11311
roslaunch yolo2025 2026.launch
```

当 CymPlanner、2026 任务脚本或 2026 地图有变更时，先从本机同步以下文件，再在小车端执行上面的 `catkin_make`。常规变更不创建新备份：

```bash
scp ucar_ws/src/cym_planner/src/cym_planner.cpp \
  ucar@192.168.8.231:~/ucar_ws/src/cym_planner/src/
scp ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/cym_planner/config/
scp ucar_ws/src/yolo2025/scripts/2026.py \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/scripts/
scp ucar_ws/src/yolo2025/launch/2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
scp ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.pgm \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/maps/
```

构建只写入新的库文件，不会替换已运行进程内存中的插件。请在当前 launch 终端按 `Ctrl-C` 后，使用以下命令手动启动；`startup_goal_enabled:=false` 会确保启动时不自动行车：

```bash
cd ~/ucar_ws
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.231:11311
roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

小车主机名当前只解析到无作用域的 IPv6 link-local 地址；启动前必须设置上述 `ROS_IP`，否则 roslaunch 的 XML-RPC 服务可能无法自检并导致无法启动。

RViz 在控制电脑上启动，通过 ROS Master 连接小车；不要在小车端的 `2026.launch` 中启动 RViz。控制电脑的 RViz 预设文件为 `ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`。

在运行 RViz 的电脑终端设置 ROS 网络环境后加载预设（将 `<控制电脑IP>` 替换为该电脑在小车同一网段的 IP）：

```bash
export ROS_MASTER_URI=http://192.168.8.231:11311
export ROS_IP=<控制电脑IP>
rviz -d /path/to/navigation_2026.rviz
```

也可在已打开的 RViz 中选择 **File → Open Config**，打开上述预设文件。

启动后在 RViz 中将 Fixed Frame 设为 `map`，按以下顺序操作：

1. 等待 `map -> odom` TF 稳定；节点会使用 launch 中配置的初始位姿。
2. 如实际位置与初始位姿有偏差，使用 **2D Pose Estimate** 重新定位。

局部膨胀半径更新为 `0.05 m`、全局膨胀半径更新为 `0.40 m` 后，必须重启小车端 `roslaunch yolo2025 2026.launch`，使 move_base 重新加载代价地图参数。
3. 使用 **2D Nav Goal** 发送导航目标。

当 `move_base` 返回 `Goal reached.` 后，再使用 `rostopic echo -n 1 /odom` 确认线速度和角速度均为 `0.0`；这表示该目标已经结束且底盘没有继续接收运动速度。

`lidar_loc` 替换了 AMCL；初始位姿由 `2026.launch` 的 `initial_pose_x/y/a` 控制，默认值为 `(-0.25, 2.75, 0)`。

当前 CymPlanner 的正常命令上限为线速度 `max_vel_x: 0.5 m/s`、横向速度 `max_vel_y: 0.1 m/s`、行进和末端对准角速度 `max_vel_theta: 1.0 rad/s`、`final_yaw_max_vel: 1.0 rad/s`。线速度 P/D 参数为 `linear_x_gain: 1.5`、`linear_x_kd: 0.5`；行进航向角速度 P/D 参数为 `angular_gain: 2.5`、`angular_kd: 0.4`。最终朝向和靠近目标点的增益仍为 `final_yaw_gain: 2.0`、`final_linear_x_gain: 1.0`。`move_base` 直接发布 `/cmd_vel`，不经过 `2026.py` 的速度中继或缩放；底盘驱动限幅仍为线速度 `3.0 m/s`、角速度 `3.14 rad/s`。这些是 ROS 命令上限，绝不等同于实际车速；`carry_speed_scale` 保持 `1.0`，因为源代码会将它钳制在 `1.0` 以内。

CymPlanner 只保留并加载 `$(find cym_planner)/config/ucar_cym_planner_params.yaml`；参数根键为 `cym_planner/CymPlanner`。插件同时兼容 move_base 传入的短名称和完整名称，并始终回退读取该规范命名空间，防止参数缺失时悄悄退回到源码默认的 `0.2 m/s`、`0.5 rad/s`。正常行进的线速度不再按车头与路径夹角做 25%～100% 的额外缩放；仍受 `max_vel_x`、搬运模式比例、碰撞检查和底盘 `linear_speed_max` 限制。修改 `cym_planner.cpp` 后必须先执行本节的 `catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware"`，再重启 launch 才会加载新插件库。旧 JSON 示例已删除，不参与真机运行。

当前 2026 导航使用的局部代价地图障碍层 `inflation_radius` 为 `0.05 m`，全局代价地图 `inflation_radius` 为 `0.21 m`。全局插件顺序为 `static_layer → obstacle_layer → inflation_layer`。`2026.py` 将原始 `/scan_raw` 原样转发为 `/scan` 供定位和局部避障使用，同时发布 `/scan_global_obstacles` 给全局障碍层；该话题会滤掉落在静态墙 `0.22 m` 范围内或落在静态地图范围外的回波，避免轻微错位的激光墙重复封死窄门。修改代价地图或 CymPlanner 参数后，必须停止并重新执行上述 `roslaunch` 命令，运行中的 `move_base` 不会自动重新加载 YAML。

当前激光数据经 `/scan_raw → 2026.py → /scan` 中继；`scan_scale` 为 `1.0`，因此 `/scan` 保持原始距离。`jie_ware` 与代价地图均订阅 `/scan`。

`navigation_2026` 仅负责 `/scan_raw → /scan` 转发和可选的单次默认目标；它不包含历史验证路线、AMCL、TEB、语音唤醒或 RViz 目标观察。RViz 的 **2D Nav Goal** 直接发送给 `move_base`。

当前 `2026.launch` 默认在 move_base 就绪后发送一次启动测试目标：`map (-1.734, 2.305, yaw 1.570796 rad)`。该 `yaw` 对应起点朝向的顺时针 `270°`，使 CymPlanner 到点时已对准二维码 `d`；后续朝向 `π`、`-π/2` 也由 move_base/CymPlanner 向同一坐标发送目标，最终停在顺时针 `90°`（二维码 `i`）。`navigation_2026` 会在定位有效且 `/move_base/make_plan` 已能生成全局路径后才发送该目标，最长等待 `90` 秒；因此启动后短暂停车属于正常的安全等待，而不是速度限制。如需只进行 RViz 手动导航，使用以下命令禁用该自动目标：

```bash
roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

## 取消导航速度中继的部署与验证

在本机完成备份后，将以下四个文件同步到小车；不在小车端创建备份：

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
export ROS_MASTER_URI=http://192.168.8.231:11311
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

完成静态检查、确认路径清空且急停可用后，以下命令会重启并等待定位、全局路径就绪，再发送 `2026.launch` 内置的默认目标 `map (-1.734, 2.305, yaw -1.570796)`：

```bash
pkill -INT -f 'roslaunch yolo2025 2026.launch' || true
cd ~/ucar_ws
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.231:11311
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

扫码完成后的三点路线由独立的一次性 ROS 定时回调在 `0.1 s` 后启动；该短暂延时只用于让扫码定时器完全退出，不会改变二维码扫描停留时间、目标点或速度。后三点在目标已经发送后才用普通终端输出 `[POST_QR] FIRST_GOAL`、`SECOND_GOAL` 和 `THIRD_GOAL` 状态，避免 ROS 日志通道阻塞任务控制。若终端已显示三个 `QR_SCAN_RESULT` 但未出现 `[POST_QR] FIRST_GOAL`，应先停止并重新启动 `roslaunch yolo2025 2026.launch`，不要用手动目标覆盖该任务状态。

当前扫码后三点以 `2026.launch` 为准：第一点 `map (-1.737, 1.003, yaw=3.140)`，第二点 `map (-1.722, -0.269, yaw=-3.140)`，第三点 `map (-2.265, -0.001, yaw=-1.557)`。第一、二点为非全向，第三点为全向；三段平移速度均由 `task_linear_speed=0.1` 限制。

### Dynamic obstacle propagation

After the first post-QR goal succeeds, the second and third goals run in holonomic mode. Laser obstacles are loaded into both costmaps. The local costmap consumes unfiltered `/scan`; the global 2D obstacle layer consumes `/scan_global_obstacles`, which removes returns already represented by the static map before normal inflation. Thus newly scanned obstacles can still trigger global replanning without duplicate static walls closing a narrow doorway. The global costmap and global planner run at `3 Hz`; the local costmap runs at `8 Hz` with a `1.8 m × 1.8 m` window. CymPlanner checks `0.8 m` of the path ahead, matching the original standalone planner's safety horizon.

Deploy these files together, then restart the navigation launch so move_base reloads all costmap layers:

```bash
scp ucar_ws/src/yolo2025/scripts/2026.py ucar@192.168.8.231:~/ucar_ws/src/yolo2025/scripts/
scp ucar_ws/src/yolo2025/launch/2026.launch ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
scp ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml ucar@192.168.8.231:~/ucar_ws/src/cym_planner/config/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/global_costmap_params.yaml ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/costmap_common_params.yaml ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/move_base_params.yaml ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
pkill -INT -f 'roslaunch yolo2025 2026.launch' || true
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
pkill -INT -f 'roslaunch yolo2025 2026.launch' || true
roslaunch yolo2025 2026.launch startup_goal_enabled:=false
rosservice call /move_base/make_plan "start: {header: {frame_id: map}, pose: {position: {x: -0.25, y: 2.75, z: 0.0}, orientation: {w: 1.0}}} goal: {header: {frame_id: map}, pose: {position: {x: -1.734, y: 2.305, z: 0.0}, orientation: {z: 0.707107, w: 0.707107}}} tolerance: 0.0"
```

The response must contain a nonempty `plan.poses` list. Stop the probe with
`pkill -INT -f 'roslaunch yolo2025 2026.launch'`, then start the normal task:

```bash
roslaunch yolo2025 2026.launch
```

Confirm that the filtered global scan is live before the task is started:

```bash
rostopic hz /scan_global_obstacles
```

It should publish while `/scan` continues to feed lidar localization and the
local costmap. The filtered topic intentionally has fewer valid returns because
mapped static walls are removed from global obstacle marking. Until the static
wall mask is ready at launch, this topic intentionally contains no valid
returns; the global static layer remains active throughout that short interval.

The normal launch waits 15 seconds before it begins the startup readiness
check, then requires three seconds of stable `map -> base_link` translation and
five consecutive nonempty global plans. Do not shorten this delay while the
lidar localization is still settling after power-on.

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

`roslaunch yolo2025 2026.launch` 的默认导航目标已是顺时针 `270°`（二维码 `d`）。到点后每个朝向静止 `3.5 s` 扫描，再由 move_base/CymPlanner 向当前 `map -> base_link` 位置依次发送 `yaw=π` 扫描 `a`、`yaw=-π/2` 扫描 `i`；最终停在顺时针 `90°`（ROS `yaw=-π/2`）。3.5 秒来自实车扫码器约 `2~3.4 s` 的稳定解码延迟。二维码在整个扫描阶段均计入结果。每个定向目标最多等待 `6 s`，超时后任务取消该旋转并扫描当前朝向，避免持续原地旋转。终端输出 `QR_SCAN_RESULT`、`QR_SCAN_GOAL_TIMEOUT` 与 `QR_SCAN_FINISHED`。该任务会启动并独占摄像头，不要同时运行独立的 `qrcode.launch`。

当二维码序列正常完成且已识别至少 `3` 个不同二维码时，任务以 `0.1 m/s` 的平移限速依次执行三点：第一点 `map (-1.134, 1.505, yaw=π)`；第二点为第一点到达后的实时 `map -> base_link` 位置向 `-Y` 移动 `1.2 m`，`yaw=π`；第三点 `map (-2.134, -0.095, yaw=π/2)`。二维码不足、扫描超时、定向失败或无法读取第二点实时位姿时，不会继续后续目标。

第一、二点使用非全向模式。发送第三点前任务写入 `/move_base/cym_planner/CymPlanner/holonomic_mode=true`，规划器按车体系位置误差输出 `linear.x` 和 `linear.y`，并保持 `angular.z=0`；仅在位置进入 `0.05 m` 到点阈值后才原地旋转至最终 `90°` 朝向。`task_max_vel=0.1` 只限制上述三段的平移速度，第三点完成或任一阶段失败后自动清零并恢复普通模式。定位节点为 `lidar_loc`，不应添加仅 AMCL 支持的 `odom_model_type` 参数。

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
export ROS_MASTER_URI=http://192.168.8.231:11311
roslaunch yolo2025 mapping.launch
```

确认终端显示 `slam_gmapping` 已启动后，在**第二个终端**设置相同 ROS 环境并运行：

```bash
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.231:11311
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
`roslaunch yolo2025 2026.launch`，新地图才会被 `map_server` 和导航代价地图加载。

## Git 与 GitHub 私有仓库

首次将本工作区发布到 GitHub 前，先确认忽略规则没有遗漏本地归档、模型权重或设备配置：

```bash
git status --short
git check-ignore -v ucar_ws_source.tar.gz \
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
