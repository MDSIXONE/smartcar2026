# 任务运行中的 RViz 调试观察

本文说明如何在任务已经运行时，用本机 RViz 只读观察仿真或实机状态。RViz 本身不启动
`roscore`、导航、底盘或相机，不会接管 `/cmd_vel`；但 RViz 中的交互工具可能发布目标，
因此实机观察时只做显示，不点击会发布消息的工具。

## 0. 现场可改参数总表（无需重新编译）

本节只列现场调试最常用、改配置后不需要重新编译的内容。YAML、XML launch 和 JSON
都是运行时读取的配置：**不用 `catkin_make`，但通常必须停止旧节点并重新启动对应
launch，运行中的节点不会自动读取文件新值**。当前项目的实际加载链路是：

```text
ucar_2026/launch/2026.launch
ucar_2026_national/launch/2026.launch
ucar_2026_extra/launch/2026.launch
        └─ ucar_nav/launch/cym_move_base_omni_2026.launch
             ├─ ucar_nav/config/testnav20260721/*.yaml
             └─ cym_planner/config/ucar_cym_planner_params.yaml
        └─ ucar_controller/launch/ucar_bringup.launch
             └─ ucar_controller/config/driver_params_mini.yaml
        └─ production_task_2026 的 launch 参数和任务 JSON
```

### 0.1 先停、再改、再启动

现场改参数前先取消任务，在车端动态设置好的 ROS 环境中发布零速度：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

修改本机文件后同步单个文件到车端；`<CAR_IP>` 使用当前网络发现到的地址，不要固定写旧
IP，也不要在车端建立备份目录：

```bash
CAR_IP=<当前车端IP>
scp ucar_ws/src/<包名>/<文件> \
  ucar@$CAR_IP:~/ucar_ws/src/<包名>/<文件>
sha256sum ucar_ws/src/<包名>/<文件>
ssh ucar@$CAR_IP 'sha256sum ~/ucar_ws/src/<包名>/<文件>'
```

然后停止旧的 `2026.launch`，按国赛或额外任务重新启动。只改导航 YAML 时至少要重启
`move_base`；改底盘 YAML、任务 launch 或 JSON 时直接重启对应的完整 `2026.launch` 最
清楚。重启后先检查 `/odom_raw` 是有限值，且 `odom -> base_link`、`map -> base_link`
TF 正常，再发送运动目标。

### 0.2 现场最常改的速度、转向和碰撞参数

文件：
`ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
（车端：`~/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`）。该文件由
`ucar_nav/launch/cym_move_base_omni_2026.launch` 加载。

| 参数位置 | 作用 | 当前/使用说明 |
| --- | --- | --- |
| `mode1_point.linear_x_gain`、`max_vel_x` | 普通 `point` 模式前向响应和速度上限 | 当前 `1.5`、`0.35 m/s`；增大 `linear_x_gain` 会更快顶到上限，不是独立物理加速度参数 |
| `mode1_point.angular_gain`、`max_vel_theta` | 普通模式转向 P 和角速度上限 | 现场转弯慢时优先小幅调整；过大可能摆动 |
| `mode1_point.heading_slowdown_min_scale` | 大航向误差时保留的最低线速度比例 | 当前 `0.00`；数值越大，转弯时越不容易完全降为零 |
| `mode1_point.final_yaw_gain`、`final_yaw_max_vel`、`final_yaw_tolerance` | 到点后的朝向对准 | 当前容差 `0.05 rad`；改小更准但更容易超时 |
| `mode1_point.final_linear_x_gain` | 到点末段是否继续用线速度修位置 | 当前 `0.0`；不要用它解决地图点坐标错误 |
| `mode2_body_projection.*` | `body_projection`/`footprint` 模式的同类速度、转向、到点和扫掠参数 | 当前 `max_vel_x=0.07`、`max_vel_theta=0.35`；只有确认实际切到该模式时才调这一组 |
| `mode3_sprint.linear_x_gain`、`max_vel_x` | 国赛 70→288 冲刺段前向响应和速度上限 | 当前 `13.5`、`2.7 m/s`；本项目没有独立加速度字段，增大 gain 会增加麦轮打滑风险 |
| `mode3_sprint.angular_gain`、`max_vel_theta` | 冲刺段航向响应和角速度上限 | 当前 `5.0`、`0.80 rad/s`；航向 P 已按现场试跑减半 |
| `mode3_sprint.approach_decel_distance`、`approach_min_vel_x` | 接近冲刺终点时开始减速及最低速度 | 当前 `1.0 m`、`0.12 m/s`；高速试跑刹不住时先检查这里 |
| `mode3_sprint.final_yaw_*`、`final_linear_x_gain` | 冲刺终点最后对向和位置回拉 | 当前 `final_yaw_tolerance=0.05`、`final_linear_x_gain=0.6` |
| `mode3_sprint.lateral_gain`、`max_vel_y` | `transverse` 横向平移实验参数 | 当前 `12.5`、`2.5 m/s`；只有 `sprint_transverse_enabled=true` 才使用 |
| `carry_speed_scale` | 载物模式速度倍率 | 取值不要超过 `1.0`；它不是基础速度上限 |
| `obstacle_lookahead_distance`、`obstacle_cost_threshold` | 前视距离和触发障碍判定的代价值 | 调整前先看 RViz 的 local costmap，不要用增大阈值掩盖真实碰撞 |
| `elastic_enabled`、`elastic_*` | body projection 的局部弹性绕行 | 属于安全策略参数，现场只做小范围对比；`escape_enabled` 当前被源码硬关闭，改成 `true` 也不会启用 |
| `debug_images_enabled` | 是否发布 CymPlanner 调试图像 | 当前 `false`；临时改 `true` 要重启导航，并会增加 Wi-Fi/WSLg 负载 |

任务扫描旋转参数位于三套流程各自的 `launch/2026.launch`，不在 CymPlanner YAML 中：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `fixed_heading_rotation_speed` | `0.70 rad/s` | 二维码固定面之间的同点原地转向 |
| `qr_rotation_speed` | `0.18 rad/s` | 二维码未在固定面识别到时的完整 360°扫描 |
| `ocr_scan_rotation_speed` | `0.35 rad/s` | OCR 的完整 360°扫描 |

三项只影响任务节点的旋转指令；修改后停止旧任务并重启对应 `2026.launch`，运行中的节点不会热加载。

速度的实际生效上限还受底盘驱动文件限制。文件：
`ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`。

| 参数 | 作用 | 当前/注意事项 |
| --- | --- | --- |
| `linear_speed_max` | 底盘对 `linear.x`、`linear.y` 的最终裁剪 | 当前 `3.0 m/s`；实际速度不会超过规划器上限和此值中的较小者 |
| `angular_speed_max` | 底盘对 `angular.z` 的最终裁剪 | 当前 `3.14 rad/s` |
| `linear_cali`、`angular_cali` | 轮速换算校准，不是常规加速旋钮 | 只有在实测“指令速度与 odom/实际速度比例错误”时调整 |
| `cmd_timeout` | 多久收不到速度命令后自动置零 | 当前 `0.2 s`，属于安全参数，不要为了消除断续随意调大 |

修改 `driver_params_mini.yaml` 后必须重启底盘驱动；不要为了提高速度现场随意改
`wheel_radius`、`base_shape_a`、串口 `port` 或 `baud`，这些会改变里程计/运动学或直接
导致底盘无法通信。

### 0.3 导航和代价地图参数

以下文件都由同一个导航 launch 加载，修改后不需要编译，但要重启 `move_base`：

| 文件 | 现场可调位置 | 用途 |
| --- | --- | --- |
| `ucar_ws/src/ucar_nav/config/testnav20260721/move_base_params.yaml` | `controller_frequency`、`controller_patience`、`planner_frequency`、`planner_patience`、`oscillation_timeout`、`oscillation_distance` | 控制循环、重规划和振荡判定；当前 `planner_frequency=0.0` 是事件驱动重规划，不要直接改回高频而不做对比 |
| `ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_common.yaml` | `footprint`、`obstacle_range`、`raytrace_range`、`inflation_radius`、`cost_scaling_factor` | 全局障碍和安全膨胀；当前常态膨胀半径 `0.224 m` |
| `ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml` | `footprint`、`obstacle_range`、`raytrace_range`、`expected_update_rate`、`inflation_radius`、`cost_scaling_factor` | 局部激光障碍和实时安全；当前常态膨胀半径 `0.224 m` |
| `ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml` | `update_frequency`、`publish_frequency`、`width`、`height`、`resolution` | 局部地图刷新和窗口大小；当前窗口 `1.0 m × 1.0 m` |
| `ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_params.yaml` | `update_frequency`、`publish_frequency`、`resolution`、`rolling_window` | 全局地图刷新和是否滚动窗口；通常不作为现场速度调参入口 |
| `ucar_ws/src/ucar_nav/config/testnav20260721/global_planner_params.yaml` | `allow_unknown`、`default_tolerance`、`use_dijkstra`、`use_grid_path` | 全局路径规划策略；路径绕不过去时先观察地图/footprint，不要直接放宽未知区 |

`footprint` 必须按真实车体尺寸填写。不能为了让规划器“通过”而缩小 footprint；
`inflation_radius` 变小会减少安全距离，`obstacle_range`/`raytrace_range` 变大也不能修复
传感器或 TF 异常。

### 0.4 三份现场参数文档

现场参数已按比赛流程拆成三份独立文档，每份文档开头都提供“候选搜索词”索引。
例如直接搜索 `OCR常规点局部膨胀`，即可定位到普通 OCR 点使用的 local costmap
常态 `inflation_radius`；搜索 `OCR停车局部膨胀`，即可定位到停车阶段的
`processing_parking_inflation_radius_m`。

| 比赛流程 | 独立参数文档 | 主流程入口 | 适合搜索的关键词 |
| --- | --- | --- | --- |
| 省赛 | [省赛现场参数](field-parameters-provincial.md) | `ucar_2026/launch/2026.launch` | `OCR常规点局部膨胀`、`OCR路线`、`普通点速度` |
| 国赛 | [国赛现场参数](field-parameters-national.md) | `ucar_2026_national/launch/2026.launch` | `OCR常规点局部膨胀`、`70点冲刺`、`终点地图坐标闭环`、`绕板` |
| 额外任务 | [额外任务现场参数](field-parameters-extra.md) | `ucar_2026_extra/launch/2026.launch` | `OCR常规点局部膨胀`、`OCR快捷路线`、`stop_mode`、`target_texts` |

三份文档都遵守同一现场规则：先停止任务和发布零速度，再改文件、同步、重启对应流程；
运行中不会自动读取 launch/YAML/JSON 新值。

### 0.5 不重启即可做的运行时对比

CymPlanner 已订阅 `/ucar/navigation_mode`，只切换已加载的参数组，不会重新读取 YAML。
只能在车辆静止、任务已取消的情况下手动对比：

```bash
rostopic pub -1 /ucar/navigation_mode std_msgs/String "data: 'point'"
rostopic pub -1 /ucar/navigation_mode std_msgs/String "data: 'sprint'"
rostopic pub -1 /ucar/navigation_mode std_msgs/String "data: 'transverse'"
```

任务运行时由 `production_task_2026` 自动切换模式，现场不要手动抢发。`/ucar/carry_mode`
可以切换载物速度倍率，但同样只建议在静止诊断时使用：

```bash
rostopic pub -1 /ucar/carry_mode std_msgs/Bool "data: false"
rostopic pub -1 /ucar/carry_mode std_msgs/Bool "data: true"
```

不要用 `rosparam set` 代替修改文件来调 CymPlanner 速度：规划器在初始化时把 YAML 读入
内存，运行中改参数服务器不会更新已缓存的 `mode1/mode2/mode3` 控制参数。需要重新加载
参数时，按前面的流程修改文件并重启节点。

### 0.6 明确不能按“现场参数”处理的内容

- `ucar_ws/src/cym_planner/src/cym_planner.cpp`、`include/`、插件源码、CMake 或依赖：
  改动后必须在小车 Ubuntu 18.04 / ROS Melodic 上重新编译，不能在本机编译后直接当成
  参数使用。
- 底盘 C++ 驱动、消息/服务定义和硬件接口改动：需要车端构建、部署和重启。
- `escape_enabled`：当前源码硬关闭，YAML 改成 `true` 也不会启用。
- 串口 `port`、`baud`、车体尺寸、轮半径和 TF frame：虽然文件上可以改且不需要编译，
  但它们不是现场速度调参；除非已经确认硬件或标定值错误，否则不要动。

现场调参的建议顺序是：先确认 odom/TF 和传感器正常，再只改一组参数，记录原值与新值，
重启后用 RViz 和 `/cmd_vel`、`/odom_raw` 对比，确认没有滚子打滑、急停距离变长或路径
偏移，再进入下一轮。

## 1. 先区分两个 ROS Master

本项目同时存在两个独立的 ROS Master：

| 对象 | `ROS_MASTER_URI` | RViz 观察内容 |
| --- | --- | --- |
| 本机仿真 | `http://127.0.0.1:11312` | Gazebo、仿真地图、仿真小车 |
| 实机任务 | `http://<VEHICLE_IP>:11311` | 小车真实 TF、雷达、地图、代价地图和导航状态 |

`<VEHICLE_IP>` 是任务启动脚本显示的小车当前局域网地址；`<PC_LAN_IP>` 是本机当前局域网
地址。两者随 Wi-Fi 变化，不要写死旧 IP。

每个终端都有自己的环境变量。仿真 RViz 和实机 RViz 必须在两个终端中分别设置
`ROS_MASTER_URI`，不能把仿真 Master 的环境带到实机 RViz。

### 1.1 国赛任务中的启动时机

如果使用 `start_2026.sh <电脑IP> mission`，脚本询问“输入 yes 继续”时，按下面顺序操作：

1. 确认车辆确实在起点后输入 `yes`，让国赛导航和任务节点启动。
2. 等待 `rosnode list` 能看到 `/map_server`、`/move_base`、`/base_driver`，并确认 `/map`
   和 TF 已有数据。
3. 在另一个终端设置同一个 ROS Master，启动本机 RViz。
4. RViz 显示地图、TF、雷达后，再进行语音唤醒或发送任务指令；不要在 RViz 还空白时
   开始让车运动。

因此：**RViz 在 `yes` 之后启动，在真正下达任务指令之前启动。** `yes` 之前启动时，
导航节点和 Master 可能还没有就绪，RViz 容易打开但没有内容；若已经提前打开，应在
Master 就绪后关闭并重新启动 RViz。

## 2. 打开实机 RViz

任务启动后，在本机 WSL 新开一个终端：

```bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://<VEHICLE_IP>:11311"
export ROS_IP="<PC_LAN_IP>"

# 先确认连到的是小车 Master
rosnode list
```

确认能列出小车节点后，再启动 RViz。国赛配置：

```bash
rviz -d /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/ucar_source_code/ucar_ws/src/ucar_2026_national/rviz/navigation_2026.rviz
```

额外任务配置：

```bash
rviz -d /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/ucar_source_code/ucar_ws/src/ucar_2026_extra/rviz/navigation_2026.rviz
```

如果代码仓库不在 `/mnt/d/...`，把 `-d` 后面的路径替换为本机实际路径；尖括号只是占位符，
不要原样输入。RViz 使用 Noetic 也可以连接小车的 Melodic ROS 1 Master。

## 3. 打开仿真 RViz

如果仿真一键启动脚本已经自动打开 RViz，不要重复启动。需要手动打开时，在另一个终端
使用仿真 Master：

```bash
source /opt/ros/noetic/setup.bash
source ~/smartcar2026/simulation/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1

rviz -d ~/smartcar2026/simulation/src/car3/rviz/navigation.rviz
```

仿真 RViz 和实机 RViz 可以同时打开，因为它们连接的是不同 Master。启动顺序不重要，
但每个终端必须先设置对应的 ROS 环境再启动 RViz。

## 4. 推荐只读显示项

实机观察建议保持 `Fixed Frame = map`，只打开或观察：

- `/map`；
- `/scan`；
- `TF` 中的 `base_link`；
- `/move_base/global_costmap/costmap`；
- `/move_base/local_costmap/costmap`；
- 全局路径和当前目标。

相机图像、CymPlanner 调试图像会增加 Wi-Fi 和 WSLg 负载。网络卡顿时先关闭这些 Image
显示项，只保留地图、TF、雷达和代价地图。

实机运行时不要点击以下工具：

- `2D Nav Goal`：会向 `/move_base_simple/goal` 发布目标；
- `2D Pose Estimate`：会向 `/initialpose` 发布定位初值；
- `Publish Point`：会向 `/clicked_point` 发布点；
- `Set Goal`、`Set Initial Pose` 等同类交互工具。

只进行缩放、平移和显示项开关操作。

## 5. 连不上或显示异常时

- `Unable to communicate with master`：检查 `ROS_MASTER_URI` 是否指向当前目标；实机应为
  小车 `11311`，仿真应为本机 `11312`。
- `rosnode list` 能通但 RViz 没有数据：检查 `ROS_IP` 是否为本机当前局域网地址，确认
  WSL 使用 mirrored networking；NAT 模式可能导致小车无法回连本机的 ROS TCPROS 端口。
- RViz 窗口打开但完全空白：必须在**启动 RViz 的同一个终端**先执行 `rosnode list`，确认
  能看到目标任务的 `/map_server`、`/move_base`、`/base_driver` 等节点，再启动 RViz：

  ```bash
  source /opt/ros/noetic/setup.bash
  unset ROS_HOSTNAME
  export ROS_MASTER_URI="http://<当前Master_IP>:11311"
  export ROS_IP="<本机当前局域网IP>"
  rosnode list
  rviz -d /home/car/ucar_ws/src/ucar_2026_national/rviz/navigation_2026.rviz
  ```

  RViz 启动后再次执行 `rosnode list | grep -i rviz`，应能看到 `/rviz`。如果目标 Master
  是任务启动后才出现，已提前打开的 RViz 不一定会自动重新注册；关闭该窗口，在任务和
  Master 就绪后重新启动。若 `rosnode list` 能看到节点但画面仍空白，检查 `Fixed Frame`
  是否为 `map`，并确认 `/map`、`/scan`、`/tf` 至少有发布者：

  ```bash
  rostopic info /map
  rostopic info /scan
  rostopic info /tf
  ```

- 实机 RViz 显示仿真内容：说明当前终端仍使用 `127.0.0.1:11312`，重新设置实机
  `ROS_MASTER_URI` 后再启动 RViz。
- 窗口黑屏、卡顿或出现 COPY MODE：关闭 RViz，先确认 WSLg 状态；两条检查都必须满足：

  ```bash
  grep -c 'use_gfxredir = 0' /mnt/wslg/weston.log
  findmnt /mnt/shared_memory
  ```

  第一条必须为 `0`，第二条必须显示 `tmpfs` 挂载。

- 日志出现 `/odom_raw` 为 `NaN` 或 `TF_NAN_INPUT`：立即停止运动并按主流程安全规程重启
  底盘/定位里程计链路；在 `/odom_raw`、`odom -> base_link`、`map -> base_link` 恢复正常前，
  不得继续导航、旋转或现场测试。

## 6. 结束观察

只在启动 RViz 的终端按 `Ctrl-C` 关闭 RViz，不要因此停止小车任务、实机 ROS Master 或仿真
主流程。任务结束后仍按对应主流程文档停止全部后台节点，不能留下残留启动终端。
