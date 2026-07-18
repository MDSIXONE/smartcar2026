# 操作命令

日常启动只需要查看 [quickstart.md](quickstart.md)；本文件保留部署、回滚和故障排查的完整命令。

## WSL ROS Master 与 RViz

本机 WSL 的 Ubuntu 20.04 使用 ROS Noetic，只作为局域网 ROS Master 和 RViz 客户端；
地址固定为控制电脑的 192.168.8.197。在 WSL 终端执行以下命令启动 Master：

    ~/start_ros_master.sh

WSL 的 ROS 环境已设置 DISABLE_ROS1_EOL_WARNINGS=1，因此不会再显示 ROS 1
Noetic 生命周期结束的提示窗口。

`~/start_rviz.sh` 会固定加载本地
`ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`：其中已启用 `/map`、全局/局部代价地图、
`/scan` 激光点云和 `base_link` 朝向箭头。点云轨迹与朝向 TF 的失效时间均为 `0.3 s`，因此
不会把断流后的旧朝向继续显示为实时数据。

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

若只验证小车到 WSL 的数据链路、绝不发送默认导航目标，小车端使用：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

小车的唯一 ROS Master 是本机 WSL Ubuntu 20.04 的
`http://192.168.8.197:11311`。因此以上两条 `export` 必须在所有 `source` 命令**之后**执行；
同时先 `unset ROS_HOSTNAME`，避免其与 `ROS_IP` 冲突。不得使用或启动小车本机
`192.168.8.231:11311` 的 `roscore`，也不要只单独执行最后一条 `roslaunch`。

随后在 WSL 终端检查；下列命令只读取数据，不会移动小车：

```bash
rostopic echo -n 1 /map
rostopic echo -n 1 /scan
rostopic echo -n 1 /move_base/global_costmap/costmap
rostopic echo -n 1 /move_base/local_costmap/costmap
rosrun tf tf_echo map base_link
```

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

### 2026.launch 异常终端关闭后的停止

正常停止始终优先在启动终端按 `Ctrl-C`。若终端已被关闭但真机任务仍在运行，不要使用
宽泛的 `pkill -f roslaunch` 或 `rosnode kill -a`。在小车的新终端运行专用脚本：

```bash
bash ~/ucar_ws/src/yolo2025/scripts/stop_2026_task.sh
```

脚本会在所有 `source` 后显式恢复 WSL Master，先向 `/cmd_vel` 发布零速度，暂停精确匹配
的 `yolo2025 2026.launch` 父进程以阻止 `move_base` 重生，关闭其直接子进程，最后清理该
launch 父进程。它不启动或停止 `192.168.8.197:11311` 的 Master，也不匹配其他 launch。

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
局部地图把路径判为不可行。

本地 WSL Noetic 没有安装 TEB 插件，不能在本地运行时加载该 planner。小车
端先同步以下文件，再进行静态解析和构建检查：

```bash
scp ucar_ws/src/ucar_nav/launch/teb_move_base_omni_2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/launch/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/teb_move_base_params_2026.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
scp ucar_ws/src/ucar_nav/config/omni_test20250620/teb_local_planner_params.yaml \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/config/omni_test20250620/
scp ucar_ws/src/yolo2025/launch/2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
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
bash ~/ucar_ws/src/yolo2025/scripts/stop_2026_task.sh
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

## 真机 RViz 车体模型

`navigation_2026.rviz` 的 **UCar 2026 Visual Model** 显示
`yolo2025/urdf/ucar_2026_visual.urdf` 中的简化蓝色车体。它以已存在的 `base_link`
为根，尺寸与全局、局部代价地图足迹一致（`0.342 m × 0.256 m`）；不发布 TF、不参与碰撞检测，
也不改变底盘控制。红黄足迹 Marker 仍用于观察足迹和 5 cm 安全边界。

部署可视车模或足迹 Marker 修复时，同步 URDF、2026 launch 与任务脚本；不在小车端创建备份：

```bash
scp ucar_ws/src/yolo2025/urdf/ucar_2026_visual.urdf \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/urdf/
scp ucar_ws/src/yolo2025/launch/2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
scp ucar_ws/src/yolo2025/scripts/2026.py \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/scripts/
```

停止旧的 `2026.launch` 后，在小车端以无自动目标模式重启，使其加载
`/robot_description`；然后在 WSL 重启 `~/start_rviz.sh`：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

If `/odom_raw` contains `x: nan` or `y: nan`, or the terminal reports
`TF_NAN_INPUT` for `wheelodom`, publish zero velocity and restart the
navigation/odometry chain before continuing.  Do not run a localization or
costmap test against that state:

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
pkill -INT -f 'roslaunch yolo2025 2026.launch' || true

# In a clean terminal, then start the no-goal launch again.
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
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
export ROS_MASTER_URI=http://192.168.8.197:11311
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
scp ucar_ws/src/jie_ware/src/lidar_loc.cpp \
  ucar@192.168.8.231:~/ucar_ws/src/jie_ware/src/
scp ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.pgm \
  ucar@192.168.8.231:~/ucar_ws/src/ucar_nav/maps/
```

构建只写入新的库文件，不会替换已运行进程内存中的插件。请在当前 launch 终端按 `Ctrl-C` 后，使用以下命令手动启动；`startup_goal_enabled:=false` 会确保启动时不自动行车：

```bash
cd ~/ucar_ws
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

小车主机名当前只解析到无作用域的 IPv6 link-local 地址；启动前必须设置上述 `ROS_IP`，否则 roslaunch 的 XML-RPC 服务可能无法自检并导致无法启动。

RViz 在控制电脑上启动，通过 ROS Master 连接小车；不要在小车端的 `2026.launch` 中启动 RViz。控制电脑的 RViz 预设文件为 `ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`。

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

局部膨胀半径更新为 `0.05 m`、全局膨胀半径更新为 `0.40 m` 后，必须重启小车端 `roslaunch yolo2025 2026.launch`，使 move_base 重新加载代价地图参数。
3. 使用 **2D Nav Goal** 发送导航目标。

当 `move_base` 返回 `Goal reached.` 后，再使用 `rostopic echo -n 1 /odom` 确认线速度和角速度均为 `0.0`；这表示该目标已经结束且底盘没有继续接收运动速度。

2026 任务使用 `lidar_loc` 发布 `map -> odom`；初始位姿由 `2026.launch` 的
`initial_pose_x/y/a` 控制，默认值为 `(-0.25, 2.75, 0)`。它以 `/scan` 匹配静态地图，
并使用同一帧激光时间戳的 `odom -> base_link` 计算 `map -> odom`，避免旋转时将新里程计
和旧激光位姿混用。RViz 的 **2D Pose Estimate** 可在实际摆放位置存在偏差时重新定位。

部署激光定位与局部 TF 修复时上传源码和两个 launch 文件，在小车端构建 `jie_ware` 后，
再用无自动目标模式重启：

```bash
scp ucar_ws/src/yolo2025/launch/2026.launch \
  ucar@192.168.8.231:~/ucar_ws/src/yolo2025/launch/
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
python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

建图和导航发布的 `/scan` 仍遵循 ROS 标准 `LaserScan` 约定（`+X` 前、`+Y` 左）。但当前 YDLidar 任务地图与 `lidar_loc` 内部的 OpenCV 图像行坐标相配，`lidar_loc.cpp` 必须保留 `y_laser = -range*sin(angle)` 的内部镜像；不要仅依据通用 ROS 约定把它改成正号。更改该符号前必须以静态墙体重合率验证。

当前 CymPlanner 的正常命令上限为线速度 `max_vel_x: 0.5 m/s`、横向速度 `max_vel_y: 0.1 m/s`、行进和末端对准角速度 `max_vel_theta: 1.0 rad/s`、`final_yaw_max_vel: 1.0 rad/s`。线速度 P/D 参数为 `linear_x_gain: 1.5`、`linear_x_kd: 0.5`；行进航向角速度 P/D 参数为 `angular_gain: 2.5`、`angular_kd: 0.4`。最终朝向和靠近目标点的增益仍为 `final_yaw_gain: 2.0`、`final_linear_x_gain: 1.0`。`move_base` 直接发布 `/cmd_vel`，不经过 `2026.py` 的速度中继或缩放；底盘驱动限幅仍为线速度 `3.0 m/s`、角速度 `3.14 rad/s`。这些是 ROS 命令上限，绝不等同于实际车速；`carry_speed_scale` 保持 `1.0`，因为源代码会将它钳制在 `1.0` 以内。

CymPlanner 只保留并加载 `$(find cym_planner)/config/ucar_cym_planner_params.yaml`；参数根键为 `cym_planner/CymPlanner`。插件同时兼容 move_base 传入的短名称和完整名称，并始终回退读取该规范命名空间，防止参数缺失时悄悄退回到源码默认的 `0.2 m/s`、`0.5 rad/s`。正常行进的线速度不再按车头与路径夹角做 25%～100% 的额外缩放；仍受 `max_vel_x`、搬运模式比例、碰撞检查和底盘 `linear_speed_max` 限制。修改 `cym_planner.cpp` 后必须先执行本节的 `catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware"`，再重启 launch 才会加载新插件库。旧 JSON 示例已删除，不参与真机运行。

当前 2026 导航的局部代价地图与全局代价地图使用同一足迹 `0.342 m × 0.256 m`（`±0.171 m`、`±0.128 m`），使全局路径、局部碰撞检查和 RViz 车体模型一致。局部滚动窗口为 `5.0 m × 5.0 m`、分辨率 `0.03 m`、更新/发布频率 `12/5 Hz`、障碍物/清除范围 `3.0/4.0 m`、`inflation_radius: 0.07 m`、`cost_scaling_factor: 4.0`。真机仍保留 `map` / `base_link` 坐标系和 `/scan_filtered` 局部观测；这两个接口不能改为仿真的 `odom` / `base_footprint` / `/scan`，否则会破坏 `lidar_loc` 定位与实车离群点过滤。全局代价地图 `inflation_radius` 为 `0.21 m`，全局插件顺序为 `static_layer → obstacle_layer → inflation_layer`。修改代价地图或 CymPlanner 参数后，必须停止并重新执行上述 `roslaunch` 命令，运行中的 `move_base` 不会自动重新加载 YAML。

当前激光数据经 `/scan_raw → 2026.py → /scan` 中继；`scan_scale` 为 `1.0`，因此 `/scan` 保持原始距离。`lidar_loc` 始终订阅原始 `/scan`。`2026.launch` 同时启动 `jie_ware/lidar_filter_node`，以 `0.10 m` 的近邻差阈值删除单束离群回波并发布 `/scan_filtered`；局部代价地图订阅该过滤话题。全局代价地图仍只订阅 `2026.py` 发布的 `/scan_global_obstacles`，不得把定位或全局静态墙过滤改接为 `/scan_filtered`。

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
python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

重启后保持车静止，先检查 `base_link -> laser_frame` 的 yaw 为 `0`，再在 RViz 对照 `/scan` 和 `/map`。若仍存在恒定角差，只能按实测安装角重新标定该单个 yaw；不得改回定位内部的 `-sin(angle)` 或通过旋转地图掩盖问题。

`navigation_2026` 仅负责 `/scan_raw → /scan` 转发和可选的单次默认目标；`lidar_loc` 由 `2026.launch` 直接启动，并且是 `map -> odom` 的唯一发布者。两者均不包含历史验证路线、TEB、语音唤醒或 RViz 目标观察。RViz 的 **2D Nav Goal** 直接发送给 `move_base`。

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

完成静态检查、确认路径清空且急停可用后，以下命令会重启并等待定位、全局路径就绪，再发送 `2026.launch` 内置的默认目标 `map (-1.734, 2.305, yaw -1.570796)`：

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
`map <- laser_frame` transform is unavailable at a scan timestamp, 2026.py
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

部署中部生产网格任务时同步以下文件；包重新构建后再由用户启动导航：

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

`roslaunch yolo2025 2026.launch` 的默认导航目标已是顺时针 `270°`（二维码 `d`）。到点后每个朝向静止 `3.5 s` 扫描，再由 move_base/CymPlanner 向当前 `map -> base_link` 位置依次发送 `yaw=π` 扫描 `a`、`yaw=-π/2` 扫描 `i`；最终停在顺时针 `90°`（ROS `yaw=-π/2`）。3.5 秒来自实车扫码器约 `2~3.4 s` 的稳定解码延迟。二维码在整个扫描阶段均计入结果。每个定向目标最多等待 `6 s`，超时后任务取消该旋转并扫描当前朝向，避免持续原地旋转。终端输出 `QR_SCAN_RESULT`、`QR_SCAN_GOAL_TIMEOUT` 与 `QR_SCAN_FINISHED`。该任务会启动并独占摄像头，不要同时运行独立的 `qrcode.launch`。

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
`roslaunch yolo2025 2026.launch`，新地图才会被 `map_server` 和导航代价地图加载。

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
python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch startup_goal_enabled:=false
```

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

先在本机检查脚本和配置，再同步四个文件；不要在小车端创建备份：

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

在小车端先静态验证，再停止旧导航并以无自动目标模式启动。所有 `source` 完成后都要
显式恢复唯一的 WSL Master：

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
