# 任务三启动与运行

本文只描述当前正式流程。安装依赖、路径和官方规则验收见
[部署说明](DEPLOYMENT.md)。

## 1. 比赛前检查

在 WSL Ubuntu-20.04 的普通用户下进入工作区：

```bash
cd ~/smartcar2026/simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
```

确认没有旧仿真残留：

```bash
pgrep -af 'rosmaster|rosout|roslaunch|gzserver|gzclient|move_base|task3_pick_deliver'
```

若命令有输出，先回到原启动终端按 `Ctrl-C` 正常退出。不要在不确认 PID 来源时
直接批量终止其他用户的 ROS 进程。

正式比赛不得使用无界面模式；Gazebo 和 RViz 必须同时启动、保持可见并按要求投屏。

## 2. 终端 A：准备仿真

```bash
roslaunch car3 task3_prepare.launch
```

默认参数已经是：

```text
gui=true
rviz=true
navigation_mode=main_legacy
grasp_attachment_enabled=true
```

等待日志出现：

```text
calibrated initial arm pose applied smoothly
```

可以在终端 B 核对：

```bash
rostopic echo -n 1 /sim_task3/arm_initial_pose_ready
rosservice call /controller_manager/list_controllers
```

正确状态为 `arm_initial_pose_ready=True`，准备阶段结束后
`arm_controller` 和 `gripper_controller` 保持 `stopped`，直到视觉确认目标。

仅用于自动化回归、不得用于正式比赛的无界面命令：

```bash
roslaunch car3 task3_prepare.launch gui:=false rviz:=false
```

## 3. 终端 B：执行任务

终端 B 先重新执行第 1 节的环境命令，然后直接输入物品：

```bash
roslaunch car3 task3_execute.launch cargo_item:="苹果"
```

内置名称：

| 类别 | 示例 |
| --- | --- |
| 食品 | 苹果、香蕉、可乐、牛奶、面包、饼干、零食、饮料 |
| 日用品 | 牙刷、毛巾、纸巾、肥皂、洗发水、水杯 |
| 电子产品 | 手机、平板、耳机、键盘、鼠标、相机、充电器 |

未收录名称明确指定类别：

```bash
roslaunch car3 task3_execute.launch \
  cargo_category:="电子产品" \
  cargo_name:="待处理物品"
```

类别可写 `food/食品`、`daily/日用品`、`electronics/电子产品`。

## 4. 仿真执行后给裁判展示场景

任务完成并输出 `DONE:` 后，如需重新随机生成物块和锥桶供裁判查看，保持
Gazebo 和终端 A 的主流程继续运行，打开新的终端 C。终端 C 必须完整执行下面
的环境命令，不能只复制最后一行 `rosrun`：

```bash
cd ~/smartcar2026/simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
```

先确认终端 C 已连接到仿真 Master：

```bash
rostopic list
```

能列出 ROS 话题后，再执行：

```bash
rosrun car3 spawn_cubes.py
```

如果提示 `Unable to register with master node`，说明仿真 Master 没有运行或终端
C 仍连接到了默认的 `11311`；不要继续重试 `rosrun`，回到终端 A 确认主流程仍在
运行，并重新执行本节的环境命令。看到终端输出物块坐标、`锥桶: 10/10 个` 和
`完成` 后，即可让裁判查看 Gazebo 窗口中的新场景。

该命令会先删除旧的 `cube_*` 和 `cone_*`，再生成一组新的随机场景；不会重启
Gazebo、复位车辆或重新执行任务。不要在任务执行过程中运行。

## 5. 正常日志顺序

搜索先在同一 XY 原地按以下方向进行：

```text
到达夹取区
开始识别第一个物块（左侧）
第一个不是目标
顺时针旋转 90 度，识别第二个物块（上方）
第二个不是目标
再顺时针旋转 90 度，识别第三个物块（右侧）
```

若三个方向都未确认目标，任务会切换到旧版左、中、右三组独立 XY 和朝向依次
复查；仍未找到时重新开始完整搜索。

确认目标后会视觉闭环靠近并执行一次官方吸附。吸附确认后先恢复携带姿势、
再启动底盘导航，日志包括：

```text
吸附完成；先恢复携带姿势，再开始底盘导航
Navigating to ...加工车间
```

到达正确位置并完成最终朝向后：

```text
DONE: ... delivered to ...加工车间
```

## 5. 运行状态

```bash
rostopic echo /sim_task3/status
rostopic echo -n 1 /grasp_attach/state
rostopic echo -n 1 /sim_task3/carry_mode
rostopic echo -n 1 /sim_task3/done
```

正确夹取时 `/grasp_attach/state` 为 `GRASPING`；载物导航时
`/sim_task3/carry_mode` 为 `True`；任务完成时 `/sim_task3/done` 为 `True`。

检查视觉画面：

```text
/sim_task3/vision/debug_image
```

RViz 已配置对应 Image 面板。不要关闭实时画面或后台标注节点。

## 6. 雷达与规划器检查

正式配置使用：

```text
base_global_planner = global_planner/GlobalPlanner
base_local_planner  = cym_planner/CymPlanner
scan_topic          = /scan
```

检查激光数据和局部控制状态：

```bash
rostopic hz /scan
rostopic echo -n 1 /move_base/CymPlanner/laser_points
rostopic echo /move_base/CymPlanner/safety_state
```

局部规划器直接以 `/scan` 点云进行车辆投影碰撞检查。全局路径由官方
`GlobalPlanner` 生成；任务代码只发送补充规则允许的地图分段目标，不发布预设
全局路径。

比赛运行期间不要启动其他人工速度发布器，也不要在任务中途修改参数。

## 7. 仿真时间比例

任务启动时会用 5 秒墙钟时间测量 `/clock`。

- 修改最低性能检查门槛：
  `src/car3/config/task3_vision.yaml` 中的 `rtf_minimum`。
- 低于门槛是否终止：
  同文件中的 `rtf_preflight_strict`。
- Gazebo 目标时间比例：
  `src/car3/world/math.world` 中的 `<real_time_factor>`。

`rtf_minimum` 不会改变仿真速度。URDF 和 world 属于官方基线，正式分支禁止修改；
因此正式部署只能调整检查门槛或优化机器负载，不能修改 world 来“加速”。

当前默认值：

```yaml
rtf_minimum: 0.30
rtf_preflight_strict: false
```

低于 0.30 会打印警告并继续。任务结束日志会给出 `wall`、`sim` 和
`effective_RTF`。

## 8. 关闭与再次启动

任务完成后先在终端 B 按 `Ctrl-C`，再在终端 A 按 `Ctrl-C`。确认进程退出：

```bash
pgrep -af 'rosmaster|rosout|roslaunch|gzserver|gzclient|move_base|task3_pick_deliver'
```

重新运行必须从终端 A 的 `task3_prepare.launch` 开始，不能在旧 Gazebo 上叠加
第二套控制器或任务节点。

## 9. 常见故障入口

- 缺少 `onnxruntime`：重新执行
  `python3 -m pip install --user -r requirements-vision.txt`。
- 场区文字不显示：确认启动日志有
  `Scene sign compatibility ready: 3/3 overlays active`。
- 机械臂初始姿态异常：检查
  `/sim_task3/arm_initial_pose_ready` 和 controller 状态。
- 物品名无法推断：显式传入 `cargo_category`。
- 更完整的原因与恢复方式见 [常见问题](FAQ.md)。
