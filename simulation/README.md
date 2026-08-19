# SmartCar 2026 任务三仿真：快速开始

这是 ROS 1 Noetic + Gazebo Classic 的 catkin 工作区。正式比赛启动默认同时打开
Gazebo 和 RViz，任务节点使用相机识别、官方全局规划器与激光雷达局部避障完成
物块夹取和配送。

## 拿到仿真包后

以下命令都在 Ubuntu 20.04 / WSL Ubuntu-20.04 的普通用户下执行。工作区可以放在
任意目录，示例使用 `/home/car/smartcar2026-simulation`；路径改变时只需修改
`cd` 命令，不要修改 URDF、world 或源码中的资源路径。

```bash
cd /home/car/smartcar2026-simulation
source /opt/ros/noetic/setup.bash

sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-opencv \
  ros-noetic-navigation \
  ros-noetic-gazebo-ros-control

python3 -m pip install --user -r requirements-vision.txt
find src/car3/scripts -type f -name '*.py' -exec chmod +x {} +

catkin_make -j2
source devel/setup.bash
```

本仓库已包含真车联动所需的 `bridge/sim_bridge.py`。它不替代仿真任务，
而是让真车通过局域网 HTTP 请求启动 `task3_execute.launch` 并轮询完成状态；启动方法、
`11313` 防火墙和独立 Master 的约束见 [bridge/README.md](bridge/README.md)。

确认视觉模型已经包含在包内：

```bash
test -f src/car3/models/vision/cube_yolov5_best.onnx
test -d src/car3/models/cube/meshes
```

包内只保留正式运行使用的一个 ONNX 权重
`src/car3/models/vision/cube_yolov5_best.onnx`；训练图片、标签、预览图和其他
候选权重不随仿真包发布。

## 正式启动

两个终端都先执行：

```bash
cd /home/car/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
```

终端 A 启动 Gazebo、RViz、导航和机械臂初始化：

```bash
roslaunch car3 task3_prepare.launch
```

比赛期间 Gazebo 和 RViz 必须始终可见，不得最小化或遮挡。等待日志出现：

```text
calibrated initial arm pose applied smoothly
```

终端 B 下发任务：

```bash
roslaunch car3 task3_execute.launch cargo_item:="苹果"
```

常用物品名也可以换成 `牙刷`、`手机` 等。未收录的名称需要明确给出类别：

```bash
roslaunch car3 task3_execute.launch \
  cargo_category:="电子产品" \
  cargo_name:="待处理物品"
```

任务完成时终端会输出 `DONE:`，也可以查询：

```bash
rostopic echo -n 1 /sim_task3/done
```

## 仿真执行后给裁判展示场景

任务完成并输出 `DONE:` 后，如需重新随机生成物块和锥桶供裁判查看，保持
Gazebo 运行，在新的终端执行：

```bash
cd ~/smartcar2026/simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
rosrun car3 spawn_cubes.py
```

该命令会先删除旧的 `cube_*` 和 `cone_*`，再生成一组新的随机场景；不会重启
Gazebo、复位车辆或重新执行任务。不要在任务执行过程中运行，以免改变比赛场景。

## 常改配置

| 目的 | 文件 |
| --- | --- |
| 视觉搜索位置、对位范围、RTF 检查门槛和超时 | `src/car3/config/task3_vision.yaml` |
| 局部规划器原生参数和载物速度 | `src/cym_planner/config/cym_planner_params.json` |
| 代价地图与官方全局规划器参数 | `src/gazebo_nav/launch/config/move_base/` |
| 正式启动默认值、机械臂初始姿态 | `src/car3/launch/task3_prepare.launch` |
| YOLO 模型路径、抓取与携带姿态 | `src/car3/launch/task3_execute.launch` |

`rtf_minimum` 只是任务启动时的性能检查门槛，不会改变 Gazebo 的仿真速度。Gazebo
目标时间比例位于 `src/car3/world/math.world` 的 `<physics>` 段，但该文件属于
官方模型基线，本项目禁止修改。详细解释见 [部署说明](DEPLOYMENT.md#仿真时间比例与-rtf)。

## 文档

- [部署说明](DEPLOYMENT.md)：拿到包后的完整安装、路径、网络、配置和验收说明。
- [启动与运行](TASK3_RUNBOOK.md)：正式比赛启动、运行日志和状态检查。
- [常见问题](FAQ.md)：Gazebo、RViz、视觉、夹取和导航故障。
- [真车 HTTP bridge](bridge/README.md)：真车双物品主流程的 bridge 启动与接口说明。

## 主要目录

| 目录 | 内容 |
| --- | --- |
| `src/car3` | 车辆模型、机械臂、相机、物块、视觉任务和 Gazebo 启动文件 |
| `src/car3/models/vision` | 正式运行唯一使用的 YOLOv5 ONNX 权重 |
| `src/gazebo_map` | 地图和加工区资源 |
| `src/gazebo_nav` | `move_base`、官方全局规划器配置和代价地图 |
| `src/cym_planner` | 允许自研的局部控制器及其原生参数 |
