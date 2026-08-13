# 部署说明

本文说明拿到 Git 仓库或 ZIP 仿真包后需要完成的安装、路径配置、构建、启动和
官方规则验收。正式运行环境为 Ubuntu 20.04、ROS Noetic 和 Gazebo Classic。

## 1. 解压或克隆

推荐放在普通用户可写目录。工作区目录名和绝对路径可以自行选择：

```bash
cd /home/car
git clone \
  https://github.com/MDSIXONE/smartcar2026-simulation.git \
  smartcar2026-simulation
cd smartcar2026-simulation
git switch main
```

ZIP 包解压后，进入含有 `src/`、`src/CMakeLists.txt` 和
`requirements-vision.txt` 的目录。不要再套一层空的 catkin 工作区，也不要把
不同 Git 分支的 `src/`、`build/` 或 `devel/` 混在同一目录。

通过 ZIP 获取时恢复脚本执行权限：

```bash
find src/car3/scripts -type f -name '*.py' -exec chmod +x {} +
```

## 2. 安装依赖

如果尚未安装 ROS，先安装 ROS Noetic Desktop-Full。已有 Noetic 时安装本项目
运行依赖：

```bash
source /opt/ros/noetic/setup.bash
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-opencv \
  ros-noetic-navigation \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control

python3 -m pip install --user -r requirements-vision.txt
```

`requirements-vision.txt` 固定了已验证的 NumPy 和 ONNX Runtime 版本。不要把
其他 Python 环境中的二进制包直接复制到 Ubuntu 20.04。

## 3. 哪些路径需要修改

正常部署只需要把命令中的工作区路径改成实际路径，源码内路径无需修改。

| 路径或资源 | 是否需要修改 | 说明 |
| --- | --- | --- |
| `/home/car/smartcar2026-simulation` | 按实际目录修改 | 只出现在本文的 `cd` 示例中 |
| ROS 包路径 | 不需要 | 启动文件使用 `$(find car3)`、`$(find gazebo_map)` 动态发现 |
| YOLO ONNX | 通常不需要 | 默认是 `src/car3/models/vision/cube_yolov5_best.onnx` |
| 标签模板 | 通常不需要 | 默认是 `src/car3/models/cube/meshes/` |
| `GAZEBO_MODEL_PATH` | 不需要手工导出 | `src/car3/launch/gazebo.launch` 自动加入 `src/car3/models` |
| 官方 `/home/ucar/gazebo_ws/...` 挂牌 URI | 不要修改或建系统软链 | 启动兼容节点在旧路径不存在时加载同一份官方 OBJ |
| URDF 和 world | 禁止修改 | 必须由官方基线校验；不能用改模型解决控制或性能问题 |

正式包只保留
`src/car3/models/vision/cube_yolov5_best.onnx` 这一个运行权重。更换模型时直接
替换该文件；不要额外提交训练权重、导出中间文件、数据集或绝对路径。

## 4. ROS 网络配置

### 同一台 WSL 运行

这是最稳定且不受 Wi-Fi 地址变化影响的方式。每个终端都使用同一个回环地址：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
```

### 多台机器运行

不要在启动文件中写死 Wi-Fi IP。应为 ROS Master 主机配置局域网可解析的主机名，
其他机器使用主机名：

```bash
export ROS_MASTER_URI=http://<可解析的主机名>:11312
export ROS_HOSTNAME="$(hostname)"
unset ROS_IP
```

先用 `getent hosts <可解析的主机名>` 和 `ping` 验证双向解析。网络变化后只需
保持主机名解析正常，无需修改仓库配置。

## 5. 构建与基础校验

```bash
cd /home/car/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash

catkin_make tests -j2
cd build
ctest --output-on-failure
cd ..
catkin_test_results --all
```

还应确认模型和视觉资源存在：

```bash
test -f src/car3/urdf/car3.urdf
test -f src/car3/world/math.world
test -f src/car3/models/vision/cube_yolov5_best.onnx
test -f src/car3/models/sign/meshes/wall_Food.obj
```

## 6. 必须了解的配置

| 配置文件 | 可配置内容 | 正式比赛注意事项 |
| --- | --- | --- |
| `src/car3/config/task3_vision.yaml` | 搜索观察位、视觉框、增益、超时、RTF 门槛 | 坐标是建图后的分段目标，不能改成读取 Gazebo 真值 |
| `src/cym_planner/config/cym_planner_params.json` | 局部控制器原生参数、雷达安全参数、载物速度 | 补充规则允许自研局部规划，但 `/scan` 必须是核心输入 |
| `src/gazebo_nav/launch/config/move_base/*.yaml` | 代价地图和官方全局规划器原生参数 | 不得替换、封装或修改官方全局规划器源码 |
| `src/car3/launch/task3_prepare.launch` | Gazebo/RViz、初始机械臂姿态、吸附开关 | 正式默认必须保持 `gui=true`、`rviz=true` |
| `src/car3/launch/task3_execute.launch` | 任务输入、视觉模型、抓取和携带姿态 | 比赛中途不得修改参数 |

所有配置修改后都要重新构建、测试并重启完整仿真。不要在任务进行中动态改参数。

## 7. 仿真时间比例与 RTF

这里有两个容易混淆的设置：

1. Gazebo 目标仿真时间比例在 `src/car3/world/math.world` 的 `<physics>` 段：

   ```xml
   <max_step_size>0.001</max_step_size>
   <real_time_factor>1</real_time_factor>
   <real_time_update_rate>1000</real_time_update_rate>
   ```

   `real_time_factor=1` 表示目标为 1 秒墙钟时间推进 1 秒仿真时间。该 world 属于
   官方基线，本项目禁止修改；需要官方验收时必须保留上述值。

2. 任务允许的最低实测 RTF 在
   `src/car3/config/task3_vision.yaml`：

   ```yaml
   rtf_minimum: 0.30
   rtf_preflight_strict: false
   ```

   `rtf_minimum` 只控制任务启动前的通过/警告门槛，不会加快 Gazebo。设为
   `strict: true` 后，低于门槛会终止任务；默认只警告。要改善实际 RTF，应减少
   后台负载、使用合适显卡驱动并关闭无关程序，不能通过修改官方 world 冒充性能提升。

非正式研究若确需改变 Gazebo 目标比例，应在独立实验分支研究
`src/car3/world/math.world` 的 `<physics>`，不得提交到正式分支，也不能把结果
称为满足官方模型基线。

## 8. 正式启动

两个终端都执行：

```bash
cd /home/car/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
```

终端 A：

```bash
roslaunch car3 task3_prepare.launch
```

默认会同时打开 Gazebo 和 RViz。等待机械臂初始化完成后，终端 B：

```bash
roslaunch car3 task3_execute.launch cargo_item:="苹果"
```

比赛全程保持 Gazebo 与 RViz 可见并按要求投屏，不要启动其他人工速度发布器或
在任务中途修改参数。

## 9. 官方硬性要求检查

检查依据为项目外层的《比赛仿真硬性要求》。代码检查只能验证程序结构；窗口投屏、
不最小化、不中途人工操作等行为要求必须由现场操作保证。
下表资源哈希统一把 CRLF/LF 视为 LF，并忽略文件末尾换行，便于 Windows 与
WSL 复核同一份内容。

| 要求 | 当前证据 | 状态 |
| --- | --- | --- |
| Gazebo 与 RViz 同时可见 | `task3_prepare.launch` 默认 `gui=true`、`rviz=true` | 代码满足；现场仍需保持可见 |
| 使用官方全局规划器 | `base_global_planner=global_planner/GlobalPlanner`，仓库不包含其修改源码 | 满足 |
| 局部避障以激光为核心 | `cym_planner` 直接订阅 `/scan`，代价地图也使用 `/scan` | 满足 |
| 不读取 `/gazebo/model_states` 定位 | 正式任务节点没有该订阅；分段目标通过 `move_base` 在地图坐标系发布 | 满足 |
| 不硬编码全局路径或障碍物坐标 | 只保存规则允许的地图分段目标，完整路径由官方全局规划器生成 | 满足 |
| 不后台遥控或中途改参 | 正式启动只使用 prepare/execute 两个 launch | 需现场遵守 |
| 源码完整、未混淆 | Python/C++、launch、配置和依赖清单均在仓库中 | 满足 |
| world 保持已确认的官方基线 | 归一化 SHA-256 锁定为 `0318bc8e...` | 满足 |
| URDF 保持已确认的官方基线 | 归一化 SHA-256 锁定为 `d54efc16...` | 满足 |

当前仓库中的 URDF 和 world 已由负责人确认为正式官方基线。验收只检查上述仓库
基线未被修改，不再与其他解压副本做内容差异判定。

## 10. 本机跑通记录

2026-07-27 在本机 WSL Ubuntu-20.04、普通用户 `car` 下完成以下验证：

- 功能基线提交：`56be35ec62ae9d773cd41c671822843efc55b6ea`；
- `catkin_make -j2` 成功；
- CTest 9/9 通过，`catkin_test_results --all` 为 0 error / 0 failure；
- Gazebo GUI 中场区文字可见，兼容节点报告三块挂牌 3/3 加载；
- 输入 `cargo_item:=牙刷`，相机确认目标、单次吸附、收臂完成后再启动导航；
- 到达日用品加工车间的距离和朝向容差后输出 `DONE`；
- 本次 GUI + RViz/视觉负载下预检 RTF 约 0.208、全流程有效 RTF 约 0.256；
  低于 `0.30` 时按默认配置记录警告但不中止。

这证明当前功能流程在本机可运行，不等同于消除上节记录的 URDF 官方基线差异。

## 11. 主分支部署流程

只有第 9 节所有代码级阻塞项消除、当前分支测试和完整实跑通过后，才执行：

```bash
git fetch origin
git switch main
git pull --ff-only origin main
git merge --ff-only <已验收分支>
git push origin main
```

WSL 部署端随后执行：

```bash
cd /home/car/smartcar2026-simulation
git fetch origin
git switch main
git pull --ff-only origin main
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

最终必须核对开发机、GitHub `origin/main` 和 WSL 部署的 `git rev-parse HEAD`
完全一致，并确认没有遗留 `roslaunch`、Gazebo 或任务进程。
