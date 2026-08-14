# SmartCar 2026 结构索引

- 更新时间：2026-08-13

统一仓库：仿真环境（Gazebo）+ 实车 ROS 源码一体管理。比赛项目（2026 生产任务：导航 + QR/OCR 识别 + 抓取搬运 + 巡线交接）。

## 总体结构

```
simulationforreal/
├── simulation/               # 仿真环境（Gazebo + move_base + car3 任务包）
│   ├── bridge/               # 真车↔仿真 HTTP 桥（sim_bridge.py）
│   ├── src/car3/             # 仿真主任务包：取放货任务状态机（task3_pick_deliver.py）
│   ├── src/cym_planner/      # 自研局部规划器（move_base nav_core 插件，C++）
│   ├── src/gazebo_map/       # 静态栅格地图（map_server 用）
│   ├── src/gazebo_nav/       # 导航装配：map_server + move_base + cym_planner
│   ├── src/roboticsgroup_gazebo_plugins/  # 第三方 Gazebo 插件
│   ├── tmp/                  # bridge 备份 + 抓取调试产物
│   └── *.md                  # 部署/FAQ/运行手册（TASK3_RUNBOOK.md 等）
├── ucar_source_code/         # 实车 ROS 源码镜像（18.04/Melodic/Py2）
│   ├── ucar_ws/src/          # catkin 工作区：18 自研包 + 8 第三方包
│   ├── rosmaster/            # WSL 端 ROS Master 启动方案（HTTP/1.0 兼容补丁）
│   ├── tools/                # 工具脚本（网格资产生成）
│   ├── docs/                 # 实车操作指南 + changes/ 改动记录
│   └── production_*.json/png # 生产任务 1-418 编号网格地图导出
├── competition-rules/        # 空（待放比赛规则）
├── docs/                     # 仓库级文档：lingo.md、ai-records/、changes/
├── ucar_message/             # ucar_info.md（小车 SSH 连接信息，含凭据勿外传）
├── tmp/                      # 临时工作区（空）
└── .agents/skills/           # 项目技能（project-index 等）
```

## 功能索引

| 功能/模块 | 位置 | 一句话说明 | 详情 |
| --- | --- | --- | --- |
| 仿真桥接 | `simulation/bridge/sim_bridge.py` | HTTP 桥：真车 POST /start → 桥启动仿真任务 launch → 回报 done/failed，含 Gazebo 物理速率降速/恢复 | |
| 仿真任务包 car3 | `simulation/src/car3/` | task3_pick_deliver.py 状态机：YOLO 视觉搜索→对齐→抓取→搬运→投放；launch 分 prepare/execute 两步 | |
| 仿真局部规划器 | `simulation/src/cym_planner/` | 自定义 BaseLocalPlanner：巡线控制 + costmap/激光判障，支持 carry_mode | |
| 仿真导航装配 | `simulation/src/gazebo_nav/launch/gazebo_nav.launch` | map_server + move_base（global_planner + cym_planner），无 amcl（静态 map→odom） | |
| 仿真地图 | `simulation/src/gazebo_map/map/ros_map_thin.{yaml,pgm}` | 栅格地图（分辨率 0.01185） | |
| 实车主流程 2026 | `ucar_source_code/ucar_ws/src/ucar_2026/` | 2026 比赛主包：production_task_2026.py 状态机（导航→扫 QR→分类→OCR 取物→交接巡线）；主入口 `launch/2026.launch` | |
| 实车局部规划器 | `ucar_source_code/ucar_ws/src/cym_planner/` | 实车版 CymPlanner（C++，直连 /scan_filtered 激光避障，6 单测） | |
| 底盘驱动 | `ucar_source_code/ucar_ws/src/ucar_controller/` | base_driver.cpp 串口驱动（CRC、里程计/电池/速度服务）+ odom EKF | |
| 导航 | `ucar_source_code/ucar_ws/src/ucar_nav/` | amcl + move_base 配置集（omni 多版本）+ 历届比赛地图 | |
| 建图 | `ucar_source_code/ucar_ws/src/ucar_map/` | gmapping/cartographer launch + 多张 pgm/yaml 地图 | |
| 相机驱动 | `ucar_source_code/ucar_ws/src/ucar_cam/` | usb_camera.py、标定 yaml、多套 yolo 配置 | |
| YOLO 检测 | `ucar_source_code/ucar_ws/src/ucar_yolo/` | darknet 绑定 + 自研检测节点（果蔬/植被等） | |
| 2025 遗留脚本 | `ucar_source_code/ucar_ws/src/yolo2025/` | 旧版任务/巡线/二维码/建图/语音试验脚本（2026.py 旧入口） | |
| 历届比赛合集 | `ucar_source_code/ucar_ws/src/darren_launch/` | 2019-2024 launch + 巡线/标定脚本，lax_startup.launch 入口 | |
| 速度标定 | `ucar_source_code/ucar_ws/src/lax_calibrate/` | 线/角速度标定（dynamic_reconfigure） | |
| 雷达定位 | `ucar_source_code/ucar_ws/src/jie_ware/` | 极简激光雷达定位（lidar_loc/lidar_filter/costmap_cleaner） | |
| ROS Master 启动 | `ucar_source_code/rosmaster/` | start_ros_master.sh + HTTP/1.0 兼容补丁（车端内核兼容） | |
| 网格资产工具 | `ucar_source_code/tools/update_production_grid_assets.py` | 生产网格 JSON/PNG 中间区域标签补全 | |
| 仓库文档 | `docs/` | lingo.md 黑话词典、ai-records/（CHANGE_LOG、FAILED_APPROACHES、MISTAKE_INDEX、mistakes/12 篇）、changes/ | |
| 实车连接信息 | `ucar_message/ucar_info.md` | 小车 SSH 凭据（含凭据，勿外传） | |

第三方包（ucar_ws/src/）：darknet_ros（YOLO ROS 包装）、geometry/geometry2（tf/tf2 官方元包）、usb_cam（官方相机驱动）、xf_mic_asr_offline / xf_tts_offline（讯飞离线语音）、ydlidar（雷达厂商驱动）、sort-deepsort-yolov3-ROS（多目标跟踪）、fdilink_ahrs（IMU 厂商驱动）、robot_pose_ekf / robot_pose_publisher（ROS 标准包）。

## 关键入口

- 仿真（WSL 内）：`roslaunch car3 task3_prepare.launch` → `roslaunch car3 task3_execute.launch`，详见 `simulation/TASK3_RUNBOOK.md`
- 实车主流程：`ucar_source_code/ucar_ws/src/ucar_2026/launch/2026.launch` → `production_task_2026.py`，启停脚本 `start_2026.sh` / `stop_2026_task.sh` / `handoff_lane.sh`
- 规则文档：根 `AGENTS.md`（通用）、`simulation/AGENTS.md`（仿真）、`ucar_source_code/AGENTS.md`（实车）
