# 融合 CymPlanner 前视车体尺寸与可视化

## 目的

将 `MDSIXONE/smartcar2026-simulation` 的
`fix/cym-planner-vehicle-size-lookahead` 分支中，与 CymPlanner 前视碰撞检查
直接相关的改动融合到本仓库 `main` 的新分支。未包含该源分支中独立的 YOLO
物块数据集采集功能。

## 涉及文件

- `simulation/src/cym_planner/`：加入 `visualization_msgs` 依赖，发布前视
  footprint Marker，并在 OpenCV 调试图上绘制同一车体轮廓；默认前视距离改为
  `0.30 m`。
- `simulation/src/gazebo_nav/launch/config/v3_move_base/local_costmap_common.yaml`：
  将局部碰撞 footprint 调整为 `0.30 m × 0.20 m`。
- `simulation/src/car3/rviz/v3_cym_nav.rviz`：默认显示
  `/move_base/CymPlanner/lookahead_footprint`。
- `simulation/src/cym_planner/config/README.md` 与 `docs/operations.md`：说明参数含义、
  构建和无运动检查步骤。

## 验证结果

- 确认目标分支以 `origin/main`（`d23d198`）为基线。
- 确认所移植的 7 个 CymPlanner、local costmap 与 RViz 文件与源分支提交
  `f6a9dbf` 对应内容一致。
- 已完成 CMake、包清单、JSON 和 YAML 的静态检查；当前 Windows 工作区未运行 WSL
  ROS Noetic 的 `catkin_make` 或 Gazebo 启动验证。

## 已知限制

前视 Marker 需要 `move_base` 已运行并产生局部规划输出才会出现。进行会导致车辆
运动的目标测试前，必须确认里程计为有限值且 `odom -> base_link`、
`map -> base_link` TF 正常；若存在 `NaN` 或 `TF_NAN_INPUT`，先发布零速度并
重启导航/里程计链路。
