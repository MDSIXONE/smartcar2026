# 2026-08-19 导航到点位置容差收紧为 3cm

## 目的

试验将实车 2026 主流程的导航到点位置容差收紧到 `0.03m`，观察定位误差和终点调整是否仍能稳定收敛。

## 改动

- `cym_planner/src/cym_planner.cpp`：CymPlanner 进入终点姿态调整的位置阈值 `0.05m → 0.03m`。
- 三套 `production_task_2026.py`：`arrival_tolerance` 默认值 `0.10m → 0.03m`。
- 三套 `launch/2026.launch`：任务层 `arrival_tolerance` `0.15m → 0.03m`。
- 航向容差保持 `0.05rad` 不变。

## 部署与验证

- 按动态 DNS `ucar-mini`（本次解析为 `192.168.8.231`）同步 7 个源码/运行文件。
- 车端 Ubuntu 18.04 / ROS Melodic 白名单编译 `cym_planner` 成功，随后恢复原 catkin 白名单 `usb_cam`。
- 7 个文件本地/车端 SHA-256 一致。
- 车端 3 个 Python2 AST、3 个 launch XML 解析通过；CymPlanner 动态库存在。
- 部署、构建和静态验证未启动 ROS、move_base、2026 主流程或车辆运动。

## 已知限制

尚未进行车端运动实测。3cm 可能小于实际定位噪声，若出现反复终点调整、导航超时或任务层拒绝到点，应回看 `/odom_raw`、TF 和 `arrival_error` 日志。
