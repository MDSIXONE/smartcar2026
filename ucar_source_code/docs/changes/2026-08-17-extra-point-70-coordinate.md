# 额外任务 70 号点坐标校准

- 日期：2026-08-17
- 目的：将额外任务使用的国赛地图 70 号冲刺前点与国赛主流程同步。
- 涉及文件：`ucar_ws/src/ucar_2026_extra/config/production_full_grid_all_numbered.json`、`docs/operations.md`。
- 改动：70 号点由 `(2.25, 1.75)` 调整为 `(2.32, 1.68)`，即 `x + 0.07m`、`y - 0.07m`；同时更新 JSON 中顶层 `points` 与 `grouped_points.centers` 两份记录。
- 验证：国赛与额外任务两份 JSON 的 70 号点联合容差核对通过；额外任务几何回归 101 项通过、86 项因本机缺 ROS 跳过；`git diff --check` 通过。
- 已知限制：尚未在小车 Ubuntu 18.04 / ROS Melodic 上启动导航或进行实车验证。
