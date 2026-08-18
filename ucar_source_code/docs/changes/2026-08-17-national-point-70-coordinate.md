# 国赛 70 号点坐标校准

- 日期：2026-08-17
- 目的：根据现场标定结果调整国赛地图 70 号导航点。
- 涉及文件：`ucar_ws/src/ucar_2026_national/config/production_full_grid_all_numbered.json`、`docs/operations.md`。
- 改动：70 号点由 `(2.25, 1.75)` 调整为 `(2.32, 1.68)`，即 `x + 0.07m`、`y - 0.07m`；同时更新 JSON 中顶层 `points` 与 `grouped_points.centers` 两份记录。
- 验证：JSON 解析、70 号点唯一性和目标坐标核对通过；国赛几何回归 86 项通过、71 项因本机缺 ROS 跳过；`git diff --check` 通过。
- 已知限制：尚未在小车 Ubuntu 18.04 / ROS Melodic 上启动导航或进行实车验证。
