# 2026-07-14：补充速度配置 YAML 注释

## 目的

为 CymPlanner 与底盘驱动的 YAML 参数补充中文注释，说明单位、用途、调大或调小的方向及安全限制，便于手动调参。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`

## 验证

- 两个 YAML 均通过本地 YAML 解析。
- 本次仅新增注释，不修改键、数值或 YAML 层级；已同步到小车端，但不需要重启或重新定位。

## 已知限制

- `max_vel_*` 和 `*_speed_max` 是 ROS 命令上限，不代表可达到的实车速度。
- `linear_cali`、`angular_cali` 会改变底盘轮速换算比例；它们不是独立的安全速度上限。
