# 2026-07-14：增加 CymPlanner 行进航向角速度 P/D 控制

## 目的

按要求将行进航向角速度 P 设置为 `10.0`，并为该控制环新增独立的 D 参数，改善转向响应同时抑制快速航向误差变化带来的过冲。

## 涉及文件

- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
  - 新增 `angular_kd` 参数读取、航向误差微分状态和限幅后的 P/D 控制计算。
  - 每次收到新路径时重置航向微分状态，避免上一段路径的误差影响下一段。
  - 启动日志增加实际角速度 P/D 值。
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - `angular_gain: 10.0`。
  - 新增 `angular_kd: 0.2`。
- `docs/operations.md`
  - 更新当前角速度 P/D 参数说明。

## 验证

- 本地 YAML 与源码静态检查已通过。
- 已上传并校验 SHA-256：源码为 `772258df…53fc3d9a`，参数 YAML 为 `231455fc…126ce73d`。
- 小车端 `catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware"` 已完成；新库构建时间为 `2026-07-14 20:48:49 +0800`，并包含角速度 P/D 启动日志格式。
- 当前 `move_base`、`base_driver`、`navigation_2026` 均未运行；本次未启动车辆。

## 已知限制

- 行进角速度仍被 `max_vel_theta: 1.0 rad/s` 限制；提高 P 或 D 不会超过该上限。
- `angular_kd` 仅作用于行进航向控制，不改变最终朝向阶段的 `final_yaw_gain`。
