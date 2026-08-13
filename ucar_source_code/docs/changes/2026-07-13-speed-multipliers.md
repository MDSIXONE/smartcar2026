# 速度与旋转控制倍率调整

日期：2026-07-13

## 目的

按确认的调整要求，将实际线速度上限从 5 m/s 提升到 15 m/s（×3），旋转最大速度提升 4 倍，旋转比例控制增益提升 6 倍。

## 变更

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`：
  - `max_vel_x` `7.0` → `21.0`，`linear_x_gain` `35.0` → `105.0`。
  - `max_vel_theta` `0.5` → `2.0`，`angular_gain` `1.5` → `9.0`。
  - 同步提升末端对准的 `final_yaw_max_vel` `0.4` → `1.6` 与 `final_yaw_gain` `1.5` → `9.0`。
- `ucar_ws/src/yolo2025/scripts/2026.py`：更新速度中继注释；线速度缩放系数保持 `5/7`，角速度缩放系数保持 `0.5`。
- `ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`：`linear_speed_max` `5.0` → `15.0`，避免底盘截断线速度命令。
- `ucar_ws/src/cym_planner/config/README.md`：更正速度中继系数说明。
- 变更前备份：`back/2026-07-13-speed-multipliers-before-change.tar.gz`。

## 验证与限制

- 本地 YAML 解析、Python 编译检查均已通过；小车端 `2026.py` Python 编译检查也已通过。
- 已核对小车端三个运行文件的参数值，与本地配置一致。
- 新上限远高于原配置，不会自动启动导航；重启后需在安全的空旷区域内测试。
