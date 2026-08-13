# 将导航线速度最大指令调为 5.0 m/s

日期：2026-07-13

## 改动

- 保持 CymPlanner `max_vel_x: 7.0 m/s`，将 `linear_x_gain` 从 `1.0` 调为 `35.0`，使刚超过 0.2 m 的前视点也能达到规划器速度上限。
- 将 `2026.py` 的线速度中继设为 `5/7≈0.714286`，对应最大 `/cmd_vel.linear.x≈5.0 m/s`；角速度中继保持 `0.5`。
- 将底盘驱动 `linear_speed_max` 从 `3.0 m/s` 调为 `5.0 m/s`，避免底盘驱动提前限幅。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/yolo2025/scripts/2026.py`
- `ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`

## 操作

- 修改部署后重启 `roslaunch yolo2025 2026.launch` 才会生效。
- 当前 launch 默认自动发送一次目标；执行 5.0 m/s 指令测试前，确认路径、人员和障碍物均已清空。

## 验证

- 已在小车端通过 Python 语法检查，并读取确认 CymPlanner `linear_x_gain: 35.0`、线速度中继 `5.0/7.0`、底盘 `linear_speed_max: 5.0` 三层参数。
- 尚需在空旷路径上采样 `/teb_cmd_vel`、`/cmd_vel` 与里程计实际速度；`5.0 m/s` 是最大指令目标，不等同于已验证的实际车速。
- 变更前本地备份：`back/2026-07-13-command-speed-before-5mps.tar.gz`。
