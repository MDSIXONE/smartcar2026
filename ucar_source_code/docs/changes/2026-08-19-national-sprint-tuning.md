# 2026-08-19：国赛冲刺速度与航向参数调整

## 目的

提高国赛 70→288 冲刺段的有效速度，并放宽高速冲刺终点的独立位置验收范围。

## 改动

- `ucar_ws/src/ucar_2026_national/launch/2026.launch`
  - `sprint_arrival_tolerance`：`0.20m→0.30m`。
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
  - 冲刺容差默认值：`0.20m→0.30m`，与 launch 显式值一致。
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - `mode3_sprint.angular_gain`：`5.0→10.0`，航向 P 翻倍。
  - `mode3_sprint.approach_decel_distance`：`1.0m→0.5m`，减速起点后移，保持更高速度。
  - `approach_min_vel_x=0.12m/s` 保持不变，避免终点附近速度地板变小后进一步减速。
- `test/test_national_sprint_speed_debug.py` 与操作文档同步更新。

## 生效与验证

本轮只涉及 YAML、Python2 脚本和 launch，不需要 catkin 编译；必须在车辆零速并完成
`/odom_raw`、两个 TF、`/scan` 安全检查后，重启国赛主流程才会加载。已同步到
`ucar-mini (192.168.8.231)`，三份运行文件本地/车端 SHA-256 一致；当前未重启 ROS、
未发送运动指令。
