# 国赛冲刺航向环 P 减半

- 日期：2026-08-18
- 目的：改善国赛 70→坡顶冲刺段到达 70 附近的航向角度控制。
- 涉及文件：`ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`、`docs/operations.md`。
- 改动：`mode3_sprint.angular_gain` 从 `10.0` 调整为 `5.0`；`linear_x_gain=12.5`、`max_vel_x=2.5`、任务层 `sprint_yaw_deg=175` 保持不变。
- 验证：本机 YAML 解析通过；确认 `mode3_sprint.angular_gain=5.0`、`linear_x_gain=12.5`、`max_vel_x=2.5`，国赛 launch 仍为 `sprint_yaw_deg=175`；`git diff --check` 通过。
- 已知限制：尚未部署或进行实车冲刺验证。
