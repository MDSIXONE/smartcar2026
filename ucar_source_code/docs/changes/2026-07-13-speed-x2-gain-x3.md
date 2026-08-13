# 速度上限翻倍与控制增益三倍

日期：2026-07-13

## 目的

按确认要求，将 CymPlanner 与底盘的基础速度上限提升为当前值的两倍，并将所有实际参与控制的 `*_gain` 参数提升为当前值的三倍；随后重新运行默认导航目标验证。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`
- `docs/operations.md`

## 改动

- 速度上限：`max_vel_x` `21.0 → 42.0`、`max_vel_theta` `2.0 → 4.0`、`final_yaw_max_vel` `1.6 → 3.2`。
- 控制增益：`linear_x_gain` `105.0 → 315.0`、`angular_gain` `9.0 → 27.0`、`final_yaw_gain` `9.0 → 27.0`、`final_linear_x_gain` `1.0 → 3.0`。
- `linear_x_kd` 当前为 `0.0`，三倍后仍为 `0.0`，因此保持不变。
- 底盘上限：`linear_speed_max` `15.0 → 30.0`、`angular_speed_max` `3.14 → 6.28`，避免规划器速度被旧上限截断。
- `carry_speed_scale` 保持 `1.0`：该参数是仅在载物模式启用的速度倍率，代码会将其限制在 `0.05–1.0`，并非基础速度上限。

## 验证

- 本地：两个 YAML 均已通过解析与数值断言；速度中继移除检查仍通过。
- 部署前：规划器与底盘旧配置已完整下载至本地 `back/2026-07-13-speed-x2-gain-x3-before-deploy/`，并以 SHA-256 校验；小车端没有创建备份。
- 车端：已同步配置、重启 `2026.launch`（禁用自动目标）并读取运行时参数。规划器实际为 `42.0 / 4.0 / 3.2` 和 `315.0 / 27.0 / 27.0 / 3.0`，底盘实际为 `30.0 / 6.28`；`/cmd_vel` 仍由 `/move_base` 直接发布给 `/base_driver`。
- 实车默认点：向 `map (-1.534, 2.105, yaw -2.950)` 发送测试目标，`/move_base/status` 最终为 `status: 3`、`Goal reached.`；随后采样到的 `/odom.twist` 线速度和角速度均为 `0.0`。安全取消指令发出时该目标已成功结束，未中断到达。

## 已知限制

- 更高上限和增益会显著增大加速、转向和末端对准的激进程度；实际速度仍受电机、供电、轮胎、负载与障碍物条件限制。
- 修改 YAML 后必须重启 `roslaunch yolo2025 2026.launch`，运行中的 move_base 与 base_driver 不会自动重载这些参数。
- 本次确认的是参数生效与目标可达，未采集足以量化实际峰值速度或加速度的连续 `/cmd_vel`、`/odom` 数据；不能据此认定实车速度严格翻倍。
