# 2026-07-14：恢复底盘初始限速

## 目的

按要求将底盘驱动的速度限幅恢复为初始版本数值，用于与 CymPlanner `300.0 m/s` 命令上限进行对比。

## 涉及文件

- `ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`
  - `linear_speed_max: 300.0` 改为 `3.0`。
  - `angular_speed_max: 31.4` 改为 `3.14`。
- `docs/operations.md`
  - 更新运行时参数预期值。

## 验证

- 本地已通过 YAML 解析，确认两个限幅值为 `3.0` 与 `3.14`。
- 已上传至小车，并完成远端 SHA-256 一致性校验。
- 未执行 `roslaunch`、未重启 ROS、未发送导航目标；运行时参数将在下次手动启动后加载。

## 已知限制

- CymPlanner 的 `max_vel_x` 仍为 `300.0 m/s`，但 `/base_driver` 会将收到的线速度命令裁剪到 `3.0 m/s`。
- 实际车速仍由底盘固件、电源和机械负载决定。
