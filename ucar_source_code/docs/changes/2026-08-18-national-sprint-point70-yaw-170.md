# 国赛 70 号点冲刺朝向调整为 170°

日期：2026-08-18

## 改动

文件：`ucar_ws/src/ucar_2026_national/launch/2026.launch`

- `sprint_yaw_deg`：`175` → `170`
- 该参数用于到达 70 号点、进入 70→288 冲刺段时的目标车头角度。
- 冲刺速度 `max_vel_x=2.7`、前向增益 `linear_x_gain=13.5`、航向 P `angular_gain=5.0`
  保持不变。

## 生效方式

launch 参数是运行时配置，不需要重新编译；同步到小车后，重启国赛 `2026.launch` 才会
生效。额外任务没有这组国赛冲刺参数。

## 验证与限制

- 已完成 XML 静态解析和参数文本核对。
- 已同步到车端 `/home/ucar/ucar_ws/src/ucar_2026_national/launch/2026.launch`，
  并核对 `sprint_yaw_deg=170`。
- 未启动任务或发送运动命令；现场试跑前仍须确认 `/odom_raw` 与 TF 有限且正常。
