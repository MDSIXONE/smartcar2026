# 2026-07-14：移除 CymPlanner 朝向线速度缩放

## 目的

移除 CymPlanner 在车头与路径存在夹角时将正常行进线速度额外缩放为 25%～100% 的硬编码逻辑，避免赛道转弯或朝向误差时出现非配置性的低速。

## 涉及文件

- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
  - 删除 `heading_speed_scale` 的计算。
  - 正常行进线速度仅保留 `linear_control`、`max_vel_x` 和 `motion_scale` 的控制。
- `docs/operations.md`
  - 说明新的速度行为及重新编译、重启要求。

## 验证

- 本地静态检查确认不再存在 `heading_speed_scale`，且线速度裁剪仍使用 `max_vel_x`。
- 已上传 `cym_planner.cpp` 至小车并完成 SHA-256 一致性校验。
- 已在小车端重新编译 `cym_planner;jie_ware`；新 `devel/lib/libcym_planner.so` 时间戳为 `2026-07-14 17:52:10 +0800`。
- 未重启 ROS、未自动发送导航目标；正在运行的 `move_base` 仍使用旧插件库，重启后才会加载本次修改。

## 已知限制

- 搬运模式、障碍碰撞检查、全局路径有效性和底盘 `linear_speed_max` 仍会限制或停止运动。
- 移除朝向减速后，转弯时车体会以更高线速度前进，需在空旷赛道验证。
