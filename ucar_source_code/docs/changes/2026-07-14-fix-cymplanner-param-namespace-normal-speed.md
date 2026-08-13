# 2026-07-14：修复 CymPlanner 参数命名空间并恢复正常速度

## 目的

修复参数服务器中存在高值、但 CymPlanner 初始化时仍退回源码默认 `0.2 m/s` 与 `0.5 rad/s` 的问题；将导航命令恢复至约 `1.0 m/s` 的正常、安全量级。

## 涉及文件

- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
  - 参数读取依次兼容 move_base 传入的运行时插件名、规范命名空间 `~/cym_planner/CymPlanner` 和旧命名空间 `~/CymPlanner`。
  - 启动日志增加实际插件名，便于确认名称解析。
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - 线速度、行进角速度、末端角速度均设置为 `1.0`。
  - 线速度 P/D 设置为 `4.0 / 0.2`，其余控制增益恢复为正常量级。
- `docs/operations.md`
  - 更新构建、启动后的参数和日志验证预期。

## 验证

- 本地 YAML 解析和源码静态检查已通过。
- 已上传并校验 SHA-256：`cym_planner.cpp` 为 `4ba6b0d8…21151ee5`，参数 YAML 为 `7a36a8c2…ada31042`。
- 小车端 `catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;jie_ware"` 已完成；新库 `devel/lib/libcym_planner.so` 的构建时间为 `2026-07-14 18:38:25 +0800`，且包含新的启动日志格式。
- 用户手动重启 launch 后，已确认新的 `move_base` 进程映射 `~/ucar_ws/devel/lib/libcym_planner.so` 的新 inode；运行时参数 `max_vel_x`、`max_vel_theta`、`final_yaw_max_vel` 均为 `1.0`。

## 已知限制

- 构建不会替换正在运行的 `move_base` 已加载库；必须在构建后重启 launch。
- `1.0 m/s` 是 ROS 命令上限，实际速度仍受底盘、供电、轮胎和场地影响。
