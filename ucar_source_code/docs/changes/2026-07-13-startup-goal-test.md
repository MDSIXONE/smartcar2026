# 启动后自动发送一次导航测试目标

日期：2026-07-13

## 改动

- `navigation_2026` 增加一次性启动目标回调：等待 move_base action server 就绪后发送目标。
- `2026.launch` 默认启用启动目标，坐标为 `map (-1.534, 2.105)`，朝向为 `yaw -2.950 rad`。
- 启动目标延迟 2 秒触发，并最多等待 move_base 15 秒；超时仅打印错误，不会重复发送。
- 增加 `startup_goal_enabled` launch 参数，可用 `startup_goal_enabled:=false` 禁用。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/2026.py`
- `ucar_ws/src/yolo2025/launch/2026.launch`

## 操作

- 直接启动会自动发送一次测试目标：`roslaunch yolo2025 2026.launch`。
- 仅使用 RViz 手动导航：`roslaunch yolo2025 2026.launch startup_goal_enabled:=false`。

## 验证

- 已在本地通过 Python 语法检查和 launch XML 解析，并在小车端再次通过 Python 语法检查、读取确认目标参数与回调存在。
- 变更前本地备份：`back/2026-07-13-startup-goal-before-change.tar.gz`。
- 已实际启动：move_base 接收目标 `/navigation_2026-1-1783952840.480`，最终返回 `status: 3`、`Goal reached.`。
