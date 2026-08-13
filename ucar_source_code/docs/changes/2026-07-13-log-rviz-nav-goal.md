# 在终端记录 RViz 导航目标

日期：2026-07-13

## 改动

- `navigation_2026` 节点订阅 `/move_base_simple/goal`。
- 每次在 RViz 使用 **2D Nav Goal** 后，在小车端 launch 终端输出目标坐标 `x/y` 与朝向 `yaw`（弧度和角度）。
- 该订阅只记录消息，不向 move_base 发送、取消或修改任何目标。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/2026.py`

## 操作

- 修改部署后重启 `roslaunch yolo2025 2026.launch`。
- 在 RViz 使用 **2D Nav Goal**，观察 launch 终端中的 `RViz 2D Nav Goal:` 日志行。

## 验证

- 已在小车端通过 Python 语法检查，并确认 `/move_base_simple/goal` 订阅、`rviz_goal_cb` 回调和 `RViz 2D Nav Goal:` 日志格式存在。
- 变更前文件备份：`/home/ucar/ucar_ws/.rviz_goal_log_backup_before_20260713/yolo2025/`。
