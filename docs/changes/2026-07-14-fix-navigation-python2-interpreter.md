# 2026-07-14：修复默认导航脚本的 Python 解释器

## 目的

修复默认导航脚本在车上启动后立即退出的问题。根因是脚本 shebang 指向 Python 3，而车载 ROS Melodic 的 `tf`、`nav_msgs.srv` 模块由 Python 2 环境提供。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - shebang 从 `python3` 改为 `python2`。
- `docs/operations.md`
  - 明确该脚本的 Python 2 运行要求。

## 验证

- 首次受控运行中，`navigation_2026` 于启动后退出，日志显示 Python 3 无法导入 `tf`。
- 修复后，小车端 Python 2 语法检查以及 `rospy`、`tf`、`GetPlan`、`PoseStamped` 模块导入均已通过。
- 修复后的 30 秒受控导航验证已能够完成路径就绪检查并发送目标；运行结束后已确认 `move_base`、`base_driver`、`navigation_2026` 均退出。

## 已知限制

- Python 2 仅用于兼容 ROS Melodic；新增 Python 代码必须保持 Python 2.7 语法兼容。
