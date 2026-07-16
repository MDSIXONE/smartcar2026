# 2026-07-14：默认导航目标等待定位与路径就绪

## 目的

修复 `roslaunch yolo2025 2026.launch` 启动后默认目标可能在定位或全局代价地图尚未就绪时发送，导致 GlobalPlanner 报 `NO PATH!` 且车辆不动的问题。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - 默认目标先等待 `move_base` action server，再循环查询 `map -> base_link` 与 `/move_base/make_plan`。
  - 仅在存在至少两个路径点时发送动作目标；超时则不发送任何运动命令。
  - 使用 Python 2 shebang，与车载 ROS Melodic 的 `tf` 和服务模块保持兼容。
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - 新增 `startup_goal_ready_timeout: 90.0` 参数。
- `docs/operations.md`
  - 更新默认目标行为、部署文件和启动说明。

## 验证

- 已用 `/move_base/make_plan` 从当前定位 `map (0.28, 2.65)` 到默认目标计算出完整路径；该服务调用不发送运动命令。
- 本地 Python 语法、launch XML 与就绪门控静态检查已通过。
- 已上传并校验 SHA-256：Python 2 修复后的 `2026.py` 为 `57aeafc4…6f5856e5`，`2026.launch` 为 `3c15a9cd…10fa8af6`。
- 小车端的首次受控运行发现 Python 3 无法导入 ROS Melodic `tf`，因此脚本在发目标前退出；已改用 Python 2 shebang，并将以 Python 2 重新验证。
- 修复后的 30 秒受控运行中，就绪检查成功得到 `293` 个路径点并发送默认目标；约 8 秒后 move_base 仍报告 `NO PATH!`。因此“过早发送默认目标”已排除，但全局路径会在运行期间变为不可达，需继续检查全局代价地图和定位稳定性。
- 用户手动重启后，`navigation_2026` 日志应依次出现 `Waiting for localization and global plan`、`Localization and global plan ready`，之后才发送默认目标。

## 已知限制

- 若 90 秒内定位未收敛、地图不可达或全局代价地图持续阻塞，脚本会安全地不发送默认目标。
- 默认目标发送后仍受 CymPlanner、局部碰撞检查及底盘状态影响。
