# 取消导航速度中继

日期：2026-07-13

## 目的

让 CymPlanner / move_base 直接向底盘速度话题 `/cmd_vel` 发布命令，移除 `2026.py` 中的速度话题中继以及线速度 `5/7`、角速度 `0.5` 的隐式缩放。

## 涉及文件

- `ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch`
- `ucar_ws/src/yolo2025/scripts/2026.py`
- `docs/operations.md`

## 改动

- 删除 move_base 从 `/cmd_vel` 到 `/teb_cmd_vel` 的重映射。
- 删除 `2026.py` 对 `/teb_cmd_vel` 的订阅、速度缩放和向 `/cmd_vel` 的二次发布。
- 保留 `2026.py` 的激光转发、语音、导航目标与任务基础设施。
- 更新部署、验证与本地回滚命令；小车端不创建或保留备份。

## 验证

- 本地：`2026.py` 已通过 Python 语法检查，`cym_move_base_omni_2026.launch` 已通过 XML 解析；两个运行文件均不再包含 `/teb_cmd_vel`、缩放系数或速度中继回调。
- 部署前：已确认小车未运行导航进程、没有遗留备份目录；原车端两个文件已下载到本地 `back/2026-07-13-remove-velocity-relay-before-deploy/`，并以 SHA-256 校验。
- 车端：两个运行文件已同步并逐文件以 SHA-256 校验；`python3 -m py_compile`、可执行权限检查、速度中继引用检查和 `roslaunch --nodes yolo2025 2026.launch` 均通过。节点图包含 `/move_base`、`/base_driver` 和 `/navigation_2026`，静态检查未启动底盘。
- 运行态：以 `startup_goal_enabled:=false` 启动后，`/move_base`、`/base_driver`、`/navigation_2026` 与 `/lidar_loc` 均存活。`rostopic info /cmd_vel` 显示唯一发布者为 `/move_base`、唯一订阅者为 `/base_driver`，两者的 TCPROS 连接已建立；`/teb_cmd_vel` 不存在。
- 实车导航：向 `map (-1.534, 2.105, yaw -2.950)` 发送一个 `move_base` 目标，动作结果为 `SUCCEEDED`，文本为 `Goal reached.`；目标完成后采样到的 `/odom.twist` 线速度和角速度均为 `0.0`。

## 已知限制

- 取消缩放后，CymPlanner 的速度命令会直接抵达底盘；底盘仍由 `linear_speed_max: 15.0` 和 `angular_speed_max: 3.14` 限制。
- 实际行驶速度仍取决于规划器、路径、障碍物、底盘电源和机械状态，必须在安全区域进行实车测试。
- 已在运行中的 ROS 图确认 `/move_base -> /cmd_vel -> /base_driver` 的实时连接，并完成一次成功导航；后续更改速度上限或定位/代价地图参数后，仍须在安全区域重新验证。
- 运行时发现 `ucar_cam/cv_bridge_flip.py` 持续报 `cv_bridge_boost` 的 Python 2/3 模块兼容错误；本次未修改摄像头文件，速度链路、定位和导航节点仍已正常运行。该错误会影响图像翻转节点，需另行修复。
