# ucar_2026 无 RViz 单目标实车测试

## 目的

停止 GitHub `main` 的历史 yolo2025 对照任务后，使用已部署的新
`ucar_2026/launch/2026.launch`，在不启动 RViz 的条件下发送一个标准
move_base 目标，验证当前 CymPlanner、定位、里程计和底盘的完整运动链路。

## 测试条件

- WSL Ubuntu 20.04 是唯一 ROS Master：
  `ROS_MASTER_URI=http://192.168.8.199:11311`。
- 小车地址为 `192.168.8.231`，没有启动本机 `roscore`。
- 车端以下四个关键文件与本地 SHA-256 完全一致：
  - `ucar_2026/launch/2026.launch`
  - `ucar_2026/scripts/navigation_scan_relay.py`
  - `ucar_nav/launch/cym_move_base_omni_2026.launch`
  - `cym_planner/src/cym_planner.cpp`
- 全程没有 RViz 节点。
- 目标为 `map (-1.734, 2.305, yaw=1.571)`，通过标准
  `/move_base_simple/goal` 发布；没有启动 yolo2025 任务节点。

## 静态安全门

- `/odom_raw` 连续 10 帧位置、姿态和速度均为有限值，静止速度全零。
- `/odom_raw` 约 `19.98 Hz`，`/imu` 约 `49.94 Hz`，
  `/scan_filtered` 约 `11.97 Hz`。
- `odom -> base_link` 与 `map -> base_link` 连续可用。
- `/cmd_vel` 唯一发布者为 `/move_base`，唯一订阅者为 `/base_driver`。
- 只读 `/move_base/make_plan` 成功生成到目标的路径。
- 启动早期存在 TF 建链的短暂 past extrapolation，随后恢复；发送目标前未见
  CRC、`head_len`、TF_NAN 或非有限里程计。

## 运动结果

- move_base 接收目标并驱动车辆运动。
- 运行约 5 秒时全局规划短暂两次报告 `NO PATH!`，同一窗口有一次
  global costmap 更新耗时 `0.678 s`；目标未 aborted，随后重新规划恢复。
- 约 31 秒后状态为 `3`，文本为 `Goal reached.`。
- 最终 `map -> base_link` 约为
  `(-1.72, 2.30, yaw 93.7°)`。
- 到达后再次发布零速度，`/odom_raw` 线速度、角速度均为有限的 `0.0`。
- 停车后继续观察约 45 秒，没有出现 AHRS/IMU CRC、`head_len` 或 TF_NAN。
- `dmesg` 没有新的 CP2102 disconnect、reset 或重新枚举。

## 测试工具失误

临时 Python 2 安全监控器误用了 Python 3 才有的 `math.isfinite()`，导致
`/odom_raw` 回调持续报错，无法使用该监控器的位移和有限值统计。move_base 状态
监控仍完成，之后使用独立 `rostopic echo`、`tf_echo` 和零速度命令重新核验结果。
该临时脚本没有写入仓库；错误与预防措施已记录到 `犯错档案.md`。

## 收尾

- 先停止 `ucar_2026 2026.launch`，确认所有子节点退出。
- 再停止 WSL ROS Master。
- 最终检查小车和 WSL 均无 `roslaunch`、`roscore`、`rosmaster`、RViz、
  `move_base`、`base_driver`、`navigation_scan_relay` 或 `lidar_loc` 残留，
  WSL 不再监听 11311。

## 已知限制

- 本轮只执行一次目标，没有覆盖多次冷启动或长时间运行。
- 停车后 45 秒未复现 CRC 不能证明串口问题已经修复；此前无 RViz 的 main
  yolo2025 测试曾在停车约 44 秒后出现一次 CRC。
- 本轮没有修改导航源码或车端文件。
