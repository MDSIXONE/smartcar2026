# 无 RViz 运行 main yolo2025 自动目标

## 目的

在关闭本机 RViz 的条件下运行 GitHub `main` 的 `yolo2025` 自动启动目标，判断跨机
可视化流量是否是 AHRS CRC 的必要条件，并观察运动负载下的里程计、TF 和 USB。

## 测试基线

- 小车 `yolo2025/launch/2026.launch` 与 `scripts/2026.py` 的 Git blob 分别为
  `59d713180d33c36e4f15a8a6a2b74af38b566c6d` 和
  `43637f0423d566f16291901f86fc0e587ae86c99`，与 `origin/main` 一致。
- WSL `192.168.8.199` 为唯一 ROS Master；小车地址为 `192.168.8.231`。
- ROS 图中没有 RViz。
- CP2102 位于外置 Hub 的 `1-2.3`，测试期间物理拓扑没有改变。

## 静态安全门

先以 `startup_goal_enabled:=false` 启动完整任务：

- `/odom_raw` 约 20 Hz，10 个样本无 NaN/Inf；
- `/imu` 约 50 Hz；
- `/scan` 约 12 Hz；
- `odom -> base_link` 与 `map -> base_link` 连续可读；
- 本地和全局代价地图话题存在；
- 没有 CRC 或新的内核 USB 事件。

启动期出现过旧版 `navigation_2026.scan_cb` 初始化竞态，以及地图/TF 建立前的短暂
extrapolation；节点随后恢复并通过安全门。发布零速度并停止无目标 launch 后才启动
自动目标。

## 自动目标结果

- 显式设置 `/navigation_2026/startup_goal_enabled=true`。
- 脚本取得连续 5/5 个稳定定位与全局规划样本后发送目标
  `map (-1.734, 2.305, yaw 1.571)`。
- `/cmd_vel` 采到线速度约 `0.20～0.32 m/s`，角速度最高 `1.0 rad/s`；
  `/odom_raw` 始终为有限值。
- `move_base` 报告启动目标和后续两个二维码朝向目标均为 `Goal reached`。
- 二维码扫描识别到 `a`、`d` 两个不同代码，低于要求的 3 个，因此按旧版逻辑跳过
  生产路线。
- 运动中出现过约 27～160 ms 的 `odom -> base_link` TF 落后和一次代价地图变换
  超时，但随后恢复，没有 `TF_NAN_INPUT`。
- 二维码流程结束约 44 秒、里程计速度已经为零后，出现一次
  `check crc16 faild(ahrs)`；随后立即发布零速度并停止。
- 内核没有新增 USB disconnect、reset 或 CP2102 重枚举。

## 结论

关闭 RViz 后，小车可以完成自动目标和二维码朝向运动，CRC 的复现时间也较晚；RViz
跨机流量或其带来的 CPU 调度负载可能提高故障概率。但无 RViz、停车后仍复现 AHRS
CRC，说明网络、RViz 和电机瞬时负载都不是唯一根因。剩余重点仍是驱动短读/重同步、
CP2102 UART 侧、线束/接地和底盘控制板数据质量。

## 涉及文件与清理

- 更新 `docs/operations.md` 和 `犯错档案.md`；
- 新增本记录；
- 没有修改、构建或部署源码、参数和资源；
- 测试结束后小车与 WSL 无 ROS 启动进程，WSL 11311 已停止监听。
