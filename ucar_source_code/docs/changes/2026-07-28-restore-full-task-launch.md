# 恢复 2026 真机完整任务入口

## 目的

为正式真机启动补回可显式启用的完整任务模式，使小车依次执行默认启动目标、二维码方向扫描和生产区编号路线，同时避免任务节点与独立激光中继重复发布 `/scan`。

## 涉及文件

- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/yolo2025/launch/2026.launch`
- `docs/operations.md`

## 实现

- `ucar_2026/2026.launch` 新增 `scan_relay_enabled`，默认保持手动导航模式不变。
- `yolo2025/2026.launch` 新增 `full_task_enabled`。启用后：
  - 停用独立 `navigation_scan_relay`；
  - 复用正式入口已启动的摄像头，只启动二维码扫描器；
  - 启动 `navigation_2026`，启用启动目标、二维码扫描和生产路线；
  - 显式传入 `production_square_centers.json`。
- 完整任务首次运行发现 CymPlanner 初始化超过旧代码固定的 15 秒 action 等待时间；
  `navigation_2026` 现复用 `startup_goal_ready_timeout`（默认 90 秒）等待
  `move_base`，并忽略 roslaunch 关闭发布者后仍到达的末尾激光回调。

## 验证与限制

- XML 解析、Python 2 编译、车端哈希/换行同步及 ROS 静态启动检查通过。
- 真机冷启动后 `move_base` 在新等待预算内就绪，定位和规划连续 5 次通过，
  启动目标成功发出。
- 首次目标被 `laser_avoidance` 的车体投影阻止。对照仿真前端后确认真机配置带错模式：
  仿真默认是无车体投影的 `main_legacy`。真机现已同步改回 `main_legacy`，保留局部
  代价地图避障。
- 继续验证发现 YAML 原根键 `cym_planner/CymPlanner` 与插件实际私有命名空间不匹配，
  参数虽出现在参数服务器上但插件仍使用默认值。根键已修正为 `CymPlanner`；运行时需以
  `/move_base/CymPlanner/navigation_mode=main_legacy` 和初始化日志双重确认。
- 修正模式后的首次完整任务冷启动中，底盘串口短暂报告 Odom/IMU inactive，并出现
  AHRS/IMU CRC 错误。虽然传感器随后自动重新激活且启动目标通过 5 次规划检查后发出，
  仍按安全门发布零速度并中止该轮，重启底盘/里程计链路后再验证。
- 完整任务会驱动车辆，只能在 `/odom_raw` 和关键 TF 通过安全门后启用。
