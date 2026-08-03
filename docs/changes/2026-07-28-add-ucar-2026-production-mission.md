# 新增 ucar_2026 正式生产任务

## 目的

基于 `production_full_grid_all_numbered.json` 新增一条显式启用、无 RViz 的
正式任务：

`52 → 面向 262/232/295 扫码 → body_projection →
12 → 24 → 16 → 28 → 19 → 170（朝向 319）`

## 坐标与任务语义

- 52：`(-1.75, 2.25)`，扫码阶段车辆停留的中心点。
- 262：`(-2.50, 2.25)`，从 52 朝西观察。
- 232：`(-1.75, 3.00)`，从 52 朝北观察。
- 295：`(-1.75, 1.50)`，从 52 朝南观察。
- 生产中心路线：
  - 12 `(-1.75, 0.75)`
  - 24 `(-0.75, 0.25)`
  - 16 `(0.25, 0.75)`
  - 28 `(1.25, 0.25)`
  - 19 `(1.75, 0.75)`
- 终点 170：`(0.00, -0.50)`。
- 朝向参考点 319：`(0.00, -0.75)`，因此终点 yaw 为 `-π/2`。

262、232、295 是墙边观察点，不是行驶目标；任务只用它们计算 52 处的车头方向。

## 实现

- 新增 `production_task_2026.py`：
  - 以后台状态机串联 move_base、二维码结果、CymPlanner 模式和原地旋转。
  - 每个二维码方向先静止识别 4 秒；没有新结果时以 `0.18 rad/s` 最多搜索
    一整圈，识别到立即停止该圈并进入下一观察方向。
  - 默认要求三个二维码结果互不重复；一圈后仍没有新结果则安全终止，不进入生产区。
  - 三个结果齐全后向 `/ucar/navigation_mode` 发布并确认连接
    `body_projection`。
  - 生产路线每点到达后以 `0.35 rad/s` 完整转一圈，并复核位置漂移。
  - 最终到达 170 并验证车头朝 319。
  - 全程监控有限且新鲜的 `/odom_raw`、两段 TF、全局路径、action 状态以及
    CRC、帧长、传感器掉线和 TF_NAN 日志；异常时取消目标并连续发布零速度。
- 新增 `production_task_geometry.py`，隔离 JSON、坐标、方位角和跨 `±π`
  转角累计逻辑，兼容 Python 2。
- `ucar_2026/launch/2026.launch` 新增默认关闭的 `task_enabled`。
  开启时复用现有相机，启动二维码节点和正式任务节点；不启动 RViz。
- `start_2026.sh` 新增 `mission` 模式。默认 `manual` 行为不变；历史
  yolo2025 任务仍保留为 `full`。
- 将用户提供的完整 418 点 JSON 原样复制到
  `ucar_2026/config/production_full_grid_all_numbered.json`，本地两份文件
  SHA-256 均为
  `2347ab3abb73c0cae3f98a87beab714f97bd68b937b58e36e7b2d5e3b028bfb7`。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026/config/production_full_grid_all_numbered.json`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
- `ucar_ws/src/ucar_2026/CMakeLists.txt`
- `ucar_ws/src/ucar_2026/package.xml`
- `docs/quickstart.md`
- `docs/operations.md`
- `犯错档案.md`

## 验证结果

- 本机 Python 3.13：5 项几何/坐标测试通过，两个 Python 文件可编译，
  launch 和 package XML 可解析。
- 小车 Ubuntu 18.04 / Python 2.7：
  - 两个脚本 `py_compile` 通过。
  - 直接 unittest：5 项通过。
  - `catkin_make --pkg ucar_2026` 通过。
  - `catkin_make run_tests_ucar_2026`：
    `5 tests, 0 errors, 0 failures, 0 skipped`。
  - `roslaunch --nodes ... task_enabled:=true` 展开包含
    `/qrcode_scanner` 与 `/production_task_2026`，不含 RViz。
- 所有部署文件逐项 SHA-256 与本地一致，脚本权限为 `0755`，shebang 为 LF。
- 使用 WSL `192.168.8.199` 唯一 Master 做延迟静态启动：
  - 将 `task_start_delay` 设为 3600 秒，任务保持 `WAITING_START`。
  - 二维码扫描参数为 `false`，move_base goal 列表为空。
  - `/cmd_vel` 两秒内无消息，`/odom_raw` 有限且速度全零。
  - `/ucar/navigation_mode` 已连接 task → move_base；
    `/qrcode_start_flag` 已连接 task → qrcode_scanner。
  - 无 RViz、无 CRC、无运动。
- 静态验证后已停止车端 launch 和 WSL Master；两端无 ROS 进程或 11311
  监听残留。

## 已知限制

- 本轮没有执行长路线或任何旋转，只完成了静态接线验证。
- 第一次动态测试仍必须先用 manual 模式通过有限里程计、两段 TF 和串口日志安全门。
- 默认策略把每个观察位置的“识别成功”定义为该阶段新出现且未用于前一位置的
  `/qr_result` 文本；如果三张二维码允许内容相同，需要将
  `require_distinct_qr_codes` 改为 `false`。
- 原地旋转时 task 节点是预期的第二个 `/cmd_vel` 发布者；它只在 move_base
  目标完成或取消后发布纯角速度。

## 实车完整验收

2026-07-28 在车辆重新放回起点、CP2102 正常枚举后完成无 RViz 冷启动实测：

- 静态门禁：`/odom_raw≈20 Hz`、`/imu≈50 Hz`、`/scan_filtered≈12 Hz`，
  里程计连续有限，两条 TF 正常，到 52 可生成路径；
- 到达 52 后依次识别 `…/a`、`…/d`、`…/i`；
- body projection 切换成功；
- 12、24、16、28、19 均到达并完成 360°；
- 最终 170 到达误差 `0.018 m`，朝向 319 校验通过；
- `/ucar_2026/task_state=SUCCEEDED`，
  `/ucar_2026/task_result` 中 `success=true`；
- 全程无 CRC、`head_len`、NaN、TF_NAN 或串口掉线。
