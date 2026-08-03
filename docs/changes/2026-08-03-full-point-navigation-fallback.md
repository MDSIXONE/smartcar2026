# 2026 全程前视点导航回退

## 目的

上一轮实车在 `52 → 12` 的 `body_projection` / 局部车体投影链路返回 move_base status `4`，
并在门洞附近卡住。按用户决定，暂不微调局部 footprint、弹性带或代价地图；自动任务先全程
回退到前视点模式验证其路线行为。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `docs/operations.md`
- `犯错档案.md`

## 实现

- 安全门通过、首次导航至 52 前，任务等待 CymPlanner 的 `/ucar/navigation_mode` 订阅并锁存
  发布三次 `point`；无订阅连接仍会安全中止。
- 移除了 QR 后切换 `body_projection` 的调用，所以 QR、生产五段和 OCR 复拍导航保持同一模式。
- 将连接超时参数名明确为 `navigation_mode_connect_timeout`，兼容读取旧
  `body_mode_connect_timeout` 配置作为回退。
- 没有删除 CymPlanner body/elastic 代码，也没有改 footprint、代价地图、动态障碍链、阈值或
  `mode1_point` 速度参数。

## 验证

- 新增四项车端 Python 2 回归：锁存发布只能是三次 `point`；普通任务在前往 52 前、
  `resume_production_only` 任务在首个生产腿前都必须先选择 `point`；完成 QR 后至首个生产腿
  的完整路径也不得调用 `body_projection`。变更前发布测试因方法不存在红灯；变更后四项均通过。
- 本机 `unittest`：32 项无失败，21 项因缺 ROS Python 按设计跳过；Python 语法、launch XML
  和 `git diff --check` 均通过。
- 已同步到 Ubuntu 18.04 / Melodic 小车，`catkin_make --pkg ucar_2026
  -DCATKIN_ENABLE_TESTING=ON` 成功，直接 Python 2 回归 32 项均通过。整个阶段未启动 ROS
  节点或车辆。

## 已知限制

- 前视点模式不执行 body projection 的完整 footprint/Twist sweep；这个回退只用于隔离门洞
  status `4`，不证明门洞对真实车体连续安全。
- 当前 `mode1_point` 上限仍为 `0.50 m/s`、`1.00 rad/s`；用户要求本轮不微调参数。实车测试
  仍须用户确认起点位置、有人看护和急停，且失败后不得自动重跑。

## 2026-08-03 实车试跑结果

- 小车通过本次 WSL Master `192.168.8.198` 连接；安全门后锁存的
  `/ucar/navigation_mode` 为 `point`，`/odom_raw` 与 `map → base_link` 均为有限值。
- 任务到达 52 后即时读取 QR `a → d → i`，随后 `52 → 12` 成功到达，位置误差 `0.035 m`。
  说明前视点模式可完成本轮到达 12 前的路线验证；启动早期的一次 `NO PATH!` 未复发。
- 在点 12 的原地 OCR 旋转中，识别到“食品加工车间”（置信度 `73.4`，转角 `3.850 rad`），
  但任务安全中止，尚未继续至 23。
- 中止原因是异步 OCR 的返回值没有带回任务侧已保存的
  `capture_requested_pose_map`：`start_async_motion_ocr()` 将位姿放入 task 元数据，
  而 `finish_async_motion_ocr()` 只返回 helper response。随后的回到抓拍朝向步骤拒绝缺失位姿，
  以 `OCR candidate has no capture pose` 结束。这是实现缺陷，不是代价地图或前视点碰撞检查
  的失败。
- 任务按异常路径发布零速、关闭相机与 OCR；随后显式执行 `stop_2026_task.sh` 并停止本次 WSL
  Master。没有自动重跑；车辆停在点 12 附近。
