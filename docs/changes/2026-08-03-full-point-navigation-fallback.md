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
