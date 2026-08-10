# 主流程改版：双物品输入（现实+仿真）→ 双码扫描 → 双类别巡航 → 停入播报 → 联动本机 WSL 仿真 → 终点

## 目的

按用户要求把 2026 生产主流程升级为「现实物品 + 仿真物品」双物品任务：

1. **双物品输入**：任务启动后依次等待操作员输入两个物品名——第一个为现实物品
   （放置在场上的货品），第二个为仿真物品（交给本机 WSL 仿真任务处理的货品）。
   两者必须非空且不相同。
2. **双码扫描**：QR 区有 3 个二维码（分别对应 3 个物品），本次任务只取与输入的两个
   物品名匹配的 2 个码；同一码只取第一次扫描结果，非目标码忽略继续扫，集齐 2 个
   才进入下一步（最多完整扫 2 轮×3 个观察面）。
3. **双类别巡航**：大模型分别对两个物品名分类得到两个目标类别。
   - 第一轮沿完整生产路线找**现实物品类别**，找到后停入加工区，停留 3 秒改为
     **同步播报**「已将[货品名称]放入[仓库类别]」，确认播报完成后**从找到的点
     继续**（不再从路线开头重跑）找第二个类别；
   - 第二轮沿剩余路线找**仿真物品类别**，找到后停入对应类别，**不发消息给本机
     WSL 仿真**：HTTP POST /start 启动仿真任务（把第二个物品输入仿真），随后小车
     保持静止轮询 GET /status 等待仿真完成；仿真完成后播报「仿真任务已完成，
     已将[货品名称]放入[仓库类别]」，再前往终点 35/36 中点（车头朝 170）SUCCEEDED。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`（任务状态机，Python 2）
- `ucar_ws/src/ucar_2026/launch/2026.launch`（新增仿真通信参数）
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`（单测同步+新增）
- `simulation/bridge/sim_bridge.py`（新建，仿真端 HTTP 桥接服务，Python 3）
- `simulation/bridge/README.md`（新建，桥接服务部署/启动/协议文档）

## 行为

### 任务节点 production_task_2026.py

- **双物品输入**：`wait_for_item_input` → `wait_for_item_inputs`（依次两个提示
  「请输入现实物品名称后回车」「请输入仿真物品名称后回车」），任一为空或两者相同
  → MissionAbort。`expected_real_item_text` / `expected_sim_item_text` 存储，
  `expected_item_text` 保留为现实物品（兼容）。
- **扫码收集**：新增 `collect_target_qr_codes(targets, rounds=2)`——按
  `qr_observation_numbers`（262/232/295）顺序面朝观察点，每个点用「导航面朝 +
  fresh 等待 + 超时转一圈」骨架，接受条件参数化（只收 `text in targets` 且未收集的
  码）；非目标码/重复码 `_reject_qr_code` 记入 `used_qr_codes` 后继续；2 轮未集齐
  → MissionAbort。`scan_observation_point` / `wait_for_fresh_qr` /
  `rotate_full_revolution` 增加 accept 参数。集齐后对两个物品名各调一次
  `classify_qr_text(obs, item_text)`（分类仍以物品名为准，非二维码文本）。
- **双类别校验**：`qr_classification_category_for_item` 分别取现实/仿真类别，
  任一缺失 → MissionAbort。
- **第一轮巡航**：`cruise_production_route(legs, start_segment_index, target,
  item)`——按段 `navigate_target_and_scan`（target_category 透传），本轮新增观察
  且命中目标类别即返回全局 leg 索引。找到后沿用 PROCESSING_STOP 停入逻辑
  （墙交点向内 25cm、车头朝外），随后 `PROCESSING_ANNOUNCE_%03d` 状态 +
  `speak_wait(u"已将%s放入%s")` 同步播报（播报完成才继续）。
- **第二轮巡航**：`second_legs = production_navigation_legs[found_leg_index+1:]`
  （从找到的点继续，不重跑）；found 在最后一段 → MissionAbort。找到仿真类别后同样
  停入，无 dwell。
- **仿真联动**：停入后依次 `SIMULATION_START`（`simulation_request_start`：POST
  `/start`，body `{"item_name","category"}` UTF-8；HTTP 409 → MissionAbort，其余
  重试 `simulation_start_retries` 次）→ `SIMULATION_WAIT_DONE`
  （`simulation_wait_done`：GET `/status` 轮询，state=done 返回、failed MissionAbort、
  总超时 `simulation_done_timeout` MissionAbort；每轮 `require_safe()`，车保持静止）
  → `SIMULATION_DONE` → `speak_wait(u"仿真任务已完成，已将%s放入%s")` → 终点
  35/36 中点朝 170 → SUCCEEDED。
- **同步播报**：新增 `speak_wait(text, timeout)`——Popen 后轮询 poll（0.1s），
  tts_say.py 播放完进程才退出；超时 terminate/kill + logwarn **不中止任务**；
  `speak()` 异步版本保留。
- **仿真主机解析**：`resolve_simulation_host`——`simulation_host` 显式配置优先，
  否则从 `ROS_MASTER_URI`（http://host:11311）去 scheme/端口推导；都不可得 →
  MissionAbort。
- **OCR 去重语义**：`scan_production_point` 增加 `target_category` 参数（匹配该类别
  才处理）；停入按 `(category, wall_point_number)` 经 `served_wall_points` 集合去重
  ——允许同类别不同墙点多次停入（两个物品同类别时第二轮找下一个同类别点），已停入
  墙点标记 `processing_category_already_served`。
- **结果字段**：`publish_result` / `save_observation_summary` 增加 `items`、
  `target_categories`。
- **移除**：`PROCESSING_DWELL` 状态与 `processing_dwell` sleep（参数保留在 launch，
  注释说明已被播报替代）。

### launch/2026.launch

- 新增：`simulation_port=11313`、`simulation_host=""`（空=从 ROS_MASTER_URI 推导）、
  `simulation_start_timeout=30`、`simulation_start_retries=3`、
  `simulation_done_timeout=900`、`simulation_poll_period=2.0`、`speak_wait_timeout=60`。
- `processing_dwell_seconds` 注释更新为「已被停入播报替代，不再使用」。

### 仿真端桥接 simulation/bridge/sim_bridge.py（新建）

- HTTP 服务监听 `0.0.0.0:11313`（`--port` 可改），状态机
  waiting →(POST /start)→ running →(done 话题)→ done / failed。
- 启动前就绪检查：轮询 `rostopic echo -n 1 /sim_task3/arm_initial_pose_ready`
  输出含 `data: True` 即仿真环境就绪（`--wait-ready-timeout` 默认 300s，0 跳过），
  就绪打印 `SIMULATION_BRIDGE_READY`。
- POST /start：body `{"item_name","category"}` → 后台 Popen
  `roslaunch car3 task3_execute.launch cargo_category:=<类别> cargo_name:=<名称>`
  （参数列表形式，无 shell；日志重定向 `task3_run_<时间戳>.log`）；running 中 →
  HTTP 409；done/failed 后 → HTTP 409（需重启 bridge 再跑）。
- GET /status → `{"state","detail","item_name","category"}`。
- 后台线程轮询 `rostopic echo -n 1 /sim_task3/done`（`data: True` → done）；roslaunch
  非零退出 → failed；`--done-timeout`（默认 1800s）超时 → failed。
- 仅标准库，Python 3.8+；只做「物品名透传 + 完成状态回读」，不触碰仿真 src /
  规划器 / URDF / world（符合比赛硬性要求）。

## 协议（小车 ↔ 仿真桥，已定死）

| 方向 | 请求 | 响应 |
| --- | --- | --- |
| 小车 → 桥 | POST /start `{"item_name":"苹果","category":"食品"}` | 200 `{"accepted":true,"state":"running"}`；409 已在跑/已完成；400 JSON 无效 |
| 小车 → 桥 | GET /status | 200 `{"state":"waiting\|running\|done\|failed","detail":"...","item_name":"...","category":"..."}` |

## 验证结果

- 本机 `python -m py_compile`：production_task_2026.py / test_production_task_geometry.py
  通过；2026.launch XML 解析通过；sim_bridge.py 通过。
- 本机 `python test/test_production_task_geometry.py`：68 tests OK（53 skipped =
  任务类需 ROS，按设计跳过）。
- 已部署小车（ssh ucar@192.168.8.231，3 文件 SHA256 与本地一致）；车端
  `catkin_make -DCATKIN_WHITELIST_PACKAGES="ucar_controller;ucar_2026" run_tests_ucar_2026`
  + `catkin_test_results`：**80 tests, 0 errors, 0 failures, 0 skipped**。
- 首轮车端测试暴露 3 个测试缺口（`cruise_production_route` 引用 `observations`、
  `run_mission` OCR 分支引用 `camera_image_topic`，测试 setUp 未初始化）——已修复
  （test setUp 补 `observations=[]`、`camera_image_topic`），详见犯错档案 2026-08-10。

## 已知限制

- 仿真桥接服务需要在仿真任务（task3_prepare.launch）就绪后、小车任务开始前由操作员
  提前启动；未启动时小车 `simulation_request_start` 重试后 MissionAbort。
- 双物品同类别（如苹果/香蕉都属食品）时，第二轮会找同一类别的下一个墙点停入；
  若路线上同类别的墙点已全部停入，第二轮 MissionAbort。
- 两个物品名必须不同（否则扫码无法区分，启动即 MissionAbort）。
- 小车等待仿真期间保持静止（仅在加工区停点），期间安全门持续监控。
- 仿真端一次运行结束后 bridge 保持 done 状态，重跑需重启 bridge。
- 仿真桥尚未在 WSL + Gazebo 环境实机联调（需 prepare.launch + 真仿真环境），
  属于部署验证待办。

## 追加：本机 WSL 消息链路联调验证（2026-08-10，通过）

在本机 WSL（Ubuntu 20.04 + ROS Noetic，无 GPU 软件渲染）+ 小车
（192.168.8.231）完成「小车模拟发送 → 仿真执行 → 完成回传」全链路联调：

1. **WSL 常驻配置**：WSL2 默认在最后一个 `wsl` 会话退出后关闭发行版，后台
   roslaunch 被连带杀掉（master 日志出现 `keyboard interrupt, will exit`）。
   已在 `C:\Users\<用户>\.wslconfig` 的 `[wsl2]` 段设 `vmIdleTimeout=-1` 并
   `wsl --shutdown` 重启生效；跨会话后台进程验证存活。
2. **就绪判定修正**：bridge 原轮询 `/sim_task3/arm_initial_pose_ready`，实测该
   话题由 set_arm_initial_pose 节点发布一次后即注销（launch 中
   `hold_initial_arm_pose=false`，节点发布后退出），bridge 会等到超时。已改为
   轮询常驻的 `/map` 话题（map_server 全程发布），`SIMULATION_BRIDGE_READY`
   正常触发。
3. **无界面启动**：`roslaunch car3 task3_prepare.launch gui:=false rviz:=false`
   在无 GPU 环境下成功（机械臂校准 `calibrated initial arm pose applied
   smoothly`；有 GUI 时首次尝试因渲染负载导致 gazebo 服务无响应、各节点
   Traceback 退出）。
4. **小车模拟客户端**（`/tmp/sim_client_test.py`，Python 2 urllib2，与
   production_task_2026 同路径实现）：
   `POST /start {"item_name":"苹果","category":"食品"}` → 200 accepted →
   roslaunch `task3_execute.launch cargo_category:=食品 cargo_name:=苹果`
   （pid 1781）→ 每 2 s 轮询 `GET /status` → 约 60 s 后
   `SIMULATION_BRIDGE_DONE item=苹果 category=食品` →
   `/sim_task3/status = DONE: 苹果 delivered to 食品加工车间; wall=61.6s
   sim=16.6s effective_RTF=0.269` → 小车侧最终 `GET /status` 返回
   `{"state":"done","detail":"done","item_name":"苹果","category":"食品"}`。
5. **防火墙**：无需新建规则——既有 `ROS TCPROS from UCar to WSL` 已是
   `port=Any + RemoteAddress=LocalSubnet`，11313 自动覆盖（实测小车
   `>/dev/tcp/192.168.8.198/11313` 通过）。
6. **清理**：任务结束后 kill task3_execute roslaunch → prepare 进程树 →
   残留 gzserver，`ps` 确认无任何 roslaunch/gzserver/rosmaster/move_base/
   task3/sim_bridge 进程（无后台残留终端）。

待办：仿真桥接脚本与 README 的本地修改需提交仿真仓库并 fast-forward 到
WSL `/home/car/smartcar2026-simulation`（按仿真侧 AGENTS 规则 7/8）；小车
端正式 mission 全流程（双码扫码 → 双类别巡航 → 停入播报 → 仿真联动 → 终点）
实车验收。

## 追加：实车全流程验证 SUCCEEDED（2026-08-10 晚）

带界面仿真 + 全部新功能实车全流程跑通（run_20260810_165902，SUCCEEDED）：

```
输入 蛋糕（现实）+ 耳机（仿真）→ 扫码 API 直连（蛋糕 262 / 耳机 232，毛巾忽略）
→ 星火分类（蛋糕→食品、耳机→电子产品，source=spark）
→ 第一轮巡航：点 12/22 守卫跳过 → 点 13 命中「食品加工车间」wall 454
   → 停入 PROCESSING_STOP_454（intersection=[-2.5,0.646] stop=-2.25,0.645）
   → 播报「已将蛋糕放入食品加工车间」（TTS_WAIT_FINISHED）
→ 第二轮巡航：点 23~18 守卫跳过 → 点 19 命中「电子产品生产车间」wall 455
   → 停入 PROCESSING_STOP_455 → SIMULATION_START → POST /start accepted（耳机/电子产品）
   → SIMULATION_WAIT_DONE（约 60s，仿真 task3 完成 耳机 delivered to 电子产品生产车间）
   → SIMULATION_DONE → 播报「仿真任务已完成，已将耳机放入电子产品生产车间」
→ DESTINATION_35_36 → SUCCEEDED
```

- 目标守卫全部跳过为实车真实障碍（用户确认），非误报；点 19 为电子产品类别点。
- 期间修复：navigation_scan_relay `global_transform_max_age` 0.20→**0.30**
  （实车 TF 瞬时抖动 0.221s 导致守卫扫描不可用中止；与 laser_map_pose 的
  tf_lookup_retry 同族处理）；全局 inflation_radius 0.205→**0.215**（用户指定）。

## 追加：终点后自动交接 lane_proto 巡线（2026-08-10 晚）

用户需求：到达终点（35/36）后接入 lane_proto 巡线（黄线对齐 → 等绿灯认箭头 →
进三岔口 → 巡线 → 终点横线 STOPPED）。

- **硬约束**：lane_proto.launch 自带 ucar_controller_simple 底盘驱动 + V4L2
  直连相机，与 2026.launch **不能同时跑**（同串口、相机被抢）。交接必须严格
  串行：2026 SUCCEEDED → 任务节点退出（required=true 触发 2026.launch 整体
  关闭，释放串口/相机）→ 交接脚本检测退出+串口可用 → 前台启动 lane_proto。
- 新增 `scripts/handoff_lane.sh`：等待 2026.launch 退出（30s 轮询）→ 等待
  /dev/ttyUSB0 可打开（10s）→ 动态推导 ROS_MASTER_URI/ROS_IP → 前台运行
  lane_proto（固定参数：dry_run:=false linear_speed:=0.2 gain:=1.0
  template:=red_template_band.png is_fork:=yolo yellow_target:=0.90
  align_offset:=0.15 start_offset:=0.25）。
- `production_task_2026.py`：新增 `lane_handoff_enabled`（默认 true）、
  `lane_handoff_script`、`lane_handoff_delay` 参数；SUCCEEDED 后
  publish_result → sleep(delay) → `setsid Popen(handoff_lane.sh MASTER_IP)`
  → `rospy.signal_shutdown("lane handoff")` 退出任务节点；Popen 失败仅
  logwarn 不 abort（人工可手动跑脚本）。`master_uri_host()` 抽自
  resolve_simulation_host 复用。
- `launch/2026.launch` / `CMakeLists.txt`：新增 3 参数与 handoff_lane.sh 安装。
- **测试契约教训**：run_mission 成功路径的 `rospy.signal_shutdown` 在 nosetests
  共享进程里真实执行会污染全局 shutdown 状态，导致按字母序其后执行的
  RecenteringPolicyTest 6 个用例失败（camera_stop 参数/2 帧超时/cancel ack/
  fresh_qr None/MoveBase.cancel_goal 缺方法）。修复：cruises 流程测试用
  try/finally 临时替换 signal_shutdown 为 no-op 并断言其被调用。车端最终
  **86 tests, 0 errors, 0 failures**。
