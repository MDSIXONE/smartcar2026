# 主流程改版：物品输入 → 单次 QR → 目标类别命中即停 → 加工区停留 → 终点 17

## 目的

按用户要求重构 2026 生产主流程：

1. 任务不再「启动即运行」：任务节点启动后先等待操作员在终端输入本次放入的物品名
   （stdin 回车），收到后才进入安全等待与 QR 阶段。
2. 只扫描一个二维码（`qr_observation_numbers[0]`，即点 262）：扫码得到物品文本后
   不再扫描其余二维码；由星火大模型（本地关键词表兜底）判定该物品的加工类别
   （日用品/食品/电子产品）。
3. 生产巡检不再收集 3 类：只按目标类别巡航识别，**首次命中目标类别即停止旋转并
   提前退出生产循环**，其余区域不再识别。
4. 命中后沿用主流程既有机制（OCR 居中 + 前向激光射线与墙求交，observe_wall）得到
   墙壁交点编号与坐标，停车位 = 墙交点坐标向场内垂直 `square_side_m/2`（25 cm），
   例如交点 300（顶墙 (0.75,1.5)）→ 停点 (0.75,1.25)=点 7；交点 455（右墙
   (2.5,0.75)）→ 停点 (2.25,0.75)=点 20；454→点 11；448→11 与 21 连线中点
   (-2.25,0.5)；307→点 34。
5. 到达加工区停点后停留 `processing_dwell_seconds`（默认 3 s），随后前往终点
   点 17（(0.75,0.75)，车头朝向点 300），任务 SUCCEEDED。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`

## 行为

### geometry 模块（新增 2 个纯几何函数）

- `load_middle_zone_geometry(path)`：读 JSON 的 `square_side_m` 与
  `middle_zone_bounds_m`（实测 `{"x":[-2.5,2.5],"y":[-0.5,1.5]}`），校验有限/正值/
  边界顺序，返回 `(x_min, x_max, y_min, y_max, side)`。
- `stop_point_for_wall_point(wall_coordinate, square_side_m, middle_bounds)`：墙点必须
  落在中区四边（容差 1e-6；水平墙 x 需在界内、垂直墙 y 需在界内），
  `offset = square_side_m/2`；顶墙 `y_max`→`(wx, y_max-offset)`、底墙 `y_min`→
  `(wx, y_min+offset)`、左墙 `x_min`→`(x_min+offset, wy)`、右墙 `x_max`→
  `(x_max-offset, wy)`；不在边界抛 `TaskDefinitionError`。

### 任务节点 production_task_2026.py

- 新增 `import sys`；import `load_middle_zone_geometry`、`stop_point_for_wall_point`。
- 新参数 `~processing_dwell_seconds`（默认 3.0），校验有限且 ≥0。
- 新实例属性：`expected_item_text=u""`、`expected_production_category=None`、
  `_ocr_turn_stop_flag=False`；加载 `middle_zone_bounds` 与 `middle_zone_square_side`。
- `run_mission` 重写：`WAITING_FOR_ITEM` → `wait_for_item_input()`（stdin 阻塞读行，
  UTF-8 解码去空白，空输入 MissionAbort，ASCII 错误消息）→ 既有安全等待/点模式 →
  QR 阶段**只扫 `qr_observation_numbers[0]` 一个点** → 目标类别 =
  `qr_classification_category(observation_key)`（None 或 "null" → MissionAbort）→
  生产循环每段 `navigate_target_and_scan` 后
  `if self.production_category_recorded(target_category): break`（首次命中即停）→
  `last_recorded_observation(target_category)` 取带墙点编号的观察 →
  `stop_point_for_wall_point` 算停点 → `parking_yaw=bearing(停点, 墙点)` →
  `PROCESSING_STOP_%03d` state → `navigate_coordinates` 前往停点 →
  `PROCESSING_DWELL_%03d` state → `rospy.sleep(processing_dwell_seconds)` →
  终点 `DESTINATION_%d` → SUCCEEDED（reason 纯 ASCII）。
- 新方法：`wait_for_item_input()`、`qr_classification_entry(obs)`、
  `qr_classification_category(obs)`、`production_category_recorded(category)`、
  `last_recorded_observation(category)`。
- `scan_production_point` 候选 handler 增加期望类别过滤（非目标类别 →
  `PRODUCTION_CATEGORY_IGNORED` 日志并 return False 继续转圈）；目标类别记录成功后
  置 `_ocr_turn_stop_flag=True`。
- `rotate_full_revolution_for_ocr` 两处 handler 返回后检查 `_ocr_turn_stop_flag`：
  转圈中途命中 → 停车并提前返回；整圈末尾命中 → 提前返回（车已停）。
- `publish_result` 增加 `item_text`、`target_category` 字段。
- 删除「集齐 ≥3 类」逻辑与对应 MissionAbort。
- 日志约定：含中文一律 `log_safe_text`/`category.encode("utf-8")`；`publish_state`
  与 MissionAbort 消息纯 ASCII（py2 中文崩溃教训）。

### launch/2026.launch

- `destination_point_number` 170→**17**；`destination_heading_point_number`
  319→**300**；新增 `processing_dwell_seconds` 3.0（0 可禁用停留）。
- 路由注释同步更新。

### start_2026.sh

- mission 模式在 roslaunch 前提示：任务节点会等待物品输入，请在本终端输入物品名并
  回车（示例：苹果、可乐、螺丝刀）。

### 测试 test_production_task_geometry.py

- 新增 `test_middle_zone_geometry_matches_grid_document`、
  `test_stop_point_for_wall_point_is_25cm_inside_the_field`（300/455/454/448/307 五组
  映射全过）、`test_stop_point_for_wall_point_rejects_off_boundary_point`。
- 三个 run_mission 流程测试补 `wait_for_item_input` stub 与事件断言
  `item_input`；QR 测试 stub `scan_observation_point` 写入带类别 entry、
  `stop_qr_classifier`/`post_qr_waypoint_number` 打桩；resume 测试 stub
  `classify_qr_text`。
- 顺带修复 3 个既有失败断言（16 点路由与 16 点守卫映射、headings 长度，模块默认
  早已改 16 点而测试未同步）。

### TTS 语音播报（2026-08-07 追加）

- 新参数：`tts_enabled`（默认 true）、`tts_python`（默认 /usr/bin/python3）、
  `tts_helper_path`（默认 /home/ucar/wake/tts_say.py，车端讯飞 TTS 助手，不在仓库）。
- 新方法 `speak(text)`：`subprocess.Popen([tts_python, tts_helper_path, text])`
  异步 fire-and-forget（py2 无 subprocess.DEVNULL，用 `open(os.devnull,"wb")`
  重定向 stdout/stderr 后关闭句柄）；失败仅 `PRODUCTION_TASK_TTS_FAILED`
  logwarn，绝不中止任务。
- 播报点：① 任务进入 `WAITING_FOR_ITEM` 后、等待物品输入**之前**播放「初始化完成，
  准备开始任务」；② 扫码得到物品文本且大模型分类返回后播放
  「取得*<物品名>*属于*<类别>」（`*` 分段，缓存命中可断网播放；物品名取自终端输入
  `expected_item_text`，类别取大模型分类结果；2026-08-07 修正：不再播 QR 网址）。
- 输入仍为物品名（用户撤回「输入类别」需求，类别由大模型推断）。

## 验证结果

- 本机 `python -m unittest test_production_task_geometry`：48 tests OK
  （skipped=33，ROS 类本机按设计跳过；15 个纯 geometry 全过，含 3 个新增）。
- `ast` 语法检查 production_task_2026.py / production_task_geometry.py /
  production_qr_classifier.py 通过。
- 已部署到小车（ssh ucar@192.168.8.231，5 文件 SHA256 与本地逐字节一致，shebang
  `#!/usr/bin/env python2` + LF 校验通过）；车上 catkin 构建 + 单测
  `catkin_test_results --verbose build/test_results/ucar_2026`：**60 tests, 0 errors,
  0 failures, 0 skipped**（车上无 skipIf 跳过，全部真实运行）。
- 车端首轮暴露并修复 5 个测试缺口：3× `test_alignment_timeout_settle_*`
  （setUp 缺 `ocr_alignment_min_speed`）、`test_qr_seen_while_facing_...`
  （setUp 缺 `spark_classify_enabled`）、`test_arrival_scan_restores_capture_yaw_...`
  （5cdb222 移除观察后第二次 restore 后陈旧断言 `calls[3]=="restore"` 未同步，
  改 `"save"`）。详见犯错档案 2026-08-07 条目。
- 车上 CMakeCache 白名单现值 `ucar_controller;ucar_2026`（含历史残留
  ucar_controller），operations.md 构建命令已同步带白名单覆盖。
- 实车联调（2026-08-07 晚，完成）：
  - 第一次运行 run_20260807_182029：QR 分类成功（蛋糕→食品，spark-x），生产循环
    首点 12 的 `laser_map_pose` 遇 TF 外推 9 ms 抖动（scan stamp 超前 TF 时钟）
    MissionAbort。修复：`laser_map_pose` 对 `tf.ExtrapolationException` 增加
    `tf_lookup_retry_seconds=0.5` 重试窗（0.01 s 间隔），超窗仍致命，其余 TF 错误
    保持致命；launch 同步参数。
  - 第二次运行 run_20260807_183301：**SUCCEEDED**。完整状态序列：
    `WAITING_FOR_ITEM`（初始化播报已触发）→ 输入蛋糕 → `STAGING_52` → `QR_SEQUENCE`
    → `QR_FACE_262`（单点扫描即中）→ `WAYPOINT_3` → `TARGET_CATEGORY 食品` →
    点 12 守卫跳过（TARGET_GUARD_SKIP_012_419）→ 点 22 转圈 1.38 rad 命中
    「食品加工车间」→ `PRODUCTION_TARGET_CATEGORY_FOUND` → `PROCESSING_STOP_454`
    （wall=(-2.5,0.75) → stop=(-2.25,0.75)=点 11）→ `PROCESSING_DWELL_454`
    seconds=3.0 → `DESTINATION_17` → `SUCCEEDED`。TTS 播报无失败
    （TTS_FAILED=0），缓存已生成「初始化完成_准备开始任务」「取得/蛋糕/属于/食品」
    分段；QR 只扫一次、命中即停、停留 3 s、终点 17 全部符合需求。
- 后台清理：任务结束后 `MASTER_IP=192.168.8.198 bash
  ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh`（脚本要求 ROS_MASTER_URI 或
  MASTER_IP），roslaunch 进程归零，无残留终端。

## 追加更新（2026-08-07 深夜，全部实车验证通过）

### 停点正对修正（连续交点停车）

- 用户观察：目标招牌在墙交点 448=(-2.5,0.5) 附近，但车停到了点 11（454 正对处）。
  根因：旧逻辑把连续激光交点取整到最近编号墙点再算停车点；且摄像头居中容差
  （23 px≈3~10°）+ 定位误差使交点漂移。
- 修改（production_task_2026.py run_mission 停车段）：停车点改用
  `observation["forward_ray_wall_intersection_map"]`（连续交点）而非
  `wall_point_coordinate`（最近编号墙点）计算 `stop_point_for_wall_point`；日志增加
  intersection 字段。实车验证：PROCESSING_STOP_456 时
  `intersection=[-2.5,0.357] stop=-2.250,0.357`（不再取整到墙点 456=(-2.5,0.25)）。

### 三处新需求（用户确认）

1. **停车后车头朝外**：`parking_yaw = normalize_angle(bearing(停点, 交点) + math.pi)`。
2. **终点改 35/36 连线中点，车头朝 170**：新参数
   `destination_midpoint_point_numbers`（逗号分隔字符串，默认空；非空必须恰好 2 个
   否则 TaskDefinitionError；加入 all_required_numbers）。终点段：设置则取两坐标中点，
   否则用 `destination_point_number`；state 用 `DESTINATION_35_36` 形式。
   launch 设 `value="35,36"`、`destination_heading_point_number=170`。坐标：
   35=(-0.25,-0.25)、36=(0.25,-0.25)、中点=(0.0,-0.25)、170=(0.0,-0.5)，
   bearing=-90°（正南）。
3. **扫码后到点 3 车头朝 13**：新参数 `post_qr_waypoint_heading_point_number`
   （默认 0=不转向；加入 all_required_numbers），post_qr 段
   `waypoint_yaw=bearing(3, 13)=-90°`。launch 设 13。

### 大模型分类修复（用户询问提示词后确认）

- 原提示词（production_qr_classifier.py:53-56）：
  「用户会给你一个二维码扫描得到的物品文本，请判断该物品属于三个加工厂类别中的
  哪一个…只输出 JSON {"category": "食品"}」；请求体 model=spark-x、
  temperature=0.5、thinking=disabled。
- 根因：`classify_qr_text` 把**二维码扫描文本**（实车是 URL
  "http://192.168.8.1:3663/a"）喂给大模型，只能瞎猜（首次 attempts=3 猜出食品）。
- 修复：① 任务节点扫码后改喂终端输入的**物品名**
  （`self.expected_item_text.encode("utf-8")`，与 resume 分支一致）；
  ② 提示词改为「用户会给你一个**物品名称**」。实车验证：
  `SPARK_CLASSIFY observation=262 qr="蛋糕" category="食品" source=spark`。

### 运动中取消停车确认超时修复

- 现象：守卫在导航途中命中（点 12 有障碍物→跳过是设计行为），取消目标后
  `wait_for_chassis_stop` 2.0 s 超时 ABORTED（车从点 3 出发途中被取消，惯性 +
  move_base 残余速度，2 s 内 odom 未降到 0.02 以下）。
- 修复：`stop_confirmation_timeout` 默认 2.0→**4.0**（production_task_2026.py 默认值
  与 launch 同步，launch 带注释）。实车验证：后续运行点 12 守卫 before_goal 跳过，
  无 abort。

### USB autosuspend 持久化加固（start_2026.sh）

- 现象：车重启后根 hub 1-1 仍 auto+2000 ms，dmesg 反复 usb_suspend_both → CP2102
  串口数据流间歇掐断 → odom/imu gap → move_base cancel 不确认/停车失败。
- 修复：`start_2026.sh` mission 分支在 yes 确认后、exec roslaunch 前自动关闭
  autosuspend（sudo -n sh -c 遍历 /sys/bus/usb/devices/*/power/，
  autosuspend_delay_ms→-1、control→on，失败仅警告不阻塞）；udev 规则
  /etc/udev/rules.d/50-usb-autosuspend-off.rules 对根 hub 不可靠，靠脚本兜底。
  实车验证：运行期间 base_driver 0 个 PUBLISH_GAP。

### 运维规则（用户明确）

- **操作顺序必须：等用户确认放回起点 → 先 stop_2026_task.sh 停止旧任务 → 再启动
  新任务**，不得提前启动。
- 教训：曾因旧进程未清理导致新旧实例并存、日志混乱；曾误把 2026.launch scp 到
  scripts/ 目录导致 `RLException: multiple files named [2026.launch]`——部署时必须
  确认目标路径，任务日志多目录并存时按进程 PID 的 __log 参数确认日志文件。

### 实车最终验证（mission_run11，全部新功能通过）

```
21:36:53 WAITING_FOR_ITEM → 21:37:37 ITEM "蛋糕" → STAGING_52
21:38:10 QR_SEQUENCE → QR_FACE_262（只扫1个QR）→ SPARK_CLASSIFY qr="蛋糕" category="食品"
21:38:14 WAYPOINT_3 → 21:38:21 TARGET_CATEGORY "食品" → OPEN_ROS_IMAGE_OCR
21:38:26 TARGET_012 → TARGET_GUARD_SKIP_012_419（before_goal）→ TARGET_022
21:38:38 OCR_TURN_022 → 21:38:52 命中"食品" → 21:39:02 TARGET_CATEGORY_FOUND route_point=22
21:39:03 PROCESSING_STOP_456（wall=[-2.5,0.25] intersection=[-2.5,0.357] stop=-2.250,0.357）
21:39:10 PROCESSING_DWELL_456 seconds=3.0 → 21:39:13 DESTINATION_35_36 → 21:39:23 SUCCEEDED
```

- 全程无中止、无 TTS 失败；任务结束后 stop_2026_task.sh 清理，roslaunch 归零。

## 已知限制

- 终端输入在 Windows 管道下可能 GBK 乱码，仅影响本机模拟；车端真实终端无此问题。
- `wait_for_item_input` 阻塞 stdin：若以 `roslaunch` 后台/非交互方式启动将永远等待，
  必须保证任务终端可输入（start_2026.sh mission 模式已提示）。
- 停点只支持「墙交点向场内垂直 25cm」的单边贴墙格；若交点位于墙角附近
  （如四角顶点），`stop_point_for_wall_point` 会因不落在某一边而抛
  TaskDefinitionError，需另行约定。
- 目标类别若在整条 16 点路由上始终未识别，任务以 MissionAbort 结束（不再有
  「集齐 3 类」兜底）。
- 终点为 35/36 连线中点 (0.0,-0.25)，车头朝 170（正南）；launch 参数
  `destination_midpoint_point_numbers=35,36`、`destination_heading_point_number=170`。

### 实车运行验证（mission_run12，连续第二次成功）

```
21:43:29 WAITING_START → 21:43:31 WAITING_FOR_ITEM → 21:44:12 ITEM "蛋糕"
→ STAGING_52 → QR_FACE_262（只扫1个QR）→ SPARK_CLASSIFY qr="蛋糕" category="食品"
→ WAYPOINT_3 → TARGET_CATEGORY "食品" → OPEN_ROS_IMAGE_OCR
→ TARGET_012 守卫跳过（before_goal 419）→ TARGET_022
→ OCR_TURN_022 命中「食品加工车间」(conf 72.5, turn 1.389 rad 即停)
→ TARGET_CATEGORY_FOUND route_point=22
→ PROCESSING_STOP_454：wall=[-2.5,0.75] intersection=[-2.5,0.628] stop=(-2.250,0.628)
  yaw=0.000（车头朝外），GOAL_REACHED error=0.056
→ PROCESSING_DWELL_454 seconds=3.0 → DESTINATION_35_36 中点 (0.000,-0.250) yaw=-1.571（朝170）
→ GOAL_REACHED error=0.029 → 21:46:02 SUCCEEDED
```

- task_result：success=true, item_text=蛋糕, target_category=食品, range_residual_m=0.0438,
  wall_match_error_m=0.1217, result_file=run_20260806_214456/observations.json。
- 全程无中止、无 TTS 失败；任务结束后 `stop_2026_task.sh` 清理，roslaunch 归零。
- 停车点连续交点（intersection=[-2.5,0.628] 而非最近墙点 [-2.5,0.75]）与终点 35/36
  中点、车头朝外均实车验证生效。
