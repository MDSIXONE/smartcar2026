# 修复常驻交接漏传起跑序列导致巡线被跳过

## 现象

2026-08-15 现场：主流程在 441 交接给常驻 lane_proto 后，巡线在黄线前立即结束。
日志显示激活后 2.8 秒即进入 `== APPROACH == 打点完成, 继续巡线 0.50m 后停`，
前进 0.5m 后 `== STOPPED ==`、播报"任务完成"，主流程报
`PRODUCTION_LANE_HANDOFF_COMPLETED`——整段巡线（黄线对齐 → 认灯 → 进三岔口 → 巡线）
被完全跳过。

## 原因

`2026.launch` 的 lane_proto 常驻 include 缺少 `is_fork:=yolo` 及整套起跑序列参数：

- 交接点 441 面向 170，画面中有一条横贯的黄线（launch 注释
  "so the lane-follow handoff sees the yellow line"）。
- `is_fork` 缺省为 `false` → 交接后 `phase=FOLLOW` 直接进行终点横线检测，
  起跑序列（ALIGN 黄线对齐 → 盲走 → YOLO 认灯 → 拐弯）整个不走。
- 黄线在分割 mask 中被判为线类，`goal_block` 立即命中终点框；`goal_pause=0`
  且 `goal_confirm=1` 时第一帧命中即进入 APPROACH，前进 0.5m 后 STOPPED。

旧流程（`handoff_lane.sh`，2026-08-12 最终值）固定传
`is_fork:=yolo yellow_target:=0.90 align_offset:=0.14 start_offset:=0.23
goal_y_lo:=0.85 rate:=20 linear_speed:=0.35 gain:=1.2 template:=red_template_band2.png`，
8-14 改为常驻交接时参数平移遗漏。与 TrackSeg/CUDA 延迟加载本身无直接因果，
但两个改动同批部署，现场首次交接才暴露。

## 修改

- `ucar_ws/src/ucar_2026/launch/2026.launch`：lane_proto 常驻 include 补齐
  `is_fork:=yolo`、`template:=red_template_band2.png`、`yellow_target:=0.90`、
  `align_offset:=0.14`、`start_offset:=0.23`、`goal_y_lo:=0.85`、
  `linear_speed:=0.35`、`gain:=1.2`、`rate:=20`、`dump_every:=3`；
  `goal_pause` 由 0 恢复为 1.0（检出后先刹停 1s 打点，与旧流程一致）。
- `ucar_ws/src/lane_proto/test/test_lane_runtime.py`：新增回归
  `test_resident_handoff_keeps_start_sequence_parameters`，锁住 2026.launch 的
  交接 include 必须传 `is_fork=yolo` 及起跑/性能参数，防止再次遗漏。

## 验证

- 本机：`2026.launch` XML 解析通过，include 参数完整；lane_proto 8 项
  unittest 0 errors / 0 failures（3 项因本机无 rospy 跳过）；`git diff --check`
  通过。
- 车端：两文件已 scp 部署，SHA-256 与本地一致；`python2 -m py_compile` 通过；
  lane_proto 定向回归 8 项 0 errors / 0 failures / 0 skipped；catkin 回归
  lane_proto 8 项 + ucar_2026 96 项均 0 errors / 0 failures / 0 skipped。
- 待现场：交接后应看到 `ALIGN`/`START_MOVE`/`YOLO` 起跑序列相位，而非直接
  APPROACH。

## 生效方式

下一次 `start_2026.sh <电脑LAN_IP> mission` 启动即生效；常驻 lane_proto 在
STANDBY 期间不触发起跑序列，交接后 `phase=FOLLOW` 才进入 ALIGN。

## 已知限制

- 起跑序列在交接时首次执行，YOLO 认灯与 TrackSeg 共享 Nano GPU，交接段比
  仅巡线耗时更长，属旧流程已验证行为。
- `handoff_lane.sh` 仍保留为旧实现记录，不在主流程调用，参数若再调整需
  同步本 include。
