# 扫码固定观察面序列修正

## 目的

保持二维码固定观察顺序为 180°（点 262）→90°（点 232）→-90°（点 295）。
当某一面收到二维码但解析出的物品不属于输入目标时，记录并继续下一固定面，不在该面
启动原地转圈。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`

## 行为与验证

- `accepted_qr_after` 与 `wait_for_fresh_qr` 向扫描状态机明确返回“收到但被拒绝”的状态。
- `scan_observation_point` 仅在该面完全没有二维码事件时进入 `QR_SEARCH_TURN`。
- 新增回归用例：非目标二维码不调用转圈兜底，而是返回给采集循环以推进下一固定观察面。
- 本机 `python ucar_ws/src/ucar_2026/test/test_production_task_geometry.py` 通过：
  `77 tests, 0 failures`；其中 62 个 ROS 任务类用例（含新增回归）按既有约定跳过。
- 2026-08-11 已同步到小车，并在 Ubuntu 18.04 / ROS Melodic 运行
  `catkin_make run_tests_ucar_2026`：共 `90 tests, 0 errors, 0 failures, 0 skipped`。

## 已知限制

- 本次仅修正“非目标二维码误触发当前面转圈”；现有“无二维码时一整圈兜底”策略保持不变。
