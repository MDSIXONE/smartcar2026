# OCR 连续对准、停稳后测距

## 目的

使小车在 OCR 框未居中时连续原地转动，避免每一个小角度修正后的停车；仅当 OCR 误差首次落入
容差后停车确认，再用激光完成测距和墙点匹配。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `docs/operations.md`

## 行为

- 首帧在静止状态获取 OCR 框；未对准时启动异步抓拍，同时以 `rotation_control_rate` 持续发布
  P/D 计算的角速度。
- 后续 OCR 帧仅更新转向速度，不在帧与帧之间发布零速。
- OCR 对准后、OCR 空帧、抓拍超时/异常、尝试耗尽和安全异常都会发零速；对准成功时必须等待
  连续低速里程计，再进入激光测距。
- 低于底盘旋转死区的非零速度仍钳制为 `ocr_alignment_min_speed`。

## 验证

- 新增回归用例：异步 OCR 尚未返回时持续发布转向速度，且 0.05 rad/s 被钳制到 0.12 rad/s。
- 本机 `python ucar_ws/src/ucar_2026/test/test_production_task_geometry.py` 通过：
  `78 tests, 0 failures`；其中 63 个 ROS 任务类用例（含新增回归）按既有约定跳过。
- 2026-08-11 已在小车 Ubuntu 18.04 / ROS Melodic 执行 `catkin_make run_tests_ucar_2026`：
  共 `90 tests, 0 errors, 0 failures, 0 skipped`。

## 已知限制

- 连续对准的相邻 OCR 帧存在推理延迟，速度可能为跟随最新帧而反向；首次实车需观察
  `PRODUCTION_OCR_ALIGNMENT_CONTINUOUS` 的误差与速度是否单调收敛。
