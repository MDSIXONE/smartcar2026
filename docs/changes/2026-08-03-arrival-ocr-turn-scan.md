# 到点 OCR 转圈扫描路线

## 目的

按用户确认的流程执行生产目标 `12 → 23 → 14 → 25 → 16`：车辆必须先到达每一个目标，
然后原地最多旋转一整圈寻找 OCR。发现候选后居中并测距；整圈未发现则直接导航至下一个点。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `docs/operations.md`

## 实现

- 生产段改为纯导航到点；移动中不发起 OCR，不因 OCR 取消或跳过当前导航目标。
- 到点后才开启 ROS 相机并暖机。异步 OCR 使转圈期间仍持续发布受控角速度；候选或整圈结束后
  都关闭相机，避免两个目标之间的无用采集。
- 候选出现时先零速并通过新鲜里程计停车门。任务回到异步请求保存的候选帧 yaw，再通过同一
  受保护转向/停车门复拍、像素居中、前方雷达测距和墙点匹配。
- 转满 360° 时会等待最后一个在圈内请求的 OCR 完成再决定是否无候选，防止遗漏圈末画面。
  每个目标的结果写入 `target_scan_events`；有效识别仍写入 `observations`。

## 验证

- 本机标准库 `unittest` 共 `28` 项，无失败；`17` 项因本机没有 ROS Python 模块按设计跳过。
  Python 语法、launch XML 与 `git diff --check` 均通过。
- 已在车端 Ubuntu 18.04 / Melodic 构建 `ucar_2026`；`catkin_test_results` 汇总
  `35 tests, 0 errors, 0 failures, 0 skipped`。整个阶段未启动 ROS 或实车。

## 已知限制

- 异步任务保存的是 OCR 请求时的 map yaw，近似相机实际曝光时刻。若实车 OCR 推理延迟显著，
  后续可将相机帧回执与 TF 时间戳精确关联；当前版本已在复拍前回到该近似 yaw。
