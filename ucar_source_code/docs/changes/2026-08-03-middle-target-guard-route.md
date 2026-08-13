# 中间区目标守卫路线（已被到点 OCR 转圈流程替代）

> 该记录描述提交 `0a8d3b1` 的历史实现。后续用户明确要求“到点后转 360° 扫描”，当前
> 生效实现与验证见 `2026-08-03-arrival-ocr-turn-scan.md`；本记录不再代表运行行为。

## 目的

将二维码后的生产路线切换为 `12 → 23 → 14 → 25 → 16`。对每个目标，移动中的 OCR
候选经安全停车、居中和前向雷达投影确认后，如果落在该中心点相邻的四个中间区边线端点，
则放弃当前目标并直接进入下一目标，避免车辆驶向已确认有障碍物的方格。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `docs/operations.md`

## 实现

- 从 JSON 的 `square_side_m`、类型为 `middle` centre 的目标坐标，自动匹配相距半边长的
  四个 `line_endpoint` vertex；12 的回归结果为 `419, 420, 428, 429`。如果未来地图缺失、
  重复或非中间区目标没有恰好四个端点，任务初始化失败关闭。
- 生产路线按目标腿执行：先 `52 → 12`，再依次连接配置的五个目标。没有使用合并直线段，
  以保证每个目标都有独立守卫。每腿的既有目标朝向参数也实际应用；首腿到 12 使用
  `12 → 23` 的 `-45°` 朝向，不会先以 `52 → 12` 的 `-90°` 入位再在近墙处额外转向。
- OCR 候选照旧先安全取消 goal、确认车辆停止、居中、采集新鲜雷达。匹配候选集合同时包含
  正常墙参考点和当前目标的守卫端点，所以守卫不会丢弃正常的三点识别结果。
- 守卫命中会先完成 observation 标记和 `target_guard_events` 审计记录，再原子写入一次结果
  摘要并跳过目标；已得到三个识别点后仍保留守卫扫描能力。最终目标被守卫时不再发送额外运动
  目标。

## 验证

- 本机标准库 `unittest`：共 `29` 项，无失败；`17` 项因本机没有 ROS Python 模块按设计跳过。
- `python -m py_compile` 通过三个任务 Python 模块。
- `2026.launch` XML 解析通过，`git diff --check` 通过。
- 已在车端 Ubuntu 18.04 / Melodic 构建 `ucar_2026`，`catkin_test_results` 汇总为
  `36 tests, 0 errors, 0 failures, 0 skipped`。整个阶段未启动 ROS 或实车。

## 已知限制

- 守卫决定需要至少一个通过阈值的 OCR 候选，之后还需完成安全停车、居中、雷达和 TF 投影；
  它不是仅由 OCR 文本触发的快速猜测。
- 最终目标 16 命中守卫没有下一个路线点，任务会安全结束并在结果文件中注明守卫事件；它不会
  假装车辆已实际到达 16。
