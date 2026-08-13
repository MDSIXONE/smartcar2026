# 生产路由改 16 点 + 提前 3 类去终点 170 朝向 319

## 目的

按用户要求调整生产巡检路由：扫码后改为依次前往
`12→22→13→23→14→24→15→25→16→26→17→27→18→28→19→29` 共 16 个点，每点都做
360° OCR 旋转扫描并记录新类别；一旦累计找到 3 个不同类别（日用品/食品/电子产品），
不再前往后续点，直接导航到终点点 170（(0.0, -0.5)），车头朝向点 319
（(0.0, -0.75)，bearing≈-90° 正南），任务 SUCCEEDED。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`（默认常量）
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `docs/operations.md`

## 行为

- `DEFAULT_PRODUCTION_ROUTE` 与 `DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG`（geometry.py
  15-20）改为 16 点与 16 个交替航向（-45/45 交替）。
- 新增参数 `destination_point_number`（默认 170）、`destination_heading_point_number`
  （默认 319）；两者加入 `all_required_numbers` 由 `require_points` 校验存在。
- run_mission 生产循环每段 `navigate_target_and_scan` 后检查
  `select_three_processing_observations(self.observations)`，≥3 类即
  `PRODUCTION_CATEGORIES_COMPLETE` 日志并 break。
- 循环结束后：不足 3 类保留原 MissionAbort；≥3 类则先停 OCR/相机、保存摘要，
  再 `publish_state("DESTINATION_170")` → `navigate_coordinates(0.0, -0.5,
  bearing(170→319), require_plan=True)` → 停车 → `SUCCEEDED` →
  `publish_result`（终点文案改为点 170）。
- 每点 OCR 行为不变：记录后继续旋转收集不同类别，同类别丢弃，转完一整圈。

## 验证结果

- 本机 `py_compile` 两个 Python 文件通过；车端 Ubuntu 18.04 Python 2 编译通过
  （VEHICLE_OK）；2026.launch 参数已同步。
- 实车验证待做：预期 task_state 序列
  `PRODUCTION_TARGET_*` → `PRODUCTION_CATEGORIES_COMPLETE early_stop_at_route_point=*`
  → `DESTINATION_170` → `SUCCEEDED`。

## 已知限制

- 16 点路由中每点 360° 扫描耗时约 35s（0.18 rad/s），全部走完约 9 分钟；提前 3 类
  停止可大幅缩短。
- 点 170/319 坐标来自 `production_full_grid_all_numbered.json`（170=(0.0,-0.5)
  type=vertex；319=(0.0,-0.75) type=edge_midpoint），若场地网格更换需同步核对。
- 若 3 类在很晚的点（如 29）才集齐，终点导航为最后一段长距离行驶，耗时以
  move_base 规划为准。
