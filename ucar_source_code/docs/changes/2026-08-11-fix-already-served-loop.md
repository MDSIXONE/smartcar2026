# 2026-08-11：修复第二轮巡航 ALREADY_SERVED 死循环

## 目的

2026-08-11 实车运行两次出现同一死循环：第二轮巡航（找仿真类别）回到点 13
墙点 448 时，`PRODUCTION_CATEGORY_ALREADY_SERVED` 永远命中，任务不停车、
无限转圈对准「电子产品生产车间」。

根因：第一轮巡航（找现实类别）在点 13 转圈时先识别到仿真类别
（电子产品,448），代码把它**无条件加入 `served_wall_points`** 但不停车
（设计意图：预记录仿真类别、继续转圈找现实类别）。第二轮回来需要停入该
墙点时，`served_key in served_wall_points` 恒命中 → 永不 RECORDED → 死循环。
18:12 那次任务同样卡在此，只是被 odom stale 中止掩盖。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`：
  `handle_candidate` 的 `ALREADY_SERVED` 分支新增：当 `category ==
  target_category`（第二轮回来找它停入）时，覆盖第一轮预记录——记录
  observation（若该墙点类别尚未在 observations 中）、设置
  `_ocr_turn_stop_flag = True` 停车；仅当类别不是当前目标时才走
  `ALREADY_SERVED` 跳过。日志新增 `(second-pass stop overriding pre-record)` 标记。
- 已同步小车 `~/ucar_ws/src/ucar_2026/scripts/production_task_2026.py`。

## 验证结果

- 本地 Python 语法检查通过；小车 `python2 -m py_compile` 通过。
- 小车单元测试 `test_production_task_geometry.py`：82 tests, OK（67 skipped）。
- 实车复跑验证进行中。

## 已知限制

- 若同一墙点第一轮已真正停入过（现实类别），第二轮 target 不同类别不受影响；
  同一 (category, wall_point) 仍只停入一次。
- 该修复只解除预记录阻塞，不改变第一轮「预记录仿真类别后继续转圈」的既有行为。
