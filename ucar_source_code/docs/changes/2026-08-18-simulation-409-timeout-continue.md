# 仿真 `/start` 409 进入 120 秒兜底

## 目的

修复 HTTP bridge 返回 `409 already running` 时生产任务直接进入 `ABORTED` 的问题。409 现在按仿真状态未确认处理，进入已有的 `/status` 轮询；仿真在 120 秒内没有报告完成时，车辆继续终点流程。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026_extra/test/test_production_task_geometry.py`
- `docs/operations.md`

## 修改

- `/start` 返回 HTTP 409 时记录 `PRODUCTION_SIMULATION_START_409_CONTINUE`，返回 `False`，不再抛出 `MissionAbort`。
- `run_mission()` 保持无条件进入 `simulation_wait_done()`；因此 409、断连、仿真 failed 或持续 running 都遵循同一 120 秒继续语义。
- 回归测试改为断言 409 返回 `False`，并确认主流程在仿真启动返回失败状态后仍进入等待和继续路径。

## 验证

- 三套主流程源码和两套测试文件通过本机 Python 3 AST 语法解析。
- `ucar_2026` 本机发现式回归：86 tests，71 skipped（缺 ROS/Python2），其余通过。
- `ucar_2026_extra` 本机发现式回归：101 tests，86 skipped（缺 ROS/Python2），其余通过。
- 相关文件 `git diff --check` 通过。

## 车端部署

- 动态确认 `ucar-mini`（192.168.8.231）后，五个源码/测试文件已同步，五份 SHA-256 与本地一致。
- 车端 `/home/ucar/ucar_ws` 使用 ROS Melodic Python2 完成白名单 Catkin 构建，exit 0。
- 车端 `ucar_2026` 定向回归 86 tests，全部通过。
- 车端额外任务新增 409 回归用例单独通过；额外任务全量 101 tests 有 1 个既有测试桩错误：`observe()` 不接受 `stop_mode` 参数，与本次 409 改动无关。
- 未启动 ROS Master、生产任务或车辆运动。

## 已知限制

409 只表示 bridge 已占用当前单次运行槽位，不代表本次请求对应的物品已完成；若 bridge 是上一轮残留，车辆按 120 秒后继续，仿真结果仍视为未确认。下一轮任务仍需停止并重启 bridge，使其回到 `state=waiting`。本次尚未启动真实任务或运动验证。
