# 车端仿真兜底等待缩短至 75 秒

## 目的

将车端等待仿真完成的兜底时限从 120 秒调整为 75 秒。仿真在时限内完成时，仍按原流程处理；超时、断连、`failed` 或 `/start` 失败的继续语义不变，主流程进入车端终点流程。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py`
- 三套 `launch/2026.launch`
- 三套 `test/test_production_task_geometry.py`
- `docs/operations.md`

三套任务脚本的默认 `simulation_done_timeout`、launch 覆盖参数、相关回归基准值和当前运维说明均已统一为 75 秒。历史变更文档中的 120 秒保留为当时的事实记录。

## 验证

- 三套任务脚本 Python 语法/AST 检查通过。
- 三套 launch XML 解析通过。
- 相关仿真超时回归通过。
- `git diff --check` 通过。
- 三套任务脚本和 launch 共 6 个运行文件已同步到 `ucar-mini`，本地与车端 SHA-256 一致；远端参数读回均为 75 秒。

## 已知限制

本次未启动 ROS、bridge 或实车任务；正在运行的任务需重启对应主流程后才会加载 75 秒参数。
