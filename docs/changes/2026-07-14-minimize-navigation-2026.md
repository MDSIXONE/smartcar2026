# 2026-07-14：精简 2026 导航节点

## 目的

删除 `2026.py` 中未启用的多路点验证路线和无用任务基础设施，保留当前 CymPlanner 导航链路所需的最小结构。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - 保留 `/scan_raw → /scan`、`move_base` action 客户端和可选默认目标。
  - 删除 `run_navigation_test`、两段固定路线、语音唤醒/播报、RViz 目标观察、`goto_yaw` 和等待结果逻辑。
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - 删除不再存在的 `run_navigation_test` 参数。
- `docs/operations.md`
  - 更新节点职责说明。

## 备份

- 改动前脚本已备份到本机 `back/2026-07-14-clean-2026py-before-deploy/2026.py`。

## 验证

- 本地已通过 Python 语法检查、launch XML 解析、AST 方法列表检查和旧功能关键词扫描。
- 2026-07-14 两次尝试连接小车端 SSH 均超时，故车端尚未备份或修改；恢复 SSH 连接后，先下载活动文件到本机 `back/`，再同步 `2026.py` 与 `2026.launch`。
- 同步后使用 `startup_goal_enabled:=false` 重启并检查 `/move_base → /cmd_vel → /base_driver`。

## 已知限制

- 默认目标仍由 `startup_goal_enabled:=true` 控制；该目标不会等待到达结果，也不会执行多路点测试。
- `2026.py` 不会再打印 RViz 目标或播报导航结果；这些功能不影响 `move_base` 和 CymPlanner 本身。
