# 2026-08-13 统一仓库整合

## 目的

将仿真与小车源代码合并为单一统一仓库，定位为 https://github.com/MDSIXONE/smartcar2026 的 main 分支。

## 背景

- `simulation/` 仿真部分：源自 [MDSIXONE/smartcar2026-simulation](https://github.com/MDSIXONE/smartcar2026-simulation) 原 main 分支（df34221）。
- `ucar_source_code/` 源代码部分：源自 smartcar2026 仓库原 `simulation_real` 分支（5ea5c32）。
- GitHub 仓库名大小写不敏感，`SmartCar2026` 即 `smartcar2026`，故直接覆盖推送现有仓库 main。

## 涉及文件/操作

- 移除嵌套仓库 `.git`：`ucar_source_code/.git`、`simulation/smartcar2026-simulation/.git`（内容直接纳入统一仓库）。
- `simulation/` 上提：克隆仓库 `smartcar2026-simulation` 的 `src/`、`bridge/`、`tmp/` 及文档（README、DEPLOYMENT、FAQ、TASK3_RUNBOOK、requirements-vision、.catkin_workspace、.gitignore）上提到 `simulation/` 根；bridge 以克隆版本为准（外层旧 bridge 备份于 `simulation/tmp/bridge_outer_backup_20260813/`）。
- `simulation/AGENTS.md`：合并中文规则与克隆英文同步规则（One-way local, WSL, and GitHub synchronization）。
- 根 `AGENTS.md`：更新为统一仓库说明；Playwright 规则从仿真侧提升为全局 Universal Rule。
- 根 `README.md`：新建，说明仓库结构。
- 根 `.gitignore`：合并两来源规则（ROS/catkin 产物、vision 模型例外、日志等）。
- `ucar_source_code/AGENTS.md`：更新仓库来源说明；`.gitignore` 移除失效的 `/simulation/`、`/.agents/` 条目。
- 上级 `D:\WORK\ALLCODE\smartcar2026\.gitignore`：追加 `/simulationforreal/`，使统一仓库与上级旧仓库隔离。
- `.workspace-init.json`：保留。

## 验证结果

- `git init -b main` + 首次提交（2575 文件），`git push -u origin main --force` 成功：`d23d198...61541f2 main -> main (forced update)`。
- GitHub main 顶层内容确认：`.agents`、`.gitignore`、`.workspace-init.json`、`AGENTS.md`、`README.md`、`docs`、`simulation`、`ucar_message`、`ucar_source_code`。
- 上级仓库工作树保持干净（`/simulationforreal/` 已忽略）。

## 已知限制

- 旧 main 分支历史（d23d198 及其祖先）仅保留在本地上级仓库 `D:\WORK\ALLCODE\smartcar2026\.git` 与 GitHub reflog 中，可通过 `git reflog` 恢复。
- `simulation/tmp/` 下保留了克隆未跟踪的诊断日志/脚本（未提交），以及 bridge 外层备份目录。
- 原 `simulation_real` 分支历史未并入统一仓库（用户选择"移除嵌套 .git 直接纳入"），远程分支 `simulation_real` 仍保留于 GitHub。
