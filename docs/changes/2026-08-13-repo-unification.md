# 2026-08-13 统一仓库整合

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

## 跟进：技能文档融合（同日）

统一仓库后，根 `.agents/skills/` 的四个技能引用的文档分散在 ucar 与仿真两侧，按用户指示融合到根：

- **犯错档案融合**：`ucar_source_code/犯错档案/`（11 个日期文件 + 索引）与仿真侧 `simulation/犯错误案.md`（07-26/07-27/08-10/08-11）全部并入 `docs/ai-records/mistakes/`，格式统一为四段式（现象/原因/修复/预防），共 12 个日期文件、168 条记录；索引重建为"14 主题 + 按日期"结构。
- **ai-records 融合**：ucar 的 `CHANGE_LOG.md`（7 条）、`FAILED_APPROACHES.md`（1 条）并入根同名文件；根原有 pgrep 条目保留。
- **lingo 融合**：`ucar_source_code/docs/lingo.md` → 根 `docs/lingo.md`，权威来源路径改为 `ucar_source_code/docs/...`；删除冗余技能 `ucar_source_code/.opencode/skills/ucar-lingo/`（词条已并入，与根 `project-lingo` 重复）。
- **规则更新**：`ucar_source_code/AGENTS.md` 删除犯错档案引用（由技能接管，避免冲突），WSLg 预检引用改指向 `docs/ai-records/mistakes/2026-08-10.md`；`simulation/AGENTS.md` 第 6 条改为指向技能维护的 `docs/ai-records/`。
- **技能修正**：`project-memory-records/SKILL.md` 引用从 `MISTAKE_LOG.md` 更正为实际结构 `MISTAKE_INDEX.md + mistakes/YYYY-MM-DD.md`。
- **提交**：`0c0563d` 已推送 main；`ucar_source_code/docs/changes/` 历史文档保留原文（含旧引用，作为历史记录不改写）。
