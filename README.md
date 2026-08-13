# SmartCar 2026

统一仓库：仿真与小车源代码一体管理（main 分支）。

## 目录结构

| 目录 | 内容 | 来源 |
|------|------|------|
| `simulation/` | 仿真环境（Gazebo、car3 导航、bridge 等） | 原 [smartcar2026-simulation](https://github.com/MDSIXONE/smartcar2026-simulation) main 分支 |
| `ucar_source_code/` | 小车 ROS 源代码（ucar_ws、tools、docs 等） | 原 smartcar2026 `simulation_real` 分支 |
| `docs/` | 仓库级文档 | — |
| `.agents/skills/` | 通用技能（project-memory-records、github-commit、project-lingo、project-index） | — |

## 快速开始

- 仿真：见 `simulation/TASK3_RUNBOOK.md`、`simulation/FAQ.md`、`simulation/DEPLOYMENT.md`
- 小车源代码：见 `ucar_source_code/README.md`

## 规则

- 仓库级规则见根 `AGENTS.md`
- 仿真部分规则见 `simulation/AGENTS.md`
- 小车源代码部分规则见 `ucar_source_code/AGENTS.md`
