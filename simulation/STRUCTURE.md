# 仿真目录结构

`simulation/` 是仿真内容的唯一根目录。根目录只保存分支工作区和管理说明，
代码、配置、资源、生成物及临时文件均应留在所属分支的文件夹内。

## 分支工作区

| 文件夹 | Git 分支 | 用途 |
| --- | --- | --- |
| `smartcar2026-simulation/` | `yolo_cym1` | YOLO 与 CYM 集成开发 |
| `smartcar2026-simulation-cym-mode/` | `fix/cym-planner-vehicle-size-lookahead` | CYM 规划器模式 |
| `smartcar2026-simulation-yolo-mode/` | `yolo_cym1-local-mode` | YOLO 本地模式 |

不同工作区之间不得直接混放或复制生成物。需要在分支之间同步源码时，应使用
Git 提交、合并或拣选。

## 单个工作区

- `src/`：ROS/Gazebo 源码、启动配置、模型与地图。
- `datasets/`：该分支使用的数据集。
- `resources/reference/`：已归档但不直接参与运行的输入或对照资源。
- `build/`、`devel/`、`install/`、`logs/`：本工作区的本地生成物，不跨分支复用。
- `README.md`：快速开始。
- `DEPLOYMENT.md`：部署说明。
- `TASK3_RUNBOOK.md`：任务启动与排障说明，属于快速开始/部署类文档。

当前运行文件仍以 `src/` 下的配置为准；`resources/reference/` 中的文件不会自动
覆盖或参与构建。
