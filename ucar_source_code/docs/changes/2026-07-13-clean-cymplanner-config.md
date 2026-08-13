# 清理 CymPlanner 未使用配置

日期：2026-07-13

## 改动

- 删除未被当前 launch 或代码读取的 `config/cym_planner_params.json`。
- 删除引用旧 JSON 的过期配置说明，并改为当前 YAML 的简明说明。
- 保留实际运行配置 `config/ucar_cym_planner_params.yaml`，其参数根键保持为 `cym_planner/CymPlanner`。

## 原因

- 当前真机 launch 只加载 YAML；旧 JSON 的参数与真机参数不一致（例如最大速度为 `14.0`），容易误导调参。

## 验证

- 已在小车端确认配置目录仅包含 `README.md` 与 `ucar_cym_planner_params.yaml`，launch 仍只引用 YAML。
- 删除前备份：`/home/ucar/ucar_ws/.cymplanner_config_cleanup_backup_20260713/cym_planner/config/`。
