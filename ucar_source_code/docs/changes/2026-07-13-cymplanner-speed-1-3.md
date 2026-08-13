# CymPlanner 最大线速度调整

日期：2026-07-13

## 改动

- 将 `cym_planner/CymPlanner/max_vel_x` 从 `1.0 m/s` 调为 `1.3 m/s`；后续调整记录见 `2026-07-13-cymplanner-speed-1-7-global-inflation-023.md`。
- `yolo2025/scripts/2026.py` 保持速度中继系数 `0.5` 不变；当前实际速度说明见 `2026-07-13-cymplanner-speed-1-7-global-inflation-023.md`。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`

## 操作

- 修改部署后重启 `roslaunch yolo2025 2026.launch`，使 move_base 重新加载插件参数。

## 验证

- 已在小车端读取配置文件，确认 `max_vel_x: 1.3`。
- 变更前配置备份：`/home/ucar/ucar_ws/.cymplanner_speed_backup_before_1_3_20260713/`。
