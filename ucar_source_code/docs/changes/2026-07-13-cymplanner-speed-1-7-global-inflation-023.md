# CymPlanner 速度与全局代价地图通道适配

日期：2026-07-13

## 改动

- 将 `cym_planner/CymPlanner/max_vel_x` 从 `1.3 m/s` 调为 `1.7 m/s`；后续调整记录见 `2026-07-13-cymplanner-speed-3-5.md`。
- 保持 `2026.py` 的速度中继系数 `0.5`，底盘实际最大前向指令约为 `0.85 m/s`。
- 将 `global_costmap.inflation_radius` 从 `0.40 m` 调为 `0.23 m`，适配约 `0.50 m` 宽的通道；后续调整记录见 `2026-07-13-global-costmap-inflation-020.md`。
- 局部代价地图 `inflation_radius` 保持 `0.05 m`。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/ucar_nav/config/omni_test20250620/global_costmap_params.yaml`

## 操作

- 修改部署后重启 `roslaunch yolo2025 2026.launch`，使 move_base 重新加载参数。

## 验证

- 已在小车端读取两个配置文件，确认 `max_vel_x: 1.7` 与 `inflation_radius: 0.23`。
- 变更前配置备份：`/home/ucar/ucar_ws/.cymplanner_speed_backup_before_1_7_20260713/`。
- 变更前配置备份：`/home/ucar/ucar_ws/.global_costmap_backup_before_inflation_023_20260713/`。
