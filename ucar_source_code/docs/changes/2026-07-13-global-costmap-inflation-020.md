# 全局代价地图 0.20 m 膨胀调整

日期：2026-07-13

## 改动

- 将 `global_costmap.inflation_radius` 从 `0.23 m` 调为 `0.20 m`，进一步适配约 `0.50 m` 宽的通道。
- 保持局部代价地图 `inflation_radius: 0.05 m` 不变。
- 当时保持 `2026.py` 的激光距离缩放系数 `0.95` 不变；后续移除记录见 `2026-07-13-remove-scan-scaling.md`。

## 涉及文件

- `ucar_ws/src/ucar_nav/config/omni_test20250620/global_costmap_params.yaml`

## 操作

- 修改部署后重启 `roslaunch yolo2025 2026.launch`，使 move_base 重新加载参数。

## 验证

- 已在小车端读取配置文件，确认 `inflation_radius: 0.20`。
- 变更前配置备份：`/home/ucar/ucar_ws/.global_costmap_backup_before_inflation_020_20260713/`。
