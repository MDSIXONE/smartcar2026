# 全局代价地图膨胀范围调整

日期：2026-07-13

## 改动

- 将 `global_costmap.inflation_radius` 从 `0.30 m` 调为 `0.40 m`；后续针对 50 cm 通道的调整记录见 `2026-07-13-cymplanner-speed-1-7-global-inflation-023.md`。
- 保持局部代价地图 `inflation_radius: 0.05 m` 不变。
- 全局代价地图仍使用静态地图范围（`rolling_window: false`），未设置有限的 `width` 或 `height` 窗口。

## 涉及文件

- `ucar_ws/src/ucar_nav/config/omni_test20250620/global_costmap_params.yaml`

## 操作

- 修改部署后，重启 `roslaunch yolo2025 2026.launch` 使 move_base 重新加载参数。

## 验证

- 已在小车端读取配置文件，确认 `inflation_radius: 0.4`。
- 变更前配置备份：`/home/ucar/ucar_ws/.global_costmap_backup_before_inflation_04_20260713/`。
