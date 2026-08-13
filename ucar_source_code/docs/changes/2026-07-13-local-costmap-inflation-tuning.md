# 局部代价地图膨胀调小

日期：2026-07-13

## 目的

降低局部代价地图的障碍物膨胀范围，减少过度保守的可行路径限制，同时保留车体外的最小安全余量。

## 改动

- 将 `local_costmap.inflation_radius` 从 `0.30 m` 调为 `0.15 m`；本次根据实车碰撞恢复日志进一步调为 `0.05 m`（5 cm）。
- 未修改机器人 footprint、致命障碍物判断或全局代价地图参数。

## 涉及文件

- `ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml`

## 验证

- 已同步到车端 `/home/ucar/ucar_ws`。
- 已在车端读取确认：`inflation_radius: 0.05`。
- 本次车端变更前配置备份：`/home/ucar/ucar_ws/.local_costmap_backup_before_inflation_5cm_20260713/`。

## 操作

需要重启 `roslaunch yolo2025 2026.launch` 后参数才会被 move_base 重新加载。
