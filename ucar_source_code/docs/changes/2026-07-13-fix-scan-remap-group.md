# 修复激光 remap 的 launch 作用域

日期：2026-07-13

## 问题

- 实际启动时 ROS Melodic 报告：`WARN: unrecognized 'remap' tag in <include> tag`。
- 原写法使 `/scan → /scan_raw` 重映射无效，`2026.py` 会订阅并重新发布同一个 `/scan`，存在消息自循环风险。

## 改动

- 将 `<remap from="scan" to="scan_raw"/>` 从 `<include>` 内移到 `<group>` 内。
- 将 `ucar_bringup.launch` 放入该 `<group>`，使重映射仅作用于底盘/雷达驱动节点。
- 保持 `2026.py` 的 `/scan_raw → /scan` 中继和 `scan_scale: 1.0` 不变。

## 涉及文件

- `ucar_ws/src/yolo2025/launch/2026.launch`

## 操作

- 部署后重启 `roslaunch yolo2025 2026.launch`。
- 启动日志中不应再出现 `unrecognized 'remap' tag in <include> tag`。

## 验证

- 本地已完成 launch XML 解析；小车端实际 roslaunch 验证未再出现 `unrecognized 'remap' tag in <include> tag`。
- 运行时拓扑已确认：`/ydlidar_node → /scan_raw → /navigation_2026 → /scan → /lidar_loc、/move_base`，不存在自循环。
- 变更前本地备份：`back/2026-07-13-scan-remap-before-group-fix.tar.gz`。
