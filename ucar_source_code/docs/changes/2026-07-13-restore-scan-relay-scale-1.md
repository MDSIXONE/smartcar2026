# 恢复激光中继并将尺度设为 1.0

日期：2026-07-13

## 改动

- 恢复 `2026.launch` 中 `/scan` 到 `/scan_raw` 的重映射。
- 恢复 `2026.py` 对 `/scan_raw` 的订阅与向 `/scan` 的转发。
- 将 `scan_scale` 设为 `1.0`：保留中继拓扑，但 `ranges`、`range_min` 与 `range_max` 不改变距离数值。

## 原因

- 直接由驱动发布 `/scan` 的方案在实车测试中出现问题。
- 以 `scan_scale=1.0` 恢复原有话题拓扑，同时避免旧版 `0.95` 带来的 5% 距离缩放。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/2026.py`
- `ucar_ws/src/yolo2025/launch/2026.launch`

## 操作

- 修改部署后重启 `roslaunch yolo2025 2026.launch`。
- 重启后使用 `rostopic list | grep -E '^/scan$|^/scan_raw$'` 检查：应同时存在 `/scan` 与 `/scan_raw`。

## 验证

- 本地已通过 `2026.py` 的 Python 语法检查与 `2026.launch` 的 XML 解析，确认 `scan_scale: 1.0`、`/scan_raw` 订阅和 `/scan` 转发均已恢复。
- 已同步到小车端并通过 Python 语法检查；已读取确认 `scan_scale = 1.0`、`/scan_raw` 订阅与 `/scan → /scan_raw` remap 均存在。
- 变更前文件备份：`/home/ucar/ucar_ws/.scan_relay_backup_before_restore_scale_1_20260713/yolo2025/`。
- 后续实际启动发现 Melodic 不支持在 `<include>` 内直接使用 `<remap>`；该 launch 语义问题由 `2026-07-13-fix-scan-remap-group.md` 修复。
