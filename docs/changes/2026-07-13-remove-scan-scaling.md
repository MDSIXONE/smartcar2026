# 移除激光距离缩放中继

日期：2026-07-13

## 改动

- 移除过 `2026.py` 对 `/scan_raw` 的订阅、`0.95` 距离缩放与 `/scan` 二次发布；后续因实车验证问题恢复中继，记录见 `2026-07-13-restore-scan-relay-scale-1.md`。
- 移除过 `2026.launch` 中将底盘驱动 `/scan` 重映射为 `/scan_raw` 的规则；后续已恢复。

## 原因

- 实车验证中 `/scan_raw` 与静态地图贴合更好；保留缩放会引入约 5% 的几何误差。
- 当时为移除 Python 消息转换与第二次发布，降低消息处理开销；该方案在实车验证中出现问题，已撤回。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/2026.py`
- `ucar_ws/src/yolo2025/launch/2026.launch`

## 操作

- 修改部署后重启 `roslaunch yolo2025 2026.launch`。
- 重启后使用 `rostopic list | grep -E '^/scan$|^/scan_raw$'` 检查：应存在 `/scan`，不应存在 `/scan_raw`。

## 验证

- 本地已通过 `2026.py` 的 Python 语法检查与 `2026.launch` 的 XML 解析。
- 小车端已通过 `2026.py` 的 Python 语法检查，且 `2026.py` 与 `2026.launch` 中均不再包含 `scan_raw`、`scan_scale`、`LaserScan` 或 `scan_cb`。
- 变更前配置备份：`/home/ucar/ucar_ws/.scan_scaling_backup_before_removal_20260713/yolo2025/`。
