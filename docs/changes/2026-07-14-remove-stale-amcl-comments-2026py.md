# 2026-07-14：清理 2026.py 中的过时 AMCL 注释

## 目的

将 `2026.py` 中两条把当前定位描述为 AMCL 的历史注释改为中性的“定位”，避免与当前 `lidar_loc` 方案混淆。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/2026.py`

## 验证

- 全文检查确认 `2026.py` 不包含 `amcl` 或 `teb` 文本。
- Python 编译检查通过。
- 2026-07-14 尝试同步时小车端 SSH 连接超时，故车端脚本尚未修改；恢复连接后需重新同步。该变更仅修改注释，不需重启。

## 已知限制

- `2026.py` 仍需要 `move_base` action 客户端发送导航目标；CymPlanner 是由 `move_base` 加载的局部规划器插件，而非 Python 直接调用的对象。
- `/scan_raw → /scan` 转发仍是 `lidar_loc` 和代价地图的输入链路，不能因清理 AMCL/TEB 名称而移除。
