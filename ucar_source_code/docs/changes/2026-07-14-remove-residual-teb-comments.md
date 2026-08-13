# 2026-07-14：清理残留 TEB 注释

## 目的

删除工作区源码与配置中所有整行、仅作说明且包含 `teb` 的历史注释，避免当前 CymPlanner 方案被旧 TEB 文字误导。

## 涉及文件

- `ucar_ws/src/darren_launch/launch/` 下 17 个历史 launch 文件，以及 `task_control2.py`、`task_control3.py` 中已注释的 TEB 文本。
- `ucar_ws/src/yolo2025/launch/` 下 5 个未启用历史 launch 文件。
- `ucar_ws/src/ucar_nav/config/` 下 25 个历史 `costmap_converter_params.yaml` 示例和一个旧 `move_base_params.yaml` 说明。

## 验证

- 按“行首为 `#`、`//` 或 `<!--` 且含 `teb`（不区分大小写）”的规则，机械删除 51 个文件中的 85 行注释。
- 全量扫描结果为 0 条残留 TEB 注释。
- 27 个受影响 launch 文件通过 XML 解析，27 个受影响 YAML 文件通过 YAML 解析。
- 清理过程中移除了批量处理意外添加的 UTF-8 BOM；未修改参数键、参数值、实际 TEB 配置键或启动逻辑。

## 已知限制

- 历史 TEB launch、插件名和参数文件仍保留，便于旧方案参考；本次仅删除注释。
- 当前 `2026.launch` 使用 CymPlanner，且未包含历史 TEB launch。
