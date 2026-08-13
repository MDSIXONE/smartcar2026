# 2026-07-14：清理启动文件中的过时 TEB 注释

## 目的

修正 `2026.launch` 中仍描述 `/teb_cmd_vel` 中继的过时注释，避免将历史 TEB 配置误认为当前运行链路。

## 涉及文件

- `ucar_ws/src/yolo2025/launch/2026.launch`

## 验证

- 当前实际运行的 `ucar_nav/config/omni_test20250620/move_base_params.yaml` 将 `base_local_planner` 设置为 `cym_planner/CymPlanner`。
- `cym_move_base_omni_2026.launch` 未配置 `/cmd_vel → /teb_cmd_vel` remap。
- `2026.py` 不创建 `/teb_cmd_vel` 的发布者或订阅者；它仅中继激光 `/scan_raw → /scan`。
- 已同步修正后的 `2026.launch` 到小车端并校验 SHA-256：`aacccdcd02aac30e857ed5f0c72958f572a56c8ae3bd6057bf4b062af2bf8059`；仅修改注释，无需重启。

## 已知限制

- 工作区仍保留历史 TEB launch 和参数文件，供旧方案参考；当前 `2026.launch` 不包含它们。
