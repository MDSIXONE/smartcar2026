# 2026-08-20 CymPlanner 点模式前视距离收紧

## 目的

减少普通 `point` 模式到目标点时，因前视范围过长而提前采样到远处膨胀区并触发局部路径重规划的情况。

## 改动

- `cym_planner/config/ucar_cym_planner_params.yaml`：仅将 `mode1_point.obstacle_lookahead_distance` 从 `0.8m` 调整为 `0.35m`。
- `mode2_body_projection` 和 `mode3_sprint` 保持 `0.8m`；`obstacle_cost_threshold` 保持原值 `1`。
- 更新实车操作文档，明确该参数只影响点模式。

## 验证与上线

- 已完成 YAML 参数静态核对和 `git diff --check`。
- 参数文件已同步到 `ucar-mini`，本地/车端 SHA-256 一致；车端当时已有 `move_base`/`2026.launch`，未强制重启。下一次安全重启 `move_base`/2026 主流程后生效。
- 尚需在实车目标守卫备用点导航中验证局部阻挡重规划是否减少，同时确认真实障碍仍能触发停止。
