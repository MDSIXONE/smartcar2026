# 膨胀区非零代价触发重规划（2026-08-18）

## 目的

点/冲刺模式的路径进入代价地图膨胀区时，不再只在 raw cost 达到 `253` 后才判定阻塞；任意非零 raw cost 都触发 CymPlanner 返回失败，由 `move_base` 按当前事件式机制重新生成全局路径。

## 改动

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - `mode1_point.obstacle_cost_threshold: 253 → 1`
  - `mode2_body_projection.obstacle_cost_threshold: 253 → 1`
  - `mode3_sprint.obstacle_cost_threshold: 253 → 1`
- 保留 `planner_frequency: 0.0`：不恢复周期性重规划，只在局部前视判定失败时触发事件式重规划。
- `mode2_body_projection` 的实际判定仍经过完整 footprint 逻辑；本次没有改变其弹性带策略。

## 验证

- Windows 本机 YAML 解析通过，三个模式阈值均为 `1`。
- 未在本机编译或启动 ROS；未发送运动命令。

## 已知限制

- CymPlanner 当前检查的是 local costmap；后续 `2026-08-18-expand-local-costmap-lookahead.md` 已将 local inflation 同步扩大到 `0.22 m`，但仍需重启主流程后才会生效。
- 车端已有运行中的 `move_base` 不会自动重新读取 YAML，必须按安全流程重启导航主流程后生效。
