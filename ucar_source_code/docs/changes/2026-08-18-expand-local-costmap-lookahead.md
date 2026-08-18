# 扩大局部代价地图与前视膨胀范围（2026-08-18）

## 目的

解决 local costmap 窗口和局部膨胀带过小，导致 CymPlanner 前视点在接近墙体后才获得代价、来不及触发重规划的问题。

## 改动

- `ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml`
  - local costmap `width/height: 1.0 → 1.8 m`。
- `ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml`
  - local inflation `inflation_radius: 0.07 → 0.22 m`。
  - `cost_scaling_factor: 4.0` 保持不变。
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - 三个模式 `obstacle_lookahead_distance: 0.25 → 0.8 m`。
  - 与上一项改动配合，点/冲刺模式仍按任意非零 local raw cost 触发事件式重规划。

## 验证

- 三份 YAML 解析通过。
- `1.8 m` local window 在 `0.03 m` 分辨率下为 `60×60` 栅格，足以覆盖 `0.8 m` 前视距离。
- 未在本机编译、启动 ROS 或发送运动命令。

## 已知限制

- YAML 修改需重启 `move_base`/2026 主流程后才会加载；车端正在运行任务，本轮只同步文件，不强制重启。
- local inflation 扩大到 `0.22 m` 后，窄通道可能更早触发重规划；若全局地图没有零代价可行通道，系统会停住并暴露无路可走，而不是继续贴墙通过。
