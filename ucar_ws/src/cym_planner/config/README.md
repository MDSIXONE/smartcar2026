# CymPlanner 真机配置

真机只加载：

```text
config/ucar_cym_planner_params.yaml
```

`cym_move_base_omni_2026.launch` 在 `move_base` 节点内加载该文件。YAML 根键必须保持为：

```yaml
cym_planner/CymPlanner:
```

当前控制逻辑直接订阅 `/scan_filtered`，将每帧激光转换到 `base_link`，再对候选
`(linear.x, angular.z)` 做真实车体 footprint 扫掠。激光缺失、超时、没有有效点，
或所有候选轨迹都会碰撞时，规划器输出零速度并返回失败，让 `move_base` 停车或重规划。

主要参数：

- `scan_topic`、`scan_timeout`、`scan_min_range`、`scan_max_range`：直接激光输入与时效。
- `safety_margin`、`braking_deceleration`、`reaction_time`：车体外扩与停车距离。
- `simulation_time`、`simulation_step`、`v_samples`、`w_samples`：候选轨迹预测窗口与采样量。
- `path_distance_weight`、`heading_weight`、`clearance_weight`：路径、朝向和净空评分。
- `max_vel_x`、`max_vel_theta`：真机命令速度上限。
- `obstacle_lookahead_distance`、`obstacle_cost_threshold`：辅助代价地图重规划条件。

修改 YAML 后无需重新编译，但必须重启 `roslaunch ucar_2026 2026.launch`。旧的
`roslaunch yolo2025 2026.launch` 仅作为兼容 wrapper。修改
`src/cym_planner.cpp`、`include/cym_planner.h`、`CMakeLists.txt` 或 `package.xml`
后必须重新构建 `cym_planner`。
