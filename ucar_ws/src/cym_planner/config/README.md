# CymPlanner 真机配置

真机只加载：

```text
config/ucar_cym_planner_params.yaml
```

`cym_move_base_omni_2026.launch` 在 `move_base` 节点内加载该文件。YAML 根键必须保持为：

```yaml
cym_planner/CymPlanner:
```

当前控制逻辑采用仿真仓库的 `main_legacy` 路径跟踪律，并默认启用
`navigation_mode: laser_avoidance`。规划器直接订阅 `/scan_filtered`，将每帧激光转换到
`base_link`，再沿全局路径前视段投影真实车体 footprint；任一激光点触碰投影车体时
立即返回失败，让 `move_base` 停车并调用全局规划器重规划。

主要参数：

- `scan_topic`、`scan_timeout`、`scan_min_range`、`scan_max_range`：直接激光输入与时效。
- `navigation_mode`：真机默认 `laser_avoidance`，也可通过 `/ucar/navigation_mode` 切换。
- `main_legacy_target_distance`、`main_legacy_*_gain`：仿真同源的路径跟踪控制律。
- `main_legacy_max_vel_x`、`main_legacy_max_vel_theta`：真机命令速度上限。
- `main_legacy_obstacle_lookahead_distance`、`laser_projection_step`：激光车体投影范围与步长。
- `safety_margin`：在真实 footprint 外增加的硬安全边界。

修改 YAML 后无需重新编译，但必须重启 `roslaunch ucar_2026 2026.launch`。旧的
`roslaunch yolo2025 2026.launch` 仅作为兼容 wrapper。修改
`src/cym_planner.cpp`、`include/cym_planner.h`、`CMakeLists.txt` 或 `package.xml`
后必须重新构建 `cym_planner`。
