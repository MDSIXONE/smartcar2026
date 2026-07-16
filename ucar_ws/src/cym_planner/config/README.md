# CymPlanner 配置

真机运行只使用一个配置文件：

```text
config/ucar_cym_planner_params.yaml
```

2026 导航 launch 通过以下配置加载它：

```xml
<rosparam file="$(find cym_planner)/config/ucar_cym_planner_params.yaml" command="load" />
```

YAML 的根键必须保持为 `cym_planner/CymPlanner`，不要额外添加 `CymPlanner:` 层：

```yaml
cym_planner/CymPlanner:
  base_link_frame: base_link
  odom_frame: odom
  max_vel_x: 7.0
```

常用参数：

- `base_link_frame`：机器人本体坐标系。
- `odom_frame`：里程计坐标系。
- `max_vel_x`：规划器最大线速度；`2026.py` 会将其输出乘以 `5/7` 后发送给底盘。
- `max_vel_theta`：路径跟踪阶段的最大角速度；`2026.py` 会将其输出乘以 `0.5` 后发送给底盘。
- `obstacle_lookahead_distance`：沿全局路径检查局部障碍物的前视距离。

修改 YAML 后无需重新编译，但必须重启 `roslaunch yolo2025 2026.launch`，使 move_base 重新加载参数。
