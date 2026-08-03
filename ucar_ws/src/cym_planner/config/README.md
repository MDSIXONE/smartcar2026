# CymPlanner 真车参数

`ucar_cym_planner_params.yaml` 由 `move_base` 节点加载，根键固定为：

```yaml
cym_planner/CymPlanner:
```

目标点固定选择车前 `0.20 m` 外的第一个全局路径点，终点位置阈值固定为
`0.05 m`；这两个几何常量有意共享，不是速度调参项。两种碰撞模式使用同一控制
算法，但运行控制参数完全独立：

- `mode1_point/*`：`point`、`main`、`main_legacy`；
- `mode2_body_projection/*`：`body_projection`、`footprint`、
  `laser_avoidance`。

两组都可分别设置直线 P/D、角度 P/D、直线/角速度上限、最终朝向参数、前视距离和
搬运速度比例。修改生产路线速度通常只需调整
`mode2_body_projection/max_vel_x`、`max_vel_theta` 和
`heading_slowdown_min_scale`。

碰撞检查提供两种形状，但不切换控制器：

- 默认 `point`：沿前方路径检查单个路径点的代价值。
- `body_projection`：在同一批前视路径点上把完整车体 footprint 投影到
  move_base 注入规划器的 local `Costmap2D` 周期快照。原始 253 是内切膨胀，
  不代表真实 footprint 已接触；只有 254（致命障碍）和 255（未知区）阻止运动。
  局部 rolling window 同时包含 `/map` 静态墙和 `/scan_filtered` 动态障碍，
  且任一动态源超时导致 local costmap 不再 current 时保持零速。

模式 2 默认把直线速度上限降到 `0.22 m/s`、角速度上限降到 `0.55 rad/s`。
普通路径跟随时还按航向误差的 `cos²` 缩小线速度，最低比例由
`heading_slowdown_min_scale` 控制；模式 1 默认设为 `1.0`，保持原行为。
模式 2 还会按实际候选 Twist 每 `0.025 s` 投影一次未来 `0.40 s` 的真实车体姿态，
当前 footprint、最近路径点前视和候选 Twist 扫掠共用同一份局部快照。车体尺寸仍
直接读取 local costmap 配置。

受限横移是次级兜底，当前被源码硬关闭，不会发布横移速度。即使 YAML 误写
`escape_enabled: true`，规划器也会忽略并记录错误。Sol 安全复核发现启用前还必须
修复完整状态保持、全接触格统计和本地动态障碍预览。下列参数只是保留的设计值：

- 根据致命格相对 `base_link` 的主方向判断前、后、左、右接触侧；
- 保持角速度为零，沿相反方向以 `0.04 m/s` 平移最多 `0.02 m`；
- 每步后原地等待 `0.4 s` 重新规划；
- 最多 4 步且累计不超过 `0.08 m`；
- 每步前沿 2 cm 平移每 5 mm 投影一次未膨胀的真实 footprint；中途接触不得变差，
  终点必须完全无碰撞或比当前位置严格减少致命格接触；
- 未知区、地图越界、姿态漂移和上限耗尽都不会触发或继续平移，而是返回原有控制失败。

该功能不缩小 footprint、不切到点碰撞，也不直接绕过 `move_base`：横移速度仍由
本地规划器作为本次合法速度命令返回。

运行时通过话题切换：

```bash
rostopic pub -1 /ucar/navigation_mode std_msgs/String "data: 'point'"
rostopic pub -1 /ucar/navigation_mode std_msgs/String "data: 'body_projection'"
```

为兼容现有任务，`main_legacy` 等同于 `point`，`laser_avoidance` 等同于
`body_projection`。

`debug_images_enabled` 默认是 `false`，不会创建 OpenCV 调试地图，也不会发布
调试话题。临时排障时可改成 `true`，保留的调试内容包括：

- `/move_base/cym_planner/CymPlanner/debug_map`
- `/move_base/cym_planner/CymPlanner/debug_plan`

修改 YAML 后重启 `roslaunch ucar_2026 2026.launch` 即可；修改 C++、头文件或依赖后
必须重新执行 `catkin_make --pkg cym_planner`。
