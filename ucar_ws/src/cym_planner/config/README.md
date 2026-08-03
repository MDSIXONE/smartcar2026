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

模式 2 默认把直线速度上限降到 `0.07 m/s`、角速度上限降到 `0.35 rad/s`。
普通路径跟随时还按航向误差的 `cos²` 缩小线速度，最低比例由
`heading_slowdown_min_scale` 控制；模式 1 默认设为 `1.0`，保持原行为。
模式 2 还会按实际候选 Twist 每 `0.025 s` 投影一次未来 `0.40 s` 的真实车体姿态，
当前 footprint、最近路径点前视和候选 Twist 扫掠共用同一份局部快照。车体尺寸仍
直接读取 local costmap 配置。

当且仅当当前 footprint 完全安全、只是前视的原全局路径命中 254/255 时，模式 2 会先
在 `0.25 m` 前方生成左右带状局部候选：`±0.02` 至 `±0.10 m`，以不超过 `0.015 m`
的间隔把每个中间完整 footprint 投影到同一份 local 快照。候选从原路径平滑偏出并回接，
任何 lethal、unknown、越界或非有限值都会淘汰该候选；启用候选时线速度额外限为
`elastic_max_vel_x`（默认 `0.07 m/s`）、角速度额外限为 `elastic_max_vel_theta`
（默认 `0.30 rad/s`）。候选使用变形后路径的切线，段内以平移 `0.015 m` 和旋转
`0.05 rad` 的更严格采样数同时插值位置/朝向；实际 Twist 仍必须通过既有 0.40 秒扫掠。
若两侧均不可行，才返回 `false` 请求全局重规划。当前 footprint 已接触、TF/代价图异常
时不会尝试弹性路径，仍保持零速失败关闭。合并 master costmap 不保留 StaticLayer 与
ObstacleLayer 来源，因此第一版不按来源放宽任一规则。

move_base 周期性送来 global plan 时，规划器会对整段路线按弧长取 7 个样本，逐个比较
位置（最多 `0.04 m`）和切线角（最多 `0.20 rad`）；全段等价才保留已验证的局部带与
搜索计时，避免每个规划周期重新选边。任何中段绕行、目标或切线显著变化都会清除带并
重新开始安全判定。

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
