# 省赛现场可修改参数

本文件只对应省赛 `ucar_2026`，用于现场查找“改哪个参数、改哪个文件”。参数文件修改后
不需要重新编译，但必须停止旧任务并重启实际使用的 launch；运行中的节点不会自动读取新值。

## 候选搜索词（可直接 Ctrl+F）

| 现场行为 / 候选词 | 对应参数（当前值） | 人话解释 | 修改位置 |
| --- | --- | --- | --- |
| `OCR常规点局部膨胀`、`OCR普通点局部膨胀`、`常规局部膨胀` | `inflation_radius=0.224` | 常规 OCR 点附近的局部代价地图膨胀半径，影响车辆离障碍物的安全距离。 | `ucar_nav/config/testnav20260721/local_costmap_common.yaml` |
| `OCR停车膨胀`、`内墙停车膨胀` | `processing_parking_inflation_radius_m=0.07` | OCR 最后靠内墙停车阶段 local/global 同步使用的临时膨胀半径，不是常规点膨胀。 | `ucar_2026/launch/2026.launch` |
| `OCR停车距离`、`内墙停车偏移` | `ocr_stop_offset_m=0.25`、`ocr_recheck_backoff_m=0.25` | 先停在墙内 25cm，再普通导航到后方 25cm 验证位，在墙面法向 -45° 到 +45° 范围内二次识别；确认后回到认定停车区，车尾朝挡板停车。 | `ucar_2026/launch/2026.launch` |
| `点3前全局膨胀` | `pre_point_3_global_costmap_inflation_radius_m=0.21` | 到达点 3 之前使用的全局代价地图膨胀半径。 | `ucar_2026/launch/2026.launch` |
| `点3后全局膨胀`、`全局常态膨胀` | `global_costmap_inflation_radius_m=0.224` | 点 3 之后以及常态导航使用的全局膨胀半径。 | `ucar_2026/launch/2026.launch` |
| `OCR路线`、`常规生产路线`、`扫码点顺序` | `production_route_numbers`、`production_observation_headings_deg` | 决定常规生产任务依次访问哪些点，以及到点时使用的观察朝向。 | `ucar_2026/launch/2026.launch` |
| `OCR五组`、`分组扫描` | `production_route_groups` | 把 OCR 点按组组织，决定每组扫描的点号范围。 | `ucar_2026/launch/2026.launch` |
| `外围兜底路线` | `fallback_production_route_numbers`、`fallback_production_observation_headings_deg` | 常规路线不可用时使用的备用点号顺序和观察朝向。 | `ucar_2026/launch/2026.launch` |
| `二维码方向`、`QR扫描方向` | `qr_observation_numbers` | 配置二维码观察点及其扫描方向。 | `ucar_2026/launch/2026.launch` |
| `二维码固定面旋转`、`固定朝向切换` | `fixed_heading_rotation_speed=0.70 rad/s` | 二维码固定面之间的同点原地转向速度。 | `ucar_2026/launch/2026.launch` |
| `二维码完整旋转`、`QR 360°扫描` | `qr_rotation_speed=0.18 rad/s` | 二维码未在固定面识别到时的完整 360°扫描速度。 | `ucar_2026/launch/2026.launch` |
| `OCR识别阈值`、`OCR扫描速度`、`OCR对准速度` | `ocr_min_confidence`、`ocr_scan_rotation_speed=0.35 rad/s`、`ocr_alignment_*` | 分别控制 OCR 最低置信度、完整 360°扫描速度和识别后的对准动作。 | `ucar_2026/launch/2026.launch` |
| `起点初始位姿`、`启动位置` | `initial_pose_x`、`initial_pose_y`、`initial_pose_a` | 启动任务时给导航系统的初始地图坐标和车头角度。 | `ucar_2026/launch/2026.launch` 的 launch arg |
| `省赛巡线速度`、`终点巡线速度` | `linear_speed`、`gain`、`rate`、`goal_pause` | 控制省赛终点巡线的线速度、控制增益、循环频率和到点停留时间。 | `ucar_2026/launch/2026.launch` 的 `lane_proto` include |
| `终点雷达角落闭环` | `use_lidar=self`、`goal_mode=visual` | 视觉命中终点后，由巡线节点读取 `/scan` 完成雷达角落闭环停车。 | `ucar_2026/launch/2026.launch` 的 `lane_proto` include |
| `普通点速度`、`point模式速度` | `mode1_point.linear_x_gain`、`mode1_point.max_vel_x` | 普通 point 导航模式的线速度增益和最大线速度。 | `cym_planner/config/ucar_cym_planner_params.yaml` |

注意：`OCR常规点局部膨胀`指 local costmap 的常态 `inflation_radius`；
`OCR停车膨胀`才指任务 launch 中的临时 `processing_parking_inflation_radius_m`，
停车 profile 会将 local/global 两个 inflation layer 同步到该值，退出时分别恢复，
两者不要混改。

## 文件入口

- 主流程：`ucar_ws/src/ucar_2026/launch/2026.launch`
- 任务网格：`ucar_ws/src/ucar_2026/config/production_full_grid_all_numbered.json`
- 省赛地图：`ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.yaml`
- 局部代价地图：`ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml`
- 规划器参数：`ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`

## 现场修改规则

1. 先停止任务并发布零速度，确认车辆静止。
2. 只修改本文件列出的参数；同步后核对本地与车端 SHA-256。
3. 修改 YAML 后至少重启 `move_base`；修改 launch、JSON 或任务参数后重启完整省赛主流程。
4. 重启后先确认 `/odom_raw` 有限、`odom -> base_link` 和 `map -> base_link` TF 正常，再开始运动。

省赛没有国赛专用的 `sprint_*` 参数，也不加载额外任务的
`ucar_2026_extra/config/ocr_route_profile.yaml`。

## 常用任务参数

| 参数 | 当前值 / 说明 |
| --- | --- |
| `ocr_stop_offset_m` / `ocr_recheck_backoff_m` | `0.25 m / 0.25 m`，先到原停车点，再普通导航到后方 25cm 验证位，在墙面法向 -45° 到 +45° 范围内二次识别；确认后回到停车区并车尾朝挡板 |
| `processing_parking_profile_enabled` | `true`，启用 OCR 停车阶段的局部膨胀切换 |
| `processing_parking_inflation_radius_m` | `0.07 m`，OCR 最终内墙停车阶段 local/global 同步使用 |
| `production_route_numbers` | `11,12,13,14,15,16,17,18,19,20,30,29,28,27,26,25,24,23,22,21` |
| `production_route_groups` | `[11,12,21,22]`、`[13,14,23,24]`、`[15,16,25,26]`、`[17,18,27,28]`、`[19,20,29,30]` |
| `destination_point_number` | `441` |
| `destination_heading_point_number` | `170` |
| `lane_proto.linear_speed` | `0.2` |
| `lane_proto.gain` / `rate` / `goal_pause` | `1.2` / `20` / `1.0` |
| `lane_proto.use_lidar` / `goal_mode` | `self` / `visual` | V29 省赛终点由巡线节点自己读取 `/scan` 做雷达闭环 |
| `lane_proto.board_in_lane` / `go_around` | `false` / `false` | 省赛关闭拦路板检测与绕板 |

## 地图点坐标

修改编号点时，必须同时修改 JSON 顶层 `points` 和 `grouped_points.centers` 中的同编号记录。
省赛 70 号点当前为 `(x_m=2.25, y_m=1.75)`。JSON 不需要编译，但必须重启任务节点。
