# 国赛现场可修改参数

本文件只对应国赛 `ucar_2026_national`，用于现场查找“改哪个参数、改哪个文件”。参数文件修改后
不需要重新编译，但必须停止旧任务并重启实际使用的 launch；运行中的节点不会自动读取新值。

## 冲刺参数（搜索“冲刺”即可集中查看）

| 现场行为 / 候选词 | 对应参数（当前值） | 人话解释 | 修改位置 |
| --- | --- | --- | --- |
| `国赛冲刺`、`冲刺是否开启` | `sprint_enabled=true` | 冲刺总开关。`true` 才进入冲刺流程；`false` 时保持原来的直航流程。 | `ucar_2026_national/launch/2026.launch` |
| `70点冲刺`、`冲刺起点` | `sprint_start_point_number=70` | 冲刺前先导航到哪个编号点。当前从 70 号点开始；它是编号，不是坐标。 | `ucar_2026_national/launch/2026.launch` |
| `冲刺终点`、`冲到288点` | `sprint_end_point_number=288` | 冲刺终点的备用编号。只有 `sprint_end_x` 和 `sprint_end_y` 没有同时填写时，才使用 288 号点。 | `ucar_2026_national/launch/2026.launch` |
| `冲刺终点坐标`、`坡顶冲刺终点` | `sprint_end_x=0.875` / `sprint_end_y=1.75` | 直接指定冲刺终点的地图坐标，单位是米。两者同时有值时，优先覆盖编号终点。 | `ucar_2026_national/launch/2026.launch` |
| `70点车头角度`、`冲刺起点航向` | `sprint_yaw_deg=180` | 到达 70 号点后，车辆开始冲刺前使用的车头角度，单位是度。 | `ucar_2026_national/launch/2026.launch` |
| `横向冲刺`、`transverse` | `sprint_transverse_enabled=false` | 是否使用横向冲刺参数。当前为 `false`，使用普通前向 sprint 参数。 | `ucar_2026_national/launch/2026.launch` |

当前实际逻辑：起点 → 70 号点 → 进入 sprint → 冲刺终点 → 切回普通 point 模式。虽然
`sprint_end_point_number` 当前显示为 `288`，但因为 `sprint_end_x/y` 已填写，实际终点是
坐标 `(0.875, 1.75)`，不是按 288 号点坐标执行。修改 `sprint_end_x/y` 时必须两个一起改。

## 候选搜索词（可直接 Ctrl+F）

| 现场行为 / 候选词 | 对应参数（当前值） | 人话解释 | 修改位置 |
| --- | --- | --- | --- |
| `OCR常规点局部膨胀`、`OCR普通点局部膨胀`、`常规局部膨胀` | `inflation_radius=0.224` | 常规 OCR 点附近的局部代价地图膨胀半径，影响车辆离障碍物的安全距离。 | `ucar_nav/config/testnav20260721/local_costmap_common.yaml` |
| `OCR停车膨胀`、`内墙停车膨胀` | `processing_parking_inflation_radius_m=0.07` | OCR 最后靠内墙停车阶段 local/global 同步使用的临时膨胀半径，不是常规点膨胀。 | `ucar_2026_national/launch/2026.launch` |
| `OCR停车距离`、`内墙停车偏移` | `ocr_stop_offset_m=0.25`、`ocr_recheck_backoff_m=0.25` | 先停在墙内 25cm，再普通导航到后方 25cm 验证位并扫读 -45° 到 +45°；若复核墙点错一格，纠正墙点重新吸附 0.25m 网格后计算停车位，最终先车头朝墙导航再原地转为车尾朝挡板。 | `ucar_2026_national/launch/2026.launch` |
| `点3前全局膨胀` | `pre_point_3_global_costmap_inflation_radius_m=0.21` | 到达点 3 之前使用的全局代价地图膨胀半径。 | `ucar_2026_national/launch/2026.launch` |
| `点3后全局膨胀`、`全局常态膨胀` | `global_costmap_inflation_radius_m=0.224` | 点 3 之后以及常态导航使用的全局膨胀半径。 | `ucar_2026_national/launch/2026.launch` |
| `OCR路线`、`扫码点顺序`、`OCR五组` | `production_route_numbers`、`production_route_groups` | 决定常规生产任务依次访问哪些点，以及如何分组扫描。 | `ucar_2026_national/launch/2026.launch` |
| `二维码固定面旋转`、`固定朝向切换` | `fixed_heading_rotation_speed=0.70 rad/s` | 二维码固定面之间的同点原地转向速度。 | `ucar_2026_national/launch/2026.launch` |
| `二维码完整旋转`、`QR 360°扫描` | `qr_rotation_speed=0.18 rad/s` | 二维码未在固定面识别到时的完整 360°扫描速度。 | `ucar_2026_national/launch/2026.launch` |
| `OCR识别阈值`、`OCR扫描速度`、`OCR对准速度` | `ocr_min_confidence`、`ocr_scan_rotation_speed=0.35 rad/s`、`ocr_alignment_*` | 分别控制 OCR 最低置信度、完整 360°扫描速度和识别后的对准动作。 | `ucar_2026_national/launch/2026.launch` |
| `70号点坐标`、`70点坐标还原`、`国赛70点原坐标` | `points`、`grouped_points.centers` 中的 70 号记录 `(2.25, 1.75)` | 同时修改网格中的两处 70 号坐标，恢复点位时不能只改一处。 | `ucar_2026_national/config/production_full_grid_all_numbered.json` |
| `终点雷达角落闭环`、`终点停车` | `use_lidar=true`、`goal_mode=visual` | 视觉命中终点后，由巡线节点读取 `/scan` 完成雷达角落闭环停车。 | `ucar_2026_national/launch/2026.launch` 的 `lane_proto` include |
| `地图坐标终点触发` | `goal_mode=both`、`goal_map_xy`、`goal_map_dist` | 用地图坐标和距离阈值触发终点；当前国赛入口未启用单点地图坐标。 | `lane_proto/launch/lane_proto.launch` |
| `拦路板检测`、`绕板` | `board_in_lane`、`go_around`、`board_stop_dist`、`go_around_keepout` | 控制是否检测拦路板、是否绕板，以及板前停车和绕板安全距离。 | `ucar_2026_national/launch/2026.launch` 的 `lane_proto` include |
| `国赛巡线速度`、`终点巡线速度` | `linear_speed`、`gain`、`rate`、`goal_pause` | 控制国赛终点巡线的线速度、控制增益、循环频率和到点停留时间。 | `ucar_2026_national/launch/2026.launch` 的 `lane_proto` include |
| `起点初始位姿`、`启动位置` | `initial_pose_x`、`initial_pose_y`、`initial_pose_a` | 启动任务时给导航系统的初始地图坐标和车头角度。 | `ucar_2026_national/launch/2026.launch` 的 launch arg |

注意：`OCR常规点局部膨胀`指 local costmap 的常态 `inflation_radius`；
`OCR停车膨胀`才指任务 launch 中的临时 `processing_parking_inflation_radius_m`，
停车 profile 会将 local/global 两个 inflation layer 同步到该值，退出时分别恢复，
两者不要混改。

## 文件入口

- 主流程：`ucar_ws/src/ucar_2026_national/launch/2026.launch`
- 任务网格：`ucar_ws/src/ucar_2026_national/config/production_full_grid_all_numbered.json`
- 国赛地图：`ucar_ws/src/ucar_nav/maps/iflysse_field_walls_national.yaml`
- 局部代价地图：`ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml`
- 规划器参数：`ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`

## 现场修改规则

1. 先停止任务并发布零速度，确认车辆静止。
2. 只修改本文件列出的参数；同步后核对本地与车端 SHA-256。
3. 修改 YAML 后至少重启 `move_base`；修改 launch、JSON 或任务参数后重启国赛主流程。
4. 重启后先确认 `/odom_raw` 有限、`odom -> base_link` 和 `map -> base_link` TF 正常，再开始运动。

## 国赛特有参数

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `use_lidar` / `goal_mode` | `true` / `visual` | V29 视觉命中后由巡线节点自读 `/scan` 做雷达角落闭环；`self` 同义，`false` 为旧 50cm 进给 |
| `board_in_lane` / `go_around` | `true` / `true` | 开启拦路板检测和绕板 |
| `board_stop_dist` / `go_around_keepout` | `0.321` / `0.08` | 板前停车距离和绕板安全距离 |
| `board_arc_lat_scale` | `0.3` | 自动绕板横向让距中的板半长投影比例 |
| `linear_speed` / `gain` / `rate` / `goal_pause` | `0.2` / `1.2` / `20` / `1.0` | 国赛终点巡线参数 |

## 地图点坐标

修改编号点时，必须同时修改 JSON 顶层 `points` 和 `grouped_points.centers` 中的同编号记录。
国赛 70 号点已恢复为原坐标 `(x_m=2.25, y_m=1.75)`，不再使用 `x + 0.07m`、`y - 0.07m`
偏移。JSON 不需要编译，但必须重启任务节点。
