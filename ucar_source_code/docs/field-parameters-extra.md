# 额外任务现场可修改参数

本文件只对应额外任务 `ucar_2026_extra`，用于现场查找“改哪个参数、改哪个文件”。参数文件修改后
不需要重新编译，但必须停止旧任务并重启实际使用的 launch；运行中的节点不会自动读取新值。

## 候选搜索词（可直接 Ctrl+F）

| 现场行为 / 候选词 | 对应参数（当前值） | 人话解释 | 修改位置 |
| --- | --- | --- | --- |
| `OCR常规点局部膨胀`、`OCR普通点局部膨胀`、`常规局部膨胀` | `inflation_radius=0.224` | 常规 OCR 点附近的局部代价地图膨胀半径，影响车辆离障碍物的安全距离。 | `ucar_nav/config/testnav20260721/local_costmap_common.yaml` |
| `OCR停车膨胀`、`内墙停车膨胀` | `processing_parking_inflation_radius_m=0.07` | OCR 最后靠内墙停车阶段 local/global 同步使用的临时膨胀半径，不是常规点膨胀。 | `ucar_2026_extra/launch/2026.launch` |
| `OCR停车距离`、`内墙停车偏移` | `ocr_stop_offset_m=0.25`、`ocr_recheck_backoff_m=0.25` | 先停在墙内 25cm，再普通导航到后方 25cm 验证位并扫读 -45° 到 +45°；若复核墙点错一格，纠正墙点重新吸附 0.25m 网格后计算停车位，最终先车头朝墙导航再原地转为车尾朝挡板。 | `ucar_2026_extra/launch/2026.launch` |
| `点3前全局膨胀` | `pre_point_3_global_costmap_inflation_radius_m=0.21` | 到达点 3 之前使用的全局代价地图膨胀半径。 | `ucar_2026_extra/launch/2026.launch` |
| `点3后全局膨胀`、`全局常态膨胀` | `global_costmap_inflation_radius_m=0.224` | 点 3 之后以及常态导航使用的全局膨胀半径。 | `ucar_2026_extra/launch/2026.launch` |
| `额外OCR路线`、`OCR快捷路线`、`随机任务路线` | `ocr_route_profile` | 是否启用额外任务的 OCR 快捷路线模板；空数组表示关闭模板。 | `ucar_2026_extra/config/ocr_route_profile.yaml` |
| `70号点坐标`、`70点坐标还原`、`额外任务70点原坐标` | `points`、`grouped_points.centers` 中的 70 号记录 `(2.25, 1.75)` | 同时修改网格中的两处 70 号坐标，恢复点位时不能只改一处。 | `ucar_2026_extra/config/production_full_grid_all_numbered.json` |
| `快捷路线点号` | `point` | 快捷路线条目要执行的任务点号。 | `ucar_2026_extra/config/ocr_route_profile.yaml` |
| `快捷路线朝向` | `heading_deg` | 到达快捷路线点后，车辆保持的车头朝向，单位是度。 | `ucar_2026_extra/config/ocr_route_profile.yaml` |
| `快捷路线旋转角度`、`旋转方向` | `rotate_angle_deg`、`rotate_dir` | 到点后原地旋转多少度，以及按顺时针还是逆时针旋转。 | `ucar_2026_extra/config/ocr_route_profile.yaml` |
| `wall停车`、`free停车`、`随机位置停车` | `stop_mode` | 选择靠墙停车、自由位置停车等停车方式。 | `ucar_2026_extra/config/ocr_route_profile.yaml` |
| `额外任务识别目标`、`OCR目标文字` | `target_texts` | 配置本次额外任务要识别和匹配的文字列表。 | `ucar_2026_extra/config/ocr_route_profile.yaml` |
| `OCR路线`、`默认生产路线`、`扫码点顺序` | `production_route_numbers`、`production_observation_headings_deg` | 决定默认生产任务依次访问哪些点，以及到点时使用的观察朝向。 | `ucar_2026_extra/launch/2026.launch` |
| `二维码固定面旋转`、`固定朝向切换` | `fixed_heading_rotation_speed=0.70 rad/s` | 二维码固定面之间的同点原地转向速度。 | `ucar_2026_extra/launch/2026.launch` |
| `二维码完整旋转`、`QR 360°扫描` | `qr_rotation_speed=0.18 rad/s` | 二维码未在固定面识别到时的完整 360°扫描速度。 | `ucar_2026_extra/launch/2026.launch` |
| `OCR识别阈值`、`OCR扫描速度`、`OCR对准速度` | `ocr_min_confidence`、`ocr_scan_rotation_speed=0.35 rad/s`、`ocr_alignment_*` | 分别控制 OCR 最低置信度、完整 360°扫描速度和识别后的对准动作。 | `ucar_2026_extra/launch/2026.launch` |
| `额外任务巡线速度`、`终点巡线速度` | `linear_speed`、`gain`、`rate`、`goal_pause` | 控制额外任务终点巡线的线速度、控制增益、循环频率和到点停留时间。 | `ucar_2026_extra/launch/2026.launch` 的 `lane_proto` include |
| `起点初始位姿`、`启动位置` | `initial_pose_x`、`initial_pose_y`、`initial_pose_a` | 启动任务时给导航系统的初始地图坐标和车头角度。 | `ucar_2026_extra/launch/2026.launch` 的 launch arg |

注意：`OCR常规点局部膨胀`指 local costmap 的常态 `inflation_radius`；
`OCR停车膨胀`才指任务 launch 中的临时 `processing_parking_inflation_radius_m`，
停车 profile 会将 local/global 两个 inflation layer 同步到该值，退出时分别恢复，
两者不要混改。

## 文件入口

- 主流程：`ucar_ws/src/ucar_2026_extra/launch/2026.launch`
- 任务网格：`ucar_ws/src/ucar_2026_extra/config/production_full_grid_all_numbered.json`
- 地图：`ucar_ws/src/ucar_nav/maps/iflysse_field_walls_national.yaml`
- OCR 快捷路线：`ucar_ws/src/ucar_2026_extra/config/ocr_route_profile.yaml`
- 局部代价地图：`ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml`
- 规划器参数：`ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`

## 现场修改规则

1. 先停止任务并发布零速度，确认车辆静止。
2. 只修改本文件列出的参数；同步后核对本地与车端 SHA-256。
3. 修改 YAML 后至少重启 `move_base`；修改 launch、JSON 或 OCR 模板后重启额外任务主流程。
4. 重启后先确认 `/odom_raw` 有限、`odom -> base_link` 和 `map -> base_link` TF 正常，再开始运动。

## OCR 快捷路线模板

默认内容是：

```yaml
[]
```

该文件由 launch 的 `param="ocr_route_profile"` 直接加载，因此根节点必须是列表；空数组表示关闭快捷模板，恢复 launch 中的固定生产路线。启用模板时，每个条目可包含：

| 字段 | 说明 |
| --- | --- |
| `point` | 必填，任务网格中的点号 |
| `heading_deg` | 到点时的车头朝向 |
| `rotate_angle_deg` | 到点后原地旋转的角度 |
| `rotate_dir` | `ccw` 或 `cw` |
| `stop_mode` | `wall` 或 `free` |
| `target_texts` | 目标文字列表；空列表沿用默认三类 |

额外任务不使用国赛 `sprint_*` 参数，也不使用国赛终点地图坐标闭环配置。

## 地图点坐标

修改编号点时，必须同时修改 JSON 顶层 `points` 和 `grouped_points.centers` 中的同编号记录。
额外任务 70 号点已恢复为原坐标 `(x_m=2.25, y_m=1.75)`，不再使用 `x + 0.07m`、`y - 0.07m`
偏移。JSON 不需要编译，但必须重启任务节点。
