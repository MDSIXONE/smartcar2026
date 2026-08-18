# 国赛主流程接入新版 lane_proto（2026-08-17）

## 目标

让 `ucar_2026_national` 在 OCR 完成后的常驻交接阶段使用从小车同步的新版
`lane_proto`，启用新版板检测、BoardKF 闭环绕板和终点板面雷达复核。

## 改动

- 仅修改 `ucar_2026_national/launch/2026.launch` 的常驻 `lane_proto` include；省赛
  `ucar_2026` 和国赛随机任务 `ucar_2026_extra` 不改。
- 保留国赛主流程的共享相机、单底盘和待命交接约束：`use_ros_camera=true`、
  `start_base_driver=false`、`start_enabled=false`、`exit_on_stop=true`。
- 按新版实车参考参数设置起跑与巡线：`is_fork=yolo`、band2 模板、
  `yellow_target=0.90`、`align_offset=0.14`、`start_offset=0.23`、`goal_y_lo=0.85`、
  `linear_speed=0.2`、`gain=1.2`、`rate=20`、`dump_every=3`、`goal_pause=1.0`。
- 启用绕板：`board_in_lane=true`、`go_around=true`、`board_stop_dist=0.321`、
  `go_around_keepout=0.15`。其余新版 BoardKF、雷达分簇、终点板面否决和无板回退逻辑
  使用 `lane_proto.launch` 的默认值。

## 交接时机

`lane_proto` 在任务启动时常驻于 `STANDBY`；OCR 和生产任务完成后，任务节点调用
`/lane_proto/set_active true`，模型加载完成后再通过 `/cmd_vel_owner/set_lane_mode true`
切换巡线控制权。`take_cam_on_start=true` 只适用于独立 `roslaunch lane_proto` 测试，
国赛主流程不能使用它，否则会与 `ucar_2026_national` 的共享相机抢占设备。

## 验证与风险

- 本机检查 `2026.launch` XML、参数完整性和 lane_proto 静态回归；本机没有 ROS Melodic，
  ROS 依赖用例需在小车 Ubuntu 18.04 / ROS Melodic 上执行。
- 启用绕板会改变 OCR 后的运动路径；部署后应观察 `/lane_proto/state`、`[拦路板]`、
  `AVOID_*` 和 `/cmd_vel_owner` 日志，并在真实赛道完成一次低速复测。
- 本次不启动 ROS、不启动车辆、不上传小车。
