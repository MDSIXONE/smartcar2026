# 国赛视觉巡线参数与完成播报（2026-08-18）

## 目的

将国赛 `lane_proto` 常驻交接入口切换到现场指定的视觉巡线参数，并保证只有
雷达角落闭环连续稳定到位后进入 `GOAL/STOPPED` 时播报“任务完成”。

## 改动

- `ucar_ws/src/ucar_2026_national/launch/2026.launch`
  - 模板改为 `red_template_band.png`；`goal_y_lo=0.75`、`goal_half=40`。
  - 保持 `is_fork=yolo`、`yellow_target=0.90`、`align_offset=0.14`、
    `start_offset=0.23`、`linear_speed=0.2`、`gain=1.2`、`rate=20`、
    `dump_every=5`、`goal_pause=1.0`。
  - 绕板参数改为 `board_in_lane=true`、`go_around=true`、
    `board_stop_dist=0.321`、`go_around_keepout=0.1`、
    `board_arc_lat_scale=0.5`。
- `ucar_ws/src/lane_proto/launch/lane_proto.launch` 与
  `ucar_ws/src/lane_proto/scripts/lane_follow.py`
  - 新增并实际使用 `board_arc_lat_scale`，控制自动横让距离中板半长投影的比例。
  - 播报绑定到雷达角落闭环连续稳定到位事件；只有该事件进入 `STOPPED/GOAL`
    时播报，视觉普通停车、`ABORT`、`ESTOP`、`BOARD`、`CONFIG` 等路径不播报。
- `ucar_ws/src/lane_proto/test/test_lane_runtime.py`
  - 锁定国赛参数和“仅 GOAL 播报”规则。

## 验证

- `lane_proto` 定向测试：15 项通过，3 项因本机没有 ROS Melodic Python 运行时跳过。
- 两个 launch XML 解析通过；Python 语法检查通过；`git diff --check` 通过。
- 已按动态发现的车端地址同步 3 个运行文件：`lane_follow.py`、
  `lane_proto.launch`、国赛 `2026.launch`；两端 SHA-256 一致。
- 车端 Ubuntu 18.04/Python2 语法检查和 launch XML 解析通过；未启动 ROS、未重启任务、未启动车辆。

## 已知限制

`board_arc_lat_scale=0.5` 会缩短自动横让距离，必须在 Ubuntu 18.04/ROS Melodic
车端完成零速、里程计/TF、雷达新鲜度检查后低速复测；本机静态测试不能替代实车
绕板安全验证。完成播报以雷达角落稳定到位为前提，雷达拟合超时不会播报。
