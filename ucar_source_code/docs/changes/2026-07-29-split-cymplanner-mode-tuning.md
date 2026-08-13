# 拆分 CymPlanner 两模式参数

## 目的

- 把“避免因惯性靠墙”放在脱困之前，通过生产路线模式的独立低速参数预防。
- 让 `point` 与 `body_projection` 不再共用 `linear_x_gain`、速度上限等参数，
  用户可直接在 YAML 中分别调整。
- 保留首版受限横移源码供后续修复，但默认禁用，避免安全复核发现的路径被执行。

## 涉及文件

- `ucar_ws/src/cym_planner/include/cym_planner/planner_tuning.h`
- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/config/README.md`
- `ucar_ws/src/cym_planner/test/planner_tuning_test.cpp`
- `ucar_ws/src/cym_planner/CMakeLists.txt`
- `docs/operations.md`
- `犯错档案.md`

## 配置结构

- `mode1_point/*` 对应 `point`、`main`、`main_legacy`。
- `mode2_body_projection/*` 对应 `body_projection`、`footprint`、
  `laser_avoidance`。
- 两组分别包含：
  `linear_x_gain`、`linear_x_kd`、`max_vel_x`、`angular_gain`、
  `angular_kd`、`max_vel_theta`、`heading_slowdown_min_scale`、
  `final_yaw_gain`、`final_yaw_max_vel`、`final_yaw_tolerance`、
  `final_linear_x_gain`、`obstacle_lookahead_distance`、
  `obstacle_cost_threshold`、`carry_speed_scale`、`command_sweep_time`、
  `command_sweep_step`。

模式 1 保留原上限 `0.50 m/s`、`1.00 rad/s`，转角最低速度比例设为 `1.0`
以保持原行为。模式 2 默认：

```yaml
linear_x_gain: 0.9
linear_x_kd: 0.2
max_vel_x: 0.22
angular_gain: 2.0
angular_kd: 0.2
max_vel_theta: 0.55
heading_slowdown_min_scale: 0.15
final_yaw_max_vel: 0.35
```

普通路径跟随时，模式 2 的线速度再乘
`max(heading_slowdown_min_scale, cos(abs(heading_error))²)`，让紧转弯先减速。
碰撞检查从当前车体和全局路径最近点开始；候选 Twist 还会每 `0.025 s` 对未来
`0.40 s` 的车体姿态做扫掠，并同时检查全局静态格和本地动态代价地图。
模式 2 的 `command_sweep_time` 若被误设为 `0` 或负数，启动时强制恢复为
`0.40 s`；`command_sweep_step` 若为非有限值或非正数则恢复为 `0.025 s`，
避免配置误操作关闭或破坏前向扫掠。footprint 位姿与全局或局部代价地图 frame
不一致时失败关闭（兼容前导 `/` 差异）；控制周期超过 `50 ms` 时节流输出错误
日志，供车端评估扫掠计算的实时性。

## 脱困状态

- `escape_enabled` 的 YAML 值为 `false`；源码硬制为 `false`，即使配置请求
  `true` 也会拒绝并记录错误。
- Sol 最终复核发现首版横移存在三个 P1：前视阻挡消失会提前重置状态、只统计第一个
  接触格、未把本地动态障碍纳入横移预览。
- 在这三项修复并建立集成测试前，不得把 `escape_enabled` 改为 `true`。

## 验证

- 先新增 `planner_tuning_test.cpp` 并在车端显式构建目标，按预期因
  `planner_tuning.h` 尚不存在而红灯。
- 车端 Ubuntu 18.04 成功构建 `cym_planner`。
- `catkin_make run_tests_cym_planner` 汇总 48 项，0 error、0 failure、
  0 skipped。
- 新增 4 项测试覆盖两模式独立取值、模式 2 的 `cos²` 转角降速、模式 1
  可保持不降速，以及 NaN/Inf 限幅失败关闭。
- 未启动 ROS、move_base 或生产任务，未发布速度。
- 最终扫掠步长与局部 frame 补丁已在车端重新构建；再次汇总 48 项测试，
  0 error、0 failure、0 skipped。
- 最终 Sol 复核确认未发现 P1；非有限扫掠步长和局部 frame 漏检两项 P2
  均已关闭。

## 已知限制

- 低速默认值尚未进行实车路线验证；需要用户确认后从固定起点完整跑一次，并根据
  点 1、7→8 的实际横向误差继续调 `mode2_body_projection`。
- 模式切换只允许在车辆已停止、两个导航目标之间执行；当前未为运动中并发切模
  提供线程级一致性保证。
- `50 ms` watchdog 只负责报告超时，不会让当前控制周期自动失败；首轮低速测试
  若重复出现超时日志，必须立即停车并评估性能。
- 修改 YAML 后必须重启 move_base 才会重新加载，不支持运行时动态 reconfigure。
