# 缩小局部图并降低 body-projection 速度

## 目的

把 local costmap 从 `2.0 × 2.0 m` 缩为 `1.0 × 1.0 m`，降低局部快照复制与 footprint
候选检查成本；同时把生产第二阶段的速度降到约 `0.07 m/s`。

## 涉及文件

- `ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/config/README.md`
- `docs/operations.md`

## 实现

- local rolling window 改为 `1.0 × 1.0 m`，保持 `0.03 m` 分辨率、静态墙、雷达层和
  footprint 不变。
- `mode2_body_projection/max_vel_x` 与 `elastic_max_vel_x` 都设为 `0.07 m/s`；模式 2
  最大角速度设为 `0.35 rad/s`，最终朝向上限 `0.25 rad/s`，弹性带最大角速度 `0.30 rad/s`。
- 因半边局部图只有 `0.5 m`，弹性带前视同步收缩为 `0.25 m`，源码最小允许值调整为
  `0.20 m`；否则完整 footprint 加偏移会安全地因越图而被拒绝，无法产生候选。

## 验证

- 待小车 Ubuntu 18.04 构建并运行 CymPlanner/ucar_2026 测试后确认；本改动未启动 ROS
  或车辆。

## 已知限制

- 1 m local window 对较远动态障碍的前视更短；安全判定仍覆盖当前 footprint、0.25 m
  前视与 0.40 秒 Twist 扫掠。首次实车只允许在用户确认、车辆置于起点且可急停时进行。
