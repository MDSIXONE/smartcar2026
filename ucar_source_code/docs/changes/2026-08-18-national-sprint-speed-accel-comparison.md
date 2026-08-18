# 国赛冲刺速度与加速响应对比参数

日期：2026-08-18

## 目的

进行国赛 70→坡顶冲刺的高速、高加速响应对比试跑。

## 参数变化

文件：`ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`

- `mode3_sprint.max_vel_x`：`2.5` → `2.7`
- `mode3_sprint.linear_x_gain`：`12.5` → `13.5`
- `mode3_sprint.angular_gain`：保持 `5.0`
- 任务层 `sprint_yaw_deg`：保持 `175`

CymPlanner 当前没有独立的加速度上限字段，因此用 `linear_x_gain` 提高前向速度指令
对前视误差的响应作为本轮加速对比项。它不是独立的物理加速度限制，实际轮端加速度仍
受底盘控制器、负载、地面附着和麦轮滚子打滑影响。

## 验证结果

- YAML 解析和目标参数断言通过。
- 已确认国赛 `2026.launch` 通过 `cym_move_base_omni_2026.launch` 加载该 YAML。
- 已同步到小车 `/home/ucar/ucar_ws`；本地与车端 YAML SHA-256 一致，车端读取到
  `linear_x_gain=13.5`、`max_vel_x=2.7`、`angular_gain=5.0`。
- 未在本机编译或启动 ROS；仍需在车端 Ubuntu 18.04 / ROS Melodic 上单独试跑。

## 已知限制

2.7 m/s 与更高前向增益可能增加麦轮滚子打滑和制动距离。试跑前必须确认
`/odom_raw` 为有限值且相关 TF 正常，并准备随时发布零速度；对比时记录实际车速、
轮端反馈和横向偏差。
