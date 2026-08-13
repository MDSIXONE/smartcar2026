# 2026-08-11：生产路线防碰撞三方案（降速/容差、重规划频率、交接提速）

## 目的

2026-08-11 实车运行中，点 13 OCR 转圈完成后前往点 16 时，因场地中间
431=(-0.50,0.50) 有真实障碍物（14/24/15/25 的守卫角点，目标 16 守卫不含
431 故放行），move_base 绕行路径（357 poses ≈ 4.2m，直线仅 1.5m）贴障碍太近，
高速下撞上。按用户三方案实施优化。

## 涉及文件（均已同步小车，YAML/launch/sh 无需编译）

1. `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`（方案2）：
   - mode1_point `max_vel_x: 0.50 → 0.35`、`max_vel_theta: 1.00 → 0.80`
     降低绕行中途障碍时的碰撞概率与冲击；
   - `final_yaw_tolerance: 0.10 → 0.15` 放宽到点朝向容差，省回部分降速时间。
2. `ucar_ws/src/ucar_2026/launch/2026.launch`（方案2）：
   - 新增 `arrival_tolerance: 0.15`（原默认 0.10），放宽任务到点容差 5cm，
     减少终点对准/回旋耗时（仍远小于 0.5m 点间距，不影响生产停入）。
3. `ucar_ws/src/ucar_nav/config/testnav20260721/move_base_params.yaml`（方案1）：
   - `planner_frequency: 3.0 → 5.0`，中途障碍出现时更快重规划绕行。
4. `ucar_ws/src/ucar_2026/scripts/handoff_lane.sh`（方案3）：
   - `lane_handoff_delay: 1.0 → 0.3`（2026.launch），
   - handoff 轮询粒度 `sleep 1s → 0.2s`（等待 2026.launch 退出与串口就绪
     各 150/50 次循环，总超时不变 30s/10s），缩短终点→巡线交接延迟。

## 验证结果

- 小车端 grep 确认 4 处参数均已生效（max_vel_x 0.35 / planner_frequency 5.0 /
  arrival_tolerance 0.15 / lane_handoff_delay 0.3）。
- 实车复跑验证待进行；431 障碍物仍在场地时，重点观察点 13→16 段绕行
  是否不再碰撞、以及任务总时长变化。

## 已知限制

- 降速会小幅增加生产巡航耗时（已用容差放宽补偿）；若后续要更快，
  可再调 max_vel_x 0.40 并用计时实测校准。
- 方案 1 的频率提升只影响重规划响应，不能修复全局路径本身贴障碍的
  质量问题；431 障碍若可移动/移除，仍是首选（守卫已正确跳过 14/24/15/25）。
