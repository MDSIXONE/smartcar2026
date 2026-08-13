# 2026-08-11：odom_timeout/tf_timeout 放宽到 4.0s

## 目的

2026-08-11 18:43 实车任务在点 13（电子产品生产车间）OCR 对准与墙点匹配全部成功后，
因 `/odom_raw is stale by 3.009 s` 中止——仅超出当时 3.0s 门限 9ms。底盘
odom/imu 每次运行有 ~1.0–1.04s 静默特性（MCU/串口特性，5 次运行均如此），
3.0s 已放大 3 倍但仍偶发卡线。本次放宽到 4.0s，给换电池后的串口恢复留出余量，
同时仍远小于真实掉线，安全门继续有效。

## 涉及文件

- `ucar_ws/src/ucar_2026/launch/2026.launch`：
  - `odom_timeout` 3.0 → 4.0
  - `tf_timeout` 3.0 → 4.0（与 odom_timeout 保持匹配，避免 TF 链冻结误触发）
- `ucar_ws/src/ucar_controller/launch/ucar_bringup.launch`：
  - `robot_pose_ekf/sensor_timeout` 3.0 → 4.0（与任务门限一致，避免 3.0-4.0s
    瞬时断流先被 EKF 判定「传感器死」——该判定被任务视为致命）
- 均已同步到小车对应路径（launch 无需编译）。

## 验证结果

- 小车端 `grep odom_timeout` 确认 4.0 已生效。
- 实车重跑验证待换电池后进行；若仍出现断流中止且时长接近 4.0s，说明问题
  不在门限而在底盘串口链路（USB Hub/CP2102/供电），需按
  `犯错档案/2026-07-28.md` 排查硬件。

## 已知限制

- 放宽门限只容忍短暂断流，不修复断流本身；连续超过 4.0s 仍会安全中止。
- 若换电池后串口设备路径变化（ttyUSB 编号），`base_driver` 的
  `driver_params_mini.yaml` 端口配置可能需要同步。
