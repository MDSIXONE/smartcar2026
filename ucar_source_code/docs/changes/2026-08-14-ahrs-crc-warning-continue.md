# AHRS CRC16 告警继续任务

## 修改

主流程的 `/rosout_agg` 回调将仅含 AHRS 的 `crc16` 日志改为
`PRODUCTION_AHRS_CRC_IGNORED` 告警，不再写入 `critical_error` 触发任务 ABORT。

## 保留行为

`head_len`、`tf_nan_input`、非有限 `/odom_raw`、里程计/IMU 不活跃等其他原有错误处理不变。

## 验证

回归分别注入 `check crc16 faild(ahrs).` 与 `head_len`：前者保持任务错误为空且有告警，
后者仍写入任务错误。车端 ucar_2026 Catkin 全量 96 项为 0 errors / 0 failures。
