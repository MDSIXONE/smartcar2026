# 2026-08-19：IMU CRC16 仅告警不触发任务停车

## 现象

底盘发布 `check crc16 faild(imu)` 后，三套 2026 任务脚本的 `rosout_cb` 将通用
`crc16` 标记写入 `critical_error`，随后任务守护线程抛出 `PRODUCTION_TASK_ABORTED`
并停车。

## 改动

三套任务脚本在通用结构故障判断前单独识别 `crc16 + imu`，使用限频
`PRODUCTION_IMU_CRC_IGNORED` 告警并直接返回。AHRS CRC16 仍按既有告警处理；
`head_len`、TF_NAN_INPUT、odom/IMU sensor not active、非有限数据和其他 CRC16
结构故障仍会触发任务中止。

底盘驱动的 CRC 校验和坏帧丢弃逻辑未修改；本次只调整任务层对该驱动告警的升级策略。

## 验证与生效

- 三套几何回归新增 IMU CRC 告警用例，并保持 AHRS CRC 告警、head_len 致命用例。
- 修改只涉及 Python2 任务脚本、测试和文档，不需要 catkin 编译。
- 同步脚本后必须在车辆零速、`/odom_raw`、两个 TF 和 `/scan` 安全检查通过后重启实际任务；
  运行中的 Python2 节点不会热加载。
