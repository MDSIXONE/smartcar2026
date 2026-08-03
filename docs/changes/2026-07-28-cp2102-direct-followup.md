# CP2102 直连后的底盘串口故障范围收敛

## 目的

记录 CP2102 绕过 Terminus USB Hub、直接接入小车主机后的静态复现结果，避免继续
把 Hub 或新增扬声器误判为唯一原因。

## 涉及文件

- `犯错档案.md`
- `docs/operations.md`
- `docs/changes/2026-07-28-cp2102-direct-followup.md`

本次没有修改或部署 ROS 源码、参数和资源文件。

## 验证结果

- 只读检查确认 CP2102 位于主机 USB 根 Hub 的 `1-2` 端口，速率为 12 Mbit/s；
  Terminus Hub 已不在 CP2102 路径中。
- 内核日志只有本次开机时的 CP2102 枚举和 `ttyUSB0` 挂载，没有新的 USB
  disconnect、reset 或 over-current。
- 直连后仍出现 `check crc16 faild(ahrs)` 和 `head_len error (ahrs)`。
- 同一窗口内，激光帧请求的 `odom -> base_link` 时间分别比最新 TF 晚约
  2.96 秒、3.13 秒和 3.22 秒；这不是 50 ms 查询容差可以吸收的正常异步。
- `Could not transform imu message from imu to base_link` 表示该条有效 IMU 消息到达
  EKF 时，对应时间的 `base_link -> imu` 变换不可用；需要在下一次底盘串口稳定后
  单独确认静态 TF 发布节点，不能用它解释或掩盖 CRC。
- 检查时相关 ROS 启动进程均已停止，没有遗留运行终端。

## 结论

原 Terminus Hub 链路曾经整体掉线，因此它确实存在问题，但不是 AHRS/底盘串口错误
的唯一来源。直连且内核 USB 层稳定时仍产生数据帧错误，下一优先级为 CP2102 模块、
CP2102 到底盘 MCU 的串口线束/接插件/公共地、底盘控制板供电与串口信号完整性；
驱动单字节读取未校验返回长度也可能放大一次短暂读取异常。

## 已知限制

- 当前日志不能单独区分 CP2102 UART 侧电气错误、底盘 MCU 发出的错误帧和驱动解析
  失步，需要逐件替换或抓取 UART/USB 数据才能继续确认。
- 在 `/odom_raw` 连续有限且 `odom -> base_link`、`map -> base_link` 均恢复前，
  不允许执行定位、导航或旋转测试。
