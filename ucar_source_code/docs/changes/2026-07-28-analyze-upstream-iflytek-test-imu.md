# 分析上游 iFLYtek 并静态测试 IMU

## 目的

分析 `https://github.com/WXyuany/iFLYtek/tree/main` 的 IMU 数据路径和串口解析实现，
并在不运动小车的前提下对 mini 集成驱动与上游独立 IMU 驱动进行差分测试。

## 上游基线

- 仓库：`WXyuany/iFLYtek`
- 分支：`main`
- 提交：`e0fe49bc8e5f21bcce115247c0f241872300458d`
- 上游只有一个提交，README 说明 mini 的 IMU 集成在 `ucar_controller`，晓版本才
  单独使用 `fdilink_ahrs`。

协议结构体、CRC 表和独立 `fdilink_ahrs/src/ahrs_driver.cpp` 与本仓库对应文件的
Git blob 完全一致。`base_driver.cpp` 的本地差异是速度校准和用 IMU 航向替代轮速
积分，不在帧头、长度、CRC8、CRC16 的读取路径中，因此不能解释已观察到的 CRC。

上游解析器同样逐段调用 `serial_.read()`，但没有系统检查返回长度；长度、序号、
CRC 字节或完整数据体发生短读时，旧缓冲内容可能继续参加校验。上游代码可作为来源
基线，但不能作为已经修复短读/失步问题的替代驱动。

## 测试方法

1. 动态确认 WSL/小车地址为 `192.168.8.199` 和 `192.168.8.231`。
2. 只启动 WSL ROS Master，启动行确认 `XML-RPC=HTTP/1.0`。
3. 小车加载 Melodic 和工作区后重新设置 `ROS_MASTER_URI`、`ROS_IP`。
4. 显式发布零速度，只启动 `ucar_controller/base_driver`，采样 `/imu` 与 `/odom`。
5. 再次零速并停止 `base_driver`；让源码与上游一致的 `fdilink_ahrs` 独占
   `/dev/ttyUSB0`、`921600`，继续采样 `/imu`。
6. 停止独立节点与 WSL Master，检查两端无 ROS 进程或 11311 监听残留。

## 验证结果

- 测试开始时 `lsusb -t` 显示 CP2102 位于外置 Hub `1-2.3`，不是前一次测试的根
  Hub 直连。内核记录在 uptime 549 秒时直连设备断开，578 秒时 Hub 接入，609 秒
  时 Hub reset 后 CP2102 重新枚举；测试结束 uptime 1402 秒前没有新增 USB 事件。
- 集成 `base_driver` 运行约 4 分钟：
  - `/imu` 约 49.95 Hz；
  - `/odom` 约 20 Hz；
  - 未出现 CRC、`head_len` 或串口异常。
- 独立 `/ahrs_bringup` 运行约 2 分钟：
  - 它是 `/imu` 的唯一发布者；
  - `/imu` 约 49.95 Hz；
  - 未出现 CRC、`head_len` 或串口异常。
- 静止样本全部为有限值，四元数模长接近 1；角速度接近 0，加速度 Z 约
  `-9.78 ~ -9.80 m/s²`。
- 测试没有启动 EKF、定位、导航、雷达或运动目标；停止后小车与 WSL 均无残留 ROS
  进程。

## 结论

上游代码没有针对当前 CRC 提供不同或更健壮的解析实现；本仓库 CRC 路径与上游相同。
本次两种节点均未复现，说明 CP2102/串口当前可以连续输出有效 IMU，但不能推翻此前
多次 CRC、帧长和 USB 重枚举证据。故障仍属间歇性，下一轮应保持物理拓扑不变，至少
观察 10 分钟并执行多次冷启动；若再次复现，应记录每次 `read()` 的实际返回长度或
直接抓取原始串口字节，区分短读、解析失步与真实线缆/MCU 数据损坏。

## 涉及文件与已知限制

- 更新 `docs/operations.md` 和 `犯错档案.md`。
- 新增本记录；没有修改、构建或部署 ROS 源码、参数和资源。
- 本次测试时物理拓扑已经从直连变回外置 Hub，结果不能作为“直连修复”结论。
- 未进行 10 分钟稳定性和多次冷启动测试，不能宣告故障已解决。
