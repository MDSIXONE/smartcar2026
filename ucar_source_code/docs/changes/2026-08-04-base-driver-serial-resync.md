# 底盘串口短读重同步

## 目的

定位 OCR 启动附近出现的 `/odom_raw stale` 后，修复底盘驱动把 `serial_.read()` 的短读结果
继续交给 CRC/里程计解析的问题。修复必须保持任务的 `0.35 s` 新鲜度安全门，不补发旧里程计，
并能从底盘日志区分“任务未收到回调”和“底盘没有产生有效帧”。

## 涉及文件

- `ucar_ws/src/ucar_controller/src/base_driver.cpp`
- `ucar_ws/src/ucar_controller/include/ucar_controller/base_driver.h`
- `ucar_ws/src/ucar_controller/include/ucar_controller/serial_read_exact.h`
- `ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`
- `ucar_ws/src/ucar_controller/test/test_serial_read_exact.cpp`
- `ucar_ws/src/ucar_controller/CMakeLists.txt`
- `ucar_ws/src/ucar_controller/package.xml`
- `docs/operations.md`
- `犯错档案.md`

## 改动

- 新增与 ROS 无关的 `serialReadExactly()` 单元，支持一次字段被多个成功 `read()` 分段返回，并以
  基于单调时钟的字段总预算阻止“持续每次仅少量字节”无限延长读取。
- 底盘、IMU/AHRS/INSGPS 和 ground 帧的每个读取点都要求读满；短读时清空目标缓冲、丢弃当前帧并
  回到帧头搜索，不再让上一帧字节参与 CRC。
- 修正 INSGPS 帧头判断误用了旧 `head_type` 的分支，使该类帧也能进入同一完整读取路径。
- 记录 `BASE_SERIAL_RESYNC_SHORT_READ`、`BASE_SERIAL_SLOW_READ`、
  `BASE_odom_PUBLISH_GAP` 和 `BASE_imu_PUBLISH_GAP`。默认阈值由
  `serial_gap_warn: 0.35` 配置，只影响诊断，不改变导航安全门或速度。

## 验证

- 已进行源码审查和 `git diff --check`；本机不编译，因为底盘代码只能在小车 Ubuntu 18.04 /
  ROS Melodic 上构建。
- 新增单测覆盖分段拼接、零进度短读、异常超量返回、持续少量字节到期和“截断帧不派发、后续完整字段
  恰派发一次”。车端原有 Catkin 白名单未包含 `ucar_controller`，已确认必须临时切换该白名单后执行
  `catkin_make --pkg ucar_controller -DCATKIN_ENABLE_TESTING=ON`、
  `catkin_make run_tests_ucar_controller` 以及至少 10 分钟零速度静态串口观察。
- 车端已成功编译 `base_driver`；首次单测链接因该 Melodic `catkin_add_gtest` 未提供测试入口 `main`
  退出，已在测试源码显式补入 gtest 入口，待重新运行。

## 已知限制

- 若短读来自 CP2102、USB、线束、接地、供电或 MCU 固件，软件只能安全丢帧和提供证据，不能恢复
  缺失字节。
- 静态通过不等同于 OCR 启动或完整实车任务通过；必须先完成静态诊断，再由用户确认后进行受看护的
  实车验证。
