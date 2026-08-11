# 2026-08-11：OCR 巡航转速 0.30 rad/s 与车端部署

## 目的

将每个生产巡航点的完整 OCR 原地扫描命令从 `0.25 rad/s` 调整为
`0.30 rad/s`，并把此前尚未部署的生产任务流程同步到小车。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `docs/operations.md`

## 行为与安全性

- Python 参数默认值及正式 launch 显式值均为 `0.30 rad/s`，避免 launch 覆盖默认值。
- 整圈超时仍按 `2π / speed × rotation_timeout_scale + 2 s` 计算；在
  `rotation_timeout_scale=3.5` 下为约 `75.3 s`。
- 不改变 OCR 连续对准速度、20 Hz 控制周期、里程计进度累计、零速/停车确认或 NaN/TF/CRC 安全门。
- 此次部署、构建和单测不启动 ROS launch，不发送底盘运动命令。

## 验证

- 当次网络小车为 `192.168.8.231`，WSL Master 为 `192.168.8.199:11311`；五个文件逐项
  SHA-256 与本地一致。
- 小车 Ubuntu 18.04 / ROS Melodic 的 Python 2 对三个任务脚本语法检查和 launch XML
  解析通过；`catkin_make --pkg ucar_2026 -DCATKIN_ENABLE_TESTING=ON` 通过。
- 工作区级 `catkin_make -DCATKIN_ENABLE_TESTING=ON run_tests` 与
  `catkin_test_results --verbose build/test_results/ucar_2026` 通过：94 tests、0 errors、
  0 failures、0 skipped；另直接运行任务测试为 82 tests OK。
- 为兼容本车 Catkin 0.7.29，相关操作文档中的错误子包测试目标改为工作区级 `run_tests`。

## 已知限制

补充巡航路线的外圈角点（尤其点 1）尚无本次改速后的原地 360° 净空实测。OCR 转圈直接发布
角速度，不经 `move_base` footprint 碰撞检查；因此不以本次部署自动开展完整外圈运动验收。
首次实车运动前必须单独复核角点的完整车体旋转净空，并有人看护急停。
