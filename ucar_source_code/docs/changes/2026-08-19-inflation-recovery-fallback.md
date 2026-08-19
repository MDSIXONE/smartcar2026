# point3 前后分阶段全局膨胀与恢复行为逐级降低

## 目的

point3 前全局膨胀使用 `0.21m`，到达 point3 后恢复为常态 `0.224m`。车辆被障碍堵住时，现有清图、旋转和再次清图恢复行为全部失败会继续逐级降低局部/全局当前膨胀：每次各降低 `0.01m`，直到 `0.05m`；每个阶段后重新规划，仍无路径才进入下一阶段。恢复插件读取 dynamic-reconfigure 当前值，避免 point3 前后固定起始值切换造成膨胀半径反向增大。

## 涉及文件

- `ucar_ws/src/cym_planner/src/inflation_recovery.cpp`
- `ucar_ws/src/cym_planner/include/cym_planner/inflation_recovery_schedule.h`
- `ucar_ws/src/cym_planner/test/inflation_recovery_schedule_test.cpp`
- `ucar_ws/src/cym_planner/CMakeLists.txt`
- `ucar_ws/src/cym_planner/package.xml`
- `ucar_ws/src/cym_planner/cym_planner_plugins.xml`
- `ucar_ws/src/ucar_nav/config/testnav20260721/move_base_params.yaml`
- `ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_common.yaml`
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
- 三套 `ucar_2026*/launch/2026.launch`
- `docs/operations.md`

## 验证结果

- YAML 解析通过：恢复行为共 21 个，其中 3 个原有行为加 18 个膨胀阶段。
- XML 解析通过：插件描述和当前 CymPlanner 导航 launch 均有效。
- 阶段序列静态核对通过：`0.224 → 0.214 → … → 0.054 → 0.05`。
- 当前值调度静态核对通过：point3 前全局 `0.21 → 0.20`，point3 后全局 `0.224 → 0.214`。
- `git diff --check` 通过。
- 三套 Python2 源码、三套 launch 和 YAML 本地静态检查通过；车端 Ubuntu 18.04 / ROS Melodic 成功构建 `libcym_planner.so`，恢复调度 gtest `2/2` 通过。

## 已知限制

- 尚未在车端实际堵塞场景验证动态重配置调用和每阶段重新规划时序。
- 车端最后一次哈希/残留进程核对因车辆随后断电而中断；本轮未启动 ROS、任务或车辆运动。
- 车端构建前必须确认 `dynamic_reconfigure` 已安装，且构建完成后恢复原有 catkin 白名单；本轮已确认依赖存在且白名单恢复为 `usb_cam`。
- 阶段成功后半径保持当前较小值，下一次导航进程重启时恢复到 `0.224m`。

## 2026-08-19 后续修订：按栅格分离恢复步长并取消旋转恢复

实车日志显示 local 分辨率为 `0.02m`、global 分辨率为 `0.01185m`，而旧版本让两套地图
每阶段都下降 `0.01m`，会造成 global 栅格几乎每阶段变化、local 栅格约每两阶段变化。
现改为：local 每阶段下降 `0.020m`，global 每阶段下降 `0.005925m`，避免按相同米数驱动
两套不同分辨率地图；`obstacle_cost_threshold=1` 保持不变。

恢复列表同时移除 `rotate_recovery/RotateRecovery`，并将 `clearing_rotation_allowed` 设为
`false`。车载雷达为 360°，不需要通过原地旋转刷新视野；障碍层清除和后续 inflation recovery
仍保留。

本次验证：车端 `cym_planner` 构建通过，恢复调度 gtest `3/3` 通过；插件源码、测试和导航
参数已与本地 SHA-256 一致。当前运行中的 ROS 流程未重启，新参数将在下次启动导航时加载。
