# 局部动态代价层延迟到点 3 启用（2026-08-18）

## 目的

前往二维码区域和点 3 的路线不使用实时雷达障碍层与局部膨胀层；车辆确认到达点 3 后，才让局部动态障碍进入 CymPlanner 的局部判障和重规划链路。

## 实现

- 标准、省赛/国赛、额外任务主流程在安全门通过、第一段运动开始前，通过 dynamic_reconfigure 将：
  - `/move_base/local_costmap/obstacle_layer`
  - `/move_base/local_costmap/inflation_layer`
  设置为 `enabled=false`。
- `navigate_coordinates(..., require_plan=True)` 成功返回点 3 后，按 `local_costmap_enable_waypoint_number=3` 校验并将两层同时设置为 `enabled=true`。
- `resume_production_only=true` 视为车辆已经完成点 3 腿，在恢复流程开始时直接启用两层。
- 保留 local costmap 容器和 StaticLayer 运行，不能关闭整个 local costmap；CymPlanner 仍需要 local costmap 的 current 状态才能输出速度。
- 额外任务的非空 `ocr_route_profile` 是独立的、不经过点 3 的快捷流程，因此不执行点 3 时序控制；默认空 profile 的主流程执行上述逻辑。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_national/test/test_production_task_geometry.py`（同步车端已有 409 语义回归断言）
- 三个包的 `package.xml`、`launch/2026.launch`
- `docs/operations.md`

## 验证

- Windows 本机已完成 Python AST、XML/依赖静态检查；不在本机编译 ROS。
- 待执行：同步到车端后，在 Ubuntu 18.04 / ROS Melodic 重启对应主流程，确认日志依次出现 `before_point_3 ... disabled`、点 3 导航成功、`reached_point_3 ... enabled`。
- 未在本次改动中启动 ROS、重启导航或发送车辆运动命令。

## 已知限制

- dynamic_reconfigure 服务必须由 move_base 的两个 local costmap 插件提供；若服务缺失，任务会明确 `MissionAbort`，不会继续以未知的局部障碍层状态运行。
- launch/2026.launch 的参数生效需要重新启动任务节点；当前运行中的 Python2 任务不会热加载源代码。
