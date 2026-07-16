# 修复局部代价地图旋转拖影

## 目的

全局动态障碍过滤修复后，局部障碍层仍保留旧激光帧；小车旋转时旧帧按旧姿态
继续存在，局部栅格形成随车拖动的障碍带，CymPlanner 因此判断前方路径被封住。

## 改动

- `ucar_ws/src/ucar_nav/config/omni_test20250620/costmap_common_params.yaml`
  - 局部 `/scan` 观测的 `observation_persistence` 从 `0.6` 改为 `0.0`。
- `ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml`
  - `transform_tolerance` 从 `1.2 s` 改为 `0.30 s`，不接受旋转期间明显过期的
    机器人姿态。
  - 修正 `plugins` 的 YAML 缩进。此前它位于 `local_costmap` 命名空间之外，
    move_base 因而回退到旧版 pre-Hydro 代价地图逻辑。
  - 显式加载 `ObstacleLayer + InflationLayer`；局部滚动窗口不加载静态地图。
  - 局部坐标系从 `odom` 改为 `map`。实测 RViz 固定坐标系为 `map`，而
    `map -> odom` 存在动态姿态修正，导致旧配置中的局部栅格相对地图转动。
- `docs/operations.md`
  - 补充局部层实时观测策略和重启说明。

## 验证

- 本地 YAML 解析和 `git diff --check` 通过。
- 已上传到小车并以 `startup_goal_enabled:=false` 重启；未发送导航目标。
- 车端参数确认局部插件为 `ObstacleLayer + InflationLayer`，
  `observation_persistence=0.0`，`transform_tolerance=0.3`。
- 新启动日志不再出现
  `local_costmap: Parameter "plugins" not provided`。
- 修改前实测局部代价地图消息 `frame_id=odom`、RViz 固定坐标系为 `map`，且
  `map -> odom` yaw 约为 `40.6°`；重启并调用不移动小车的
  `/move_base/clear_costmaps` 后，局部代价地图消息已实测为 `frame_id=map`。

## 已知限制

此改动只移除局部层的旧帧拖影，不修正静态地图与激光之间的固定几何偏差。若实时
TF 长时间不可用，局部层会暂时丢弃观测而不是将其错误地写入代价地图。
