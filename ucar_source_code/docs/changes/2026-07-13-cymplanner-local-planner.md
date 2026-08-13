# CymPlanner 替换 UCar 局部规划器

日期：2026-07-13

## 目的

将 UCar 2026 导航栈的局部规划器从 TEB 替换为 `cym_planner/CymPlanner`，并用于 RViz 手动发送导航目标的低速验证。

## 改动

- 新增 `cym_planner` ROS 插件包至 `ucar_ws/src/cym_planner`。
- 删除 `cym_planner.cpp` 中全部 OpenCV 代价地图/路径绘制和窗口显示逻辑。
- 将 `base_local_planner` 改为 `cym_planner/CymPlanner`。
- 新增 `cym_move_base_omni_2026.launch`，加载既有 costmap 和全局规划器配置，但不加载 TEB 及 costmap converter 参数。
- 将局部代价地图 `width` 与 `height` 从 `2.0 m` 调为 `0.5 m`。
- 将局部代价地图 `inflation_radius` 从 `0.30 m` 调为 `0.15 m`，保留车体外的最小安全余量。
- 新增真机参数：`max_vel_x: 0.2`、`max_vel_theta: 0.5`。
- 删除 `2026.py` 对 TEB 参数的动态写入；默认改为 RViz 手动导航，避免启动时自动发送验证路点。

## 涉及文件

- `ucar_ws/src/cym_planner/`
- `ucar_ws/src/ucar_nav/config/omni_test20250620/move_base_params.yaml`
- `ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml`
- `ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch`
- `ucar_ws/src/yolo2025/launch/2026.launch`
- `ucar_ws/src/yolo2025/scripts/2026.py`

## 验证

- `2026.py` 已通过 Python AST 语法检查。
- 两个更新后的 launch 文件已通过 XML 解析检查。
- `cym_planner` 活跃源码中不再包含 `opencv` 或 `cv::` 引用。
- 已同步至小车 `/home/ucar/ucar_ws`，并成功执行 `catkin_make -DCATKIN_WHITELIST_PACKAGES=cym_planner`。
- 车端已生成 `devel/lib/libcym_planner.so`；`rospack plugins --attrib=plugin nav_core` 能发现 `cym_planner_plugin.xml`。
- 车端 `roslaunch --nodes yolo2025 2026.launch` 已成功展开全部预期节点，尚未实际启动底盘。

## 车端备份

部署前的原配置已备份到：

`/home/ucar/ucar_ws/.cymplanner_backup_before_cymplanner_20260713/`

## 限制

- `2026.py` 仍以 `0.5` 缩放 `/teb_cmd_vel`；因此规划器 `max_vel_x: 0.2` 对应底盘实际最大前进速度约 `0.1 m/s`。
- CymPlanner 当前遇到局部障碍物会请求 move_base 恢复/重规划，不提供 TEB 等价的动态局部绕障能力。
