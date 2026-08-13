# jie_ware 激光定位替换 AMCL

日期：2026-07-13

## 目的

将 2026 导航 launch 中的 AMCL 替换为 `6-robot/jie_ware` 的 `lidar_loc` 激光定位节点，并将 CymPlanner 最大 X 向速度设为 `1.0 m/s`。

## 改动

- 引入上游仓库 `https://github.com/6-robot/jie_ware` 至 `ucar_ws/src/jie_ware`。
- 补齐 `jie_ware` 对 `std_srvs`、`tf2`、`tf2_ros`、`tf2_geometry_msgs` 的显式构建和运行时依赖，并启用 C++11。
- 在 `2026.launch` 中用 `jie_ware/lidar_loc` 取代 `amcl_omni.launch`。
- 按 UCar 真机 TF 和话题配置定位器：`base_link`、`odom`、`laser_frame`、`scan`。
- 将 launch 的 `initial_pose_x/y/a` 传给 `lidar_loc`，并修复上游代码在地图加载后强制重置为 `(0,0,0)` 的行为。
- 移除 `2026.py` 对 AMCL `/request_nomotion_update` 服务的等待与调用；该服务在 AMCL 被替换后不再存在。
- 将 `cym_planner/CymPlanner/max_vel_x` 从 `0.2` 改为 `1.0`；后续调整记录见 `2026-07-13-cymplanner-speed-1-3.md`。
- 新增供控制电脑加载的 2026 RViz 预设，显示地图、TF、激光、全局/局部代价地图、全局路径和当前目标；小车端 launch 不启动 RViz。

## 涉及文件

- `ucar_ws/src/jie_ware/`
- `ucar_ws/src/yolo2025/launch/2026.launch`
- `ucar_ws/src/yolo2025/scripts/2026.py`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`

## 验证

- `2026.py` 已通过 Python AST 语法检查。
- `2026.launch` 已通过 XML 解析检查。
- 已确认 `lidar_loc` 所需的 UCar TF/话题分别为 `base_link`、`odom`、`laser_frame`、`/scan`。
- 已同步到车端 `/home/ucar/ucar_ws`，并成功构建 `jie_ware/lidar_loc` 与 CymPlanner。
- 车端已验证 `rospack find jie_ware`，且 `devel/lib/jie_ware/lidar_loc` 可执行。
- 车端 `roslaunch --nodes yolo2025 2026.launch` 启动图包含 `/lidar_loc`，不再包含 `/amcl`。
- 初始位姿修复已同步并在车端重新编译 `lidar_loc` 成功。

## 车端备份

部署前状态已备份至：

`/home/ucar/ucar_ws/.jie_ware_backup_before_lidar_loc_20260713/`

初始位姿修复前的文件备份：

`/home/ucar/ucar_ws/.lidar_loc_initial_pose_backup_20260713/`

## 操作与限制

- `lidar_loc` 启动时采用 launch 的 `initial_pose_x/y/a`，并可通过 RViz 的 **2D Pose Estimate** 在运行中重新定位。
- `2026.py` 仍将导航速度乘以 `0.5` 后发往底盘；当前速度上限说明见 `2026-07-13-cymplanner-speed-1-3.md`。
- `jie_ware` 的定位算法是确定性激光栅格匹配，不具备 AMCL 的粒子滤波恢复行为；首次测试应在空旷、已知初始位姿条件下进行。
