# CymPlanner 收敛为原始控制器加车体检查模式

## 目的

纠正把“前视路径点扩大为前视车体”实现成第二套局部规划器的问题。当前实现以最初
CymPlanner 的直接巡线为唯一控制器，模式切换只改变碰撞检查面积。

## 涉及文件

- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/config/README.md`
- `ucar_ws/src/cym_planner/CMakeLists.txt`
- `ucar_ws/src/cym_planner/package.xml`
- `ucar_ws/src/ucar_2026/rviz/navigation_2026.rviz`
- `docs/quickstart.md`
- `docs/operations.md`
- `犯错档案.md`

## 行为

- 只有一套直接巡线 P+D 控制和一套 15 项参数。
- 默认 `point` 检查前视路径点。
- `body_projection` 在同一前视路径位置检查 local costmap 的真实 footprint 面积。
- 两种模式共用目标点、控制增益、速度上限、终点处理和重规划逻辑。
- `/ucar/navigation_mode` 只切换碰撞几何；保留旧模式名作为兼容别名。
- 原始 OpenCV Map/Plan 改为发布
  `/move_base/cym_planner/CymPlanner/debug_map` 和 `debug_plan`，供本地 RViz 显示。

## 验证结果

- WSL Ubuntu 20.04 / ROS Noetic：`catkin_make --pkg cym_planner` 通过。
- `catkin_make run_tests_cym_planner`：22 项测试，0 错误，0 失败。
- `ldd devel/lib/libcym_planner.so`：OpenCV 与 `cv_bridge` 均解析成功，无 `not found`；
  CymPlanner 只继承 `cv_bridge` 导出的 ROS 系统 OpenCV，避免车端 3.2/3.3 混合 ABI。
- 参数静态检查：未保留 `main_legacy_*`、scan、轨迹采样、刹车或投影步长参数。
- 已同步到 `ucar@192.168.8.231`，五个部署文件的 SHA-256 与本地完全一致。
- 小车 Ubuntu 18.04 / ROS Melodic：重新构建通过；22 项测试 0 错误、0 失败；
  `libcym_planner.so` 仅加载系统 OpenCV 3.2 和 Melodic `cv_bridge`。
- 无目标启动时，roslaunch 摘要正确显示
  `/move_base/cym_planner/CymPlanner` 下的 15 项参数。

## 已知限制

- CV 图像只在 move_base 已收到有效路径并调用局部规划器时刷新。
- 两次无目标启动均卡在车端 Python 2 `xmlrpclib` 等待 WSL Master 的
  `system.multicall` 响应；WSL 发送队列保留 275 字节而车端未收到。节点尚未启动，
  因而未完成插件加载、CV 话题和运动验证。普通新建 `rosparam/rosnode` 请求正常。
- 验收后已停止车端 roslaunch 和临时 WSL Master，两端均无 ROS 后台进程残留。
