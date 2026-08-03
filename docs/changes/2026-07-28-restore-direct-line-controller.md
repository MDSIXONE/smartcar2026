# 恢复前段旧真车直接巡线控制

## 目的

恢复无车体投影阶段最初使用的真车直接巡线手感，同时保持
`laser_avoidance` 阶段现有的激光车体投影和控制参数不变。

## 涉及文件

- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/include/cym_planner/direct_line_control.h`
- `ucar_ws/src/cym_planner/test/direct_line_control_test.cpp`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/config/README.md`
- `ucar_ws/src/cym_planner/CMakeLists.txt`
- `docs/operations.md`
- `犯错档案.md`

## 控制边界

- `main_legacy`：无车体投影，恢复旧真车直接巡线。
  - 目标点距离：`0.20 m`
  - 直线控制：`P=1.5`、`D=0.5`
  - 角度控制：`P=2.5`、`D=0.4`
  - 不进行航向余弦降速
  - 终点进入 `0.05 m` 后只旋转对正
  - costmap 路径检查距离：`0.25 m`
- `laser_avoidance`：保持修改前实现。
  - 直接激光车体投影保持启用
  - `main_legacy_linear_x_kd=0.05`
  - 角度继续仅使用 P 项
  - 航向余弦降速保持启用
  - 投影距离与步长保持 `0.30 m / 0.03 m`

两种模式继续通过 `/ucar/navigation_mode` 切换。本次没有新增自动切换时机。

## 验证结果

- 新增直接巡线公式回归测试；实现前因缺少控制函数编译失败，完成实现后 4 项测试通过。
- WSL Ubuntu 20.04 / ROS Noetic 已成功编译 `libcym_planner.so`。
- WSL 全部 CymPlanner 测试通过：`22 tests, 0 errors, 0 failures, 0 skipped`。
- 7 个部署文件已同步到小车，SHA-256 与本地全部一致；小车未创建备份目录或归档。
- 小车 Ubuntu 18.04 / ROS Melodic / Python 2 已成功重建 `libcym_planner.so`，全部
  CymPlanner 测试同样为 `22 tests, 0 errors, 0 failures, 0 skipped`。
- 车端静态参数确认前段为 `linear_x_kd=0.5`、`angular_kd=0.4`、
  `obstacle_lookahead_distance=0.25`；后段保持 `main_legacy_linear_x_kd=0.05` 和
  `main_legacy_obstacle_lookahead_distance=0.30`。
- 车端动态库无缺失依赖，`roslaunch --nodes` 静态展开包含 `base_driver`、
  `navigation_scan_relay` 与 `move_base`；检查结束后没有残留 ROS 进程。

## 已知限制

- 本地测试不能替代真车低速运动验收。
- 当前任务脚本没有自动发布 `/ucar/navigation_mode`，后段切换必须由明确的任务阶段执行。
