# 增加受限 footprint 横移脱困

## 目的

- 解决小车到达外圈角点后，全局路径存在但局部规划器因惯性/到达误差使未来车体
  footprint 擦到墙体致命格，随后直接进入失败或旋转恢复的问题。
- 保留真实 `0.342 m × 0.256 m` footprint 和致命格检查，不缩小车体、不切换点碰撞。
- 允许全向底盘在保持当前航向的前提下，向接触侧反方向做极小平移，再等待重规划。

## 涉及文件

- `ucar_ws/src/cym_planner/include/cym_planner/escape_recovery.h`
- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/config/README.md`
- `ucar_ws/src/cym_planner/test/escape_recovery_test.cpp`
- `ucar_ws/src/cym_planner/CMakeLists.txt`
- `docs/operations.md`
- `犯错档案.md`

## 行为

- 触发条件是 `body_projection` 的局部前视 footprint 连续命中可定位的致命格
  `0.4 s`，不是全局 `/make_plan` 超时。
- 等待期间本地规划器返回合法的全零 Twist，阻止 move_base 过早进入旋转恢复。
- 把致命格转换到 `base_link`，用真实 footprint 半长/半宽归一化后判断接触侧：
  `front`、`rear`、`left` 或 `right`。
- 只输出接触侧反方向的单轴 `linear.x` 或 `linear.y`，速度 `0.04 m/s`；
  `angular.z` 始终为零。
- 单步最大 `0.02 m`。每步结束后输出零速度并等待 `0.4 s` 重规划。
- 2 cm 平移不是膨胀车体：航向不变、车体尺寸不变，沿平移轨迹每 `5 mm`
  投影一次真实 footprint。中途接触不得变差，终点必须完全安全或严格减少致命格。
- 单个连续阻挡事件最多 4 步、累计不超过 `0.08 m`；航向漂移超过 `0.05 rad`、
  位姿不可用、未知区、地图越界、候选轨迹不改善或预算耗尽时返回原有控制失败。

## 验证

- 先写点 1 实际致命格 `(-2.485, 1.215)` 回放测试；在车端显式构建测试目标时，
  先因 `escape_recovery.h` 不存在而失败，确认红灯有效。
- 修正 Melodic `Costmap2DROS::getRobotPose(PoseStamped&)` 接口后，车端
  Ubuntu 18.04 成功构建 `cym_planner`。
- `catkin_make run_tests_cym_planner` 与 `catkin_test_results --verbose` 通过：
  汇总 40 项、0 error、0 failure、0 skipped；新增 EscapeRecovery 6/6 通过。
- 新测试覆盖点 1 接触方向、侧向接触、无效几何、终点严格改善、5 mm 中间投影
  不恶化，以及 4 次/8 cm 双重硬上限。
- 未启动 ROS、move_base 或生产任务，未向 `/cmd_vel` 发布消息，未执行实车运动。

## 已知限制

- Sol 最终安全复核发现首版横移有 3 个 P1：活动步可能被临时路径清空提前重置、
  footprint 只统计第一个接触格、横移预览没有检查本地动态障碍。为避免该版本被
  使用，后续改动已新增 `escape_enabled`，源码默认值和 YAML 均为 `false`。
- 在完成上述三项修复和相应集成测试前，不得把 `escape_enabled` 改为 `true`。
- 单元测试验证方向、预算和投影判据，但真实底盘的 2 cm 位移控制仍需用户明确确认后
  进行低速实车验证。
- 该恢复只处理能从全局致命格确定接触方向的 `body_projection` 阻挡；未知区和地图
  越界继续失败关闭。
- 达到 4 步/8 cm 上限后不会无限继续，任务仍按原有 move_base 失败路径安全终止。
