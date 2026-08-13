# Local 最大净空弹性带与全局动态重规划

## 目的

真车在 52 → 3 的门洞入口出现“move_base 仍为 ACTIVE、`/cmd_vel` 为零”的停滞：全局路径
按点代价能贴近静态墙角，局部完整 footprint 却必须拒绝。将局部策略改为在接触前选择最大
净空带；若本地 1 × 1 m 范围无更安全带，则立即交回持续更新的全局动态代价地图重规划。

## 涉及文件

- `ucar_ws/src/cym_planner/include/cym_planner/local_elastic_path.h`
- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/config/README.md`
- `ucar_ws/src/cym_planner/test/local_elastic_path_test.cpp`
- `ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_common.yaml`
- `ucar_ws/src/ucar_2026/scripts/navigation_scan_relay.py`
- `docs/operations.md`

## 实现

- 保持既有全局动态链路：`navigation_scan_relay` 从 `/scan_raw` 生成
  `/scan_global_obstacles`，先滤除静态地图已有墙体；global ObstacleLayer 使用
  `marking: true`、`clearing: true`、`observation_persistence: 0.0`。因此新障碍会写入
  global costmap，随后无回波的有效射线会清除它，不建立会累积陈旧格的第二张全局图。
- footprint 采样现统计每个安全候选的最高 raw 代价及平均 raw 代价。候选先最小化最高值，再
  最小化平均值，最后才最小化横移量；254/255、越界、TF/地图异常仍一律淘汰。
- 新参数 `elastic_activation_cost: 220` 在真实接触前激活局部带。最大横移默认调为 `0.12 m`，
  仍受 1 × 1 m local costmap、完整 footprint、姿态插值和 0.40 s command sweep 约束。
- 前视高代价且没有低于阈值的带时，立即返回 `false` 给 move_base。全局规划器随后以 3 Hz
  读取进程内、持续 marking/clearing 的 global costmap 重规划；不修改 StaticLayer，也不把
  局部栅格直接复制到全局图。

## 验证

- `local_elastic_path_test` 新增最高代价优先、平均代价次之、最小偏移最后的排序回归测试，
  以及空评分以 0 作为 `max()` 累积恒等元的回归断言。
- 静态地图回放确认从 52 到 3 的直线斜向矩形车体会在门洞左角相交；这证明纯点中心路径
  不能作为 footprint 安全性的验收依据。
- 在小车 Ubuntu 18.04 / Melodic 构建 `cym_planner` 并运行 `run_tests_cym_planner`：
  `68 tests, 0 errors, 0 failures, 0 skipped`。整个阶段未启动 ROS 或车辆。

## 已知限制

- local costmap 只提供合并后的 raw cost，不能可靠地区分静态墙和动态激光来源；没有本地安全带
  时均交由全局代价图重新规划。
- 若全局静态地图本身不存在可通行走廊，或 1 × 1 m 内没有安全带，任务会保持失败关闭而非
  以横移强行通过；需要调整目标/地图后再试。
