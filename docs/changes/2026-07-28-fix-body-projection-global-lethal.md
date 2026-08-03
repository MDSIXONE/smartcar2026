# 修正车体投影的代价地图与接触阈值

## 目的

实车正式任务第一次运行在 `52 → 12` 阶段中止。定点诊断确认旧实现把车体
footprint 投影到局部代价地图，并把代价 253 的内切膨胀格当成碰撞；命中位置为
`(-1.425, 1.545)`，并未接触 254 致命障碍格。

按任务语义修正为：

- 车体投影读取 `/move_base/global_costmap/costmap`；
- OccupancyGrid 99（Costmap2D 原始 253）不算实际接触；
- OccupancyGrid 100（原始 254）判为实际接触；
- 未知区继续安全拒绝；
- 控制律、速度、目标点、前视距离和 footprint 尺寸不变。

## 涉及文件

- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/include/cym_planner/global_cost_semantics.h`
- `ucar_ws/src/cym_planner/test/global_cost_semantics_test.cpp`
- `ucar_ws/src/cym_planner/CMakeLists.txt`
- `ucar_ws/src/cym_planner/package.xml`
- `ucar_ws/src/cym_planner/config/README.md`
- `ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_params.yaml`
- `docs/quickstart.md`
- `docs/operations.md`
- `犯错档案.md`

全局代价地图发布频率由 1 Hz 提高到 5 Hz。订阅者和发布者都在小车上的
`move_base` 进程内/本机，不依赖 RViz，也不会把完整地图持续发送到 WSL。

## 验证

- 修正前实车复现两次，均在局部代价 253 处挡停。
- 车端 Ubuntu 18.04 编译 `cym_planner` 成功。
- `catkin_make run_tests_cym_planner` 成功。
- `catkin_test_results build/test_results/cym_planner`：
  `28 tests, 0 errors, 0 failures, 0 skipped`。
- 新增回归测试确认：
  - 99/原始 253 不代表物理接触；
  - 100/原始 254 代表致命障碍；
  - 未知格保持安全阻塞。
- 修正后的最小实车复测不再被原 253 格挡停，并能报告全局 100/原始 254
  的具体接触位置。
- 重新冷启动并把车放回起点后，完整正式任务实车通过：
  - 三个二维码依次识别为 `…/a`、`…/d`、`…/i`；
  - 12、24、16、28、19 均到达并完成 360°；
  - 各点转后位置误差最大 `0.055 m`；
  - 170 到达误差 `0.018 m`，朝向 319 校验通过；
  - 状态 `SUCCEEDED`，结果 `success: true`；
  - 全程无 CRC、`head_len`、NaN、TF_NAN 或串口掉线。

## 已知限制

- 最小复测期间操作者移动小车曾导致定位坐标变化并使 CP2102 暂时掉线；该轮没有
  用作验收证据。最终结果来自设备重新枚举、车辆放回起点、静态安全门重新通过后的
  完整冷启动运行。
- 全局 OccupancyGrid 以公开消息的 99/100/-1 表示 Costmap2D 的
  253/254/255；插件没有绕过 ROS 消息语义读取内部全局 LayeredCostmap。
