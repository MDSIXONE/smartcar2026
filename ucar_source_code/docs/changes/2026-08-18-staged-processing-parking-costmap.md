# OCR 内墙停车分阶段局部膨胀（2026-08-18）

## 目的

解决正常轨迹规划需要较大局部膨胀距离、但 OCR 识别后的 `0.29m` 内墙停车目标又可能落入同一膨胀区的冲突。

## 改动

- 三套 2026 主流程新增 `processing_parking_profile_enabled` 和
  `processing_parking_inflation_radius_m`，默认停车膨胀半径为 `0.10m`。
- 正常路线保持当前局部膨胀配置；进入 OCR 内墙最终停车前，通过
  `dynamic_reconfigure` 读取并保存当前半径，再切换到 `0.10m`。
- 停车阶段显式保持车端已验证正常的 CymPlanner `point` 模式，只改变 local inflation；静态墙体和动态障碍层保持运行。
- 停车成功、失败或超时后均恢复原局部膨胀半径，并再次确认 `point` 模式；动态重配置或模式切换失败会明确中止任务。
- 配置要求停车膨胀半径小于 `ocr_stop_offset_m`，当前为 `0.10m < 0.29m`。
- 到达点 3 后，三套主流程通过 global costmap 的 inflation layer 将全局膨胀切换并保持为 `0.235m`；点 3 后 local costmap 常态膨胀也配置为 `0.235m`，断点续跑会重新应用全局值，OCR 停车阶段不修改 global inflation。

## 涉及文件

- 三套 `scripts/production_task_2026.py`
- 三套 `scripts/production_task_geometry.py`（OCR `0.29m` 停车坐标计算依赖）
- 三套 `launch/2026.launch`
- 三套 `test/test_production_task_geometry.py`
- `docs/operations.md`
- `docs/lingo.md`

## 验证

- 本机标准/国赛版几何回归：各 87 项通过，72 项按既有 ROS 条件跳过。
- 本机额外版几何回归：102 项通过，87 项按既有 ROS 条件跳过。
- 动态重配置模拟测试确认 `0.22m → 0.10m → 0.22m`，进入和退出均只发布 `point` 模式。
- 已动态确认车端 `ucar-mini`（`192.168.8.231`），同步 12 个任务脚本、geometry、launch 和测试文件，车端/本地 SHA-256 全部一致。
- 车端 Ubuntu/ROS 环境中 `dynamic_reconfigure` 导入、Python 2 语法检查通过；标准和国赛版车端各 87 项全量回归通过，额外版新增 profile 单测通过。
- 用户反馈后已撤销 `body_projection`：补充同步 6 个 point-only 修正脚本/测试文件；车端标准/国赛各 87 项回归和额外版 point-only profile 单测通过。
- 额外版全量回归仍有 1 个既有测试桩错误：`observe()` 不接受已有调用方传入的 `stop_mode`，与本次膨胀配置无关。
- 本轮追加验证点 3 后全局膨胀切换：三套 launch 明确传入 global inflation layer 和 `0.235m` 参数，同时将 `testnav20260721` local 分辨率改为 `0.02m`、常态半径改为 `0.235m`；车端标准版全量回归通过，三套新增全局切换定向单测通过，11 个源码/launch/YAML/测试文件 SHA-256 与本地一致。
- 未执行 catkin 编译（本次无 C++ 变更），未启动 ROS Master、主流程或发送车辆运动命令；车端已有单独 `move_base` 进程，未热重启或调用运行时参数服务，配置需下次安全重启后生效。

## 已知限制

- `0.10m` 和点 3 后 `0.235m` 都是现场验证前的配置；车端应在 `/odom_raw` 和 TF 均为有限值、车辆零速的安全条件下复测。
- 点 3 后 global/local 常态 inflation 均为 `0.235m`；OCR 停车只临时切换 local inflation，不能借此切换 CymPlanner 的 `body_projection` 模式。

## 后续参数调整（2026-08-18）

- 三套 2026 主流程共用的 `testnav20260721` local costmap 常态膨胀半径调整为 `0.224m`。
- OCR 内墙停车阶段的临时 local 膨胀半径调整为 `0.05m`；全局 costmap 的目标值调整为 `0.224m`，CymPlanner 的 `point` 模式保持不变。
- 三套 launch、任务脚本默认值和几何测试已同步；三套几何回归、Python AST、launch XML、YAML 和 `git diff --check` 均通过。未启动 ROS、不发送运动命令。
