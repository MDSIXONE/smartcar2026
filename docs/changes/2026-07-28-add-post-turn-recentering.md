# 旋转后自动回中

## 目的

生产任务在点 16 完成 360° 旋转后出现 `0.102 m` 漂移，略超 `0.100 m`
位置告警阈值。全局路径仍存在，因此增加受控回中恢复；若恢复失败则记录告警并继续，
避免因微小漂移导致整场任务无法完成。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `docs/operations.md`
- `docs/quickstart.md`
- `犯错档案.md`

## 行为

- 每个生产点完成旋转后测量当前位置到该点中心的平面误差。
- 误差大于 `post_turn_recenter_trigger=0.06 m` 时，保持原定的下一点朝向，
  使用 `move_base` 重新导航到同一中心。
- 最多执行 `post_turn_recenter_attempts=2` 次。
- 每次回中都执行既有里程计、TF、全局路径和 action 安全检查。
- 回中无路径、action 失败或结束后仍超过 `arrival_tolerance=0.10 m` 时，输出
  `PRODUCTION_TASK_*_WARNING` 并继续后续任务。
- NaN、TF 失效和传感器掉线仍按原有硬安全门停车终止。

## 验证

- 本机 7 项纯 Python 测试通过，2 项 ROS 策略测试因本机没有 ROS Python 模块而按预期跳过。
- `production_task_2026.py` 与 `production_task_geometry.py` 语法检查通过。
- `2026.launch` XML 解析通过。
- 车端 Ubuntu 18.04 Python 2 单元测试通过：9 项、0 错误、0 失败；
  其中明确覆盖“转后超差只告警并继续”和“正常严格位置检查仍终止”。
- 车端 Catkin 白名单构建成功，`run_tests_ucar_2026` 汇总为
  9 项、0 错误、0 失败。
- `roslaunch --nodes` 确认任务节点存在；`--dump-params` 确认车端展开参数为
  `post_turn_recenter_trigger: 0.06` 和 `post_turn_recenter_attempts: 2`。
- 验证过程没有启动 ROS Master、导航节点或底盘运动。

## 已知限制

- 本次只做静态、零运动验证，不会自动启动正式任务。
- 真正的回中位移与连续任务结果仍需在用户确认场地安全后进行一次实车任务验证。
- 非致命策略只适用于旋转后的回中阶段；正常点位导航和最终点导航仍保留原有 action
  失败处理。
