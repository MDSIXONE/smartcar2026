# 国赛任务重定位阈值与到点容差一致化（2026-08-19）

## 症状

国赛任务脚本已能正常导入，但构造阶段退出：

```text
TaskDefinitionError: post_turn_recenter_trigger must be positive and smaller than arrival_tolerance
```

## 根因

三套主流程同时配置了：

- `arrival_tolerance=0.03m`
- `post_turn_recenter_trigger=0.06m`

任务层要求重定位触发阈值必须严格小于最终到点验收阈值，因此该配置在启动校验阶段必然失败。

## 修复与验证

- 三套 `2026.launch` 将 `post_turn_recenter_trigger` 统一为 `0.02m`。
- 三套实际 `scripts/production_task_2026.py` 默认值同步为 `0.02m`。
- 本地三套 XML 参数不变量检查通过。
- 车端三套任务脚本 Python2 编译通过。
- 车端国赛 `roslaunch --nodes` 正常列出 `/production_task_2026`。

运行中的旧 launch 不会重新读取参数；重启后必须确认任务日志不再出现该 `TaskDefinitionError`，并继续进行车辆零速、odom、TF 和雷达检查。
