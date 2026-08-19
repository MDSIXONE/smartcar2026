# 2026-08-19 导航到点偏差重试与继续流程

## 背景

实车日志先后出现：

```text
cym_planner: goal reached
PRODUCTION_TASK_ABORTED ... stopped 0.056 m from target (limit 0.030 m)
PRODUCTION_TASK_ABORTED ... stopped 0.094 m from target (limit 0.080 m)
```

这不是地图点号错误，而是 `move_base` 已返回成功后，任务层再次用 `map -> base_link` 复核位置时超出任务层容差。旧实现把这一类可恢复偏差直接转换成 `MissionAbort`，并由总异常处理调用 `stop_everything()`。

## 改动

- 三套 2026 主流程的 `navigate_coordinates()` 增加 `navigation_arrival_retry_attempts=3`；action 成功但复核超限时重发同一目标。
- `continue_on_arrival_error=true`：4 次 action 都完成但复核仍超限时只告警并返回成功，继续后续比赛流程。
- 重试和最终继续分支不调用任务层 `stop_motion()`；正常 action 成功分支也不再额外发送零速突发。
- 三套入口的任务层 `arrival_tolerance` 统一为 `0.12m`，`post_turn_recenter_trigger=0.06m` 仍满足严格小于关系。
- CymPlanner 实车库重新编译；规划器内部位置进入阈值为 `0.08m`，点/冲刺最终航向容差为 `0.10rad`。

## 保留的保护

NaN、TF、雷达、底盘传感器、通信、目标守卫、action 超时和 OCR/处理阶段的设计性停车/中止不属于本次可恢复偏差分支，仍然保留。当前“不中止”针对的是 `move_base` action 已成功、但任务层位置复核偏差这一种情况。

## 部署与验证

- 同步到 `ucar-mini`：三套任务脚本、三套 `2026.launch`、CymPlanner 源码和 YAML，共 8 个运行文件。
- 车端 `cym_planner` 白名单构建成功，之后恢复原 `CATKIN_WHITELIST_PACKAGES=usb_cam`。
- 三套 Python2 源码编译、三套 launch XML、本地 AST/参数契约均通过。
- 车端正式 `start_2026.sh 192.168.8.152 manual` 已启动；ROS Master、`lidar_loc`、`move_base` 正常，当前不自动执行比赛任务。

## 复测要求

车辆物理复位到起点、确认 `/odom_raw`、两个 TF、`/scan` 有限且新鲜后，再用正式 mission 入口开始比赛。观察日志中的 `PRODUCTION_TASK_ARRIVAL_RETRY` 和 `PRODUCTION_TASK_ARRIVAL_CONTINUE`；若再次看到旧格式的 `PRODUCTION_TASK_ABORTED ... stopped ... from target`，先检查任务脚本哈希和实际 launch 是否已重启。
