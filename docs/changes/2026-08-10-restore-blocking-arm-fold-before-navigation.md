# 2026-08-10 Restore blocking arm fold before navigation after pickup

## Purpose

官方吸附路径在夹取完成后改为「先抬完臂、再启动导航」，避免收臂过程中的
机械臂被激光雷达误判为障碍物。

## Cause

`task3_execute.launch` 默认 `attachment_fallback_enabled=true`。`_pick()`
官方吸附路径在吸附确认后只**非阻塞发布**收臂轨迹
（`_publish_arm_target()`）就立即切换 `laser_avoidance` 导航模式并启动
`move_base`。机械臂还在从下探位折叠到携带位的过程中，车已经开始跑，
伸出的机械臂被激光扫描成障碍物，干扰路径规划与避障。
纯物理夹取路径一直使用阻塞 `_move_arm()`，无此问题。

## Changed files

- `src/car3/scripts/task3_pick_deliver.py`
- `src/car3/test/test_task3_carry_sequence.py`（由
  `test_task3_concurrent_carry.py` 重命名并改写为顺序断言）
- `src/car3/test/test_task3_pick_sequence.py`
- `src/car3/CMakeLists.txt`（测试注册名）
- `DEPLOYMENT.md`、`FAQ.md`、`TASK3_RUNBOOK.md`

## Change

官方吸附路径的收臂从非阻塞 `_publish_arm_target()` 改为阻塞
`_move_arm(self.arm_carry, self.arm_carry_duration)`，等待携带位轨迹走完后
才 `_set_navigation_mode("laser_avoidance", ...)` 并发布 `carry_mode=True`。
两条夹取路径现在都是「抬完臂再跑」。状态文案同步更新为
「吸附完成；先恢复携带姿势，再开始底盘导航」。

## Verification

- `python -m py_compile` 全部改动脚本。
- WSL `catkin_make`/构建后跑 `task3_carry_sequence`、`task3_pick_sequence`
  及全套 CTest。
- WSL 完整任务：日志顺序为「吸附完成；先恢复携带姿势，再开始底盘导航」
  出现在 `Navigating to ...加工车间` 之前；抬臂期间车不移动。

## Safety

抬臂阶段车保持静止，仅多等待约 `arm_carry_duration + 0.2` 秒仿真时间；
不影响识别、吸附与最终泊车验收逻辑。
