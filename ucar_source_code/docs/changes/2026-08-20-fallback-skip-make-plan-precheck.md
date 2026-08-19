# fallback 导航跳过 make_plan 预检查

## 变更

三套 2026 主流程的目标守卫 fallback 导航改为：

```text
普通路径：require_plan=True  -> 先调用 /move_base/make_plan
fallback：require_plan=False -> 直接发送 MoveBaseGoal
```

fallback 仍保留独立的 `25s` 目标超时和失败后尝试下一个候选点的语义。这样不可达候选会进入
`move_base` action 状态机，CymPlanner/move_base 才有机会按恢复列表执行清除代价地图和逐级降低
膨胀半径；不会再停留在任务层反复调用 `make_plan` 服务的阶段。

446、447、448、449、450、451 均为墙点。本次只改变 fallback 的导航入口，并将这六个点从 fallback 导航候选中排除；不替换这些点为内侧可导航点。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py`
- 三套 `test/test_production_task_geometry.py`
- `docs/operations.md`

## 验证

- 标准、国赛、额外三套几何测试分别 `104`、`105`、`120` 项通过；按现有约定分别跳过 `88`、`89`、`104` 个 ROS 依赖用例。
- fallback 回归断言确认 `require_plan=False`；普通路径代码仍使用 `require_plan=True` 的默认分支。
- 墙点候选回归断言确认 446–451 不会进入 fallback 导航列表。
- 三套正式任务脚本和三套对应测试已同步到 `ucar-mini (192.168.8.231)`；车端 Python2 AST 通过，六个文件本地/车端 SHA-256 一致。同步后未重启 ROS、未启动任务、未发送运动指令。
