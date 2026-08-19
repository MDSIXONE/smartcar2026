# 2026-08-19：国赛 288 冲刺终点独立到达容差

## 目的

国赛主流程的冲刺段为 `70→288`，当前实际终点使用坡顶坐标 `(0.875, 1.75)`。
普通任务层到达容差保持 `0.12m`，为冲刺终点增加独立参数 `sprint_arrival_tolerance=0.20m`，
用于吸收高速冲刺后的定位/刹停误差。

## 改动

- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
  - 读取并校验 `sprint_arrival_tolerance`。
  - `70→288` 的终点导航单独传入 `0.20m`；70 起点和其他导航仍使用 `arrival_tolerance=0.12m`。
- `ucar_ws/src/ucar_2026_national/launch/2026.launch`
  - 新增 `<param name="sprint_arrival_tolerance" value="0.20"/>`。
- `ucar_ws/src/ucar_2026_national/test/test_national_sprint_speed_debug.py`
  - 锁定普通容差与冲刺终点容差为两个独立参数。

## 生效与验证

修改只涉及 Python2 脚本和 launch，不需要 catkin 编译；必须重启国赛主流程后生效。
重启前确认车辆零速、`/odom_raw` 有限、两个 TF 和 `/scan` 正常。
