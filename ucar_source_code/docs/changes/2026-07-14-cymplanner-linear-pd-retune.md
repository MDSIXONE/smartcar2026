# 2026-07-14：CymPlanner 线速度 P/D 整定

## 目的

按要求降低线速度比例增益并加入少量微分增益，测试是否减少命令饱和、突变或接近路径点时的抖动。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - `linear_x_gain: 1890.0` 改为 `1260.0`。
  - `linear_x_kd: 0.0` 改为 `30.0`。
- `docs/operations.md`
  - 更新当前 P/D 参数记录。

## 验证

- 本地已通过 YAML 解析，确认 P=`1260.0`、D=`30.0`。
- 已上传至小车，并完成远端 SHA-256 一致性校验。
- 运行时参数将在下次启动 `move_base` 后验证；本次未启动 ROS。

## 已知限制

- P/D 仅影响线速度控制曲线，不能解除代价地图、全局路径、搬运模式或底盘硬件限制。
- 当前 ROS Master 未运行，无法在本次修改前读取活跃参数。
