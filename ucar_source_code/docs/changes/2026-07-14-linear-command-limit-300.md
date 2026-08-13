# 2026-07-14：线速度命令上限 300

## 目的

按要求将导航线速度命令上限及底盘驱动的线速度限幅统一设为 `300.0 m/s`，并同步此前尚未上传的小型化 `2026.py` 与对应 launch 文件。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - `max_vel_x: 210.0` 改为 `300.0`。
- `ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`
  - `linear_speed_max: 150.0` 改为 `300.0`。
- `ucar_ws/src/yolo2025/scripts/2026.py`
  - 同步此前完成的最小导航节点；不含导航速度中继。
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - 同步此前删除 `run_navigation_test` 参数后的启动文件。
- `ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch`
  - 同步 move_base 直接发布 `/cmd_vel` 的启动文件。
- `docs/operations.md`
  - 更新运行时参数预期值。

## 验证

- 本地已通过 YAML 解析、`2026.py` Python 语法检查和两个 launch XML 解析。
- 已上传至小车，并对五个运行文件逐个完成 SHA-256 一致性校验。
- 未执行 `roslaunch`、未重启 ROS、未发送导航目标；运行时参数将在下次手动启动后加载。

## 已知限制

- `300.0 m/s` 仅为 ROS 命令与软件限幅，不代表电机可达速度。
- 角速度相关参数保持 `20.0 / 16.0 / 31.4 rad/s`，未将线速度单位错误应用到角速度。
