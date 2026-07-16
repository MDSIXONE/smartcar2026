# 2026-07-14：速度 ×5、控制增益 ×6

## 目的

按确认要求直接提高当前导航命令配置：所有实际参与控制的速度上限相对上一版提高 5 倍，所有非零实际控制增益提高 6 倍；不再执行定位操作。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - `max_vel_x: 210.0`、`max_vel_theta: 20.0`、`final_yaw_max_vel: 16.0`。
  - `linear_x_gain: 1890.0`、`angular_gain: 162.0`、`final_yaw_gain: 162.0`、`final_linear_x_gain: 18.0`。
- `ucar_ws/src/ucar_controller/config/driver_params_mini.yaml`
  - `linear_speed_max: 150.0`、`angular_speed_max: 31.4`。
- `docs/operations.md`
  - 更新部署后的运行时参数核验与本地回滚路径。

## 备份与小车端清理

- 部署前的两份小车端活动配置已下载到本机 `back/2026-07-14-speed-x5-gain-x6-before-deploy/` 并完成 SHA-256 校验。
- 同时迁移并校验了小车端遗留的 `20__backup.py`、`.rosinstall.bak` 和空的 `ucar_yolo/scripts/backup/` 目录，随后从小车清理。
- `xf_mic_asr_offline/tmp/system.tar` 虽为归档文件，但由该语音组件的 `config.txt` 和 `user_interface.h` 直接引用为运行时资源，故保留在小车端，不作为备份处理。

## 验证

- 本地 YAML 解析、参数数值断言与 launch XML 解析均通过。
- 两个配置文件已同步到小车，SHA-256 分别为 `de62304cd15757ca1e03684b97b9485faa02c24fbfd66fb70eb4f0e62645262e` 与 `b71c4caef111edb63661d699ea6b6be088e7cfc2ee2494eb189de0aa3a7dc113`。
- 小车端已重启 `roslaunch yolo2025 2026.launch startup_goal_enabled:=false`，运行时参数已核对为 `210.0`、`20.0`、`16.0`、`1890.0`、`162.0`、`162.0`、`18.0`、`150.0`、`31.4`。
- 运行话题图为 `/move_base → /cmd_vel → /base_driver`，`/teb_cmd_vel` 不存在，`/move_base/status` 为空；该重启没有自行发送导航目标。

## 已知限制

- 这些数值只是 ROS 命令上限；电机固件、PWM、电源、机械摩擦及底盘保护仍可能限制实际速度。
- `carry_speed_scale` 保持 `1.0`，因为 CymPlanner 源码将该任务模式比例钳制为不大于 `1.0`。
- 未在本次配置变更中执行物理导航，须在路径清空和急停可用时另行测试。
- `usb_cam_flip` 仍有既有的 Python/OpenCV 回调异常；它不订阅 `/cmd_vel`，不影响本次速度配置验证。
