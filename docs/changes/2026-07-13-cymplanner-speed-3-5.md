# CymPlanner 最大线速度调整为 3.5 m/s

日期：2026-07-13

## 改动

- 将 `cym_planner/CymPlanner/max_vel_x` 从 `1.7 m/s` 调为 `3.5 m/s`；后续调整记录见 `2026-07-13-cymplanner-speed-7.md`。
- `2026.py` 保持速度中继系数 `0.5`，底盘最大前向指令约为 `1.75 m/s`。

## 涉及文件

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/config/README.md`

## 操作

- YAML 已同步到小车端后，重启 `roslaunch yolo2025 2026.launch` 才会生效。
- 当前 launch 默认发送一次启动目标；高速度测试前确认路径、人员和障碍物均已清空。

## 验证

- 已在小车端读取配置，确认 `max_vel_x: 3.5`。
- 变更前本地备份：`back/2026-07-13-cymplanner-speed-before-3-5.tar.gz`。
