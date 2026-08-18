# 国赛 70→坡顶独立冲刺速度调试入口

## 目的

新增一个不进入二维码、OCR、生产任务和巡线流程的独立调试程序。小车物理放在国赛
70 号点后，程序加载国赛地图和导航链路，切换 CymPlanner 的 `sprint` 模式，导航到
67 与 290 的中点坡顶 `(0.875, 1.75)`，停车并恢复 `point` 模式。

## 涉及文件

- `ucar_ws/src/ucar_2026_national/scripts/national_sprint_speed_debug.py`：独立 ROS 节点；从网格 JSON 读取 70、67、290 坐标，校验里程计/TF/起点位置，执行一次冲刺并记录请求速度。
- `ucar_ws/src/ucar_2026_national/launch/national_sprint_speed_debug.launch`：加载底盘、雷达定位、国赛地图、CymPlanner/move_base；默认 `run:=false`，通过参数覆盖冲刺速度。
- `ucar_ws/src/ucar_2026_national/CMakeLists.txt`：安装调试节点并注册静态测试。
- `ucar_ws/src/ucar_2026_national/test/test_national_sprint_speed_debug.py`：验证脚本可解析、点 70 和坡顶点存在。

## 70 点坐标约束

点 70 是国赛共享任务坐标，本调试入口不修改它。车端已恢复并保持原坐标
`(2.25, 1.75)`；程序运行时只读取车端现有网格 JSON。部署独立调试程序时禁止同步
`production_full_grid_all_numbered.json`，避免影响国赛主任务。

## 速度参数

本入口默认使用当前国赛冲刺参数：`sprint_linear_x_gain=13.5`、
`sprint_max_vel_x=2.7`、`sprint_angular_gain=5.0`、`sprint_max_vel_theta=0.80`。
一次试跑建议同时记录并调整 `sprint_linear_x_gain` 与 `sprint_max_vel_x`；CymPlanner
没有独立加速度参数，`linear_x_gain` 主要影响前向加速响应。

## 验证结果

- 本机 Python 静态单测：2 项通过。
- 本机 Python 编译检查：通过。
- launch XML 解析：通过。
- 本机未编译 ROS；车端 Ubuntu 18.04 / ROS Melodic 构建成功，Python2 定向测试 2 项通过，launch 静态节点解析通过；本轮未启动运动测试；已将车端 70 点共享网格恢复为 `(2.25, 1.75)` 并完成只读核对。

## 已知限制

- 程序统计的是 `/cmd_vel` 请求速度，不是轮速计测得的实际车速。
- 必须先停止其它国赛/标准/额外导航流程；本入口直接使用 `/cmd_vel`，不能与其它底盘控制链路并行。部署后已确认车端无相关残留进程。
- 若 `/odom_raw` 出现 NaN/Inf 或 TF 报错，程序会零速并退出；必须按操作文档重启底盘/定位链路并确认有限值恢复后才能再次试跑。
