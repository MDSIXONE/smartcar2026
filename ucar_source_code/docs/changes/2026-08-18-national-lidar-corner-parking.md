# 国赛终点雷达角落闭环停车

## 目的

国赛巡线视觉命中终点线后，不再通过终点导航或固定前进距离停车。巡线节点直接使用 `/scan` 拟合车体附近相邻两面墙，闭环控制 `linear.x`、`linear.y` 和小角度 `angular.z`，两面墙距离稳定在 `0.24～0.26m` 后停车并退出任务节点。

## 涉及文件

- `ucar_ws/src/lane_proto/scripts/lane_common.py`
- `ucar_ws/src/lane_proto/scripts/lane_follow.py`
- `ucar_ws/src/lane_proto/launch/lane_proto.launch`
- `ucar_ws/src/ucar_2026_national/launch/2026.launch`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
- 对应 lane/国赛测试文件

## 当前控制规则

- 两面墙各至少 8 个点，墙段跨度至少 `0.12m`，拟合残差不超过 `0.025m`，墙角误差不超过 `10°`。
- 目标距离 `0.25m`，容差 `±0.01m`，连续 5 帧满足后发布 `GOAL`。
- 偏航误差超过 `5°` 时先只做小角度航向修正，不同时平移。
- 任一墙距离超过 `1.0m` 时转入原来的 `PAUSE + APPROACH` 兜底流程。
- 雷达陈旧、两墙拟合失败持续到超时则发布 `ABORT`；主流程必须同时检查 `/lane_proto/result`，不能仅因 `STOPPED` 就报告成功。
- 四个角的名义朝向由墙符号核对：下方两角 `-90°`，左上 `0°`，右上 `180°`；闭环实际以当前墙法向为准。

## 验证结果

- 本机四组角落合成点云、4mm 噪声和 1.2m 远墙回归通过。
- `lane_common.py`、`lane_follow.py`、国赛任务 Python2 语法检查通过。
- 两份 launch XML 解析通过；lane 状态机和国赛任务离线回归通过（ROS 相关车端用例在本机按既有条件跳过）。
- 已动态确认车端 `ucar-mini` 当前地址为 `192.168.8.231`，同步 5 个运行文件；本地与车端 SHA-256 逐项一致。
- 车端 Ubuntu 18.04 / Python2 语法检查通过，`roslaunch --nodes ucar_2026_national 2026.launch task_enabled:=true` 展开通过。
- 部署后未重启主流程、未启动 ROS、未发送运动命令；车端当前保持停止。

## 已知限制

- 尚未在 Ubuntu 18.04 / ROS Melodic 小车上实测四个角落的雷达回波、安装偏移和四个路线映射。
- 车端试跑前必须检查 `/odom_raw`、`odom -> base_link`、`map -> base_link` 和 `/scan` 均为有限/新鲜数据，车辆处于零速。
