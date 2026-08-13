# 本地仿真独立 Master 保活修正

## 目的

保证真车双物品任务触发本地仿真时，Gazebo、`map_server`、`move_base` 与桥接服务在独立
`127.0.0.1:11312` Master 上持续可用，不与真车使用的 WSL Master 混用。

## 现象与处理

- 一次性后台 WSL 子进程在调用结束后带走了仿真 `roscore`；prepare 进程因此退出，后续
  `task3_execute` 只剩任务节点，持续等待 `/gazebo/get_link_state` 或
  `/move_base/clear_costmaps`。
- 已验证在独立、前台保活的 WSL 终端先运行 `roscore -p 11312`，再启动
  `task3_prepare.launch gui:=false rviz:=false` 后，`/gazebo`、`/map_server`、`/move_base`、
  `/gazebo/get_link_state`、`/move_base/clear_costmaps` 与 `/map` 均稳定可用。
- `docs/operations.md` 已补全此启动顺序与就绪检查；真车仍仅连接
  `192.168.8.199:11311`，不启动小车端 `roscore`。

## 验证与限制

- 真车本轮已完成静态安全门、固定面扫码及食品/电子产品 OCR/停入；仿真后端失效时小车保持静止，
  已通过 `stop_2026_task.sh` 发布零速度并停止。
- 随后按修正顺序使用**有界面**的 `task3_prepare.launch gui:=true rviz:=true` 重跑：仿真 bridge
  成功返回 `done`，真车完成点 441 的 `SUCCEEDED`，并自动交接 lane_proto 至终点 `STOPPED`。
- 本次 QR 实测到 90°的非目标“毛巾”后直接推进 -90°；OCR 实测连续发布
  `PRODUCTION_OCR_ALIGNMENT_CONTINUOUS`，对准后才测距。
- 已停止本次车端任务、仿真 Master、Gazebo 与桥接进程，不留后台启动终端。
- 本次中止后车辆不再位于起点；重新跑完整流程前必须由用户重新放回起点。
