# 2026-08-12：仿真 bridge 纳入克隆仓库

## 目的

将 WSL 侧 `sim_bridge.py` 及其运行说明直接纳入 `smartcar2026-simulation` 仓库，消除新电脑
还需从旧电脑或交付盘单独取得 bridge 的隐含前置条件。

## 涉及文件

- `../simulation/smartcar2026-simulation/bridge/sim_bridge.py`
- `../simulation/smartcar2026-simulation/bridge/README.md`
- `../simulation/smartcar2026-simulation/.gitignore`
- `../simulation/smartcar2026-simulation/README.md`
- `docs/new-computer-gui-simulation-mission.md`

## 验证

- 仿真仓库 `main` 已提交并推送 `df3422140cb3367c1f8ff9b10bacb2dcca658019`。
- WSL 部署副本已 `pull --ff-only` 到相同提交；其既有未跟踪 `Testing/` 已登记至该副本
  `.git/info/exclude`，未删除或上传。
- WSL 中 `python3 bridge/sim_bridge.py --help` 和 `python3 -m py_compile bridge/sim_bridge.py`
  通过，未启动 Gazebo、bridge 或 ROS 任务进程。
- Noetic `catkin_make -j2` 在 `cmake_check_build_system` 阶段阻塞于 WSL `p9_client_rpc` 文件系统
  调用，已只中断本次构建进程树并确认无残留；这不是 bridge 编译/语法错误，需在 WSL 文件系统
  恢复响应后重跑构建。

## 已知限制

bridge 仍需在仿真专用 `127.0.0.1:11312` ROS Master 与 `task3_prepare.launch` 就绪后启动；
小车通过可信局域网访问 WSL 的 `11313`，Windows 防火墙必须按运行指南限制为 `LocalSubnet`。
