# AI 改动记录

## 2026-08-12

- **状态**：改动完成
- **目标**：在桥接服务等待小车开始信号时降低 Gazebo 物理更新频率，并在任务前恢复原值。
- **影响文件**：`simulation/bridge/sim_bridge.py`、`simulation/bridge/README.md`、`simulation/bridge/test_sim_bridge.py`
- **结果**：bridge 启动后保存当前物理属性并将待机 `max_update_rate` 降为 100 Hz；`POST /start` 先恢复保存值，再启动任务。
- **验证**：Windows 与 WSL Ubuntu 20.04 均通过 `test_sim_bridge.py`；WSL 已验证 `rospy`、`GetPhysicsProperties`、`SetPhysicsProperties` 可导入。
- **风险**：尚未在正在运行的 Gazebo 实例上执行服务调用，现场首次启动应核对 bridge 日志中的保存与恢复频率。
