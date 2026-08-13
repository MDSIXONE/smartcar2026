# 修复停止脚本加载 Melodic 前启用 nounset

## 目的

任务安全中止后执行 `stop_2026_task.sh`，脚本在
`/opt/ros/melodic/setup.bash` 中因 `ROS_DISTRO: unbound variable` 提前退出，
未能结束 roslaunch。

## 修改

- `ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh`
  - 初始严格模式保留 `set -e -o pipefail`；
  - 两次 ROS 环境加载完成后再启用 `set -u`。
- `犯错档案.md`
  - 记录复现、原因、修复和预防规则。

## 验证

- 车端 `bash -n` 通过。
- 在 WSL Master 存在、车端没有 2026 launch 的实际环境运行停止脚本：
  - 成功计算动态 `ROS_IP`；
  - 成功连接唯一 WSL Master并发布零速度；
  - 正确输出没有运行中的任务；
  - 未停止 WSL Master。

## 已知限制

- 停止脚本必须提供 `MASTER_IP` 或 `ROS_MASTER_URI`。
- 脚本只停止 `ucar_2026/yolo2025 2026.launch`，不会停止 WSL Master；
  Master 仍需在其启动终端正常按 `Ctrl-C`。
