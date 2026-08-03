# 增加小白安全启动脚本

## 目的

避免用户手工复制多行 `source`、`export` 和 `awk` 时漏掉 `MASTER_IP`、破坏引号，
从而产生空 `ROS_IP` 或 `http://:11311`。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
- `ucar_ws/src/ucar_2026/CMakeLists.txt`
- `docs/quickstart.md`
- `docs/operations.md`
- `rosmaster/NETWORK_CONFIGURATION.md`
- `犯错档案.md`

## 行为

- 无参数运行时只提示输入一次 WSL Master 地址。
- 自动加载 Melodic 和车端工作区、清除 `ROS_HOSTNAME`、动态推导小车 `ROS_IP`。
- 拒绝空地址、`localhost` 和 `127.0.0.1`。
- 启动前检查 WSL Master 可达；默认只启动无自动目标的 `manual` 模式。
- 提供 `check` 模式用于只读网络验证，不启动导航节点。
- `full` 模式保留给安全门通过后的维护操作。

## 验证

- `bash -n` 语法检查、LF shebang、车端可执行权限、错误输入拒绝和 `check` 模式结果
  见本次操作记录。
- 首次验证发现 Melodic 的环境脚本不能在未定义 `ROS_DISTRO` 时承受调用方提前启用
  `set -u`；脚本现仅在两个 ROS 环境加载完成后启用未定义变量检查。
