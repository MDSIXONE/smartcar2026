# 2026-07-14：修复 ROS Melodic 终端启动环境

## 目的

修复小车终端出现 `Argument list too long` 以及普通 `roslaunch` 使用 Python 3 导致 ROS Melodic 启动失败的问题。

## 涉及文件

- 小车端 `/home/ucar/.bashrc`
  - 删除重复 source `ucar_ws` 与 `catkin_ws` 的行，每个 overlay 仅加载一次。
  - 为交互式 `roslaunch` 定义 Python 2 启动函数。
- `docs/operations.md`
  - 增加终端恢复、验证及 Python 2 备用启动命令。

## 验证

- 在无登录脚本的干净环境中，`source /opt/ros/melodic/setup.bash` 和 `source ~/ucar_ws/devel/setup.bash` 均成功。
- 以 Python 2 执行 `roslaunch --nodes yolo2025 2026.launch` 成功列出完整节点图，未启动任何节点。
- 已修改 `.bashrc` 并在新的交互 shell 中验证：`roslaunch` 为 Python 2 函数，环境变量长度保持正常，`roslaunch --nodes yolo2025 2026.launch` 成功列出完整节点图，未启动任何节点。

## 已知限制

- 已经报错的旧终端无法安全恢复，必须关闭并新开终端。
- ROS Melodic 仍依赖 Python 2；不修改系统 `/usr/bin/python` 的全局指向，避免影响其他软件。
