# 2026-07-28：修复 Melodic Python 2 与 tf2 ABI 冲突

## 目的

修复小车 `navigation_scan_relay` 持续报 `init_tf2`、退出并重启，导致 `/scan`、
`map` TF 和 `move_base` 无法就绪的问题。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/run_melodic_python2.sh`
  - 清除继承环境中的 Python 3 模块目录，固定使用 `/usr/bin/python2`。
- `ucar_ws/src/ucar_2026/scripts/navigation_scan_relay.py`
  - 忽略 roslaunch 关闭发布者与最后一帧回调重叠产生的退出异常。
- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
  - 启动前检查 Melodic Python 2 的 `rospy`、`tf` 和 `LaserScan` 导入。
- `ucar_ws/src/ucar_2026/launch/2026.launch`
  - relay 节点经专用 Python 2 启动器执行。
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - 完整任务导航节点使用同一启动器。
- `ucar_ws/src/ucar_2026/CMakeLists.txt`
  - 安装新增启动器。
- `docs/quickstart.md`、`docs/operations.md`、`犯错档案.md`
  - 补充启动保护、重建命令和错误记录。

## 验证

- 本地两个 Shell 脚本的 `bash -n` 与两个 launch 文件的 XML 解析通过；部署文件
  SHA-256 与本地逐项一致，脚本权限为 `0755`。
- 原始受污染 `PYTHONPATH` 稳定复现 `init_tf2`；同一环境经过专用启动器后成功导入
  `rospy`、`tf` 和 `LaserScan`，实际路径仅包含 Python 2。
- 车端确认清除 `ucar_ws/devel/lib/python3`，Catkin 缓存为
  `PYTHON_EXECUTABLE=/usr/bin/python2` 和
  `PYTHON_INSTALL_DIR=lib/python2.7/dist-packages`；4 个白名单包重建成功。
- manual 模式中 relay 持续在线并发布 `/scan`，实际进程为
  `/usr/bin/python2.7`，进程环境不含 Python 3；未再出现 `init_tf2`、ImportError
  或 respawn。
- `/odom_raw` 为有限零速度值，`map -> base_link` 有效，本地 RViz 成功连接。
- 随后的完整安全门因独立的 AHRS/IMU `head_len`、CRC 错误按规则中止；已发布零速度，
  车端任务、RViz 和 Master 全部停止，两端无 ROS 进程残留。

## 已知限制

- Python 3 `cv_bridge_ws` 仍供其专用节点使用；本修复只隔离 Melodic Python 2
  导航进程，不删除该独立工作空间。
