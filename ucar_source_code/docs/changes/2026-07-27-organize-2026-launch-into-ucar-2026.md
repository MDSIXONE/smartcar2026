# 2026 启动入口迁移到 `ucar_2026`

## 目的

将 2026 真机导航入口和专用资源从视觉包 `yolo2025` 中分离，明确单一正式位置，避免后续部署、启动和排障继续依赖旧目录。

## 正式结构与兼容策略

- 正式 ROS 包：`ucar_ws/src/ucar_2026/`
- 正式入口：`roslaunch ucar_2026 2026.launch`
- 正式资源：
  - `ucar_2026/launch/2026.launch`
  - `ucar_2026/scripts/navigation_scan_relay.py`
  - `ucar_2026/scripts/stop_2026_task.sh`
  - `ucar_2026/urdf/ucar_2026_visual.urdf`
  - `ucar_2026/rviz/navigation_2026.rviz`
- `yolo2025/launch/2026.launch` 暂时保留为兼容 wrapper；旧启动命令仍可转发，但新文档和新部署统一使用 `ucar_2026`。
- `yolo2025` 不再保存上述 2026 专用脚本、URDF 和 RViz 资源。
- `yolo2025/scripts/2026.py` 未迁移、未删除；它是历史自动目标、二维码和生产路线任务脚本，
  不属于新的正式入口。

## 涉及文件

- `README.md`：补充正式包职责和动态 ROS 网络约束。
- `docs/quickstart.md`：更新启动、RViz、停止命令及网络配置示例。
- `docs/operations.md`：更新构建、部署、启动、停止和 RViz 路径，并注明兼容入口。
- `ucar_ws/src/cym_planner/config/README.md`：将当前参数重载入口改为正式包命令。
- `ucar_ws/src/ucar_2026/`：新增正式 Catkin 包及 launch、脚本、URDF、RViz 资源。
- `ucar_ws/src/yolo2025/launch/2026.launch`：缩减为兼容 wrapper。
- `ucar_ws/src/yolo2025/CMakeLists.txt`：移除已迁移 Python 节点的旧安装引用。
- 删除 `yolo2025` 下已迁移的 scan relay、停止脚本、URDF 和 RViz 文件。

## ROS 网络约束

- 控制电脑 WSL Ubuntu 20.04 是唯一 ROS Master，小车端不得启动 `roscore`。
- 地址按 `rosmaster/NETWORK_CONFIGURATION.md` 动态发现或显式配置，不写死旧 Wi-Fi IP。
- 小车终端必须在所有 `source` 完成后设置当前 `ROS_MASTER_URI`。
- 小车 `ROS_IP` 由到 Master 的路由动态推导，或按当前网络显式配置。

## 验证结果

- 已完成文档静态检查：推荐入口、RViz 路径和停止脚本路径统一指向 `ucar_2026`。
- `package.xml`、正式 launch、兼容 wrapper 和 URDF 均通过 XML 解析；停止脚本通过
  `bash -n`，scan relay 通过静态语法解析。
- 正式 launch 不再引用 `$(find yolo2025)`；旧四项专用资源已从 `yolo2025` 删除，
  `yolo2025/CMakeLists.txt` 也不再安装已删除的 relay。
- 迁移前后的 scan relay、URDF 和 RViz 文件内容保持一致；正式 launch 仅调整包内资源归属。
- 已核对正式 launch 只声明 `initial_pose_x/y/a`，不支持 `startup_goal_enabled`；文档中正式
  `ucar_2026` 启动命令均未传入该参数，也未将历史自动任务能力归给正式 launch。
- 已恢复 `yolo2025/scripts/2026.py` 的历史部署、权限和 Python 语法检查说明，并将相关旧命令
  明确标注为历史记录、当前不要执行。
- 当前 Windows/WSL 环境没有 ROS Melodic，因此未运行 `catkin_make` 或
  `roslaunch --nodes`，也未启动任何 ROS 节点或 Master。

## 已知限制

- 本次未连接小车，未部署资源，未启动 ROS 节点。
- 旧 wrapper 仅用于兼容，不应承载 2026 正式资源。
- `yolo2025/scripts/2026.py` 仍保留在旧包中但本次不迁移；其默认目标、二维码扫描和生产路线
  能力不属于 `ucar_2026/launch/2026.launch`，正式 launch 也不接受
  `startup_goal_enabled`。
- 仍需在小车 Ubuntu 18.04 / ROS Melodic 环境完成包发现、构建和 launch 静态展开；
  执行前必须连接 WSL 的唯一 ROS Master，且不得启动小车本机 `roscore`。
