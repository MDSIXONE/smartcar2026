# 部署当前 CymPlanner 到真机

## 目的

从 GitHub `simulation_real` 分支覆盖本地工作区后，将提交 `cd166d0` 的当前
CymPlanner、正式 `ucar_2026` 入口及其导航依赖部署到小车，并在不启动底盘的前提下完成
构建、测试和静态启动检查。

## 涉及文件

车端同步内容：

- `ucar_ws/src/cym_planner/`
- `ucar_ws/src/ucar_2026/`
- `ucar_ws/src/ucar_nav/config/testnav20260721/`
- `ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch`
- `ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.{yaml,pgm}`

本地记录与命令修正：

- `犯错档案.md`
- `docs/operations.md`
- `docs/changes/2026-07-28-deploy-current-cymplanner-to-vehicle.md`

## 网络与部署

- 控制电脑当前物理 WLAN / WSL Master 地址：`192.168.8.199`
- 小车当前地址：`192.168.8.231`
- 小车身份：`ucar-mini`，Ubuntu 18.04.6 LTS
- 车端所有 ROS 环境均显式使用
  `ROS_MASTER_URI=http://192.168.8.199:11311` 与 `ROS_IP=192.168.8.231`。
- WSL 镜像 WLAN 接口已恢复当前地址 `192.168.8.199/24`。
- 未在小车上启动 `roscore`，未创建备份目录或归档文件。

## 验证结果

- 27 个部署文件逐一通过 SHA-256 校验，车端与本地内容一致。
- Catkin 白名单为
  `cym_planner;jie_ware;yolo2025;ucar_2026`，完整构建通过。
- `catkin_make run_tests_cym_planner` 与 `catkin_test_results` 通过：
  `14 tests, 0 errors, 0 failures, 0 skipped`。
- `rospack find` 能发现 `cym_planner` 与 `ucar_2026`。
- `libcym_planner.so` 的动态依赖检查无 `not found`。
- `navigation_scan_relay.py` 与 `stop_2026_task.sh` 的车端权限均为 `0755`。
- `roslaunch --nodes ucar_2026 2026.launch` 静态展开成功，包含底盘、雷达、定位、
  `move_base` 和 scan relay 等预期节点。
- 部署完成后独立检查确认没有残留 `roslaunch`、`roscore`、`rosmaster`、`move_base`
  或 `base_driver` 进程。
- WSL 中一个从前一日遗留、仅绑定 `127.0.0.1:11311` 且没有业务节点的旧 Master 已停止；
  本轮没有留下 ROS 启动进程。

## 已知限制

- 本次没有启动新的 ROS Master、导航或底盘，也没有发送任何运动命令。实时测试前需运行
  `~/start_ros_master.sh`，让唯一 Master 使用当前 `192.168.8.199` 地址。
- 尚未进行实车运动验收。首次运动前必须确认 `/odom_raw` 为有限值，
  `odom -> base_link` 与 `map -> base_link` TF 均正常，`/scan_filtered` 新鲜有效，
  且 `/cmd_vel` 只有预期发布者。
