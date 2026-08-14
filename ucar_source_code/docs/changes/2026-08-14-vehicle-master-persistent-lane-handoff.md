# 小车本机 Master 与常驻巡线交接

## 目的

将主流程 ROS Master 移回小车，电脑只保留仿真 HTTP bridge；并把终点巡线从“退出主
launch、释放串口/相机、再启动 lane_proto”改为常驻节点服务交接，消除进程重启造成的长停顿。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`、`stop_2026_task.sh`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/scripts/cmd_vel_owner.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/lane_proto/{launch/lane_proto.launch,scripts/lane_follow.py}`
- `rosmaster/NETWORK_CONFIGURATION.md`、`docs/operations.md`

## 实现

- `start_2026.sh <SIMULATION_HOST> mission` 按到电脑的路由动态选取小车 `ROS_IP`，
  启动并监管本机 `roscore`；launch 结束或被停止时清理它自己启动的 Master。电脑地址仅作为
  `simulation_host` 传给 HTTP bridge，绝不从 `ROS_MASTER_URI` 推导。
- lane_proto 用共享 `/usb_cam/image_raw`，不启动 `ucar_controller_simple`、不独占
  `/dev/ucar_video`；初始 `STANDBY`，由 `/lane_proto/set_active` 激活。
- 新增 `cmd_vel_owner`：导航写 `/cmd_vel/navigation`，巡线写 `/cmd_vel/lane`，该节点为
  唯一 `/cmd_vel` 发布者。控制源切换在同一锁内完成。
- 到终点后不插入停车命令或停车确认：激活巡线后立即切换 owner；巡线 `STOPPED` 后才关闭
  主 launch，避免共享相机提前释放。
- 巡线主流程要求 `/odom_raw` 数据有限且新鲜；无更新即清晰进入 `STOPPED`，不将其误判为
  “车辆静止”。
- 语音非 JSON 状态只输出 `[语音] 中文`，不再向 rosout 写会显示为 `\uXXXX` 的重复日志。

## 验证

- 本地：`python -m py_compile`（三个变更 Python 脚本）、两个 launch/package XML 解析、
  WSL `bash -n`（启动/停止脚本）和 `git diff --check` 均通过。
- 小车：已同步变更文件且 SHA-256 与本地一致；`python2 -m py_compile`、`bash -n`、
  `catkin_make -DCATKIN_WHITELIST_PACKAGES='ucar_2026;lane_proto'` 均通过。
- 小车回归：`catkin_make -DCATKIN_WHITELIST_PACKAGES='ucar_2026;lane_proto' run_tests`：
  **94 tests, 0 errors, 0 failures, 0 skipped**。没有为验证启动新 ROS、相机或运动命令。
- 部署时发现的旧版独立 `lane_proto` 经用户明确要求后，以其 roslaunch 父进程 SIGINT 正常
  退出；已确认无 `roscore`、`roslaunch`、`lane_follow`、任务或仲裁节点残留。
- 已在小车执行 `start_2026.sh 192.168.8.152 check`：本机 Master 正常显示为
  `http://192.168.8.231:11311`，退出后再次确认无相关 ROS 进程残留。

## 已知限制

- `handoff_lane.sh` 保留为旧实现记录，已不在主流程调用。
- lane_proto 原型内仍存在旧的里程计冻结阈值 `100000` 秒与速度×时间测距分支；主流程使用
  新鲜度硬门避免该分支处理失联里程计，但位置持续不变而消息仍新鲜的根因仍应在底盘里程计
  链路修复，不能视为静止的正常现象。
