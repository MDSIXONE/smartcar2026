# 动态到达前目标守卫

## 目的

恢复并改为可执行的到达前守卫：生产路线固定为 `12 → 23 → 14 → 25 → 16`，在前往每个
目标之前及途中，如果动态障碍物匹配该目标相邻的四个中间区边线端点，就安全取消该目标并
直接前往下一目标。到点后的 360° OCR 流程保持不变；守卫并不重新启用移动 OCR。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_perception.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `docs/operations.md`
- `犯错档案.md`

## 实现

- 从 JSON 的 `square_side_m` 和 `middle_line_endpoints` 自动推导四角，拒绝缺点、重号或非四点
  映射。回归的精确编号为：12 `419/420/428/429`，23 `429/430/438/439`，14
  `421/422/430/431`，25 `431/432/440/441`，16 `423/424/432/433`。
- 守卫只订阅 `/scan_global_obstacles`，即全局障碍层已经滤除静态地图回波后的动态扫描；不读取
  混有静态层、膨胀层及增量发布语义的 global costmap。离线检查全部 18 个端点距离静态占据格
  至少 `0.485 m`，高于 relay 的 `0.22 m` 静态掩膜半径。
- 新目标建立独立 scan sequence epoch；零、重复或倒序的 `LaserScan.header.stamp`、超过
  `0.50 s` 的接收/源时间、无效距离、全 `inf`、缺失 `map ← laser` TF 都无效。相同端点必须由
  两个严格递增且间隔不超过 `0.50 s` 的雷达源时间戳连续匹配、每帧误差不超过 `0.12 m` 才触发；
  无可投影扫描超时将安全中止任务，绝不静默放行。目标发出后仍有相同的 `0.50 s` 看门狗，扫描
  断流或 TF 持续失败会先安全取消目标并停车，然后中止任务。
- 触发发生在发 goal 前时，不发送 `move_base` goal；发生在行驶中时由导航监督循环串行执行
  `cancel_goal → 零速 → action 确认 → stopped odom`。回调仅保存最新扫描，绝不在回调线程
  取消导航。审计事件写入 `observations.json`，该目标不启动相机或 OCR。
- 同时修复实车首轮发现的异步 OCR response 元数据缺失：helper response 现在带回任务侧抓拍
  时间和 map 位姿，供安全回到抓拍朝向使用。

## 验证

- 本机标准库 `unittest`：42 项通过，其中 30 项因本机没有 ROS Python 按设计跳过。
- 本机 Python 语法、`2026.launch` XML 解析和 `git diff --check` 通过。
- 已上传到车端 Ubuntu 18.04 / ROS Melodic；`catkin_make --pkg ucar_2026
  -DCATKIN_ENABLE_TESTING=ON` 成功，直接 Python 2 回归 42 项全部通过。
- 整个部署/验收阶段未启动 `roslaunch` 或 `roscore`，未发布任何运动命令，也未进行实车测试。

## 已知限制

- 这是顶点附近的动态障碍守卫，不是对整个目标方格的占据判断；若障碍从未在四个端点附近产生
  动态雷达回波，不会触发跳点。
- 最后目标 16 命中守卫时没有下一点；任务会保存事件并因不足三项有效 OCR 结果安全失败，不会
  声称已经完成生产路线。

## 2026-08-03 实车验证

- 从用户复位后的起点启动。安全门采到 96 帧连续有限 `/odom_raw`，`odom → base_link` 和
  `map → base_link` 均可用；`/scan`、`/scan_filtered` 和 `/scan_global_obstacles` 稳定在约
  `11.9 Hz`，`/cmd_vel` 只有 `/move_base` 发布。
- 车辆到达 52（误差 `0.022 m`），二维码即时接受 `a → d → i`。去往 12 时在 429 命中守卫，
  正确执行取消、零速和停车确认；23 因共享 429 在发送 goal 前正确跳过。审计事件记录了对应
  的 map 误差 `0.081 m` 与 `0.045 m`。
- 14、25、16 依次到达并执行到点 OCR；三次均识别“日用品加工车间”，但居中后雷达没有匹配到
  有效 `wall_reference_point_numbers`，因此最终按既有“三个不同有效墙点”约束以 `0/3` 安全中止。
  这是 OCR/墙点匹配结果不足，并非守卫、里程计、TF 或导航动作失败。
- 任务停止路径已发布零速并结束小车 launch；确认车端进程退出后清理 Master 中的死亡注册，随后
  停止 WSL Master。没有自动重跑。
