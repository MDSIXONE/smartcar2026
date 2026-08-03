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
