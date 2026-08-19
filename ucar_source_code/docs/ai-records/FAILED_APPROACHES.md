# AI 失败方案记录

> 仅记录本次部署中已确认失败并被替换的执行方案。

## 2026-08-19｜收紧到点容差后遗漏重定位触发阈值

- 原始方案：将 `arrival_tolerance` 收紧为 `0.03m`，保留原
  `post_turn_recenter_trigger=0.06m`。
- 停止原因：任务启动校验要求重定位触发阈值严格小于到点验收阈值，国赛节点在构造阶段退出。
- 替代方案：将三套入口和实际任务脚本默认值统一为 `arrival=0.03m`、`recenter=0.02m`，并加入启动前参数不变量检查。

## 2026-08-19｜只同步任务主脚本而遗漏同目录几何依赖

- 原始方案：部署新版 `production_task_2026.py` 后，只按主脚本哈希确认车端版本，直接启动国赛流程。
- 停止原因：车端同目录 `production_task_geometry.py` 仍为旧版，缺少
  `shortest_yaw_delta`，任务在 import 阶段退出，ROS 只报告 exit code 1。
- 替代方案：任务脚本及同目录依赖模块按成套哈希部署；部署后用车端 Python2 实际导入、编译和关键符号调用验收。

## 2026-08-19｜直接解压 V29 后未恢复 lane_follow.py 执行位

- 原始方案：将 Windows 侧 V29 压缩包解压/复制到车端后直接启动。
- 停止原因：车端 `lane_follow.py` 权限为 `644`，ROS 拒绝执行；文件路径、shebang 和 `rospack`
  包路径均正确。
- 替代方案：部署后明确执行 `chmod +x lane_follow.py`，再用 `test -x` 和无运动启动解析验收。

## 2026-08-19｜部署工作区旧 lane_proto 而非用户指定 V29 压缩包

- 原始方案：直接把工作区当前 `lane_proto` 的 5 个文件上传到车端，认为它们就是最新运行源。
- 停止原因：用户明确提供的 `tmp/lane_proto_v29.zip` 才是最新版本；压缩包内核心文件哈希与
  旧工作区版本不同，且 V29 参数接口不包含上一版临时的 `goal_control_mode`。
- 替代方案：先从 V29 压缩包覆盖本地 `lane_proto`，按 V29 参数接口重写主流程，再以压缩包整体
  部署到车端并用远端哈希确认版本来源。

## 2026-08-18｜直接包级构建未绕过现有白名单

- 原始方案：在车端直接执行 `catkin_make --pkg ucar_2026_national`。
- 停止原因：工作区白名单仍为 `usb_cam`，CMake 未生成 `ucar_2026_national` 构建目标并报 `No rule to make target`。
- 替代方案：临时切换到 `ucar_2026_national` 构建和测试，完成后恢复白名单为 `usb_cam`；替代方案已成功。

## 2026-08-18｜独立调试部署同步了国赛共享网格

- 原始方案：部署 70→坡顶独立调试程序时，同时把本地 `production_full_grid_all_numbered.json` 上传到车端。
- 停止原因：该 JSON 是国赛主任务共享资源，上传会改变车端 70 点坐标，违背调试入口与主任务隔离要求。
- 替代方案：恢复车端原坐标 `(2.25, 1.75)`；后续只部署独立脚本、launch、CMake 和测试，不同步网格 JSON；替代方案已验证。

## 2026-08-19｜终点两墙雷达闭环替换为地图坐标闭环

- 原始方案：终点视觉命中后从 `/scan` 拟合两面墙，用墙距和墙角度控制停车。
- 停止原因：现场出现 `两墙拟合未通过: 两面墙不完整(x=2,y=0)`；其中 `x=2,y=0` 是 X/Y 候选墙数量，Y 墙缺失时车辆保持零速并最终超时，且墙拟合不直接表达生产目标点。
- 替代方案：视觉命中后按支路选择地图点 111/120，先闭环航向，再用 `map -> base_link` 的 x/y 位姿误差停车；连续到位后才发布 `GOAL` 和播报“任务完成”。

## 2026-08-19｜恢复插件改用新的 UTF-8 插件描述文件

- 原始方案：直接在既有 `cym_planner_plugin.xml` 中追加恢复插件类。
- 停止原因：该历史插件描述文件含非 UTF-8 字节，`apply_patch` 无法安全读取和修改。
- 替代方案：新增 UTF-8 的 `cym_planner_plugins.xml`，合并保留 CymPlanner 与 InflationRecovery 两个类，并将 package export 切换到新文件；旧文件保留未删除。

## 2026-08-19｜恢复插件按车端 ROS Melodic API 修正初始化签名

- 原始方案：按旧版 `nav_core::RecoveryBehavior` 使用 `tf::TransformListener*` 加单个 costmap 参数实现 `initialize()`。
- 停止原因：车端编译明确报 `override` 不匹配；ROS Melodic 实际接口要求 `tf2_ros::Buffer*`、global costmap 和 local costmap 三个参数。
- 替代方案：按车端 `/opt/ros/melodic/include/nav_core/recovery_behavior.h` 的实际签名修正并重新部署；`libcym_planner.so` 构建成功，定向 gtest `2/2` 通过。
