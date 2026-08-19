# AI 变更记录

> 此文件由项目记忆技能维护。仅记录本项目 AI 辅助完成的源代码、配置或资源改动。

## 2026-08-20｜OCR 后退 25cm 复核并尾部朝挡板停车（改动完成）

- 状态：三套主流程已接入普通导航到原停车点后方 25cm、墙面法向 `-45°` 到 `+45°` 连续 OCR 复核；不发送倒车指令。复核成功后回到认定停车区，最终车尾朝向挡板，再执行播报或仿真。
- 语义：针对可能错一格的 `0.25m` 网格偏差；不能从当前 OCR 帧确定的纠正坐标不静默猜测。
- 验证：本轮测试、车端同步和重启要求以本次提交交接结果为准；未自动启动主流程、未发送运动指令。

## 2026-08-20｜OCR 联合实测雷达点与地图墙点（改动完成）

- 状态：三套 2026 主流程、几何辅助函数、感知回归测试和运维说明已同步。
- 改动：记录 `measured_wall_hit_map`；地图交点只判定墙边，实测点用于墙点匹配、停车坐标和停车朝向；启用 `wall_match_max_error_m=0.18m` 拒绝远距离墙点匹配。
- 验证：三套感知测试均 `14/14`；三套几何测试分别 `105/105`、`121/121`、`106/106`（ROS 依赖用例按环境跳过 `88`、`104`、`89`）；未启动 ROS、未发送运动指令。

## 2026-08-20｜fallback 导航跳过 make_plan 预检查（改动完成）

- 状态：三套 2026 主流程仅对 `fallback_navigation=True` 关闭 `make_plan` 预检查；普通导航不变；446–451 从 fallback 导航候选中排除。
- 原因：反复调用 `/move_base/make_plan` 不会发送 `MoveBaseGoal`，因此不会进入 CymPlanner/move_base recovery 状态机。
- 点位：446、447、448、449、450、451 均为墙点，本次保持不变，不替换为内侧可导航点，也不向它们发导航目标。
- 验证：本地标准、国赛、额外几何测试分别 `104`、`105`、`120` 项通过（分别跳过 `88`、`89`、`104` 个 ROS 依赖用例）；普通路径 `require_plan=True`、fallback `require_plan=False`、446–451 墙点排除回归均通过。三套正式脚本和三套测试已同步车端，车端 Python2 AST 通过，六个文件本地/车端 SHA-256 一致；未重启 ROS、未发送运动指令。

## 2026-08-20｜OCR 对准超过五次后放宽像素容差（改动完成）

- 状态：三套任务脚本和 launch 将在第 6 次 OCR 对准尝试起把基础容差增加 `30px`，基础容差仍为 `30px`，即临时容差为 `60px`。
- 验证：三套任务脚本、launch 和几何测试已同步车端；车端 Python2 编译和三套 `roslaunch --nodes` 通过，未重启 ROS 或发送运动指令。

## 2026-08-20｜恢复逻辑按栅格降低膨胀（改动完成）

- 状态：当前导航配置中的全局恢复步长为 `0.00395m`，局部保持 `0.020m`；恢复列表已改为 18 组“清除两次 + 膨胀降低一次”。
- 原因：全局分辨率为 `0.01185m`，按 1/3 栅格降低可减少每阶段对全局可通行路径的突变，局部仍按 1 栅格推进。
- 验证：车端 `cym_planner` 构建成功，恢复调度 gtest `3/3` 通过；导航参数和恢复测试文件已同步车端并完成 SHA-256 核对；catkin 白名单恢复为 `usb_cam`，未启动任务或发送运动指令。

## 2026-08-20｜OCR 单点扫描补齐全部类别（改动完成）

- 状态：三套任务脚本改为只有在当前 `record_categories` 全部记录后才提前结束单点 OCR；第一类识别成功但仍有其他待收集类别时继续旋转，单类别扫描仍可在目标记录后提前停止。
- 验证：三套任务脚本 Python 语法、几何回归测试和 `git diff --check` 已通过；已同步三套运行脚本到车端并完成车端 Python2 编译及哈希核对；未重启 ROS、未发送运动指令。
- 生效条件：运行中的 Python2 任务不会热加载，需车辆安全停止后重启对应主流程；日志应看到未集齐类别时 `PRODUCTION_OCR_TURN_COMPLETE` 的 progress 接近 `6.24`，而不是第一候选后的极小值。

## 2026-08-20｜移除 OCR 候选后的旧朝向恢复（改动完成）

- 状态：三套任务脚本在 OCR 候选停车后直接以当前姿态进入 `observe_wall`，不再恢复异步识别请求时记录的旧 yaw；对应回归测试、运维说明已更新，运行脚本已上传车端。
- 验证：本地三套几何测试分别 `100/100`、`116/116`、`101/101` 通过（ROS/Python2 用例按环境跳过）；车端 Python2 编译、三套 `roslaunch --nodes` 和六个运行文件 SHA-256 校验通过；未重启 ROS、未发送运动指令。
- 生效条件：需下一次安全重启国赛主流程后，通过日志确认 OCR 候选后不再出现 `restore capture yaw`。

## 2026-08-20｜二维码固定面与 OCR 旋转速度调整（改动完成）

- 状态：三套任务脚本和对应 `2026.launch` 已将固定面转向设为 `0.70rad/s`，OCR 完整 360°扫描设为 `0.35rad/s`；QR 完整 360°扫描保持 `0.18rad/s`；现场参数文档和运维说明已同步更新。
- 验证：本地三套脚本编译、launch XML、三套参数值核对和 `git diff --check` 通过；六个运行文件已同步车端且本地/车端 SHA-256 一致；车端三套 Python2 `py_compile` 和三套 `roslaunch --nodes` 通过；未重启 ROS、未发送运动指令。
- 生效条件：运行中的任务不会热加载源码或 launch 参数，需车辆安全停止后重启对应主流程。

## 2026-08-20｜OCR 候选框面积门调整为 1000px²（改动完成）

- 状态：三套 2026 launch、任务脚本默认值、perception 回归测试和当前运维说明已同步调整；六个运行文件已上传车端。
- 目标：将 `ocr_candidate_min_bbox_area_px` 从 `500px²` 提高到 `1000px²`，过滤更小的远距离或遮挡 OCR 框。
- 验证：三套 perception 回归均 `13/13` 通过；三套 Python AST、launch XML、旧 `500px²` 残留扫描和 `git diff --check` 通过；车端三套 Python2 `py_compile`、三套 `roslaunch --nodes` 和六个文件 SHA-256 核对通过；未重启 ROS、未发送运动命令。
- 风险：面积门提高后可能过滤远距离但真实有效的 OCR 框，需要通过下一轮日志确认有效框面积分布；运行中的 Python2 任务不会热加载。

## 2026-08-20｜OCR 固定朝向与 360° 旋转速度分离（改动完成）

- 状态：三套实际车端任务脚本和对应 launch 参数已修改并同步到 `ucar-mini`，未重启 ROS 或发送运动指令。
- 改动：固定朝向原地旋转新增独立参数 `0.35rad/s`；OCR 360°扫描为 `0.18rad/s`；QR 完整旋转保持 `0.18rad/s`。
- 验证：三套任务脚本 AST/语法、launch XML、参数值核对和 `git diff --check` 通过。
- 风险：运行中的 Python2 任务不会热加载；需下一次安全启动实际 `2026.launch` 后进行实车转向验证。

## 2026-08-20｜启动脚本清理孤儿 ROS Master（改动完成）

- 状态：启动前动态识别孤儿 PID，仅对空 Master 发送 `SIGINT`；有其他 ROS 节点时保留原有拒绝启动保护。
- 验证：车端 PID 9014 实际被 `check` 模式清理；启动器随后成功启动并退出自管 Master；本地/车端 `bash -n` 和 SHA-256 校验通过，未启动导航或发送运动指令。
- 风险：活动 ROS Master 不会自动清理；需人工确认对应会话后再停止。

## 2026-08-20｜CymPlanner 点模式前视距离收紧（改动完成）

- 状态：本地 `mode1_point.obstacle_lookahead_distance` 已从 `0.8m` 调整为 `0.35m`，参数文件已同步到 `ucar-mini`；未重启运行中的 ROS。
- 影响：只影响 CymPlanner 点模式；`mode2_body_projection`、`mode3_sprint` 和 `obstacle_cost_threshold` 不变。
- 验证：YAML 解析、`git diff --check` 通过；本地/车端 SHA-256 一致；车端已有 `move_base`/`2026.launch`，未发送运动指令。
- 风险：当前运行中的 `move_base` 不会热加载 YAML，下一次安全重启导航/2026 主流程后才生效。

## 2026-08-20｜OCR 内墙停车朝向修正（改动完成）

- 状态：三套 2026 主流程及处理区几何回归测试已同步到 `ucar-mini`，未重启 ROS 或发送运动指令。
- 改动：三套处理区最终停车 yaw 改为停车点直接指向墙面交点，修复此前额外加 `180°` 导致车头背墙、而 CymPlanner point 模式无法倒车的问题；不修改 CymPlanner。
- 验证：本地标准/国赛/额外几何测试分别 `100/100`、`101/101`、`116/116` 通过（ROS/Python2 用例按环境跳过）；六个 Python 文件 AST、`git diff --check` 通过；车端六个文件 SHA-256 一致，三套脚本 Python2 AST 通过；车端无 2026 任务、`move_base` 或 `roslaunch 2026` 进程。
- 风险：下一次启动任务节点前，运行中的 Python2 进程不会热更新；尚未进行真实处理区移动验证。

## 2026-08-20｜Extra OCR profile 参数重复包裹导致启动退出（改动完成）

- 状态：Extra OCR profile YAML 和回归测试已同步到 `ucar-mini (192.168.8.231)`，未重启 ROS 或发送运动指令。
- 现象：Extra 任务启动时报 `ocr_route_profile entry 1 must be a mapping, got 'ocr_route_profile'`，任务节点退出；local costmap 的 `static_map unused` 只是兼容性 warning。
- 原因：launch 已通过 `param="ocr_route_profile"` 指定参数名，但 YAML 内又声明顶层 `ocr_route_profile:`，参数被重复包裹成映射。
- 修复与验证：YAML 改为直接列表根节点，默认值为 `[]`；车端 Extra OCR profile 校验 `16/16`、Python2 编译、`roslaunch --nodes` 均通过；`roslaunch --dump-params` 显示 `/production_task_2026/ocr_route_profile: []`；2 个同步文件 SHA-256 一致。

## 2026-08-19｜保存 OCR 对准完成位姿并复用朝向（改动完成）

- 状态：三套 2026 主流程和停车回归测试已同步到 `ucar-mini (192.168.8.231)`，未重启 ROS 或发送运动指令。
- 改动：三套 `observe_wall()` 在 OCR 对准并确认停止后写入 `ocr_aligned_pose_map=[x,y,yaw]` 并输出 `PRODUCTION_OCR_ALIGNED_POSE`；停车 approach 使用该位姿在正常膨胀下回到识别姿态，最终墙点停车 yaw 仍由墙面几何计算。
- 验证：本地三套测试分别 `99/99`、`100/100`、`114/114` 通过（ROS 用例按本机环境跳过）；车端 Python2 编译通过；三套车端新增回归各 `3/3` 通过；6 个更新文件同步完成，车端无 2026 主流程、`move_base` 或 `roslaunch 2026` 进程。
- 风险：下一次启动任务节点前，运行中的 Python2 进程不会热更新；尚未进行真实移动验证。

## 2026-08-19｜记录点前保持安全膨胀、最终定点停车改为 0.07（改动完成）

- 状态：三套 2026 主流程、launch 和停车回归测试已同步到 `ucar-mini (192.168.8.231)`，未重启 ROS 或发送运动指令。
- 改动：三套主流程在 `park_at_recorded_production_category()` 中先用常态膨胀导航到 OCR `route_point_number`，到点后才进入停车 profile，将 local inflation 切到 0.07m，再导航到墙边停车点；profile 退出时恢复进入前的局部膨胀。三套 launch 和脚本默认值统一为 0.07m，并加入 `PRODUCTION_PROCESSING_APPROACH` 日志。
- 验证：本地三套测试分别 `98/98`、`99/99`、`113/113` 通过（ROS 用例按本机环境跳过）；车端 Python2 编译、三套 launch XML 解析通过；车端标准/国赛/Extra 各 2 个停车相关回归 `2/2` 通过；9 个同步文件 SHA-256 一致，车端无 2026 主流程、`move_base` 或 `roslaunch 2026` 进程。
- 风险：下一次启动任务节点前，运行中的 Python2 进程不会热更新；尚未进行真实移动验证。

## 2026-08-19｜目标守卫备用点导航失败切换（改动完成）

- 状态：三套 2026 主流程、launch 和几何回归测试已同步到 `ucar-mini (192.168.8.231)`，未重启 ROS 或发送运动指令。
- 修复：备用目标点使用独立 `25s` 导航超时；导航失败返回 `target_navigation_failed`，继续尝试剩余守卫候选点；普通导航仍保留失败即中止。
- 验证：车端 Python2 编译通过；三套新增/相关回归各 `4/4` 通过；9 个文件本地/车端 SHA-256 一致；本地 launch XML、AST 和 `git diff --check` 通过。完整 ROS 几何套件仍有既有测试夹具错误，与本次改动无关。
- 未解决风险：需要下一次安全启动 2026 主流程后，才能进行真实移动验证。

## 2026-08-19｜QR 同点旋转阈值与到达容差统一（改动完成）

- 状态：改动完成
- 目标：修复二维码面切换时，任务层按 `0.12m` 判定到达但同点旋转仍要求 `0.05m`，导致 `0.068m` 现场误差被错误交给 `move_base`、车辆不转的问题。
- 改动：三套 2026 主流程让同点判断复用任务到达容差；增加 `0.068m` 误差回归测试，并同步三套运行脚本与测试文件到车端。
- 验证：本地三套几何回归共 `301` 项通过、`253` 项因本机无 ROS 模块按设计跳过；三套任务脚本 Python 语法检查通过；车端三套脚本 Python2 AST 通过；运行脚本和测试文件本地/车端 SHA-256 一致；`git diff --check` 通过。
- 未解决风险：车端当前未重启 2026 主流程，已加载的旧 Python2 进程不会热更新；下次安全重启后才生效，尚未做实车原地转向验证。

## 2026-08-19｜国赛冲刺速度与航向参数调整（改动完成）

- 状态：本地配置和国赛入口已修改并同步到 `ucar-mini (192.168.8.231)`，未重启 ROS 或发送运动命令。
- 改动：`sprint_arrival_tolerance=0.20m→0.30m`；`mode3_sprint.approach_decel_distance=1.0m→0.5m`；`angular_gain=5.0→10.0`。
- 说明：减速距离调小意味着更晚开始压低速度；`approach_min_vel_x=0.12m/s` 保持不变，避免终点速度更低。
- 验证：本地国赛冲刺回归 `3/3`、Python AST、launch XML/YAML 参数核对和
  `git diff --check` 通过。

## 2026-08-19｜按栅格分离 inflation recovery 步长并取消旋转恢复（改动完成）

- 状态：车端源码和导航参数已同步，`cym_planner` 已重新构建，当前 ROS 流程未重启。
- 修复：local 每阶段下降 `0.020m`，global 每阶段下降 `0.005925m`；保留 `obstacle_cost_threshold=1`；移除 `rotate_recovery/RotateRecovery`，设置 `clearing_rotation_allowed=false`。
- 原因：local 分辨率 `0.02m`、global 分辨率约 `0.01185m`，不能继续用相同米数同步降级；车载雷达为 360°，无需旋转清除。
- 验证：车端 `cym_planner` 构建通过；恢复调度 gtest `3/3` 通过；三份部署文件车端 SHA-256 与本地一致；车端 catkin 白名单恢复为 `usb_cam`。


## 2026-08-19｜导航到点偏差重试与继续流程（改动完成）

- 状态：三套任务脚本和 launch 已同步，车端 CymPlanner 已构建，正式入口当前为 manual 待机。
- 修复：任务层 `arrival_tolerance=0.12m`；action 成功但位姿复核超限时重发 3 次，最终仍超限记录告警并继续；正常导航成功和继续分支不追加任务层零速突发。
- 保留：NaN/TF/雷达/底盘/通信/目标守卫等硬故障保护停车，以及 OCR/处理阶段的设计性停车。
- 验证：车端 Python2 编译、3 个 launch XML、本地 AST/参数契约和车端 `cym_planner` 构建通过。

## 2026-08-19｜修复任务重定位阈值与到点容差冲突（改动完成）

- 状态：车端文件已同步，等待重启国赛入口完成启动验证
- 原因：三套入口同时使用 `arrival_tolerance=0.03` 和
  `post_turn_recenter_trigger=0.06`，违反任务构造阶段的严格小于约束。
- 修复：三套 launch 和三个实际任务脚本默认值统一为 `0.02`。
- 验证结果：本地 XML 参数不变量、车端三个任务脚本 Python2 编译和国赛
  `roslaunch --nodes` 均通过；未终止当前运行中的旧 launch。

## 2026-08-19｜同步国赛任务脚本的几何依赖模块（改动完成）

- 状态：车端依赖已修复，等待重启主流程完成启动链验证
- 原因：车端 `production_task_2026.py` 已引用 `shortest_yaw_delta`，但同目录
  `production_task_geometry.py` 仍是旧版，导致 Python2 导入阶段 `exit code 1`。
- 修复：同步本地新版 `production_task_geometry.py` 到车端；未终止用户当前仍在运行的
  ROS launch 和底盘/导航进程。
- 验证结果：车端任务脚本导入、三个任务脚本 Python2 编译和
  `shortest_yaw_delta` 实际调用均通过。
- 后续：停止旧 launch 后重新启动国赛入口，确认任务节点输出
  `2026 production task node started.`，再进行 odom、TF、雷达和零速检查。

## 2026-08-19｜修复 V29 lane_follow.py 车端执行权限（改动完成）

- 状态：改动完成
- 原因：Windows 侧部署 V29 压缩包后，车端 `lane_follow.py` 权限为 `644`，ROS 无法执行节点。
- 修复：在 V29 部署流程中加入 `chmod +x ~/ucar_ws/src/lane_proto/scripts/lane_follow.py`。
- 验证结果：车端权限已为 `755`，`test -x` 通过；无运动 `roslaunch --nodes` 能列出 `/lane_follow`。
- 现场状态：用户先前启动的失败国赛 launch 仍在运行，未擅自终止；重启前需先 Ctrl-C 停止旧进程。

## 2026-08-19｜V29 lane_proto 接入主流程（改动完成）

- 状态：改动完成
- 目标：以 `tmp/lane_proto_v29.zip` 为唯一 `lane_proto` 运行包来源，替换旧的工作区/车端运行包，并将标准/省赛/国赛入口改用 V29 原生参数接口。
- 当前结论：V29 使用 `goal_mode`、`goal_map_xy` 和 `use_lidar=self/true`，不兼容上一版临时加入的 `goal_control_mode`、`goal_grid_path`、`goal_point_111/120`。
- 涉及文件：V29 `lane_proto` 包、标准/省赛/国赛/额外三套 launch、三套 handoff 脚本、额外流程缺失的 `ocr_route_profile.yaml`、测试与运维文档。
- 验证结果：本地 V29 runtime `103` 项通过、`3` 项 ROS Melodic 用例跳过，角点测试 `15` 项通过；三套 launch XML、参数契约、Python AST、Bash 语法和旧接口扫描通过；车端 V29 核心哈希与压缩包一致，Python2 语法和三套 `roslaunch --nodes` 通过，临时压缩包/解压目录及 ROS/任务/底盘进程均无残留。
- 未解决风险：未执行实车运动；正式启动前仍需确认车辆零速、`/odom_raw`、两个 TF、`/scan` 新鲜且无 NaN/TF 错误。

## 2026-08-19｜导航到点位置容差收紧至 3cm

- 状态：改动完成
- 目标：将 CymPlanner 终点位置进入阈值和三套 2026 任务层 `arrival_tolerance` 统一调整为 `0.03m`，航向容差保持不变。
- 涉及文件：`cym_planner/src/cym_planner.cpp`、三套 2026 任务脚本和 `2026.launch`。
- 验证结果：车端 Ubuntu 18.04 / ROS Melodic 白名单编译 `cym_planner` 成功并恢复原 `usb_cam` 白名单；7 个文件本地/车端 SHA-256 一致；车端 3 个 Python2 AST、3 个 launch XML 解析通过；未启动 ROS、未连接车辆、未发送运动命令。
- 未解决风险：尚未进行车端运动实测；3cm 可能因定位噪声导致终点调整反复或任务层拒绝到点。

## 2026-08-19｜QR 同点朝向切换最短旋转修复

- 状态：改动完成
- 目标：修复 QR 固定朝向从 `-90°` 切换到 `-135°` 时，车辆反向逆时针绕行一大圈的问题。
- 影响文件：三套 2026 主流程任务脚本、几何辅助模块、几何回归测试。
- 验证结果：标准/国赛/扩展几何回归分别 `92/93/107` 项通过（跳过 `76/77/91` 项，因本机无 ROS Melodic Python2 模块）；9 个 Python 文件语法检查和 `git diff --check` 通过；未启动 ROS、未连接车辆、未发送运动命令。
- 未解决风险：尚未进行 ROS Melodic 车端运动验证。

## 2026-08-19｜lane_proto 增加 use_lidar=self 三态入口

- 状态：改动完成
- 目标：增加 `use_lidar=self` 的巡线节点自读雷达模式，同时保留 `true` 的国赛地图定点方案和 `false` 的 50cm 视觉进给方案；国赛绕板参数对齐 `go_around_keepout=0.08`、`board_arc_lat_scale=0.3`。
- 涉及文件：`lane_proto` 启动文件、巡线节点、运行时回归测试、国赛启动配置及实车操作/改动文档。
- 验证结果：`lane_proto` 回归 17 项通过、3 项因本机无 ROS Melodic Python2 运行时跳过；3 个 launch XML 解析、2 个 Python AST 解析和 `git diff --check` 通过；未启动 ROS、未连接车辆、未发送运动命令。
- 未解决风险：尚未进行 Ubuntu 18.04/ROS Melodic 车端启动和运动验证。

## 2026-08-19｜恢复国赛 70 号点原坐标

- 状态：改动完成
- 目标：将国赛任务网格中的 70 号点从临时偏移坐标 `(2.32, 1.68)` 恢复为原坐标 `(2.25, 1.75)`；额外任务坐标保持不变。
- 影响文件：国赛 `production_full_grid_all_numbered.json` 的顶层 `points` 与 `grouped_points.centers`、国赛现场参数文档、操作文档、坐标回归测试和变更说明。
- 结果：国赛两处 70 号记录均恢复为 `(2.25, 1.75)`；已同步到 `ucar-mini` 对应包目录，车端 SHA-256 与本地一致；未重启任务或发送运动命令。
- 验证结果：三套 JSON 的 70 号点两处记录一致；国赛定向测试 `2/2` 通过；国赛几何回归 `15/15` 非 ROS 用例通过、`76` 项按既有环境条件跳过；`git diff --check` 通过。
- 未解决风险：尚未进行实车运动验证；正式启动国赛主流程前仍需确认 `/odom_raw` 与 TF 正常。

## 2026-08-19｜恢复国赛额外任务 70 号点原坐标

- 状态：改动完成
- 目标：将国赛额外任务网格中的 70 号点从 `(2.32, 1.68)` 恢复为原坐标 `(2.25, 1.75)`，与国赛正式版保持一致。
- 影响文件：额外任务 `production_full_grid_all_numbered.json` 的顶层 `points` 与 `grouped_points.centers`、额外任务现场参数文档、操作文档、坐标核对记录和变更说明。
- 结果：额外任务两处 70 号记录均恢复为 `(2.25, 1.75)`；已同步到 `ucar-mini` 对应包目录，车端 SHA-256 与本地一致；未重启任务或发送运动命令。
- 验证结果：国赛/额外任务坐标矩阵核对通过；额外任务两处记录一致；`git diff --check` 通过。
- 未解决风险：尚未进行实车运动验证；正式启动额外任务前仍需确认 `/odom_raw` 与 TF 正常。

## 2026-08-19｜point3 前后分阶段设置全局膨胀

- 状态：改动完成
- 目标：point3 前将全局 costmap inflation 设置为 `0.21m`，到达 point3 后恢复为 `0.224m`；恢复阶段始终基于当前实际半径每次降低 `0.01m`，避免阶段切换时反向增大膨胀。
- 涉及文件：三套 `ucar_2026*` 生产任务脚本与 launch、`cym_planner` 膨胀恢复插件及测试、相关 `docs/` 记录。
- 验证结果：三套 Python2 源码、三套 launch 和 YAML 本地静态检查通过；14 个相关文件已同步车端，Ubuntu 18.04 / ROS Melodic 成功构建 `libcym_planner.so`，恢复调度 gtest `2/2` 通过，catkin 白名单恢复为 `usb_cam`。
- 未解决风险：未进行实车堵塞恢复测试；最后的车端哈希/残留进程核对因车辆随后断电中断；本轮未启动 ROS、任务或车辆运动。

## 2026-08-19｜恢复行为末尾逐级降低局部/全局膨胀

- 状态：改动完成
- 目标：恢复行为全部失败后，追加同步降低 local/global costmap inflation 的恢复阶段，从常规 `0.224m` 每次降低 `0.01m`，最低到 `0.05m` 后仍无路径才终止。
- 涉及文件：`ucar_ws/src/cym_planner/` 恢复插件、`ucar_ws/src/ucar_nav/config/testnav20260721/` 导航参数、相关构建测试及 `docs/` 操作记录。
- 验证结果：本机 YAML/XML 解析、21 个恢复阶段和 `0.224→0.05m` 序列静态核对、纯 C++ 调度头文件语法检查、`git diff --check` 通过；已按动态 DNS `ucar-mini`（`192.168.8.231`）同步 8 个运行/构建文件；车端 Ubuntu 18.04 / ROS Melodic 成功构建 `libcym_planner.so`，恢复调度 gtest `2/2` 通过，`dynamic_reconfigure` 与插件 XML 索引可发现，catkin 白名单恢复为 `usb_cam`。
- 未解决风险：尚未启动 `move_base` 验证 dynamic-reconfigure 服务实际调用，也未进行实车堵塞恢复运动测试；本轮未启动 ROS、任务或车辆运动。

## 2026-08-19｜国赛终点改为111/120地图坐标闭环

- 状态：改动完成，已修正支路映射
- 目标：视觉命中终点后不再用两墙雷达控制；按分支将车辆转到指定朝向，并用 `map -> base_link` 位姿误差将车辆闭环拉到生产网格点 111/120，成功后播报“任务完成”。
- 涉及文件：`ucar_ws/src/lane_proto/scripts/lane_follow.py`、`ucar_ws/src/lane_proto/scripts/lane_common.py`、`ucar_ws/src/lane_proto/launch/lane_proto.launch`、`ucar_ws/src/ucar_2026_national/launch/2026.launch`、相关测试与 `docs/operations.md`。
- 验证结果：现场日志确认旧映射错误；已按用户确认改为物理左→120、物理右→111、中间→111。本机 24 项测试通过、3 项因无 ROS Melodic Python2 运行时跳过；Python2 语法、两个 launch XML 和 `git diff --check` 通过；5 个运行文件已同步车端且 SHA-256 与本机一致。
- 未解决风险：尚未再次进行实车运动验证；正式启动前必须确认 `/odom_raw`、`odom -> base_link`、`map -> base_link` TF 有限且车辆零速。车端已有一条非本轮启动的旧 `start_2026.sh ... mission` 进程，本轮不擅自终止。

## 2026-08-18｜守卫拦截后的四顶点替代 OCR

- 状态：改动完成
- 目标：中心生产点被动态守卫拦截时，从该点四个守卫顶点中选择当前未被障碍命中的顶点作为 OCR 导航位置；替代点仍监控剩余候选，四点均不安全才忽略原中心点。
- 涉及文件：三套 `production_task_2026.py` 的目标守卫监控、顶点替代导航和分组 OCR 调度，以及 `docs/changes/`、`docs/operations.md`、`docs/debug-rviz-observation.md`。
- 验证结果：三套几何测试通过（`90/91/105`，ROS 缺失测试按现有约定跳过）；六个 Python 文件 AST 语法检查、三个 launch XML 解析、五组默认路线审计、四顶点替代顺序审计和 `git diff --check` 均通过；已按动态解析的 `ucar-mini` 地址部署 11 个运行文件，车端 Python2 语法、4 个 launch XML 和 11 个文件 SHA-256 校验通过，未启动 ROS 或任务进程。
- 未解决风险：顶点是备用 OCR 观察位置，正式运行前仍需在 Ubuntu 18.04 / ROS Melodic 车端低速验证其可导航性和 OCR 视角。

## 2026-08-18｜雷达到位后播报完成

- 状态：改动完成
- 目标：将“任务完成”播报绑定到雷达角落闭环连续稳定到位事件，普通视觉停车不再触发播报。
- 涉及文件：`ucar_ws/src/lane_proto/scripts/lane_follow.py`、`ucar_ws/src/lane_proto/test/test_lane_runtime.py`、`docs/changes/2026-08-18-national-lane-proto-visual-params.md`、`docs/operations.md`。
- 验证结果：本机 `lane_proto` 定向测试 15 项通过、3 项因无 ROS Melodic Python 运行时跳过；两个 launch XML、Python 语法和 `git diff --check` 通过；已同步 `lane_follow.py` 到动态发现的车端，车端 Python2 语法通过且 SHA-256 一致；未启动 ROS、未重启任务、未动车辆。
- 未解决风险：尚未进行实车运动验证；启动前须确认 `/odom_raw` 有限、两个 TF 正常、`/scan` 新鲜且车辆零速。

## 2026-08-18｜OCR 主路线扩展为五组并纳入边界点

- 状态：改动完成
- 目标：根据生产区编号图，将 OCR 主路线从 12–29 的 16 个内侧点扩展为从 11 开始覆盖 11–30 的 20 个中间区点，形成五个 2×2 调度组。
- 涉及文件：三套 `ucar_2026*` 的几何默认配置、任务脚本、`2026.launch`、几何测试，以及中间区边界守卫点解析和 `docs/changes/`、`docs/debug-rviz-observation.md`、`docs/operations.md`。
- 验证结果：三套几何测试通过（`90/91/105`，ROS 缺失测试按现有约定跳过）；六个 Python 文件编译检查、三个 launch XML 解析、五组调度顺序审计、20 个目标守卫映射和 `git diff --check` 均通过；本轮尚未部署车端。
- 未解决风险：11、20、21、30 是中间区侧墙边界点，守卫映射必须使用 446–451 侧墙顶点；尚未进行车端构建或实车运动验证。

## 2026-08-18｜国赛视觉巡线参数与完成播报

- 状态：改动完成
- 目标：将国赛 lane_proto 常驻交接参数对齐现场视觉巡线版本，并确保只有真实终点 GOAL 停车时播报“任务完成”。
- 涉及文件：`ucar_ws/src/lane_proto/launch/lane_proto.launch`、`ucar_ws/src/lane_proto/scripts/lane_follow.py`、`ucar_ws/src/lane_proto/test/test_lane_runtime.py`、`ucar_ws/src/ucar_2026_national/launch/2026.launch`。
- 验证结果：lane_proto 定向测试 `15` 项通过、`3` 项因本机无 ROS Melodic Python 运行时跳过；本机两个 launch XML、Python 语法和 `git diff --check` 通过；本次已按动态发现的车端地址同步该改动涉及的 3 个运行文件，两端 SHA-256 一致；车端 Python2 语法和 4 个 launch XML 解析通过；未启动 ROS、未重启任务、未启动车辆。
- 未解决风险：尚未进行实车运动验证；`board_arc_lat_scale=0.5` 需先做零速/里程计/TF/雷达检查，再低速确认绕板净空。

## 2026-08-18｜OCR 主路线四组前向/反向调度

- 状态：改动完成
- 目标：将生产 OCR 主路线调整为 `12-13-14-15-16-17-18-19-29-28-27-26-25-24-23-22`，按四个 2×2 点组前向扫描，并在末组后反向补齐本轮尚未尝试的点，减少不必要的跨点导航。
- 涉及文件：三套 `ucar_2026*` 的生产任务脚本、几何默认配置、`2026.launch`、几何测试，以及 `docs/changes/`、`docs/debug-rviz-observation.md`、`docs/operations.md`。
- 验证结果：三套几何测试通过（`90/91/105`，ROS 缺失测试按现有约定跳过）；六个 Python 文件编译检查、三个 launch XML 解析、分组调度审计和 `git diff --check` 均通过；已按动态解析到的 `ucar-mini` 车端地址同步 9 个运行时文件，两端 SHA-256 一致，车端 Python2 语法 `6/6`、launch XML `3/3` 通过；未启动主流程或车辆运动。
- 未解决风险：守卫点判断仍是到当前目标前/导航中的动态判断，不预判远处同组点；尚未在车端构建或实车运动验证，正式运行前需确认局部代价地图、里程计和 TF 正常后低速验证。

## 2026-08-18｜70 点坐标回滚与调试入口隔离

- 状态：改动完成
- 目标：响应“70 坐标不能更新”，撤销独立调试部署对车端国赛共享网格的坐标影响。
- 涉及文件：`ucar_ws/src/ucar_2026_national/launch/national_sprint_speed_debug.launch`、定向测试、`docs/changes/2026-08-18-national-sprint-speed-debug.md`、`docs/operations.md`；车端 `production_full_grid_all_numbered.json`。
- 验证结果：车端 70 点两处记录均为 `(2.25, 1.75)`；车端 Python2 定向测试 2 项通过；`run:=false/true` launch 静态解析通过；未启动运动节点。
- 未解决风险：本地工作区原有未提交网格仍保留 `(2.32, 1.68)`，本次未擅自覆盖；后续部署禁止同步该共享网格文件。

## 2026-08-18｜QR 扫描方向扩展

- 状态：改动完成
- 目标：到达二维码扫描中心后，将固定观察方向从 `180°→90°→-90°` 扩展为 `180°→90°→-90°→-135°→135°→45°`，保持后续任务流程不变。
- 涉及文件：三套 `ucar_2026*` 主流程的二维码默认配置、launch、几何回归测试、`docs/changes/` 与 `docs/operations.md`。
- 验证结果：标准/国赛/额外三套几何单测分别 `88/89/103` 通过；本机六个 Python 文件编译检查通过；三套 launch XML 解析通过；六个编号点均存在且从点 52 的方位角顺序匹配目标序列；已通过动态主机名同步到车端，9 个运行时文件 SHA-256 一致，车端 Python 2 编译检查通过。
- 未解决风险：尚未在车端动态 launch 或实车运动验证；扫码固定面每轮由 3 个增加为 6 个，可能增加扫码耗时。同步后未重启现有主流程，需下次按安全检查重启实际使用的入口。

## 2026-08-18｜70 冲刺速度调试程序

- 状态：改动完成
- 目标：新增独立 ROS 调试入口，加载比赛地图，从 70 冲刺起始点运行到坡顶，用于标定合适速度。
- 涉及文件：`ucar_ws/src/ucar_2026_national/scripts/national_sprint_speed_debug.py`、`launch/national_sprint_speed_debug.launch`、`CMakeLists.txt`、定向测试、`docs/changes/2026-08-18-national-sprint-speed-debug.md`、`docs/operations.md`。
- 验证结果：本机 2 项定向单测通过；车端 Ubuntu 18.04/ROS Melodic 构建成功、Python2 定向测试 2 项通过、launch 静态节点解析通过；4 个新增文件及依赖网格 JSON 已完成 SHA-256 校验；原车端白名单已恢复为 `usb_cam`，无导航残留进程。
- 未解决风险：尚未执行实车运动验证；日志统计的是 `/cmd_vel` 请求速度，不是实际轮速。

## 2026-08-20｜OCR 同类候选重复转向抑制

- 状态：改动完成，待车端回归
- 目标：修复到点 OCR 识别到同一类别但墙面对准失败后，反复停车、反向恢复候选角度并再次丢失的问题。
- 涉及文件：三套 `ucar_2026*` 生产任务脚本和几何测试、`docs/changes/2026-08-20-ocr-candidate-retry-suppression.md`、`docs/operations.md`。
- 结果：每个到点 OCR 整圈为同一类别维护拒绝集合；第一次对准失败后，后续同类候选只丢弃并保持原方向旋转。
- 验证：三套脚本 Python 语法检查和 `git diff --check` 通过；本机无 ROS Melodic，ROS 依赖测试按现有约定跳过，待车端 Python 2/Catkin 回归。
- 未解决风险：同一物理点本轮首次对准失败后，不再在同一整圈重复尝试该类别；后续路线仍可在其他点继续识别。

## 2026-08-20｜车端任务脚本执行权限恢复

- 状态：已处理
- 问题：临时文件同步后脚本权限变为 `0644`，`roslaunch` 报无法定位 `production_task_2026.py`。
- 处理：恢复三套车端任务脚本的执行位为 `0755`，并用 Melodic 环境的 `roslaunch --nodes` 完成静态节点解析。
- 结果：`ucar_2026/production_task_2026.py` 已可被 ROS 识别；未启动任务、未发送运动指令。
