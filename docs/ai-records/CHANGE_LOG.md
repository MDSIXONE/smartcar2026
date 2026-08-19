# AI 改动记录

## 2026-08-20｜OCR 后退 25cm 复核并尾部朝挡板停车（改动完成）

- 目标：首次 OCR 识别后到达墙内 25cm，普通导航到原停车点后方 25cm，在墙面法向 `-45°` 到 `+45°` 连续扫读并再次确认类别；不发送倒车指令。
- 范围：三套 2026 主流程、三套 launch 参数、几何停车回归测试和运行说明；复核成功后回到认定停车区，最终车尾朝向挡板，再播报或启动仿真。
- 语义：针对可能错一格的 `0.25m` 网格偏差；二次 OCR 只确认类别/位置，不静默猜测缺少实测数据的纠正坐标。
- 验证：本轮测试、车端同步和重启要求以本次提交交接结果为准，未自动启动任务、未发送运动指令。

## 2026-08-20｜OCR 联合实测雷达点与地图墙点（改动完成）

- **目标**：保留雷达实测命中点与地图射线交点，使用地图点判定墙面、使用实测点计算最终沿墙停车坐标，降低地图局部偏差对停车位置的影响。
- **改动**：三套 2026 主流程记录 `measured_wall_hit_map`；地图交点只判定墙边，实测点用于墙点匹配、停车坐标和停车朝向；启用 `wall_match_max_error_m=0.18m` 拒绝远距离墙点匹配。
- **验证状态**：三套感知测试均 `14/14`；三套几何测试分别 `105/105`、`121/121`、`106/106`（ROS 依赖用例按环境跳过 `88`、`104`、`89`）；斜向射线和实测沿墙停车回归通过，Python 语法、`git diff --check` 通过。未启动 ROS、未发送运动指令。

## 2026-08-20｜fallback 导航跳过 make_plan 预检查（改动完成）

- **目标**：让目标守卫 fallback 候选进入 `move_base` action 状态机，避免在任务层反复调用 `/move_base/make_plan` 而无法触发 recovery list。
- **改动**：三套 2026 主流程仅在 `fallback_navigation=True` 时传入 `require_plan=False`；普通路径仍保留 `make_plan` 预检查。446、447、448、449、450、451 均为墙点，并从 fallback 导航候选中排除，不替换点位。
- **验证状态**：标准、国赛、额外几何测试分别 `104`、`105`、`120` 项通过（分别跳过 `88`、`89`、`104` 个 ROS 依赖用例）；普通路径 `require_plan=True`、fallback `require_plan=False`、446–451 墙点排除回归均通过。三套正式脚本和三套测试已同步车端，Python2 AST 通过，六个文件本地/车端 SHA-256 一致；未重启 ROS、未发送运动指令。

## 2026-08-20｜每个膨胀恢复阶段前重新清除代价地图（改动完成）

- **目标**：让每一次膨胀恢复前都重新执行保守清除和激进清除，而不是只在整个恢复序列开头清除两次。
- **计划改动**：重排当前 `move_base` recovery list，使每个 `relax_inflation_N` 前都执行 `conservative_reset_N` 和 `aggressive_reset_N`；同步运维文档、配置和车端验证。
- **影响范围**：`testnav20260721` 实车导航恢复顺序；不改变局部/全局膨胀降低步长、最低值和恢复 C++ 逻辑。
- **验证状态**：本地 YAML 解析及 54 项顺序/参数断言通过；车端 YAML 顺序和 Python2 AST 通过，10 个同步文件本地/车端 SHA-256 一致。当前未重启主流程，未发送运动指令。

## 2026-08-20｜OCR 单点重复目标类别反复对准（改动完成）

- **目标**：同一单点扫描中，目标类别首次处理后不再重复进入停车、对准和测距；保留预记录类别在第二轮首次正式处理的行为。
- **计划改动**：三套 2026 主流程增加单次扫描类别处理门；补充重复候选与预记录第二轮回归测试；同步车端脚本、测试和运行文档。
- **影响范围**：OCR 单点候选处理去重；不改变多类别集齐后才停止 360° 扫描的规则。
- **验证状态**：本地三套几何测试分别 `103`、`119`、`104` 项通过（任务类测试因本机无 ROS 依赖按设计跳过）；车端三套重复候选、多类别继续扫描和拒绝候选回归共 `9/9` 通过；六个 Python 文件 AST、`git diff --check` 通过；三套运行脚本已同步车端，未重启主流程，未发送运动指令。

## 2026-08-20｜OCR 对准超过五次后放宽像素容差（改动完成）

- **目标**：避免 OCR 对准在误差下降缓慢时重复重试过久。
- **计划改动**：前三套 2026 主流程保持基础 `30px` 容差；第 6 次及以后将容差临时增加 `30px`（临时值 `60px`），并记录实际容差；同步测试、launch、车端和提交。
- **影响范围**：OCR 墙面连续对准循环；不改变前五次控制速度、相机帧新鲜度和发散保护。
- **验证状态**：本地三套几何测试 `102/102`、`118/118`、`103/103` 通过（ROS 任务类用例分别跳过 `86`、`102`、`87`）；三套脚本 AST/语法、launch XML 和 `git diff --check` 通过；车端 Python2 编译、三套 `roslaunch --nodes` 和运行文件 SHA-256 核对通过，已上传车端；未重启 ROS、未发送运动指令。

## 2026-08-20｜恢复逻辑按栅格降低膨胀（改动完成）

- **目标**：减少恢复阶段全局膨胀半径变化过大造成的路径频繁丢失；局部每次降低 1 个栅格，全局每次降低 1/3 个栅格。
- **计划改动**：将当前全局恢复步长从 `0.005925m` 调整为 `0.00395m`（`0.01185m / 3`），保持局部 `0.020m`；更新恢复单测、导航文档并部署车端后提交。
- **影响范围**：`cym_planner` inflation recovery 参数、恢复调度测试、车端导航配置和运维记录；不改变恢复阶段数量、最低膨胀半径或旋转恢复策略。
- **验证状态**：车端 Ubuntu 18.04/ROS Melodic `cym_planner` 构建成功，恢复调度 gtest `3/3` 通过；活跃导航参数和恢复测试文件已上传且本地/车端 SHA-256 一致；catkin 白名单已恢复为 `usb_cam`；`git diff --check` 和 UTF-8 YAML 解析通过。未启动任务、未发送运动指令。

## 2026-08-20｜OCR 单点扫描补齐全部类别（改动完成）

- **目标**：修复同一停车点可见两个类别时，第一类别识别成功就提前结束 360° OCR，导致第二类别漏扫的问题。
- **计划改动**：扫描只有在当前 `record_categories` 全部记录后才提前停止；若仍有类别未记录，则继续完成整圈；增加三套几何回归测试并同步车端。
- **影响范围**：三套 2026 主流程的 OCR 单点扫描终止条件；不改变二维码首次识别、OCR 对准和类别去重规则。
- **验证状态**：本地三套几何测试 `101/101`、`117/117`、`102/102` 通过（ROS 任务类用例分别跳过 `85`、`101`、`86`）；三套脚本编译和 `git diff --check` 通过；车端 Python2 编译、三套运行脚本 SHA-256 核对通过，已上传车端；未重启 ROS、未发送运动指令。

## 2026-08-20｜移除 OCR 候选后的旧朝向恢复（改动完成）

- **目标**：修复 OCR 旋转识别到候选后立即反向转动的问题。
- **改动**：三套任务脚本在 OCR 候选停车后直接以当前姿态进入 `observe_wall`，不再恢复异步识别请求时记录的旧 yaw；同步更新几何回归测试、运维说明并上传车端。
- **影响范围**：OCR 旋转候选处理和三套任务几何回归；不改变 OCR 对准、激光测距或类别去重逻辑。
- **验证状态**：本地三套几何测试分别 `100/100`、`116/116`、`101/101` 通过（ROS/Python2 用例按环境跳过）；车端 Python2 编译、三套 `roslaunch --nodes` 和六个运行文件 SHA-256 校验通过；未重启 ROS、未发送运动指令。
- **已知限制**：需下一次安全重启国赛主流程后，通过日志确认 OCR 候选后不再出现 `restore capture yaw`。

## 2026-08-20｜二维码固定面与 OCR 旋转速度调整（改动完成）

- **目标**：缩短二维码固定面之间的同点转向时间，并提高 OCR 完整 360°扫描速度。
- **改动**：三套任务脚本和 `2026.launch` 将 `fixed_heading_rotation_speed` 从 `0.35` 调为 `0.70rad/s`，将 `ocr_scan_rotation_speed` 从 `0.18` 调为 `0.35rad/s`；`qr_rotation_speed` 保持 `0.18rad/s`；同步更新现场参数总表和运维说明。
- **验证状态**：本地三套脚本编译、launch XML、三套参数值核对和 `git diff --check` 通过；六个运行文件已同步车端且本地/车端 SHA-256 一致；车端三套 Python2 `py_compile` 和三套 `roslaunch --nodes` 通过；未重启 ROS、未发送运动指令。
- **生效条件**：运行中的任务不会热加载源码或 launch 参数，需车辆安全停止后重启对应 `2026.launch`。

## 2026-08-20｜OCR 候选框面积门调整为 1000px²（改动完成）

- **状态**：改动完成
- **目标**：将三套 2026 主流程的 OCR 候选框面积门从 `500px²` 调整为 `1000px²`，避免过小或被遮挡的 OCR 框进入停车与对准。
- **改动**：同步三套 launch、任务脚本默认值、感知回归断言、当前运维说明和变更记录；六个运行文件已上传车端。
- **验证**：三套 perception 回归均 `13/13` 通过；三套任务脚本 AST、三套 launch XML、旧 `500px²` 残留扫描和 `git diff --check` 通过；车端三套 Python2 `py_compile`、三套 `roslaunch --nodes` 和六个文件 SHA-256 核对通过；未重启 ROS、未发送运动命令。
- **未解决风险**：面积门提高后可能过滤远距离但真实有效的 OCR 框，需要通过下一轮日志确认有效框面积分布；运行中的 Python2 任务不会热加载，需安全重启实际主流程。

## 2026-08-20｜OCR 固定朝向与 360° 旋转速度分离（改动完成）

- **目标**：缩短 OCR 恢复/固定朝向的原地转向时间，同时保留 360° OCR 扫描较慢、便于识别的速度。
- **计划改动**：新增固定朝向旋转参数并设为 `0.35rad/s`；将三套主流程的 OCR 360° 扫描设为 `0.18rad/s`；QR 完整旋转仍使用原 `0.18rad/s`。
- **影响范围**：三套实际车端 `production_task_2026.py`、三套 `2026.launch`、参数验证与操作记录；不改变旋转角度、识别流程或运动安全门。
- **验证状态**：三套任务脚本 AST/语法、三套 launch XML、参数值核对和 `git diff --check` 通过；已同步到车端，未重启任务或发送运动指令。
- **已知限制**：运行中的 Python2 任务不会热加载；需车辆安全停止后重启实际 `2026.launch` 才会生效，尚未做实车原地转向验证。

## 2026-08-20｜启动脚本清理孤儿 ROS Master（改动完成）

- **目标**：处理上次启动脚本异常退出后只剩孤立 `roscore`、导致下一次 `start_2026.sh` 被“已有 ROS Master”检查拦截的问题。
- **计划改动**：启动前只匹配 PPID 为 1 且命令确认为车端 Melodic `roscore` 的孤儿进程；仅当 Master 节点列表只有 `/rosout` 时发送 `SIGINT`，发现其他节点则拒绝自动清理。
- **影响范围**：车端 `start_2026.sh`、启动操作文档和无运动静态验证；不写死 PID，不修改导航任务逻辑。
- **验证状态**：本地/车端 `bash -n` 通过；车端 `check` 模式实际清理孤儿 PID 9014，重新启动并退出自管 Master；启动脚本已同步且本地/车端 SHA-256 一致；未启动导航、未发送运动指令。
- **已知限制**：只有孤立 Master 且节点列表仅含 `/rosout` 才会自动清理；有活动 ROS 节点时仍需人工确认和停止对应会话。

## 2026-08-20｜CymPlanner 点模式前视距离收紧（改动完成）

- **目标**：将 `mode1_point` 的 `obstacle_lookahead_distance` 从 `0.8m` 降至 `0.35m`，减少目标导航时过远前视引起的局部路径阻挡重规划。
- **计划改动**：只修改 `ucar_cym_planner_params.yaml` 的 `mode1_point`，不改变 `obstacle_cost_threshold`，不修改 `body_projection` 参数。
- **影响范围**：CymPlanner 点模式配置、实车配置同步和参数静态核对；不修改 C++ 控制逻辑。
- **验证状态**：YAML 解析确认 `mode1=0.35m`、`mode2=0.8m`、`mode3=0.8m`；`git diff --check` 通过；参数文件已同步到 `ucar-mini` 且本地/车端 SHA-256 一致。同步时车端已有 `move_base`/`2026.launch`，未重启或发送运动指令。
- **已知限制**：当前运行中的 `move_base` 不会热加载该 YAML；下一次安全重启导航/2026 主流程后才生效。

## 2026-08-20｜OCR 内墙停车朝向修正（改动完成）

- **目标**：处理区停车目标的车头朝向墙面，避免车辆转到背向墙后因 CymPlanner 不支持倒车而停在目标前方。
- **计划改动**：三套 2026 主流程将停车朝向从“停车点→墙面方向再加 180°”改为直接使用“停车点→墙面方向”；补充处理区停车目标朝向回归断言。
- **影响范围**：三套 `production_task_2026.py` 及其几何回归测试；不修改 CymPlanner、不改变全局 `linear.x` 语义。
- **验证状态**：标准/国赛/额外几何测试分别 `OK`（100/101/116，因本机缺少 ROS/Python2 跳过 84/85/100 项）；六个 Python 文件 AST、`git diff --check` 通过；六个文件已同步到 `ucar-mini`，本地与车端 SHA-256 一致，车端三套脚本 Python2 AST 通过；同步前后均无 2026 任务、`move_base` 或 `roslaunch 2026` 进程，未重启或发送运动指令。
- **已知限制**：运行中的 Python2 任务不会热加载；尚未在真实处理区安全重启后进行移动验证。

## 2026-08-20｜QR 多码结果队列与首次物品锁定（改动完成）

- **目标**：处理同一画面/短时间内出现多个二维码时只保留最近结果的问题，并按裁判规则锁定同一二维码第一次成功解析出的物品。
- **改动**：二维码节点逐个发布同帧多码，文本/API 发布队列调整为 20；三套 2026 主流程 FIFO 消费 API 结果；按二维码 URL 保存首次成功物品；三套 launch 与脚本默认固定方向等待上限统一为 2.0 秒；补充 Python3/Python2 静态与定向回归。
- **影响范围**：`yolo2025` 二维码扫描器、标准/国赛/额外 2026 主流程及其二维码回归测试、实车 QR 操作文档。
- **验证**：二维码扫描器回归 `3/3` 通过；三套主流程合计 `317` 项测试中 `48` 项通过、`269` 项因本机无 ROS/Python2 依赖按设计跳过、`0` 项失败；7 个 Python 文件 AST 解析通过；三套 launch XML 解析和 `qr_search_timeout=2.0` 参数核对通过；`git diff --check` 通过；已同步车端 11 个文件且 SHA-256 全部一致；车端 Python2 AST、三套 `roslaunch --nodes` 及参数核对通过；未启动任务、未发送运动指令。
- **已知限制**：尚未在车端执行任务层 QR 回归或同帧多码实测，也尚未安全重启实际 `2026.launch`；运行中的任务不会热加载，需安全重启后生效。整套任务重启后首次物品锁定表会重新建立。

## 2026-08-20｜OCR 停车 profile 同步切换全局膨胀（改动完成）

- **目标**：处理停车点进入时同时将 local/global costmap 的 inflation 切换为 `0.07m`，退出时分别恢复进入前的实际值。
- **原因**：现有 `enter_processing_parking_profile()` 只动态修改 local inflation；日志中的 `local_inflation_radius=0.070` 不能代表 global 已同步，global 仍保持 `0.224m`。
- **改动**：三套 2026 任务脚本进入 profile 时分别读取并将 local/global inflation 同步设置为 `0.07m`，退出时分别恢复原值；进入/退出日志拆分输出两个半径；三套停车 profile 回归测试同步断言两个 namespace。
- **验证**：本地三套几何回归 `99/99`、`100/100`、`115/115`（ROS 用例按环境跳过）通过；三套 AST、launch XML、`git diff --check` 通过；车端六个运行文件 SHA-256 与本地一致；车端三套 Python2 AST、三套 `roslaunch --nodes` 和本次 profile 定向回归 `1/1` 均通过；未重启任务或发送运动指令。
- **已知限制**：车端全量几何测试仍有既有测试夹具缺少历史属性/旧接口的问题，额外版还保留既有 `observe(stop_mode)` 测试桩错误；不影响本次 profile 定向回归。新逻辑需下一次安全重启 2026 主流程后才会加载。

## 2026-08-20｜Extra OCR profile 参数重复包裹导致启动退出（改动完成）

- **现象**：Extra 任务启动时报 `ocr_route_profile entry 1 must be a mapping, got 'ocr_route_profile'`，任务节点退出；local costmap 的 `static_map unused` 只是兼容性 warning。
- **原因**：launch 已通过 `param="ocr_route_profile"` 指定参数名，但 YAML 内又声明顶层 `ocr_route_profile:`，参数被重复包裹成映射。
- **修复**：将 `ocr_route_profile.yaml` 改为直接列表根节点，默认值为 `[]`；同步更新 Extra 参数文档和 1 条结构回归测试。
- **验证**：车端 Extra OCR profile 校验 `16/16` 通过；车端 `python2` 编译通过；`roslaunch --nodes` 通过；`roslaunch --dump-params` 显示 `/production_task_2026/ocr_route_profile: []`；2 个同步文件 SHA-256 一致；未重启主流程或发送运动指令。

## 2026-08-19｜保存 OCR 对准完成位姿并复用朝向（改动完成）

- **目标**：记录 OCR 对准完成且底盘停止时的 `map -> base_link` 坐标和 yaw，返回该物品记录点时复用原始朝向，避免重复转向。
- **改动**：三套 `observe_wall()` 在 OCR 对准并确认停止后写入 `ocr_aligned_pose_map=[x,y,yaw]` 并输出 `PRODUCTION_OCR_ALIGNED_POSE`；停车 approach 使用该位姿在正常膨胀下回到识别姿态，最终墙点停车 yaw 仍由墙面几何计算。
- **验证**：本地三套测试分别 `99/99`、`100/100`、`114/114` 通过（ROS 用例按本机环境跳过）；车端 Python2 编译通过；三套车端新增回归各 `3/3` 通过；6 个更新文件同步完成，车端无 2026 主流程、`move_base` 或 `roslaunch 2026` 进程。
- **未解决风险**：车端尚未重启 2026 主流程或进行真实移动验证；已加载的 Python2 进程不会热更新，下一次安全启动后才会加载新位姿记录逻辑。

## 2026-08-19｜记录点前保持安全膨胀、最终定点停车改为 0.07（改动完成）

- **目标**：定点停车最终靠墙阶段使用 0.07m；从实际物品停车点返回仿真物品记录点的路段保持正常安全膨胀。
- **改动**：三套主流程在 `park_at_recorded_production_category()` 中先用常态膨胀导航到 OCR `route_point_number`，到点后才进入停车 profile，将 local inflation 切到 0.07m，再导航到墙边停车点；profile 退出时恢复进入前的局部膨胀。三套 launch 和脚本默认值统一为 0.07m，并加入 `PRODUCTION_PROCESSING_APPROACH` 日志。
- **验证**：本地三套测试分别 `98/98`、`99/99`、`113/113` 通过（ROS 用例按本机环境跳过）；车端 Python2 编译、三套 launch XML 解析通过；车端标准/国赛/Extra 各 2 个停车相关回归 `2/2` 通过；9 个同步文件 SHA-256 一致，车端无 2026 主流程、`move_base` 或 `roslaunch 2026` 进程。
- **未解决风险**：车端尚未重启 2026 主流程或进行真实移动验证；已加载的 Python2 进程不会热更新，下一次安全启动后才会加载新顺序。

## 2026-08-19｜目标守卫备用点导航失败切换（改动完成）

- **目标**：修复目标 11 的备用导航点被 CymPlanner 判定阻挡后仍按普通目标等待 180 秒，导致后续备用点不再尝试的问题。
- **改动**：备用点使用独立 `25s` 超时；导航失败返回 `target_navigation_failed` 并继续尝试剩余候选；普通目标失败语义保持不变；标准、国赛、Extra 三套主流程和 launch 参数同步。
- **验证**：本地三套脚本/测试编译、launch XML、`git diff --check` 通过；车端 Python2 编译通过；车端三套新增/相关回归各 `4/4` 通过；9 个同步文件本地/车端 SHA-256 一致。带完整 ROS 环境的历史全量几何套件仍有既有测试夹具错误（缺少 `pre_point_3_global_costmap_inflation_radius_m` 等属性、旧膨胀断言），与本次改动无关。
- **未解决风险**：尚未重启 ROS 或进行真实移动验证；新参数需下一次安全启动 2026 主流程后生效。


## 2026-08-19｜QR 同点旋转阈值与到达容差统一（改动完成）

- **状态**：改动完成
- **目标**：修复二维码面切换时，任务层按 `0.12m` 判定到达但同点旋转仍要求 `0.05m`，导致 `0.068m` 现场误差被错误交给 `move_base`、车辆不转的问题。
- **改动**：三套 2026 主流程让同点判断复用任务到达容差；增加 `0.068m` 误差回归测试，并同步三套运行脚本与测试文件到车端。
- **验证**：本地三套几何回归共 `301` 项通过、`253` 项因本机无 ROS 模块按设计跳过；三套任务脚本 Python 语法检查通过；车端三套脚本 Python2 AST 通过；运行脚本和测试文件本地/车端 SHA-256 一致；`git diff --check` 通过。
- **未解决风险**：车端当前未重启 2026 主流程，已加载的旧 Python2 进程不会热更新；下次安全重启后才生效，尚未做实车原地转向验证。

## 2026-08-19｜441 终点独立位置收敛与普通点阈值调整（改动完成）

- **目标**：只改善三套 2026 主流程到点 441 的位置收敛，不新增停车结束任务逻辑；普通 `mode1_point` 的规划器内部位置阈值由 `0.08m` 调整为 `0.07m`。
- **改动**：441 使用独立 `destination` 模式，位置阈值 `0.04m`、末端位置微调增益 `1.0`；普通 `mode1_point` 保持原任务流程并使用 `0.07m`；三套任务脚本在 441 前发布一次 `destination`，既有 lane handoff、`SUCCEEDED` 和 `signal_shutdown()` 保持不变。
- **验证**：本机脚本语法、YAML、`git diff --check` 通过；车端 `cym_planner` 编译通过，规划器 31 项 gtest 全部通过；本轮再次同步并核对国赛任务脚本、几何/感知依赖、launch 和共享规划器文件共 8 个文件，SHA-256 与本地一致；国赛 Python2 回归 94 项（16 通过、78 按设计跳过、0 失败），YAML/launch 节点解析通过；车端 `CATKIN_WHITELIST_PACKAGES` 保留为 `cym_planner`。
- **风险**：尚未进行车端真实运动验证；源码和参数不会热加载到已运行节点，需车辆零速并完成 `/odom_raw`、TF、`/scan` 检查后重启实际 2026 主流程才会生效。本次未重启 ROS、未发送运动指令。

## 2026-08-19｜国赛冲刺速度与航向参数调整（改动完成）

- **目标**：提高 70→288 冲刺段有效速度，并将高速冲刺终点的独立位置验收容差从 `0.20m` 放宽到 `0.30m`。
- **改动**：国赛 `sprint_arrival_tolerance=0.30m`；`mode3_sprint.approach_decel_distance=0.5m`（由 `1.0m` 调小，减速起点后移）；`angular_gain=10.0`（由 `5.0` 翻倍）。
- **验证状态**：国赛冲刺回归 `3/3`、任务脚本 AST、launch XML、YAML 参数核对和
  `git diff --check` 通过；已同步车端 `ucar-mini (192.168.8.231)`，三份运行文件
  SHA-256 一致；未重启 ROS、未发送运动指令。
- **风险**：减速起点后移和航向 P 翻倍需通过低风险实车观察确认，尤其关注冲过坡顶、角速度饱和和终点回拉行为。

## 2026-08-19｜仿真终点位置到达容差小幅放宽（改动完成）

- **状态**：改动完成
- **目标**：将仿真任务终点的位置到达容差从 `0.05m` 调整为 `0.08m`，缓解车辆已经接近加工车间但任务层未通过位置复核的问题；朝向容差保持 `0.10rad`。
- **改动**：同步仿真任务脚本默认值、`task3_vision.yaml` 实际配置和参数回归断言。
- **验证**：实时参数回归 `7/7` 通过；仿真任务脚本 AST、YAML 容差值和 `git diff --check` 通过。Windows 本机定点行为测试因缺少 ROS `actionlib` 跳过；WSL 未同步验证，因为部署工作区已有未提交修改且包含同一测试文件，按规则未覆盖。
- **未解决风险**：如果 `move_base` 因激光障碍无法把车送入 `0.08m` 范围，单纯放宽到点判定仍不能替代路径规划修复；运行中的仿真不会热加载，需要下一轮启动任务时生效。

## 2026-08-19｜车端仿真兜底等待缩短至 75 秒（改动完成）

- **状态**：改动完成
- **目标**：将车端等待仿真完成的兜底时限从 120 秒缩短为 75 秒，超时后继续终点流程的语义保持不变。
- **改动范围**：标准、额外、国赛三套 2026 主流程的任务脚本、launch 参数、几何回归基准值和当前运维说明。
- **验证**：三套几何回归分别 `93/77`、`108/92`、`94/78`（通过/跳过）；三套任务脚本与测试 AST、三套 launch XML、75 秒参数契约和 `git diff --check` 均通过；6 个运行文件已同步车端，本地/车端 SHA-256 一致；未启动 ROS、bridge 或实车任务。
- **未解决风险**：当前运行中的任务不会热加载新参数，需重启对应主流程后生效；本轮未做车端 Ubuntu 18.04/Python2 和真实运动验证。

## 2026-08-19｜IMU CRC16 仅告警不触发任务停车（改动完成）

- **状态**：改动完成
- **目标**：将底盘日志 `check crc16 faild(imu)` 从任务级中止改为限频告警，保持其他底盘、TF、里程计和结构性通信故障的中止行为。
- **改动**：三套任务脚本在通用 `crc16` 致命标记前单独忽略 IMU CRC16 并输出 `PRODUCTION_IMU_CRC_IGNORED`；同步几何回归、运维说明和改动文档；底盘驱动 CRC 校验逻辑未修改。
- **验证**：三套几何回归（标准 `93/77`、额外 `108/92`、国赛 `94/78`，通过/跳过）、三套 Python AST、车端 Python2 `py_compile`、三套 `roslaunch --nodes`、本地/车端 SHA-256 和 `git diff --check` 均通过；未重启 ROS、未发送运动命令。
- **未解决风险**：IMU 坏帧仍由底盘驱动丢弃；若后续出现 `imu sensor not active`、非有限 odom/TF 或其他结构性通信故障，任务仍会中止并停车。三套任务脚本已上传车端，但需下一次安全重启任务节点后加载。

## 2026-08-19｜OCR 候选框面积门、对准容差与尝试次数调整（改动完成）

- **状态**：改动完成
- **目标**：将三套 2026 OCR 候选框面积门调整为 `500px²`，横向对准容差调整为 `30px`，对准尝试次数调整为 `12`，减少有效 OCR 框被过滤并给点 15 更多收敛时间。
- **改动**：同步三套 launch、任务脚本默认值、感知回归断言、运维说明和 OCR 变更文档；面积门改为独立的 `500px²`，不再按对准容差比例计算；6 个运行文件已上传车端。
- **验证**：三套 perception 回归均 `13/13` 通过；三套任务脚本 Python AST、launch XML 和 `30px/500px²/12` 参数契约检查通过；车端三套 Python2 `py_compile`、三套 `roslaunch --nodes`、远端 launch 参数读回和 6 个文件 SHA-256 核对通过；`git diff --check` 通过；未启动 ROS、未发送运动命令。
- **未解决风险**：本轮只上传静态文件，未重启车端任务节点，参数需下一次安全启动后加载；启动前仍必须确认车辆零速、`/odom_raw`、TF、`/scan` 安全门。`500px²` 会放行此前 `733~1448px²` 的小框，现场可能增加遮挡框进入对准的概率，需要通过下一轮日志复核。

## 2026-08-19｜OCR 对准强制使用下一帧（改动完成）

- **状态**：改动完成
- **目标**：修复移动 OCR 对准时可能在 1 秒新鲜窗口内重复读取同一 ROS 相机帧，导致车辆已经旋转但 `horizontal_error_px` 不变化。
- **改动**：三套 2026 任务脚本在保存 ROS 相机帧时等待 `camera_sequence` 增加；新增相机回调回归用例、变更文档和操作说明；三套运行脚本及测试已同步车端。
- **验证**：标准/额外/国赛几何回归分别 `93/77`、`108/92`、`94/78`（通过/跳过）；国赛冲刺回归 `3/3`；车端三套 Python2 `py_compile` 通过；三套脚本本地/车端 SHA-256 一致；带 ROS 环境的国赛 `roslaunch --nodes` 通过。车端相机回归用例因测试导入条件跳过，未计入通过数。
- **影响文件**：三套 `production_task_2026.py`、三套几何测试、`ucar_source_code/docs/operations.md` 及 `ucar_source_code/docs/changes/2026-08-19-ocr-next-frame-alignment.md`。
- **未解决风险**：当前运行中的任务未重启，车端新脚本尚未加载；需在车辆静止并完成 `/odom_raw`、TF、`/scan` 安全检查后重启实际任务，之后再做低速 OCR 对准验证。未执行车辆运动。

## 2026-08-19｜按 local/global 栅格分离 inflation recovery 步长并取消旋转恢复（改动完成）

- **状态**：改动完成
- **目标**：保留 `obstacle_cost_threshold=1`，将 recovery 的 local 膨胀每阶段按 `0.020m`（1 个 local 栅格）下降，global 每阶段按 `0.005925m`（约半个 global 栅格）下降，避免两套不同分辨率地图按同一米数同步降级造成路径反复变化。
- **改动**：恢复插件改为读取独立的 local/global 步长；移除 `rotate_recovery/RotateRecovery`，并设置 `clearing_rotation_allowed: false`。360° 雷达无需原地旋转刷新视野。
- **验证**：本地 YAML/源码检查、`git diff --check` 通过；车端 `cym_planner` 构建通过；恢复调度 gtest `3/3` 通过；三份部署文件车端 SHA-256 一致；车端构建白名单已恢复为 `usb_cam`。
- **未解决风险**：尚未进行实车堵塞场景验证；当前运行中的 ROS 流程未重启，新恢复参数需下次启动导航后生效。

## 2026-08-19｜国赛 288 冲刺终点独立到达容差（改动完成）

- **状态**：改动完成
- **目标**：为国赛 `70→288` 冲刺段终点增加独立的位置到达容差 `0.20m`，普通导航和 70 起点继续使用 `0.12m`。
- **改动**：国赛任务脚本新增并校验 `sprint_arrival_tolerance`，仅传给 `70→288` 冲刺终点导航调用；新增 launch 参数和回归测试。
- **验证**：冲刺参数回归 `3/3` 通过；国赛几何回归 `93` 项通过、`77` 项跳过；本地 Python AST、launch XML 和 `git diff --check` 通过；国赛脚本与 launch 已同步车端且本地/车端 SHA-256 一致；车端 `roslaunch --nodes` 通过。
- **影响文件**：国赛任务脚本、国赛 launch、国赛冲刺回归测试、`ucar_source_code/docs/operations.md` 及 `ucar_source_code/docs/changes/2026-08-19-national-sprint-arrival-tolerance.md`。
- **未解决风险**：当前运行中的任务未重启，新参数需在车辆静止并完成 `/odom_raw`、TF、`/scan` 安全检查后，重启国赛主流程才会生效；尚未进行实车冲刺运动验证。

## 2026-08-19｜OCR 遮挡小框候选过滤（改动完成）

- **状态**：改动完成
- **目标**：避免锥桶遮挡远处 OCR 时，小尺寸/不稳定文字框触发停车、恢复角度和重复对准。
- **拟改动**：三套 2026 主流程在 OCR 候选进入对准前增加 bbox 面积门；本次将门槛从 `1200px²` 提高到 `2400px²`，并同步回归测试与配置说明。
- **验证**：三套 perception 回归各 `13/13` 通过；三套几何回归 `92/93/107` 通过（分别跳过 `76/77/91` 项）；本地 Python/launch 静态检查、车端 Python2 AST、三套 `roslaunch --nodes` 和 `git diff --check` 通过；6 个运行文件已同步车端且本地/车端 SHA-256 一致。
- **影响文件**：三套任务脚本、三套 perception 模块、三套 launch、三套 perception 测试、`ucar_source_code/docs/operations.md` 及 `ucar_source_code/docs/changes/2026-08-19-ocr-small-box-filter.md`。
- **未解决风险**：当前运行中的任务未重启，仍使用启动时加载的旧参数；新阈值需在车辆静止并完成 `/odom_raw`、TF、`/scan` 安全检查后，重启对应 2026 主流程才生效。面积门仍需现场确认有效框和锥桶遮挡框分布。

## 2026-08-19｜导航到点偏差重试并禁止任务层主动停车（改动完成）

- **状态**：改动完成
- **原因**：车端出现 `move_base: goal reached` 后，任务层复核到点偏差 `0.056m`、`0.094m`，仍按旧的 `arrival_tolerance` 直接 `PRODUCTION_TASK_ABORTED`，导致比赛流程提前结束。
- **修复**：三套 2026 主流程加入 3 次到点偏差重发；连续偏差时记录 `PRODUCTION_TASK_ARRIVAL_CONTINUE` 并继续流程；任务层位置容差统一为 `0.12m`；正常导航成功和该类继续分支不再额外发布零速突发。保留 NaN/TF/雷达/底盘/通信和目标守卫等硬故障保护停车。
- **规划器**：车端重新构建 `cym_planner`，恢复 catkin 白名单为 `usb_cam`；当前实车正式入口已按 `start_2026.sh` 启动为 `manual` 待机，等待车辆复位后再进入 mission。
- **验证**：本地 3 个 Python AST、3 个 launch XML、规划器/YAML 容差契约通过；车端 Python2 编译通过；8 个运行文件同步并核对；车端 `cym_planner` 编译成功；正式入口启动后 ROS Master、定位和 move_base 节点正常。
- **未解决风险**：当前只启动了 manual 待机，尚未在车辆复位到起点后进行完整比赛运动验证；OCR/处理/目标守卫阶段的设计性停车仍存在，硬故障停车仍保留。

## 2026-08-19｜修复 V29 lane_follow.py 车端执行权限（改动完成）

- **状态**：改动完成
- **原因**：Windows 侧部署 V29 压缩包后，车端 `lane_follow.py` 权限为 `644`，ROS 无法执行节点。
- **修复**：在 V29 部署流程中加入 `chmod +x ~/ucar_ws/src/lane_proto/scripts/lane_follow.py`。
- **验证**：车端权限已为 `755`，`test -x` 通过；无运动 `roslaunch --nodes` 能列出 `/lane_follow`。
- **现场状态**：用户先前启动的失败国赛 launch 仍在运行，未擅自终止；重启前需先 Ctrl-C 停止旧进程。

## 2026-08-19｜V29 lane_proto 接入主流程（改动完成）

- **状态**：改动完成
- **目标**：以 `tmp/lane_proto_v29.zip` 为唯一 `lane_proto` 运行包来源，替换车端及本地工作区的旧版本，并将标准/省赛/国赛主流程改用 V29 原生参数接口。
- **当前结论**：V29 不包含上一版临时的 `goal_control_mode`、`goal_grid_path`、`goal_point_111/120` 接口；V29 采用 `goal_mode`、`goal_map_xy`，终点雷达由 `use_lidar=self/true` 进入巡线节点自身的角落闭环。
- **影响文件**：V29 `lane_proto` 包、标准/省赛/国赛/额外三套 launch、三套 handoff 脚本、额外流程缺失的 `ocr_route_profile.yaml`、测试和运维记录。
- **验证**：本地 V29 runtime 103 项通过、3 项 ROS Melodic 用例跳过，角点测试 15 项通过；三套 launch XML、参数契约、Python AST、Bash 语法和旧接口扫描通过；车端 V29 核心哈希与压缩包一致，Python2 语法和三套 `roslaunch --nodes` 通过，临时压缩包/解压目录及 ROS/任务/底盘进程均无残留。
- **未解决风险**：未执行实车运动；正式启动前仍需按运维文档确认车辆零速、`/odom_raw`、两个 TF、`/scan` 新鲜且无 NaN/TF 错误。

## 2026-08-19｜导航到点位置容差收紧至 3cm

- **状态**：改动完成
- **目标**：将实车三套 2026 主流程的任务层位置到点验收从 `0.15m` 收紧为 `0.03m`，并将 CymPlanner 进入终点姿态调整的内部位置阈值从 `0.05m` 收紧为 `0.03m`；航向容差保持不变。
- **影响文件**：三套 2026 launch、三套运行任务脚本、实车 CymPlanner 源码。
- **验证**：三套几何回归 `90/105/91` 项全部通过（跳过 `75/90/76` 项，因本机无 ROS Melodic Python2 模块）；3 个 Python AST、3 个 launch XML 和 `git diff --check` 通过；未启动 ROS、未连接车辆、未发送运动命令。
- **未解决风险**：尚未进行 Ubuntu 18.04/ROS Melodic 车端运动实测；3cm 可能因定位噪声导致导航反复调整或任务层拒绝到点。

## 2026-08-19｜QR 同点朝向切换最短旋转修复

- **状态**：改动完成
- **目标**：修复 QR 固定朝向从 `-90°` 切换到 `-135°` 时，车辆反向逆时针绕行一大圈的问题。
- **影响文件**：三套 2026 主流程任务脚本、几何辅助模块、几何回归测试。
- **验证**：标准/国赛/扩展几何回归分别 `92/93/107` 项通过（跳过 `76/77/91` 项，因本机无 ROS Melodic Python2 模块）；9 个 Python 文件语法检查和 `git diff --check` 通过；未启动 ROS、未连接车辆、未发送运动命令。
- **未解决风险**：尚未进行 ROS Melodic 车端运动验证。

## 2026-08-19｜lane_proto 增加 use_lidar=self 三态入口

- **状态**：改动完成
- **目标**：增加 `use_lidar=self` 的巡线节点自读雷达模式，同时保留 `true` 的国赛地图定点方案和 `false` 的 50cm 视觉进给方案；国赛绕板参数对齐 `go_around_keepout=0.08`、`board_arc_lat_scale=0.3`。
- **影响文件**：`lane_proto` 启动文件、巡线节点、运行时回归测试、国赛启动配置及实车操作/改动文档。
- **验证**：`lane_proto` 回归 17 项通过、3 项因本机无 ROS Melodic Python2 运行时跳过；3 个 launch XML 解析、2 个 Python AST 解析和 `git diff --check` 通过；未启动 ROS、未连接车辆、未发送运动命令。
- **未解决风险**：尚未进行 Ubuntu 18.04/ROS Melodic 车端启动和运动验证。

## 2026-08-19｜OCR 停车临时局部膨胀调整

- **状态**：改动完成
- **目标**：将三套 2026 主流程 OCR 内墙停车临时 local inflation 从 `0.05m` 调整为 `0.20m`；global/local 常态值保持 `0.224m`。
- **影响文件**：三套 `2026.launch`、三套 `production_task_2026.py`、三套几何测试、实车操作文档和分阶段局部膨胀改动记录。
- **结果**：停车临时参数、脚本默认值和测试断言均改为 `0.20m`，CymPlanner `point` 模式保持不变；6 个运行文件已同步到 `ucar-mini`。
- **验证**：标准/额外/国赛几何回归分别 `90/105/91` 项通过（跳过 `75/90/76` 项）；9 个 Python 文件 AST、3 个 launch XML、YAML 参数检查和 `git diff --check` 通过；6 个文件本地/车端 SHA-256 一致；未启动 ROS、不发送运动命令。
- **已知限制**：参数需重启实际使用的 2026 主流程后生效，尚未进行车端运动实测。

## 2026-08-18｜常态与 OCR 停车局部膨胀参数调整

- **状态**：改动完成
- **目标**：将实车三套 2026 主流程使用的 global/local costmap 常态膨胀半径统一调整为 `0.224m`，OCR 内墙停车临时膨胀半径调整为 `0.05m`。
- **影响文件**：三套 2026 launch/任务脚本/几何测试、`testnav20260721` global/local costmap YAML、实车操作文档与 `docs/changes/2026-08-18-staged-processing-parking-costmap.md`。
- **结果**：三套 launch 和脚本默认值、全局 YAML 说明均改为 global/local 常态 `0.224m`，停车临时值为 `0.05m`；`point` 模式保持不变；7 个运行文件已同步到 `ucar-mini`。
- **验证**：标准/额外/国赛几何回归分别 `90/105/91` 项通过（跳过 `75/90/76` 项）；9 个 Python 文件 AST、3 个 launch XML、YAML 参数检查和 `git diff --check` 通过；7 个文件本地/车端 SHA-256 一致；未启动 ROS、不发送运动命令。
- **已知限制**：参数需重启实际使用的 2026 主流程后生效，尚未进行车端运动实测。

## 2026-08-18｜OCR 识别后内墙停车恢复为 25cm

- **状态**：改动完成
- **目标**：OCR 识别并完成墙面交点测量后，车辆停在墙内 `0.25m` 处。
- **影响文件**：三套 `2026.launch`、三套几何测试、`ucar_source_code/docs/operations.md` 和本记录。
- **结果**：三套 launch 的 `ocr_stop_offset_m` 已统一为 `0.25`；几何测试断言同步为 25cm；运维文档核对命令同步更新。
- **验证**：标准/国赛/额外三套几何测试分别 `90/91/105` 项通过（含既有条件跳过）；3 个 launch XML、9 个 Python AST 和 25cm 参数检查通过；`git diff --check` 通过；未启动 ROS、不发送运动命令。
- **已知限制**：参数需重启实际使用的 2026 主流程后生效；尚未进行车端运动实测。

## 2026-08-18｜QR 分类超时响应串线修复

- **状态**：改动完成，已部署，待车端现场复测
- **目标**：停止 QR 分类请求超时后旧响应串给下一二维码，避免语音双类别扫描阶段等待 60 秒、类别错配和重复扫描。
- **影响文件**：三套 2026 主流程 Python/QR helper、三套 `2026.launch`、对应测试、`ucar_source_code/docs/operations.md` 和改动记录。
- **结果**：分类请求/响应使用 `request_id` 配对；迟到响应被丢弃；分类超时 8 秒后销毁 helper，下一请求启动干净进程；分类失败释放二维码已用标记并在有限轮次内重试；三套 launch 均设置 `spark_retries=0`、`spark_timeout=8.0`、helper 启动等待 `10.0` 秒。
- **验证**：旧车端代码新增两条回归均按预期失败；修复后车端 ROS 环境三套包各 2/2 定向回归通过，标准包完整 90 项通过；本地 Python/launch 静态检查和 helper request_id 协议冒烟通过；车端 Python2 语法检查、helper 远端失败后本地分类协议检查通过；运行文件已同步，任务与运动进程保持停止。
- **已知限制**：国赛和额外包完整车端回归各有 1 个与本次 QR 改动无关的既有用例错误（分别为 `state_pub` 缺失、测试 stub 不接受 `stop_mode`）；真实讯飞延迟和现场扫描仍需车辆静止并通过安全门后复测。详细记录见 `ucar_source_code/docs/changes/2026-08-18-fix-qr-classifier-response-desync.md`。

## 2026-08-18｜仿真第一段导航速度小幅下调

- **状态**：改动完成
- **目标**：将仿真第一段导航（`main_legacy`）的前进速度上限由 `14.0 m/s` 小幅下调为 `13.5 m/s`。
- **影响文件**：`simulation/src/cym_planner/config/cym_planner_params.json`、`simulation/src/car3/test/test_task3_navigation_phase_contract.py`、`simulation/src/car3/test/test_task3_realtime_budget.py`。
- **结果**：第一段导航 `main_legacy_max_vel_x` 从 `14.0` 调整为 `13.5 m/s`；运行中的 WSL 仿真配置已同步并重启准备层。
- **验证**：两组导航契约测试各 7 项通过；Windows 与 WSL 配置 SHA-256 一致；运行时 `/move_base/CymPlanner/main_legacy_max_vel_x` 已读回 `13.5`。
- **已知限制**：13.5 m/s 是前向指令上限，不是全程恒定速度；实际速度仍受位置误差和转向降速影响。

## 2026-08-18｜国赛终点改为雷达角落闭环停车

- **状态**：改动完成，已部署，待车端现场复测
- **目标**：视觉发现终点后不再前进或调用终点导航，直接用雷达拟合相邻两面墙，控制车体前后/左右微调；距离稳定后发布 GOAL 并退出任务节点。
- **影响文件**：`lane_proto/scripts/lane_common.py`、`lane_proto/scripts/lane_follow.py`、`lane_proto/launch/lane_proto.launch`、国赛 `2026.launch`、生产任务交接状态检查、测试与操作文档。
- **结果**：国赛 launch 已启用雷达角落闭环；目标为两面墙 `0.25m ± 0.01m`，连续 5 帧稳定后发布 `GOAL`。任一墙面拟合距离大于 `1.0m` 时回退原来的 `PAUSE+APPROACH`；拟合失败或超时发布 `ABORT`。生产任务只有收到 `GOAL` 才发布最终 `SUCCEEDED`。
- **验证**：17 项 lane 离线测试通过（3 项 ROS 环境跳过）；production 定向文件 15 项通过（其余 74 项因本机无 ROS 模块跳过）；3 个 Python 文件 `py_compile` 通过；2 个 launch XML 解析通过。已动态确认 `ucar-mini` 当前地址 `192.168.8.231`，5 个运行文件已上传且车端 SHA-256 与本地一致；车端 Python2 语法和 `task_enabled:=true` launch 展开通过。
- **已知限制**：尚未在 Ubuntu 18.04 / ROS Melodic 车端实测四个角落的雷达点云和 profile 映射；部署后主流程保持停止，车端实测前必须确认 `/odom_raw`、TF 和 `/scan` 有限且车辆零速。

## 2026-08-18｜bridge 端口占用提示补充

- **状态**：改动完成
- **目标**：当 `11313` 被旧 bridge 占用时，在启动器错误提示中直接给出按 PID 核对和终止命令。
- **影响文件**：`simulation/scripts/start_simulation_stack.sh`、`ucar_source_code/docs/operations.md`、`ucar_source_code/docs/changes/`。
- **结果**：启动失败时动态打印 `ps` 核对、`kill -TERM PID` 和 `kill -KILL PID` 命令；更新后的启动脚本已精确部署到 WSL，SHA-256 与本地一致。
- **验证**：WSL 实际占用 PID `303639` 时成功输出三条可复制命令；Bash 语法通过。未自动终止当前 `state=done` bridge。
- **已知限制**：当前 bridge PID `303639` 仍占用 `11313`，需按提示先核对后手工终止，下一轮启动才会进入正常流程。

## 2026-08-18｜全局与局部常态膨胀调整为 0.24m

- **状态**：改动完成，待车端现场复测
- **目标**：将点 3 后 global inflation 由 `0.23m` 调整为 `0.24m`，并把 CymPlanner 使用的 local costmap 常态 inflation 由 `0.22m` 调整为 `0.24m`；OCR 停车临时局部膨胀和前式点模式保持不变。
- **计划影响文件**：三套 2026 任务 launch、三套任务脚本默认值、testnav20260721 的 global/local costmap 配置、测试、操作文档和改动记录。
- **验证计划**：本地 YAML/XML/Python 静态检查；车端 Python 2 定向回归和文件哈希；不启动 ROS Master、导航主流程或发送运动命令。
- **结果**：点 3 后 global inflation 目标和任务脚本默认值调整为 `0.24m`，`testnav20260721` 的 local 常态 inflation 调整为 `0.24m`；OCR 停车仍为 local `0.10m` 临时配置并保持 `point` 模式。10 个源码/launch/YAML/测试文件已同步到 `ucar-mini`；标准/国赛各 88 项车端回归和额外版全局切换定向单测通过，Python 2 语法检查通过。
- **已知限制**：车端当前未运行 `move_base`，因此未查询运行时 dynamic_reconfigure 服务；需在 `/odom_raw`、`odom -> base_link`、`map -> base_link` 有限且车辆零速时重启后复测。

## 2026-08-18｜点 3 后全局膨胀切换为 0.23m

- **状态**：改动完成，待车端现场复测
- **目标**：到达点 3 后将 global costmap 的 inflation radius 从当前启动值切换为 `0.23m`，并保持到任务结束；local costmap 仍按既有点 3 时序启用。
- **计划影响文件**：三套 2026 任务脚本、三套 `2026.launch`、对应改动记录和操作文档。
- **验证计划**：本地 Python/launch 静态检查；车端 dynamic_reconfigure 服务、Python 2 语法和定向测试；不启动任务、不发送运动指令。
- **结果**：点 3 到达和断点续跑均先设置 global inflation 为 `0.23m` 并回读校验，OCR 停车只切换 local inflation 且保持 `point` 模式；三套 launch 与三套脚本/测试已同步到 `ucar-mini`，标准/国赛车端各 88 项回归和额外版全局切换定向单测通过，9 个文件 SHA-256 一致。车端当前未运行 `move_base`，实际服务发现和运动复测留待安全重启后进行。

## 2026-08-18｜撤销 OCR 停车车体模式

- **状态**：改动完成
- **目标**：保留分阶段 local inflation，但严格使用车端已验证正常的前视点模式，避免 `body_projection` 的低速控制参数影响停车和后续导航。
- **触发**：用户实车反馈部署后速度明显变慢，并确认 CymPlanner 只能使用前视点模式。
- **结果**：移除 OCR 停车阶段的 `body_projection` 切换，停车阶段显式保持 `point`；已同步 6 个修正脚本/测试文件到 `ucar-mini`，车端 Python 2 语法检查、标准/国赛各 87 项回归和额外版 point-only 单测通过。

## 2026-08-18｜仿真 bridge 残留端口清理

- **状态**：改动完成
- **目标**：修复上一轮仿真结束后旧 HTTP bridge 仍占用 `11313`，导致下一轮启动误读旧 bridge 状态并继续运行的问题。
- **影响文件**：`simulation/scripts/start_simulation_stack.sh`、`simulation/bridge/sim_bridge.py`、`ucar_source_code/docs/changes/`、`ucar_source_code/docs/operations.md`。
- **结果**：启动前检查 `11313` 监听 PID，等待时绑定本次 bridge 子进程与端口所有权；`SIMULATION_BRIDGE_READY` 延后到 socket bind 后；Ctrl-C 清理增加 SIGINT/SIGTERM/SIGKILL 有界升级。
- **验证**：WSL 现场确认旧 PID `78097` 为本项目 bridge 并已定向停止，`11313` 已无监听；本地修改版 `bash -n`、`--help`、`sim_bridge.py` Python 语法、端口所有权（空闲/临时监听）、bridge `/status`、标记顺序和清理升级回归通过；随后只精确部署两个运行文件到 WSL，SHA-256 与本地一致，部署后 Bash/Python 语法和帮助命令通过。
- **已知限制**：WSL `/home/car/smartcar2026/simulation` 仍有既有 tracked/untracked 改动；本轮未做整仓库 fast-forward，也未启动完整 Gazebo/RViz 栈。

## 2026-08-18｜OCR 内墙停车分阶段局部膨胀

- **状态**：改动完成，待车端现场复测
- **目标**：在正常轨迹规划保留较大局部膨胀的同时，避免 OCR 识别后的 `0.29m` 内墙停车目标被同一膨胀层判为不可达。
- **结果**：三套 2026 主流程在最终内墙停车前临时将 local inflation 切换为 `0.10m`，始终使用 CymPlanner `point` 模式；停车结束后恢复进入前半径并再次确认 `point`。动态重配置失败会明确中止任务。
- **验证**：已动态确认并部署到 `ucar-mini`（`192.168.8.231`），12 个任务脚本/geometry/launch/测试文件 SHA-256 与本地一致；车端 `dynamic_reconfigure` 导入、Python 2 语法检查通过；标准/国赛车端各 87 项全量回归通过，额外版新增 profile 单测通过。额外版全量仍有 1 个既有 `observe(stop_mode)` 测试桩错误，与本次改动无关。
- **已知限制**：`0.10m` 尚未实车运动验证；未重启主流程，必须在 `/odom_raw` 和 TF 有限、车辆零速时再重启对应流程复测。

## 2026-08-18｜OCR 识别后内墙停车偏移调整

- **状态**：改动完成
- **目标**：将 OCR 识别成功后根据内墙交点计算的停车坐标内缩距离由 `0.25m` 调整为 `0.29m`。
- **影响文件**：三套 2026 主流程任务脚本与 `launch/2026.launch`、对应测试、`ucar_source_code/docs/changes/`、`ucar_source_code/docs/operations.md`。
- **结果**：停车坐标函数改为接收显式内缩距离；三套 launch 均设置 `ocr_stop_offset_m=0.29`，地图网格 `square_side_m=0.5` 保持不变。
- **验证**：三套几何回归通过（标准/国赛各 86 项、额外 101 项；ROS 依赖用例按既有条件跳过）；AST、launch XML、0.29m 坐标容差断言和 `git diff --check` 通过；未在本机编译、启动 ROS 或发送运动命令。
- **已知限制**：尚未在 Ubuntu 18.04 / ROS Melodic 车端现场验证实际停车误差；参数生效需要重启对应主流程。

## 2026-08-18｜usb_cam USB Hub 热重连自动恢复

- **状态**：改动完成
- **目标**：解决 USB Hub 整体断开并重新枚举摄像头后，`usb_cam` 继续使用旧 fd，导致 `/usb_cam/start_capture` 在 `VIDIOC_QBUF` 返回 `ENODEV(19)`。
- **实现方式**：在 `usb_cam` 内增加设备断开识别、旧 fd/mmap 释放、稳定别名重新 open、缓冲区重建和运行中自动恢复；start service 在 8 秒内按 0.5 秒间隔等待重连。
- **影响文件**：`ucar_source_code/ucar_ws/src/usb_cam/include/usb_cam/camera_driver.h`、`camera_driver.cpp`、`usb_cam.cpp`、对应 launch 以及 `ucar_source_code/docs/changes/`、`docs/operations.md`。
- **验证**：本地 launch XML 和差异检查通过；车端已同步 7 个源码/launch 文件，Ubuntu 18.04 `usb_cam` 白名单构建通过；新二进制包含 `USB_CAM_RECONNECT` 日志；`roslaunch --nodes` 通过；未重启当前用户 ROS 主流程，未人为断开 Hub。
- **已知限制**：尚未完成真实 USB Hub 断开/恢复回归；下一轮车辆静止且通过安全门后验证图像话题恢复和点 52 `/usb_cam/start_capture` 成功。

## 2026-08-18｜摄像头改用稳定 udev 别名

- **状态**：改动完成
- **目标**：解决 USB 摄像头重新枚举为 `/dev/video1` 后，主流程仍固定打开 `/dev/video0` 导致点 52 后 QR 阶段失败的问题。
- **实现计划**：为 RHX `0edc:2050` 摄像头新增 `/dev/ucar_camera` udev 别名；三套 2026 主流程、OCR、巡线交接和常用相机 launch 改用该别名，保留可变 `/dev/videoN` 仅在测试夹具/历史文档中。
- **影响文件**：`ucar_source_code/ucar_ws/src/startup_scripts/ucar_camera.rules`、三套 2026 launch/任务脚本、QR/巡线/相机入口、`ucar_source_code/ucar_ws/src/usb_cam/src/camera_driver.cpp`、`ucar_source_code/docs/changes/`、`ucar_source_code/docs/operations.md`。
- **验证**：本地 Python AST、launch XML、udev 规则和 `/dev/video0` 运行时引用检查通过；车端已同步 28 个文件并完成关键文件 SHA-256 校验；Ubuntu 18.04 车端 `usb_cam` 白名单构建通过；`/dev/ucar_camera -> /dev/video0` 且 `v4l2-ctl` 读取 1920×1080 MJPG 成功；未启动 ROS 或发送运动命令。
- **验证**：本地 Python AST、launch XML、udev 规则和 `/dev/video0` 运行时引用检查通过；车端已同步 28 个文件并完成关键文件 SHA-256 校验；Ubuntu 18.04 车端 `usb_cam` 白名单构建通过；`/dev/ucar_camera` 可跟随从 `/dev/video0` 切换到 `/dev/video1`；两种格式直接 V4L2 采帧成功。
- **新增发现**：真实 QR 流程在 USB Hub 热重连后仍失败；`usb_cam` 持有热重连前的旧 fd，在 `VIDIOC_QBUF` 处返回 `ENODEV(19)`。下一步需修复节点重连/重新 open 生命周期，不能继续只调整设备路径。
- **已知限制**：热重连自动恢复尚未实现；在修复并回归前，USB Hub/摄像头链路必须保持稳定，启动前仍必须按安全门检查 `/odom_raw`、TF、/scan 和零速条件。

## 2026-08-18｜局部代价地图延迟到点 3 启用

- **状态**：改动完成
- **目标**：让标准、省赛/国赛和额外任务主流程在前往点 3 前关闭 local costmap 的动态障碍与膨胀层，到达点 3 后再启用，避免过早的局部动态判障干扰点 3 之前的路线。
- **影响文件**：`ucar_source_code/ucar_ws/src/ucar_2026*/scripts/production_task_2026.py`、对应 `package.xml` 与 `launch/2026.launch`、`ucar_source_code/docs/changes/`、`ucar_source_code/docs/operations.md`。
- **实现方式**：不关闭 local costmap 容器和静态层，仅通过 dynamic_reconfigure 控制 `obstacle_layer`、`inflation_layer`；点 3 导航成功返回后才切换为 enabled。
- **验证**：三套本地 Python AST、三套 launch/package XML 和生命周期断言通过；车端 dynamic_reconfigure Python2 导入、白名单 catkin 构建、`roslaunch --nodes ucar_2026_national 2026.launch task_enabled:=true` 通过；标准与国赛任务几何/感知回归各 86 项通过，额外任务保留 1 个既有 `observe()` 测试桩错误；9 个运行时/依赖文件和 3 个测试文件已同步到 `ucar-mini` 并完成哈希校验；未启动 ROS 或发送运动命令。
- **已知限制**：尚未在真实任务运行中观察 `before_point_3` 到 `reached_point_3` 的动态重配置日志；下次启动前必须按安全门检查 `/odom_raw` 和 TF，启动后确认两个 dynamic_reconfigure 服务及点 3 前后日志时序。

## 2026-08-18｜国赛 70 号点冲刺朝向恢复 180°

- **状态**：改动完成
- **目标**：将国赛到达 70 号点、进入冲刺段时的车头角度由 170° 恢复为 180°。
- **影响文件**：`ucar_source_code/ucar_ws/src/ucar_2026_national/launch/2026.launch`、`ucar_source_code/docs/debug-rviz-observation.md`、`ucar_source_code/docs/operations.md`、`ucar_source_code/docs/changes/`。
- **结果**：`sprint_yaw_deg=180`；冲刺速度、前向增益和航向 P 保持上一轮对比值；额外任务不受此参数影响。
- **验证**：XML 解析、文本核对和 `git diff --check` 通过；已同步到车端并完成 SHA-256 校验，车端读取为 `180`；未启动任务或发送运动命令。
- **风险**：180° 尚未完成本轮现场试跑，需在 odom/TF 安全检查通过后观察到达 70 点的实际车头误差。

## 2026-08-18｜扩大局部代价地图与前视膨胀范围

- **状态**：改动完成
- **目标**：解决当前 local costmap `1.0×1.0 m`、局部膨胀 `0.07 m`、CymPlanner 前视 `0.25 m` 导致障碍判断滞后的问题。
- **影响文件**：`ucar_source_code/ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml`、`local_costmap_common.yaml`、`ucar_source_code/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`、`ucar_source_code/docs/changes/2026-08-18-expand-local-costmap-lookahead.md`、`ucar_source_code/docs/operations.md`。
- **结果**：local window `1.8×1.8 m`、local inflation `0.22 m`、三个模式前视距离 `0.8 m`；三份 YAML 已同步到 `ucar-mini`，本地与车端 SHA-256 一致。
- **验证**：三份 YAML 解析和数值断言通过，车端 grep 值正确；未在本机编译、启动 ROS 或发送运动命令。
- **已知限制**：同步期间未强制重启任务；最终核对时车端 ROS 主流程已不在运行，下一次启动会加载新配置。local inflation 扩大后窄通道可能更早暴露无路可走。

## 2026-08-18｜仿真 409 状态进入 120 秒兜底

- **状态**：改动完成
- **目标**：HTTP bridge 返回 409（已有或残留运行状态）时不再中止生产任务，进入仿真状态轮询，并在 120 秒未完成后继续车辆终点流程。
- **影响文件**：`ucar_ws/src/ucar_2026*/scripts/production_task_2026.py`、对应测试、`docs/changes/`、`docs/operations.md`。
- **结果**：标准、国赛和额外三套主流程均将 HTTP 409 记录为告警并返回 `False`，继续 `/status` 轮询及 120 秒超时继续路径；对应回归测试已改为锁定该语义。
- **验证**：三套主流程源码和两套测试文件 AST 解析通过；本机 `ucar_2026` 发现式回归 86 tests（71 skipped）、`ucar_2026_extra` 发现式回归 101 tests（86 skipped），其余通过；已在 `ucar-mini` Ubuntu 18.04 / ROS Melodic Python2 环境完成白名单 Catkin 构建（exit 0），`ucar_2026` 车端 86 tests 全部通过，额外任务新增 409 用例单独通过；相关文件 `git diff --check` 通过。
- **已知限制**：额外任务全量测试仍有一个既有 `observe()` 测试桩不接受 `stop_mode` 参数的问题；本次未启动真实任务或车辆运动。

## 2026-08-18｜膨胀区非零代价触发重规划

- **状态**：改动完成
- **目标**：将当前 CymPlanner 点/冲刺模式的路径阻塞判定从 `cost >= 253` 调整为任意非零代价，以便路径进入局部代价地图膨胀区时触发事件式全局重规划。
- **影响文件**：`ucar_source_code/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`、`ucar_source_code/docs/changes/2026-08-18-replan-on-any-inflation-cost.md`、`ucar_source_code/docs/operations.md`。
- **结果**：三个模式的 `obstacle_cost_threshold` 均改为 `1`；参数文件已同步到 `ucar-mini`，车端文件与本地 SHA-256 一致。
- **验证**：本机 YAML 解析和 `git diff --check` 通过；车端文件值为 `1`。车端当前 `move_base` 仍在运行，运行时参数仍为旧值 `253`，未强行重启任务。
- **已知限制**：当前判定读取 local costmap；本轮后续已将 local inflation 同步为 `0.22 m`，并扩大窗口/前视距离，下一次安全重启 `move_base`/2026 主流程后配置才生效。

## 2026-08-18｜生产地图墙体拐角与端点像素修正

- **状态**：改动完成
- **目标**：修正省赛运行地图正交墙拐角的缺角像素，并将 139、148、152 三处开放墙端各延长半个墙宽；国赛版在墙段由 148-159 平移至 147-158 后同步相同几何修正。
- **影响范围**：`ucar_nav/maps/iflysse_field_walls_without_middle_vertices.pgm`、`iflysse_field_walls_national.pgm`、生产编号 PNG 及地图资源生成工具、`docs/changes/`、`docs/operations.md`。
- **结果**：补齐 136、138、140、141、149、151 等拐角缺角；139、148、152（国赛对应平移后的 147）沿墙体方向延长半个墙宽；省赛、国赛、额外任务 PGM/PNG 资源已同步。
- **验证**：PGM 像素级断言、国赛墙段平移关系、PNG 资源一致性和修正工具幂等性通过；`git diff --check`（限定本轮文件）通过；已动态确认车端 `ucar-mini` 为 Ubuntu 18.04.6，并完成两份 PGM 上传及车端/本地 SHA-256 校验；未在本机编译、启动 ROS 或动车。
- **已知限制**：本次未启动或重启车端 ROS/导航主流程；若已有 `map_server` 常驻，需重启后才会重新加载地图。历史未被当前 2026 主流程引用的 `iflysse_2026_direct.pgm` 未修改。

## 2026-08-18｜国赛 70 号点冲刺朝向调整

- **状态**：改动完成
- **目标**：将国赛到达 70 号点、进入冲刺段时的车头角度由 175° 调整为 170°。
- **影响文件**：`ucar_source_code/ucar_ws/src/ucar_2026_national/launch/2026.launch`、`ucar_source_code/docs/debug-rviz-observation.md`、`ucar_source_code/docs/operations.md`、`ucar_source_code/docs/changes/`。
- **生效方式**：launch 参数改动不需要重新编译，但必须重启国赛主流程；额外任务不受此参数影响。
- **结果**：`sprint_yaw_deg=170`，冲刺速度、前向增益和航向 P 保持上一轮对比值。
- **验证**：XML 解析、文本核对和 `git diff --check` 通过；已同步到车端并完成 SHA-256 校验，车端读取为 `170`；未启动任务或发送运动命令。
- **风险**：170° 尚未完成现场试跑，需在 odom/TF 安全检查通过后观察到达 70 点的实际车头误差。

## 2026-08-18｜国赛冲刺速度与加速响应对比

- **状态**：改动完成
- **目标**：将国赛 70→坡顶冲刺最大前向速度调至 2.7，并提高前向加速响应用于对比试跑。
- **影响文件**：`ucar_source_code/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`、`ucar_source_code/docs/changes/`、`ucar_source_code/docs/operations.md`。
- **方案**：CymPlanner 当前没有独立加速度参数，本轮以 `linear_x_gain` 作为前向加速响应的控制项；不修改航向 P 和冲刺朝向。
- **结果**：`mode3_sprint.linear_x_gain=13.5`、`max_vel_x=2.7`；`angular_gain=5.0`、`sprint_yaw_deg=175` 保持不变。
- **验证**：YAML 解析、参数断言、启动链路静态检查和 `git diff --check` 通过；已同步到车端 `/home/ucar/ucar_ws`，本地与车端 SHA-256 一致并读取确认目标参数；未在本机编译或启动 ROS。
- **风险**：更高速度和前向增益可能增加麦轮滚子打滑与制动距离，需在车端完成安全试跑。

## 2026-08-18｜国赛冲刺航向环 P 减半

- **状态**：改动完成
- **目标**：降低国赛 70→坡顶冲刺段的航向角度环 P，改善到达 70 附近的角度控制表现。
- **影响文件**：`ucar_source_code/ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`、`ucar_source_code/docs/changes/`、`ucar_source_code/docs/operations.md`。
- **结果**：`mode3_sprint.angular_gain` 已由 `10.0` 调整为 `5.0`；`linear_x_gain`、`max_vel_x` 和任务层 `sprint_yaw_deg` 保持不变。
- **验证**：YAML 解析通过；确认 `mode3_sprint.angular_gain=5.0`、`linear_x_gain=12.5`、`max_vel_x=2.5`，国赛 launch 仍为 `sprint_yaw_deg=175`；`git diff --check` 通过。
- **风险**：尚未部署或在小车 Ubuntu 18.04 / ROS Melodic 上实测。

## 2026-08-18｜以 WSL 源码同步 Windows 仿真分享目录（改动完成）

- **状态**：改动完成
- **目标**：以 WSL 当前实际运行的仿真源码为唯一基准，确保 Windows 分享目录与实际运行内容一致。
- **影响范围**：WSL `/home/car/smartcar2026/simulation/` 到 Windows `simulation/` 的源码、配置、模型、测试和文档；排除 `build/`、`devel/`、`logs/`、`tmp/`、训练产物等生成文件。
- **结果**：已完成 WSL → Windows 同步，并删除 Windows 侧不属于 WSL 基准且未被排除的旧文件；后续修改应先改 WSL，再按 `docs/operations.md` 同步。
- **验证**：两端内容级 `diff -qr --strip-trailing-cr` 无差异；关键启动脚本、部署文档和 `math.world` 的 SHA-256 一致。Windows 挂载盘权限位差异不作为源码不一致；验证时 WSL 一键启动脚本已持有仿真栈，`/map` 和 bridge `state=waiting` 均已检查通过。
- **收尾**：本次同步没有主动启动或终止仿真；当前运行栈结束后必须在一键启动终端按 Ctrl-C，避免残留进程。

## 2026-08-18｜仿真三步启动合并（改动完成）

- **状态**：改动完成
- **目标**：将仿真专用 `roscore`、`task3_prepare.launch` 和 HTTP bridge 合并为一条 WSL 启动命令，并在启动前后保留 GUI 安全预检与按 Ctrl-C 清理。
- **影响文件**：`simulation/scripts/start_simulation_stack.sh`、`simulation/README.md`、`simulation/bridge/README.md`、`ucar_source_code/docs/deployment.md`、`ucar_source_code/docs/operations.md`、`ucar_source_code/docs/operations-national.md`、`ucar_source_code/docs/operations-extra.md`、`ucar_source_code/docs/changes/`。
- **结果**：新增 `simulation/scripts/start_simulation_stack.sh`，默认串行启动仿真 Master、Gazebo/RViz 和 HTTP bridge；bridge 等 `/map` 就绪并确认 `/status` 为 `state=waiting` 后输出单独一行 `OK`，Ctrl-C 按顺序清理三类进程；标准、国赛和额外流程文档均已切换到一键入口。
- **验证**：Windows 与 WSL 两端脚本 SHA-256 一致；两端均通过 `bash -n` 和一键脚本 `--help`，并通过 `git diff --check`；尚未启动真实 WSL ROS/Gazebo GUI。
- **风险**：尚未在 WSL Ubuntu 20.04 的真实 ROS Noetic/Gazebo GUI 环境运行；本机仅做脚本静态检查。

## 2026-08-17｜额外任务 70 号点坐标同步

- **状态**：改动完成
- **目标**：将额外任务使用的国赛地图 70 号冲刺前点同步为 `x + 0.07m`、`y - 0.07m`。
- **影响文件**：`ucar_source_code/ucar_ws/src/ucar_2026_extra/config/production_full_grid_all_numbered.json`、`ucar_source_code/docs/changes/`、`ucar_source_code/docs/operations.md`。
- **结果**：额外任务 70 号点由 `(2.25, 1.75)` 调整为 `(2.32, 1.68)`，顶层 `points` 与 `grouped_points.centers` 两份记录一致。
- **验证**：国赛与额外任务两份 JSON 的 70 号点联合容差核对通过；额外任务几何回归 101 项通过、86 项因本机缺 ROS 跳过；`git diff --check` 通过。
- **风险**：尚未在小车 Ubuntu 18.04 / ROS Melodic 上启动或实测。

## 2026-08-17｜国赛 70 号点坐标校准

- **状态**：改动完成
- **目标**：将国赛地图 70 号点坐标按现场标定结果调整为 `x + 0.07m`、`y - 0.07m`。
- **影响文件**：`ucar_source_code/ucar_ws/src/ucar_2026_national/config/production_full_grid_all_numbered.json`、`ucar_source_code/docs/changes/`、`ucar_source_code/docs/operations.md`。
- **结果**：国赛 70 号点由 `(2.25, 1.75)` 调整为 `(2.32, 1.68)`，顶层 `points` 与 `grouped_points.centers` 两份记录一致。
- **验证**：JSON 解析、70 号点唯一性和 `x + 0.07m` / `y - 0.07m` 容差核对通过；国赛几何回归 86 项通过、71 项因本机缺 ROS 跳过；`git diff --check` 通过。
- **风险**：尚未在小车 Ubuntu 18.04 / ROS Melodic 上启动或实测。

## 2026-08-17｜国赛主流程接入新版 lane_proto

- **状态**：改动完成
- **目标**：在 OCR 完成后的巡线交接中，将已从小车同步的新版 `lane_proto` 应用到 `ucar_2026_national` 主流程，并启用板检测与绕板；保留主流程共享相机和单底盘约束。
- **影响文件**：`ucar_ws/src/ucar_2026_national/launch/2026.launch`、`ucar_ws/src/lane_proto/test/test_lane_runtime.py`、`docs/lingo.md`、`docs/changes/`、`docs/operations.md`。
- **参数**：`is_fork=yolo`、band2 模板、`yellow_target=0.90`、`align_offset=0.14`、`start_offset=0.23`、`goal_y_lo=0.85`、`linear_speed=0.2`、`gain=1.2`、`rate=20`、`dump_every=3`、`goal_pause=1.0`、`board_in_lane=true`、`go_around=true`、`board_stop_dist=0.321`、`go_around_keepout=0.15`。
- **结果**：国赛 `2026.launch` 的常驻 `lane_proto` 在 OCR/生产任务完成后的交接阶段使用新版巡线逻辑；保留共享相机和单底盘模式，新增板检测与绕板参数。
- **验证**：国赛 XML 与参数值检查通过；include 参数名与新版 `lane_proto.launch` 的 119 个参数 schema 对齐；lane_proto Python 语法检查通过；本机发现式回归 13 项中 10 项通过、3 项因缺少 ROS Melodic 跳过；`git diff --check` 通过。
- **风险**：尚未启动 ROS 或车辆；启用板检测/绕板后需在小车 Ubuntu 18.04 / ROS Melodic 上做完整 Catkin 回归和现场复测。

## 2026-08-17

- **状态**：改动完成
- **目标**：从小车 `192.168.8.231` 拉取最新 `lane_proto` 巡线源码、配置、启动文件、测试、工具和运行库到本地。
- **影响文件**：`ucar_source_code/ucar_ws/src/lane_proto/`。
- **结果**：已同步源码、配置、launch、测试、工具、CUDA 源码和 `lib` 运行库；排除了抓拍/缓存、Python 字节码、训练权重和编译中间产物。
- **验证**：车端与本地纳入范围的 31 个文件 SHA-256 全部一致；本机 Python 语法检查通过；巡线回归 8 项中 5 项通过、3 项因缺少 ROS Melodic 跳过。
- **风险**：排除小车端抓拍/缓存、Python 字节码和 CUDA 训练/编译产物；未启动 ROS 或车辆。

## 2026-08-15

- **状态**：改动完成
- **目标**：三参数试跑——关闭 5Hz 周期性全局重规划、Point 模式转向按 cos² 降速、终点阶段只调朝向不修正位置，用于归因"近障碍突然换路来不及转弯撞击"与"墙边定点来回调整"。
- **影响文件**：`ucar_ws/src/ucar_nav/config/testnav20260721/move_base_params.yaml`、
  `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`、
  `docs/changes/2026-08-15-replan-once-turn-slowdown-final-yaw-only.md`、
  `docs/operations.md`。
- **结果**：`planner_frequency` 5.0→0.0（仅事件驱动重规划）；`mode1_point`
  `heading_slowdown_min_scale` 1.00→0.00（线速度乘 cos²(航向误差)）；
  `final_linear_x_gain` 1.0→0.0（0.05m 内只对准朝向）。mode2 与其余参数不动。
- **验证**：本机两份 YAML UTF-8 解析通过；已 scp 至小车 192.168.8.231，
  两端 SHA-256 一致；车端 grep 确认 planner_frequency 0.0 /
  mode1_point heading_slowdown_min_scale 0.00 / final_linear_x_gain 0.0
  生效，mode2 参数未动。未经实车验证，用户试跑后决定是否保留。
- **风险**：`planner_frequency=0.0` 仍会被局部判障失败、新目标、守卫取消触发重规划；
  cos² 降速对 45° 误差仍保留约 50% 线速度，非硬阈值；终点位置精度现仅由任务层
  `arrival_tolerance=0.15m` 验收。

- **状态**：改动完成
- **目标**：修复常驻交接漏传 `is_fork:=yolo` 导致交接点黄线被当终点横线、巡线全程跳过。
- **影响文件**：`ucar_ws/src/ucar_2026/launch/2026.launch`、
  `ucar_ws/src/lane_proto/test/test_lane_runtime.py`、`docs/changes/`、
  `docs/operations.md`。
- **结果**：2026.launch 的 lane_proto 常驻 include 补齐起跑序列参数
  （is_fork:=yolo、band2 模板、align_offset/start_offset/goal_y_lo 及性能参数，
  goal_pause 恢复 1.0），与 handoff_lane.sh 2026-08-12 最终值对齐；新增回归
  测试锁住 include 参数完整性。
- **验证**：本机 XML 解析与 lane_proto 8 项 unittest 0 errors / 0 failures；
  已部署小车，SHA-256 两端一致；车端 Python2 语法检查、lane_proto 8 项
  定向回归（0 skip）与 catkin 回归（lane_proto 8 项 + ucar_2026 96 项）
  均为 0 errors / 0 failures。
- **风险**：首次交接将走完整起跑序列（黄线对齐 → 认灯 → 进三岔口），现场须
  观察首个相位为 ALIGN 而非 APPROACH。

## 2026-08-14

- **状态**：改动完成
- **目标**：将 lane_proto 的 TrackSeg/CUDA 模型初始化从常驻启动期延迟到主流程激活巡线时。
- **影响文件**：`ucar_ws/src/lane_proto/scripts/lane_follow.py`、
  `ucar_ws/src/lane_proto/test/`、`docs/changes/`、`docs/operations.md`。
- **结果**：TrackSeg/CUDA 在 STANDBY 不构造；激活服务在置 FOLLOW 前加载一次，独立启动模式
  仍在 run 开始时加载。
- **验证**：车端定向回归先因缺少延迟入口失败；修复后 Python2 语法检查、两项定向回归及
  lane_proto 全量 7 项测试均为 0 errors / 0 failures。
- **风险**：首次巡线交接会承担一次 CUDA 模型初始化时长；服务在模型加载完成前不会返回成功。

- **状态**：改动完成
- **目标**：在主流程接受语音并发送 SIGTERM 时，显式停止阵列麦录音并释放 HID USB 句柄。
- **影响文件**：小车 `/home/ucar/wake/micarray/wake_listen.py`、
  `/home/ucar/wake/micarray/mic_array.py`、`docs/changes/`、`docs/operations.md`。
- **结果**：wake_listen 收到 SIGTERM 时先停止阵列麦录音并关闭 HID；正常退出走同一释放路径，
  重复清理不会重复发出停止录音指令。
- **验证**：本机与小车 Python3 语法检查通过；车端结构回归确认 SIGTERM 注册、释放调用和
  录音状态幂等保护均存在。未启动真实 HID 硬件。
- **风险**：HID 库调用属于硬件侧闭合动作；本次仅验证代码路径，不启动麦阵列。

- **状态**：改动完成
- **目标**：为 QR 扫描器增加单进程有效的“网址 → 物品”内存缓存，避免重复 HTTP 查询。
- **影响文件**：`ucar_ws/src/yolo2025/scripts/qrcode_scanner.py`、
  `ucar_ws/src/yolo2025/test/`、`docs/changes/`、`docs/operations.md`。
- **结果**：成功查询的规范化网址会在 qrcode_scanner 进程内映射到物品名；相同网址命中后
  直接发布带 `cached=true` 的结果，失败结果不缓存。
- **验证**：本机与车端 Python3 均完成语法检查和定向回归：同一网址只请求一次，第二次发布
  缓存物品。
- **风险**：缓存随 qrcode_scanner 进程退出清空，不跨任务或重启复用。

- **状态**：改动完成
- **目标**：修复共享 ROS 相机模式的 lane_follow 在性能输出时缺少 `cam_fps` 属性而退出。
- **影响文件**：`ucar_ws/src/lane_proto/scripts/lane_follow.py`、
  `ucar_ws/src/lane_proto/test/`、`docs/changes/`、`docs/operations.md`。
- **结果**：`RosFrameGrabber` 现提供与原 `FrameGrabber` 一致的 `cam_fps` 属性，并按 ROS
  图像回调每秒更新一次实际帧率。
- **验证**：车端 Melodic Python2 定向回归先稳定复现 AttributeError；修复后 Python2
  编译、定向回归与 lane_proto 全量 5 项 Catkin 测试均为 0 errors / 0 failures。
- **风险**：当前已运行的 lane_follow 已退出；部署本身不会重启用户正在运行的主流程。

- **状态**：改动完成
- **目标**：将 AHRS `crc16` rosout 从任务中止条件改为告警继续。
- **影响文件**：`ucar_ws/src/ucar_2026/scripts/production_task_2026.py`、
  `ucar_ws/src/ucar_2026/test/`、`docs/changes/`、`docs/operations.md`。
- **结果**：AHRS CRC16 只记录 `PRODUCTION_AHRS_CRC_IGNORED` 并继续；其他已存在的
  rosout 错误标记保持原处理。
- **验证**：先由定向回归复现 AHRS CRC 会写入 `critical_error`；修复后车端定向回归与
  ucar_2026 全量 96 项 Catkin 测试均为 0 errors / 0 failures。
- **风险**：AHRS CRC 出现后姿态来源可能不可靠；本改动由用户明确要求仅记录告警。

- **状态**：改动完成
- **目标**：为仿真 `/status` 断连加入重连，并在 120 秒未完成时继续小车后续任务。
- **影响文件**：`ucar_ws/src/ucar_2026/scripts/production_task_2026.py`、
  `ucar_ws/src/ucar_2026/launch/2026.launch`、`ucar_ws/src/ucar_2026/test/`、
  `docs/changes/`、`docs/operations.md`。
- **结果**：`BadStatusLine` 等 HTTP 连接关闭会在下一轮新建连接重试；120 秒未完成、failed 或
  持续断连均发布 `SIMULATION_TIMEOUT_CONTINUE` 并继续小车流程；真实 done 与超时继续都播报
  仿真任务已完成。
- **验证**：车端 Python2 语法检查通过；done、failed 超时继续、running 超时继续、
  BadStatusLine 重连后 done 共 4 项定向回归通过；完整 Catkin 回归 ucar_2026 95 项、
  lane_proto 4 项均为 0 errors / 0 failures。
- **风险**：超时继续意味着仿真结果未确认，终端日志必须明确标注该状态。

- **状态**：改动完成
- **目标**：避免 lane_follow 退出时因 `required=true` 关闭整个 2026 主流程。
- **影响文件**：`ucar_ws/src/lane_proto/launch/lane_proto.launch`、
  `ucar_ws/src/lane_proto/test/`、`docs/changes/`、`docs/operations.md`。
- **结果**：lane_follow 改为非 required 节点；其退出仅记录退出，不再触发父 launch shutdown。
- **验证**：本地 launch 属性回归 4 项通过（无 ROS 的本机跳过 1 项）；车端完整 Catkin
  回归 lane_proto 4 项、ucar_2026 95 项均为 0 errors / 0 failures。
- **风险**：lane_follow 非正常退出后主流程保持运行，须由现场日志继续定位巡线自身退出原因。

- **状态**：改动完成
- **目标**：修复 lane_proto 在 ROS Melodic Python2 启动日志中直接传入中文导致的
  `rospy.loginfo` 格式化崩溃。
- **影响文件**：`ucar_ws/src/lane_proto/scripts/lane_follow.py`、
  `ucar_ws/src/lane_proto/test/`、`docs/changes/`、`docs/operations.md`。
- **结果**：节点内统一预格式化 `info/warn/error`（含节流 error）的 Unicode/UTF-8
  参数后再交给 rospy，保留中文终端输出且不会触发 Python2 logging 的二次混合格式化。
- **验证**：本机 Python3 语法检查与 3 项回归（1 项因缺少 ROS 跳过）通过；车端
  Melodic Python2 编译与 lane_proto 3 项回归通过，0 errors / 0 failures；部署文件 SHA-256
  已核对。
- **风险**：修复仅在下一次启动 lane_follow 时装载；当前没有替用户启动主流程或车辆。

- **状态**：改动完成
- **目标**：修复常驻 lane_proto 用默认 Python3 加载 Melodic Python2 `cv_bridge_boost` 时的
  `PyInit_cv_bridge_boost` 回调异常。
- **影响文件**：`ucar_ws/src/lane_proto/scripts/lane_follow.py`、
  `ucar_ws/src/lane_proto/launch/lane_proto.launch`、`ucar_ws/src/lane_proto/CMakeLists.txt`、
  `docs/changes/`。
- **结果**：`lane_follow.py` 固定为 `python2`，launch 同时经 Melodic Python2 启动器执行；
  新增运行时解释器回归测试。
- **验证**：先复现默认 Python3 导入 cv_bridge 失败、Python2 成功；修复后本地 2 项测试、
  车端 lane 2 项与 ucar_2026 94 项 Catkin 测试均通过。
- **风险**：解释器修复在下一次 lane 节点启动时生效，运行中的旧 Python3 进程不会原地切换。

- **状态**：改动完成
- **目标**：将 2026 主流程 ROS Master 迁回小车，并将终点巡线改为常驻待命、单一速度控制权切换，移除语音监听的 Unicode 转义重复日志。
- **影响文件**：`ucar_ws/src/ucar_2026/`、`ucar_ws/src/lane_proto/`、`rosmaster/NETWORK_CONFIGURATION.md`、`docs/operations.md`、`docs/changes/`。
- **结果**：已从小车同步 `lane_proto` 源码；主流程改为小车本机 Master、常驻 lane_proto
  与 `/cmd_vel` 单一仲裁。终点不再退出/重启 launch，也未新增中途停车步骤；语音状态仅显示
  `[语音] 中文`，不再输出 Unicode 转义重复行。
- **验证**：本地语法/XML/shell/diff 检查通过；车端 Python2/shell、Catkin 构建和 94 项
  回归均通过，已逐文件 SHA-256 对齐；`check` 模式已验证小车 `192.168.8.231:11311`
  Master 正常启动并随脚本退出清理，全程未发送运动命令。
- **风险**：部署时检测到的旧版独立 lane_proto 已在用户明确要求后正常停止；新版可在下一次
  `start_2026.sh <电脑LAN_IP> mission` 启用。lane_proto 的旧里程计冻结
  测距分支仍须从底盘链路根因修复，不能把 TF/odom 新鲜度异常当作静止的正常现象。

- **状态**：改动完成
- **目标**：修复 E2 mission 终端的动态 WSL Master 传递、ABORTED 后旧巡线日志尾随，并将 move_base 就绪等待恢复为 180 秒。
- **影响文件**：WSL 侧 `~/.config/smartcar/term_e_mission.sh`、`~/.config/smartcar/ros_network.env`、`ucar_ws/src/ucar_2026/scripts/start_2026.sh`、`ucar_ws/src/ucar_2026/launch/2026.launch`、`docs/operations.md`、`docs/changes/2026-08-14-terminal-visible-logs.md`。
- **结果**：E2 从 WSL 当前 `ROS_MASTER_URI` 动态传递 Master 地址，并移除 WSL `ros_network.env` 遗留的旧 IP 覆盖；仅在操作员确认起点、实际启动 mission 后截断本次 `lane_handoff.log`，ABORTED 后不会显示旧巡线输出，取消启动仍保留旧日志；`move_base_ready_timeout` 已恢复为 180 秒，操作与变更文档已同步。
- **验证**：WSL E2 脚本与 `start_2026.sh` 均通过 `bash -n`；移除 WSL 配置覆盖后动态解析为当前 `ROS_MASTER_URI=http://192.168.8.152:11311`；确认 E2 无 `192.168.8.199` 且保持 Unix LF；`2026.launch` 通过 XML 解析；目标文件 `git diff --check` 通过。`start_2026.sh` 与 `2026.launch` 已部署到小车，远端 `bash -n`、180 秒参数检查及 SHA-256 与本地一致。
- **风险**：未启动 ROS 或车辆，仍需在下一次现场任务按「取消 mission 保留旧日志 / 实际启动清空日志 / ABORTED 不显示旧日志 / SUCCEEDED 实时显示巡线」完成行为验证。

- **状态**：改动完成
- **目标**：让 mission 终端（E2）可见三类过程日志——语音等待/识别进度、TTS 播报内容、终点后 lane_proto 巡线日志。
- **影响文件**：`ucar_ws/src/ucar_2026/scripts/production_task_2026.py`、`ucar_ws/src/ucar_2026/scripts/handoff_lane.sh`、WSL 侧 E2 终端脚本 `~/.config/smartcar/term_e_mission.sh`。
- **结果**：语音监听启动打 `PRODUCTION_VOICE_WAITING`，监听子进程非 JSON 状态行以 `PRODUCTION_VOICE_LISTENER` 转发终端；`speak`/`speak_wait` 播报前打 `PRODUCTION_TTS_SPEAK text=<全文>`；`handoff_to_lane` 的 setsid 输出重定向 `~/.ros/lane_handoff.log`（不再丢 /dev/null），`handoff_lane.sh` 的 lane_proto 加 `--screen`，E2 终端任务结束后 `tail -F` 展示巡线日志。
- **验证**：车端 `python2 -m py_compile`、`bash -n` 通过；车端 `catkin_make -DCATKIN_WHITELIST_PACKAGES=ucar_2026 run_tests` 94 tests、0 errors、0 failures、0 skipped；两文件已 scp 小车并 chmod 0755。
- **风险**：`PRODUCTION_VOICE_LISTENER` 只转发完整行（无换行的进度行会延迟到下次换行）；任务 ABORTED 不产生交接日志，E2 停在等待提示属预期。

## 2026-08-12

- **状态**：改动完成
- **目标**：在桥接服务等待小车开始信号时降低 Gazebo 物理更新频率，并在任务前恢复原值。
- **影响文件**：`simulation/bridge/sim_bridge.py`、`simulation/bridge/README.md`、`simulation/bridge/test_sim_bridge.py`
- **结果**：bridge 启动后保存当前物理属性并将待机 `max_update_rate` 降为 100 Hz；`POST /start` 先恢复保存值，再启动任务。
- **验证**：Windows 与 WSL Ubuntu 20.04 均通过 `test_sim_bridge.py`；WSL 已验证 `rospy`、`GetPhysicsProperties`、`SetPhysicsProperties` 可导入。
- **风险**：尚未在正在运行的 Gazebo 实例上执行服务调用，现场首次启动应核对 bridge 日志中的保存与恢复频率。

# AI 变更记录

## 2026-08-12｜仿真 bridge 纳入克隆仓库（改动完成）

- **目标**：将真车—仿真 HTTP bridge 作为 `smartcar2026-simulation` 的受版本控制文件，使新电脑克隆仿真仓库即可获得它。
- **涉及文件**：`../simulation/smartcar2026-simulation/bridge/`、仿真仓库 README，以及本仓库的新电脑运行指南。
- **结果**：`bridge/sim_bridge.py` 和运行说明已纳入仿真仓库 `main`，新电脑克隆后直接运行 `python3 bridge/sim_bridge.py`；运行日志已排除版本控制；真车新电脑指南与 operations 已同步去除外部交付步骤。
- **验证**：提交 `df3422140cb3367c1f8ff9b10bacb2dcca658019` 已推送并 fast-forward 到 WSL；bridge 的 Python 3 help/语法检查通过，无 ROS/Gazebo/bridge 残留。Noetic 全量构建被 WSL `p9_client_rpc` 文件系统阻塞，已中断本次构建进程树，待文件系统恢复后重跑。
- **风险**：bridge 接入了局域网 HTTP 端口 11313；仍须使用可信 Wi-Fi 的 LocalSubnet 防火墙规则，并在仿真 11312 Master 已就绪后启动。

## 2026-08-11｜生产 OCR 巡航转速与车端部署（改动完成）

- **目标**：将到点 OCR 整圈巡航角速度从 `0.25 rad/s` 提升为 `0.30 rad/s`，并将当前未上车的生产任务依赖同步至小车 Ubuntu 18.04 后完成车端构建与回归。
- **涉及文件**：`ucar_ws/src/ucar_2026/scripts/production_task_2026.py`、`ucar_ws/src/ucar_2026/launch/2026.launch`、`docs/operations.md`、`docs/changes/2026-08-11-ocr-cruise-speed-and-vehicle-deploy.md`。
- **结果**：Python 默认值和 launch 显式值均更新为 `0.30 rad/s`；完整生产任务依赖已逐路径同步至小车，并修正文档中的 Catkin 测试目标为工作区级 `run_tests`。
- **验证**：五个上传文件 SHA-256 全部一致；车端 Python 2 语法检查、launch XML、`catkin_make --pkg ucar_2026` 均通过；直接任务回归 82 tests OK，工作区 Catkin 回归 94 tests、0 errors、0 failures、0 skipped。
- **风险**：补充边界路线含角点，完整外圈运动须先核验原地旋转净空；本次未启动 ROS launch 或运动小车。

## 2026-08-11｜生产任务补充巡航与仿真顺序（改动完成）

- **目标**：扫码收齐两件物品后、驶往点 3 前完成物品播报；主巡航未定位类别时补跑指定边界路线，仍缺失则驶往 441 交接；先完成实物停靠/播报，再停靠及启动仿真；仿真完成等待上限改为 150 秒。
- **涉及文件**：`ucar_ws/src/ucar_2026/scripts/production_task_2026.py`、`ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`、`ucar_ws/src/ucar_2026/launch/2026.launch`、`ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`、`docs/changes/2026-08-11-production-fallback-and-simulation-order.md`。
- **结果**：二维码确认与仓库归属播报被移到点 3 前；首轮 OCR 可先记录仿真墙面、却强制先停靠和播报实物；主路线缺失时补跑 `1–10→20→30→40→39–31→21→11`，仍缺失则停 OCR 后驶往 441 交接；仿真完成等待统一为 150 秒。补充路线边界点不伪造中线四点守卫，改用导航安全层并在审计中标明路线。
- **验证**：本机 `python -m py_compile` 通过；`python ucar_ws/src/ucar_2026/test/test_production_task_geometry.py` 为 82 tests OK（67 个 ROS 依赖用例跳过）；`2026.launch` XML 解析通过；主路线守卫几何检查为 16 个目标、64 个顶点；`git diff --check` 通过。
- **风险**：本机未安装车端 ROS Melodic，新增 ROS 状态机用例仍需在小车 Ubuntu 18.04 上以 Python 2/Catkin 完整回归；补充边界路线依赖通用安全门和 `move_base` 动态障碍层，未采用内侧格专用的四顶点目标守卫。

## 2026-08-11｜语音双物品任务输入（改动完成）

- **目标**：将 2026 主流程启动时的两个终端物品输入替换为“小飞小飞”唤醒后的固定双类别语音指令，并由二维码解析出对应的真实物品名。
- **涉及文件**：`ucar_ws/src/ucar_2026/scripts/production_task_2026.py`、`ucar_ws/src/ucar_2026/launch/2026.launch`、`ucar_ws/src/ucar_2026/scripts/start_2026.sh`、`ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`、`docs/operations.md`、`docs/changes/2026-08-11-voice-dual-item-input.md`。
- **结果**：任务以 `wake_listen.py --loop --asr --set-wake 小飞小飞 --json` 接收不同的现实/仿真类别；二维码阶段按类别回填真实物品名，后续播报、结果和仿真桥接保持传递真实物品名。无效语音继续等待，且语音进程在接受结果或异常时被终止。
- **验证**：小车 Ubuntu 18.04 / ROS Melodic 上 `python2 -m py_compile` 通过；`python2 test/test_production_task_geometry.py` 为 81 tests OK；`catkin_make run_tests_ucar_2026 -j2` 与 `catkin_test_results build/test_results/ucar_2026` 为 93 tests、0 errors、0 failures、0 skipped。
- **风险**：语音脚本仅返回类别而非二维码上的物品名，因此 `resume_production_only=true` 不支持语音模式；现场运行依赖麦阵列、讯飞 ASR 和语音脚本密钥可用。

## 2026-08-11｜GUI 仿真联动与实车全流程（验证完成）

- **目标**：在 GUI Gazebo/RViz 已就绪的前提下，运行蛋糕、耳机双物品真车全流程并完成仿真与巡线交接。
- **结果**：桥接服务返回 `done`；真车到达 441 后报告 `SUCCEEDED`；lane_proto 最终发布 `STOPPED`。
- **验证**：小车安全门、二维码固定面推进、连续 OCR 对准、两类停入、本地仿真和终点巡线全部完成；未见 NaN、TF_NAN_INPUT 或 CRC。
- **收尾**：已停止车端导航/巡线与独立仿真 Master、Gazebo、RViz、bridge，扬声器 PCM 保持 50%。

## 2026-08-11｜OCR 连续视觉对准（改动完成）

- **目标**：OCR 对准期间连续转动并持续更新角速度；仅在像素误差达标后停车确认，再测量激光距离。
- **涉及文件**：`ucar_ws/src/ucar_2026/scripts/production_task_2026.py`、`ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`。
- **结果**：OCR 未对准时以异步抓拍和持续角速度构成闭环；首次居中后才停车确认并测距。
- **验证**：本机 `python ucar_ws/src/ucar_2026/test/test_production_task_geometry.py` 通过（78 tests，63 skipped）；2026-08-11 已同步车端 Ubuntu 18.04 / ROS Melodic，`run_tests_ucar_2026` 通过（90 tests，0 errors，0 failures，0 skipped）。
- **风险**：连续转动期间必须保持 `/cmd_vel` 周期发布，并在任何 OCR/安全异常路径立即发零速；相邻 OCR 帧的推理延迟需实车观察是否收敛。

## 2026-08-11｜扫码固定观察面推进（改动完成）

- **目标**：收到非目标二维码后继续 180°、90°、-90° 固定观察序列，避免在当前面直接转圈。
- **涉及文件**：`ucar_ws/src/ucar_2026/scripts/production_task_2026.py`、`ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`。
- **结果**：将非目标二维码与无二维码事件分开处理；前者立即推进下一固定观察面，后者保留转圈兜底。
- **验证**：本机 `python ucar_ws/src/ucar_2026/test/test_production_task_geometry.py` 通过（77 tests，62 skipped）；2026-08-11 已同步车端 Ubuntu 18.04 / ROS Melodic，`run_tests_ucar_2026` 通过（90 tests，0 errors，0 failures，0 skipped）。
- **风险**：本机缺少 ROS Melodic，包含新增用例的任务类测试会跳过。

## 2026-08-20｜OCR 同类候选重复转向抑制

- **目标**：修复到点 OCR 识别到同一类别但墙面对准失败后，反复停车、反向恢复候选角度并再次丢失的问题。
- **涉及文件**：三套 `ucar_2026*` 生产任务脚本和几何测试、`ucar_source_code/docs/changes/2026-08-20-ocr-candidate-retry-suppression.md`、`ucar_source_code/docs/operations.md`。
- **结果**：每个到点 OCR 整圈为同一类别维护拒绝集合；第一次对准失败后，后续同类候选只丢弃并保持原方向旋转。
- **验证**：三套脚本 Python 语法检查和 `git diff --check` 通过；本机无 ROS Melodic，ROS 依赖测试按现有约定跳过，待车端 Python 2/Catkin 回归。
- **风险**：同一物理点本轮首次对准失败后，不再在同一整圈重复尝试该类别；后续路线仍可在其他点继续识别。

## 2026-08-20｜车端任务脚本执行权限恢复

- **问题**：临时文件同步后脚本权限变为 `0644`，`roslaunch` 报无法定位 `production_task_2026.py`。
- **处理**：恢复三套车端任务脚本的执行位为 `0755`，并用 Melodic 环境的 `roslaunch --nodes` 完成静态节点解析。
- **结果**：`ucar_2026/production_task_2026.py` 已可被 ROS 识别；未启动任务、未发送运动指令。
