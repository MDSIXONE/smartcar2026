# AI 改动记录

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
