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
