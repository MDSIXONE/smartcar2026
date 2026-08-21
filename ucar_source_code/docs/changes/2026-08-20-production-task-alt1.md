# 省赛备用方案一：超时连续对准版生产任务（production_task_2026_alt1）

## 目的

不动省赛原 `production_task_2026.py` / `2026.launch`，复制出备用方案一入口。省赛主流程
已回退到实车验证版 `17b39b3`（完善 OCR 对准与导航恢复流程），alt1 基于该版本复制，
把 OCR 连续对齐改成与独立对齐入口（`ocr_alignment.launch`）一致的行为——固定 15s 墙钟
预算、固定 30px 容差、发散两次仅重置 PD 导数继续对准、空检测继续边转抓帧——并将任务
节点改为直接发布 `/cmd_vel` 速度到底盘驱动（不经 `cmd_vel_owner` 仲裁）。对齐成功后
的前向激光测距、墙体射线交点、墙点匹配与停车坐标决定逻辑保持不变。用于省赛主流程在
实车对齐不理想时快速切换验证。

## 涉及文件（本机与车端均已同步，SHA-256 一致）

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`：回退到 `17b39b3` 版本
  （无 OCR 后退 25cm 复核/墙点吸附纠正/尾向停车）。
- `ucar_ws/src/ucar_2026/launch/2026.launch`：回退到 `17b39b3` 版本（无
  `ocr_recheck_backoff_m` 参数）。
- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py` 与对应两个测试：回退到
  `17b39b3` 版本。
- `ucar_ws/src/ucar_2026_national/`、`ucar_ws/src/ucar_2026_extra/` 两个包的
  `launch/2026.launch`、`scripts/production_task_2026.py`、
  `scripts/production_task_geometry.py` 及对应两个测试：同样回退到 `17b39b3` 版本，
  三套主流程保持一致（均无 OCR 后退 25cm 复核/墙点吸附纠正/尾向停车）。
- `ucar_ws/src/ucar_2026/scripts/production_task_2026_alt1.py`：基于回退版复制，仅改动
  `observe_wall` 对齐循环与参数/异常。
- `ucar_ws/src/ucar_2026/launch/2026_alt1.launch`：基于回退版 `2026.launch` 复制，
  任务节点换用 alt1 脚本、`cmd_vel_topic=/cmd_vel`、新增 `ocr_alignment_timeout=15.0`、
  `result_directory=~/.ros/ucar_2026_alt1`。
- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`：新增 `mission_alt1` 模式，
  `bash start_2026.sh <PC_LAN_IP> mission_alt1` 一键启动 alt1 入口（复用 mission 的
  网络预检、起点确认与仿真 bridge 检查）。
- `ucar_ws/src/ucar_2026/test/test_2026_alt1_launch.py`：新回归测试。
- `ucar_ws/src/ucar_2026/CMakeLists.txt`：注册安装 alt1 脚本并登记新测试。

原 `production_task_2026.py` 与 `2026.launch` 在回退到 `17b39b3` 后不再改动（有测试
断言保护）。

## 行为边界

`observe_wall` 对齐改为：以 `time.time() + ocr_alignment_timeout`（15s）为墙钟截止；
连续边转抓帧；`attempt>5` 不再放宽容差（固定 30px）；发散两次只重置 PD 导数继续；
抓不到框时已在转则继续边转抓帧、未在转则静止抓帧；超时或未对齐时返回 `aligned=False`
的 observation，调用方将该类别加入 `rejected_categories`，车辆从当前朝向继续转完
剩余 360°，期间其他未处理类别仍可进入对齐；不再因对齐发散 `MissionAbort` 中止任务。
对齐成功后保留原测距/墙点匹配/停车坐标逻辑。任务节点直发 `/cmd_vel`，与 `cmd_vel_owner`
同 topic 双写（move_base 在 OCR/QR 对齐阶段已 cancel goal 空闲，实际仍为单一写者）。

## 验证

- 本机：alt1 脚本 Python AST 语法、launch XML 解析通过；新测试 `4/4`、`ocr_alignment`
  `4/4`、`ocr_search` `5/5` 共 13 项通过；`git diff --check` 通过；`diff` 确认 alt1 与
  原文件差异仅为预期改动点。
- 车端（ucar-mini，192.168.8.231）：4 个文件 SHA-256 与本地一致；Python2 `py_compile`
  通过；launch XML 解析确认 `cmd_vel_topic=/cmd_vel`、`ocr_alignment_timeout=15.0`；
  `roslaunch --nodes ... task_enabled:=true` 能解析出 alt1 节点；无 alt1/生产任务残留进程。
  未启动 ROS、未发送运动指令。

## 已知限制

- 直发 `/cmd_vel` 后不再有 cmd_vel_owner 的仲裁隔离；若验证期间另起会写 `/cmd_vel` 的
  节点会竞争，验证前应确认没有其他运动源运行。
- 备用入口为完整任务，默认 `task_enabled=false` 不会启动任务节点，跑任务须显式传
  `task_enabled:=true`。
- 对齐超时后放弃的类别要等下一圈/下一个点重新出现才会再次尝试对齐。