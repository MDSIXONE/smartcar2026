# 2026 主流程启动指南（其他电脑/新操作者）

本文档面向**首次在这台控制电脑上启动 2026 智能车主流程**的操作者。主流程
（物品输入 → 单二维码 → 大模型分类 → 目标加工区停车停留 → 终点）已在实车
连续多次验证通过（最近两次 mission_run11 / mission_run12 均 SUCCEEDED）。

只需按顺序执行下列步骤，不需要理解任务代码内部实现。

---

## 0. 前置条件（一次性的准备）

| 项目 | 要求 |
| --- | --- |
| 控制电脑 | Windows 10/11 + WSL Ubuntu 20.04，装有 ROS Noetic（仅作为 Master） |
| 小车 | Ubuntu 18.04 / ROS Melodic，`ucar_ws` 工作区，代码已部署（见第 1 节） |
| 网络 | 控制电脑与小车上同一 Wi-Fi / 同一 IPv4 局域网；小车**不得**运行 `roscore` |
| 仓库 | `https://github.com/MDSIXONE/smartcar2026` 分支 `simulation_real` |

首次使用前需在 WSL 安装控制端脚本（与仓库同步，**必须从仓库复制，不能手写**）：

```bash
# 在 WSL 终端执行；把路径换成你 clone 仓库的实际路径
mkdir -p ~/.config/smartcar/python_http10_compat
cp /mnt/<盘符>/<仓库路径>/rosmaster/start_ros_master.sh ~/start_ros_master.sh
cp /mnt/<盘符>/<仓库路径>/rosmaster/python_http10_compat/sitecustomize.py \
  ~/.config/smartcar/python_http10_compat/sitecustomize.py
chmod 0755 ~/start_ros_master.sh
```

> 网络细节与故障排查见 `rosmaster/NETWORK_CONFIGURATION.md`（地址全部动态
> 发现，不要写死 IP）。

---

## 1. 把代码同步到小车（首次部署或代码更新后）

在**控制电脑**（Windows PowerShell 或 WSL）执行。以下 `小车IP` 用小车当前
Wi-Fi 地址（可问队友或 `nmap` 扫描，历史地址 192.168.8.231 仅作示例）。

```powershell
# 需要同步的关键文件（以仓库路径为准）
$src = "D:\<你的仓库路径>\ucar_ws\src\ucar_2026"
scp $src\scripts\production_task_2026.py      ucar@<小车IP>:~/ucar_ws/src/ucar_2026/scripts/
scp $src\scripts\production_task_geometry.py  ucar@<小车IP>:~/ucar_ws/src/ucar_2026/scripts/
scp $src\scripts\production_qr_classifier.py  ucar@<小车IP>:~/ucar_ws/src/ucar_2026/scripts/
scp $src\scripts\start_2026.sh                ucar@<小车IP>:~/ucar_ws/src/ucar_2026/scripts/
scp $src\launch\2026.launch                   ucar@<小车IP>:~/ucar_ws/src/ucar_2026/launch/
```

校验文件一致（本机哈希与小车端 sha256sum 必须相同）：

```powershell
Get-FileHash "$src\scripts\production_task_2026.py" -Algorithm SHA256
ssh ucar@<小车IP> "sha256sum ~/ucar_ws/src/ucar_2026/scripts/production_task_2026.py"
```

在小车上编译并跑单测（**只能在小车上编译**，本机不可编译）：

```bash
ssh ucar@<小车IP> "source /opt/ros/melodic/setup.bash && cd ~/ucar_ws && \
  catkin_make -DCATKIN_WHITELIST_PACKAGES='ucar_controller;ucar_2026' run_tests_ucar_2026 && \
  catkin_test_results build/test_results/ucar_2026"
```

预期输出：`60 tests, 0 errors, 0 failures, 0 skipped`。

> 若报 `No rule to make target 'ucar_2026/all'`：先看车上
> `grep CATKIN_WHITELIST_PACKAGES build/CMakeCache.txt`，用其中实际值覆盖。

---

## 2. 启动 ROS Master（控制电脑 WSL）

打开 WSL 终端，**第一个启动的必须是 Master**：

```bash
unset ROS_IP ROS_HOSTNAME ROS_MASTER_URI
~/start_ros_master.sh
```

看到类似输出即成功（记下中间的地址，它是本机 IP）：

```text
Starting ROS Master at http://192.168.8.199:11311 (ROS_IP=192.168.8.199)
XML-RPC=HTTP/1.0
```

> 若显示 `localhost`/`127.0.0.1`：说明网卡选择失败，`Ctrl-C` 停止，检查
> Wi-Fi 连接后重试，不要继续。小车端**绝不运行 roscore**。

---

## 3. 把车放回起点（每次任务前必须）

- 任务定位初值固定为起点 `(-0.25, 2.75, 0)`。
- 把小车**物理推回起点**，车头方向朝场内（x 负方向指向场地内部）。
- **先放车，再启动任务；任务进行中若中止，必须先停任务、放回起点，才能重启。**
- 顺序规则（重要）：**确认放回起点 → 停止旧任务（若有）→ 启动新任务**，
  不允许提前停止或提前重启。

---

## 4. 启动主流程任务（mission 模式）

在小车终端（或 ssh 登录小车）执行：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <Master地址> mission
```

脚本会依次：

1. 自动检测 ROS 环境并连接 WSL Master（不启动小车端 roscore）；
2. 自动关闭 USB autosuspend（防止串口掉线，日志会打印“USB autosuspend 已关闭”；
   失败仅警告，不阻塞）；
3. 询问 `是否已把车放回起点？`——**输入 `yes` 回车**才继续，其他输入取消启动；
4. 启动 `2026.launch`（定位、底盘、相机、二维码、任务节点）。

任务节点启动后打印提示并**阻塞等待物品输入**：

```text
PRODUCTION_TASK_INPUT_PROMPT: 请输入本次放置的物品名称后回车
```

在**同一个终端**直接输入本次物品名并回车（如：`蛋糕`、`苹果`、`螺丝刀`）。
输入为空会中止任务；只有收到物品名任务才进入 QR 阶段。

---

## 5. 任务流程与状态观察

任务自动执行（无需人工干预）：

```
WAITING_FOR_ITEM（等输入）→ WAITING_SAFE_START（安全检查）
→ STAGING_52（去 QR 区）→ QR_FACE_262（只扫 1 个二维码）
→ WAYPOINT_3 →（星火大模型分类物品 → 确定目标类别）
→ PRODUCTION_TARGET_xxx（沿生产路线巡航，OCR 识别加工厂招牌，
   命中目标类别即停；有障碍的点自动守卫跳过）
→ PROCESSING_STOP_xxx（墙交点向内 25cm 停车，车头朝外）
→ PROCESSING_DWELL_xxx（停留 3 秒）
→ DESTINATION_35_36（终点：35/36 连线中点，车头朝 170）
→ SUCCEEDED
```

语音播报（USB 音箱，失败不影响任务）：启动时「初始化完成，准备开始任务」、
扫码分类后「取得*<物品名>*属于*<类别>」。

在控制电脑任一终端（已 `source` 且 ROS_MASTER_URI 指向 WSL Master）观察：

```bash
rostopic echo /ucar_2026/task_state   # 当前状态（见上表）
rostopic echo /ucar_2026/task_result  # 结束后输出 JSON 结果
ls -1dt ~/.ros/ucar_2026_observations/run_* | head -n 1   # 车上结果目录
```

> 任务日志在小车 `~/.ros/log/latest/production_task_2026-*.log`，或
> `rostopic echo /rosout`。若日志多目录并存，按进程 PID 的 `__log` 参数确认
> 最新文件（历史教训：旧会话目录会被复用，grep 会混入旧内容）。

---

## 6. 停止任务（每次任务结束后必须）

任务结束后（SUCCEEDED 或 ABORTED），**必须停止任务，不留后台进程**：

```bash
# 在小车终端，先恢复 Master 网络变量再停止
MASTER_IP=<Master地址> bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
```

看到 `ucar_2026 2026.launch task stopped; ROS Master remains running.` 即成功。
确认无残留：

```bash
ps aux | grep -E 'roslaunch|production_task' | grep -v grep   # 应为空
```

紧急停止：在启动 launch 的终端按 `Ctrl-C`；终端丢失时用上面的 stop 脚本。

---

## 7. 常见问题速查

| 现象 | 处理 |
| --- | --- |
| `RLException: multiple files named [2026.launch]` | 有文件被误传到 `scripts/` 目录，删除该文件重新启动 |
| 日志出现 `/odom_raw` NaN 或 `TF_NAN_INPUT` | 停止任务，重启导航/底盘链路；确认 odom 有限且 TF 恢复后再重试 |
| 任务中途 ABORTED | 看 task_result 的 reason；多数是点 12 等有障碍被守卫跳过（正常）或停车确认超时（已放宽到 4s） |
| 大模型分类结果错误 | 任务喂给大模型的是你输入的**物品名**（不是二维码文本）；确认输入的是真实物品名 |
| `move_base unavailable after 90s` | 检查 WSL Master 是否还活着、小车与 Master 网络是否正常 |
| 想重跑一次 | 先按第 6 节停止 → 放回起点 → 再按第 4 节启动 |
| 车上有残留任务进程 | 先执行第 6 节停止脚本，再启动新任务（顺序不可颠倒） |

---

## 8. 相关文档

- `docs/operations.md`：部署、构建、测试的完整操作命令。
- `docs/quickstart.md`：日常手动导航、RViz 使用。
- `rosmaster/NETWORK_CONFIGURATION.md`：ROS 网络动态配置。
- `docs/changes/2026-08-07-item-input-single-qr-target-category.md`：本主流程
  的改造记录（含实车验证序列）。
