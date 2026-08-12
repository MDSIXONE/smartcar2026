# Simulation Bridge（任务三仿真桥接服务）

WSL 侧的 HTTP 桥接服务，让小车通过局域网 **单向** 访问仿真工程：
小车 POST 启动仿真任务（物品名透传），随后轮询 GET 完成状态（完成状态回读）。
WSL 不主动回连小车。

本服务不会修改规划器 / URDF / world 等仿真工程文件，也不干预路径。bridge
在等待小车开始信号期间通过 Gazebo 物理服务将 `max_update_rate` 临时降频，降低
待机阶段的 UI 负载；收到 `POST /start` 后先恢复启动时读取的原始值，再启动任务。

- 脚本：`sim_bridge.py`（Python 3.8+、ROS Noetic 的 `rospy` 与 `gazebo_msgs`）
- 默认端口：`11313`（参数 `--port` 可改）
- 运行日志：bridge 目录下 `task3_run_<YYYYmmdd_HHMMSS>.log`（roslaunch 完整输出）

## 与小车端的接口协议

### POST /start

启动仿真任务。请求体为 UTF-8 JSON：

```json
{"item_name": "苹果", "category": "日用品"}
```

| 场景 | HTTP | 响应体 |
| --- | --- | --- |
| 当前 waiting，启动成功 | 200 | `{"accepted": true, "state": "running"}` |
| 当前 running | 409 | `{"error": "already running"}` |
| 当前 done / failed | 409 | `{"error": "already finished, restart bridge for another run"}` |
| JSON 解析失败 / 缺 item_name | 400 | `{"error": "invalid JSON body"}` 或 `{"error": "missing item_name"}` |

内部等价于执行：

```bash
roslaunch car3 task3_execute.launch \
  cargo_category:="<category>" cargo_name:="<item_name>"
```

（两个参数都传，不传 `cargo_item`，避免与 category 冲突；`category` 缺失时用
launch 默认值 `auto`。）

### GET /status

查询当前状态，HTTP 200：

```json
{"state": "running", "detail": "task running", "item_name": "苹果", "category": "日用品"}
```

`state` 取值：`waiting` | `running` | `done` | `failed`。
`detail` 为状态细节：`ready`、`task running`、`done`、
`roslaunch exited with code N`、`timeout waiting /sim_task3/done`。

### 状态机

```
waiting --POST /start--> running --/sim_task3/done=True--> done
                             |--roslaunch 非零退出且未收到 done--> failed
                             |--轮询超时（--done-timeout 默认 1800s）--> failed
```

完成信号：话题 `/sim_task3/done`（std_msgs/Bool，latched，完成后 True）。
一次运行结束后如需再次运行，需重启 bridge 进程。

## 部署方式

把本 `bridge` 目录复制到 WSL（Ubuntu 20.04）用户目录下：

```bash
mkdir -p /home/car/simulation_bridge
# 把本目录下的 sim_bridge.py 复制过去，例如：
#   scp sim_bridge.py car@<wsl-ip>:/home/car/simulation_bridge/
```

## 启动命令

先人工启动仿真准备（终端 A，等待日志出现
`calibrated initial arm pose applied smoothly` 后再启动 bridge）：

```bash
cd /home/car/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
roslaunch car3 task3_prepare.launch
```

> WSL2 默认在最后一个 `wsl` 会话退出后关闭发行版，后台的 roslaunch 会被
> 连带杀掉。建议在 `C:\Users\<用户>\.wslconfig` 的 `[wsl2]` 段设置
> `vmIdleTimeout=-1` 保持发行版常驻，再用 `wsl --shutdown` 重启 WSL 生效。
> 无法改配置时，启动命令必须用 `setsid nohup ... < /dev/null &` 脱离会话。

再启动 bridge（终端 B）：

```bash
cd /home/car/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
python3 /home/car/simulation_bridge/sim_bridge.py
```

默认待机频率为 `100 Hz`；需要调整时可传入（必须大于 0）：

```bash
python3 /home/car/simulation_bridge/sim_bridge.py --idle-update-rate 100
```

bridge 会先轮询 `/map` 话题（map_server 常驻发布，`--wait-ready-timeout`
默认 300s，0 表示跳过检查；不能用 `/sim_task3/arm_initial_pose_ready`——该
话题只发布一次且 set_arm_initial_pose 节点发布后即退出，话题会从 master
注销）。随后 bridge 调用 `/gazebo/get_physics_properties` 保存当前
`max_update_rate`，调用 `/gazebo/set_physics_properties` 设为待机频率；收到
`POST /start` 时会恢复保存值，恢复成功后才执行 `task3_execute.launch`。就绪后打印：

```text
SIMULATION_BRIDGE_READY
simulation bridge listening on 0.0.0.0:11313 (state=waiting)
```

小车即可 POST /start。任务完成时打印：

```text
SIMULATION_BRIDGE_DONE item=苹果 category=日用品
```

Ctrl-C 退出时 bridge 会尽量终止 roslaunch 子进程。

## 单独启动与验证仿真（不接小车）

用于确认 Gazebo/RViz 能正常显示，以及 bridge 的待机降频是否生效；本流程**不会**
发送 `POST /start`，因此不会启动取放任务。

1. 终端 A 启动仿真环境（Gazebo 与 RViz 会同时打开）：

   ```bash
   cd /home/car/smartcar2026-simulation
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash
   export ROS_MASTER_URI=http://127.0.0.1:11312
   roslaunch car3 task3_prepare.launch
   ```

   保持此终端和 Gazebo/RViz 窗口运行，等待 `/map` 可用。

2. 终端 B 启动 bridge，并将待机物理更新频率设为 100 Hz：

   ```bash
   cd /home/car/smartcar2026-simulation
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash
   export ROS_MASTER_URI=http://127.0.0.1:11312
   python3 /home/car/simulation_bridge/sim_bridge.py --idle-update-rate 100
   ```

   日志应包含类似内容：

   ```text
   Gazebo max_update_rate saved 1000.000 Hz; idle rate set to 100.000 Hz
   SIMULATION_BRIDGE_READY
   ```

3. 终端 C 确认 Gazebo 当前待机频率与 bridge 状态：

   ```bash
   source /opt/ros/noetic/setup.bash
   export ROS_MASTER_URI=http://127.0.0.1:11312
   rosservice call /gazebo/get_physics_properties
   curl -s http://127.0.0.1:11313/status
   ```

   第一条输出中的 `max_update_rate` 应为 `100.0`；第二条应返回
   `"state": "waiting"`。此时可观察 Gazebo/RViz 是否比默认待机状态更流畅。

4. 结束单独测试时，先在终端 B 按 `Ctrl-C` 停止 bridge，再在终端 A 按
   `Ctrl-C` 关闭仿真。由于本流程未发开始请求，退出前不需要恢复频率；下次
   `task3_prepare.launch` 启动 Gazebo 时会重新读取 world 的官方 `1000 Hz` 值。

若需要验证“开始前恢复原值”，则必须发送一次真实的 `POST /start`，它会启动完整
任务；不能把该请求当成无副作用的测试接口。

## Windows 防火墙放行说明

小车通过局域网 Wi-Fi 访问 WSL 的 11313 端口。WSL 处于镜像网络模式时，
Windows 防火墙必须放行 TCP 11313 的**入站**流量，且只允许局域网
（`LocalSubnet`），做法与
`ucar_source_code/rosmaster/NETWORK_CONFIGURATION.md` 中 ROS 规则同款：

管理员 PowerShell 创建规则（不存在时）：

```powershell
New-NetFirewallRule -DisplayName 'SimBridge 11313 from UCar to WSL' `
  -Direction Inbound -Protocol TCP -LocalPort 11313 `
  -RemoteAddress LocalSubnet -Action Allow

# WSL 镜像网络模式（Hyper-V 虚拟网卡）下还需 Hyper-V 规则：
New-NetFirewallHyperVRule -Name 'SimBridge 11313 from UCar to WSL' `
  -DisplayName 'SimBridge 11313 from UCar to WSL' `
  -Direction Inbound -Protocol TCP -LocalPorts 11313 `
  -RemoteAddresses LocalSubnet -Action Allow
```

netsh 等价写法：

```powershell
netsh advfirewall firewall add rule name="SimBridge 11313 from UCar to WSL" `
  dir=in action=allow protocol=TCP localport=11313 remoteip=localsubnet
```

既有规则若绑定旧 IP，需改回 `LocalSubnet`：

```powershell
Get-NetFirewallRule -DisplayName 'SimBridge 11313 from UCar to WSL' |
  Get-NetFirewallAddressFilter |
  Set-NetFirewallAddressFilter -RemoteAddress LocalSubnet
```

换 Wi-Fi 后端口规则无需改动（LocalSubnet 自适应），但仍需确认小车与
控制电脑在同一局域网。仅在受信任的专用局域网中使用 `LocalSubnet`。

小车侧连通性检查（WSL 的局域网 IP 用 `ip addr` 查）：

```bash
timeout 2 bash -c ">/dev/tcp/<wsl-ip>/11313" && echo OK
curl -s http://<wsl-ip>:11313/status
```

## 常见故障

| 现象 | 原因与处理 |
| --- | --- |
| `Address already in use` / 端口被占 | 11313 被占用（可能上次 bridge 未退净）。`pgrep -af 'sim_bridge|roslaunch'` 确认旧进程后退出，或用 `--port` 换端口（小车端同步修改）。 |
| `roslaunch: command not found` / `package 'car3' not found` | 启动终端的 ROS 环境未 source 或 Master 未指向仿真。确认启动前已执行 `source /opt/ros/noetic/setup.bash`、`source devel/setup.bash` 且 `export ROS_MASTER_URI=http://127.0.0.1:11312`（不要指向小车的 11311）。 |
| 一直打印 ready 轮询、超时退出（退出码 1） | `task3_prepare.launch` 未启动或未就绪。先人工启动准备，等日志出现 `calibrated initial arm pose applied smoothly` 或 `rostopic echo -n 1 /map` 有输出，再重启 bridge。 |
| bridge 报 Gazebo physics service 超时或恢复失败 | Gazebo 服务未就绪或已退出。确认 `task3_prepare.launch` 正在运行，且 `rosservice call /gazebo/get_physics_properties` 可成功返回后重启 bridge。 |
| WSL 后台 roslaunch 无故退出（master 日志出现 `keyboard interrupt`） | WSL2 会话退出导致发行版关闭，后台进程被连带终止。设置 `.wslconfig` 的 `[wsl2] vmIdleTimeout=-1` 并 `wsl --shutdown` 重启，或启动时用 `setsid nohup ... < /dev/null &`。 |
| 启动后长时间 running、`/sim_task3/done` 没数据 | 仿真任务仍在执行属正常（最多 `--done-timeout` 默认 1800s）；若 roslaunch 日志（`task3_run_*.log`）报错，按仿真 RUNBOOK/FAQ 排查（视觉、导航、物品名/类别）。 |
| 小车 POST 超时 / connection refused | 防火墙未放行 11313（见上节）；或小车访问的 IP 不是 WSL 当前局域网 IP；确认同网段后用 `curl -s http://<wsl-ip>:11313/status` 自查。 |
| 返回 `already finished, restart bridge for another run` | 一次运行结束后状态保持 done/failed；再次运行需重启 bridge 进程（Ctrl-C 后重新 `python3 .../sim_bridge.py`）。 |
