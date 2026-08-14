# 新电脑运行真车双物品主流程与 GUI 仿真

> 适用场景：一台新的 Windows 电脑通过 WSL Ubuntu 20.04 接入新的 Wi-Fi；小车运行
> Ubuntu 18.04 / ROS Melodic；电脑上同时显示 Gazebo 和 RViz，并通过 bridge 联动仿真。
>
> 本文是这项任务的唯一启动顺序。所有 IP 均为示例，**每次换 Wi-Fi 都重新获取，绝不照抄旧 IP**。

## 0. 先读完这四条

1. 电脑和小车必须接入同一个可信 IPv4 局域网，且路由器不能开启 AP/client isolation。
2. 主流程 ROS Master 是小车本机的 `11311`；电脑 WSL **不运行真车 roscore**。
3. 仿真另用 WSL 内的 `127.0.0.1:11312`，与真车 Master 完全隔离；两者只由 HTTP bridge 的
   `11313` 端口衔接。
4. 在任何会带动车轮的命令前，都必须完成“第 7 节安全检查”。发现 `NaN`、`TF_NAN_INPUT`、
   `crc16`、`sensor not active` 或串口异常时，不得启动 mission。

完整链路如下：

```text
小车 Melodic ─ROS→ 小车本机 Master :11311
小车 Melodic ─HTTP→ WSL bridge :11313 ─ROS→ 仿真 Master :11312
                                              └─ Gazebo + RViz（可见窗口）
```

> 2026-08-14 起，本文下方所有“WSL 真车 Master / start_ros_master.sh / 将 MASTER_IP
> 传给小车”的旧段落均已退役。当前启动命令为小车端
> `bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <电脑LAN_IP> mission`；电脑地址
> 只用于 bridge HTTP，不是 ROS Master。完整当前网络说明见
> `rosmaster/NETWORK_CONFIGURATION.md`。

## 1. 一次性准备：新电脑、新 WSL 和代码

### 1.1 Windows 与 WSL

- 使用 Windows 11，安装 WSL 2 和 Ubuntu 20.04；确认 `wsl -l -v` 中 Ubuntu-20.04 为版本 2。
- 安装 Ubuntu 20.04 的 ROS Noetic、Gazebo 和 RViz。仿真工作区的依赖/编译以
  `smartcar2026-simulation/README.md` 为准。
- 让 WSL 保持运行，避免关闭最后一个 WSL 窗口后连带结束 Master、Gazebo 或 bridge。在 Windows
  用户目录的 `C:\Users\<Windows用户名>\.wslconfig` 中写入：

  ```ini
  [wsl2]
  networkingMode=mirrored
  vmIdleTimeout=-1
  ```

  保存后，在管理员或普通 PowerShell 执行一次：

  ```powershell
  wsl --shutdown
  ```

  再重新打开 Ubuntu-20.04。若公司策略不允许 mirrored networking，先向网络管理员确认 WSL 与
  小车的 ROS 回连方案；不要把 WSL 虚拟地址 `172.*`、`198.18.*`、VPN `100.*` 写给小车。

### 1.2 取得两个仓库

在 Windows 中克隆真车工程（分支名以远端为准）：

```powershell
git clone --branch simulation_real https://github.com/MDSIXONE/smartcar2026.git D:\SmartCar\ucar_source_code
```

在 WSL 中克隆和构建仿真工程。下面示例假设 WSL 用户名是 `car`；若不是，所有
`/home/car` 改为自己的 `$HOME`：

```bash
cd ~
git clone https://github.com/MDSIXONE/smartcar2026-simulation.git smartcar2026-simulation
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv ros-noetic-navigation ros-noetic-gazebo-ros-control
python3 -m pip install --user -r requirements-vision.txt
find src/car3/scripts -type f -name '*.py' -exec chmod +x {} +
catkin_make -j2
```

确认仿真资源完整：

```bash
test -f ~/smartcar2026-simulation/src/car3/models/vision/cube_yolov5_best.onnx && echo ONNX_OK
test -d ~/smartcar2026-simulation/src/car3/models/cube/meshes && echo MESH_OK
```

真车—仿真 HTTP bridge 已直接纳入 `smartcar2026-simulation.git` 的 `bridge/` 目录；完成第 1.2 节
克隆后即已拥有 `bridge/sim_bridge.py`，不需要从旧电脑或交付盘另行取得，也不要用手工简化版替代。

真车代码不用在新电脑编译；它只能在小车 Ubuntu 18.04 上构建。后文第 3 节会上传并在车端构建。

### 1.3 安装 WSL 真车 Master 启动器

在 WSL 中，先把 Windows 真车仓库路径写成一个变量；下面路径必须按实际克隆位置修改：

```bash
export REAL_REPO=/mnt/d/SmartCar/ucar_source_code
test -f "$REAL_REPO/rosmaster/start_ros_master.sh"
mkdir -p ~/.config/smartcar/python_http10_compat
cp "$REAL_REPO/rosmaster/start_ros_master.sh" ~/start_ros_master.sh
cp "$REAL_REPO/rosmaster/python_http10_compat/sitecustomize.py" \
  ~/.config/smartcar/python_http10_compat/sitecustomize.py
chmod 0755 ~/start_ros_master.sh
```

首次安装还需要这个小型网络加载器。它只读取当前 Wi-Fi 的地址；每次换网时仅改
`ros_network.env`，不改代码：

```bash
mkdir -p ~/.config/smartcar
cat > ~/.config/smartcar/ros_network.sh <<'EOF'
#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
if [[ -r "$HOME/.config/smartcar/ros_network.env" ]]; then
  source "$HOME/.config/smartcar/ros_network.env"
fi
if [[ -z "${ROS_IP:-}" ]]; then
  echo 'ERROR: set ROS_IP in ~/.config/smartcar/ros_network.env first.' >&2
  return 2
fi
case "$ROS_IP" in
  127.*|198.18.*|100.*) echo "ERROR: invalid LAN ROS_IP=$ROS_IP" >&2; return 2 ;;
esac
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${ROS_IP}:11311"
EOF
chmod 0755 ~/.config/smartcar/ros_network.sh
```

## 2. 每次换 Wi-Fi：确定 IP 并配置防火墙

### 2.1 找到电脑的局域网 IP（即 `MASTER_IP`）

在 Windows PowerShell 执行：

```powershell
ipconfig
```

从 **Wireless LAN adapter WLAN**（或实际有默认网关的有线网卡）找 IPv4 地址。例如
`192.168.31.252`。不要使用：

- `127.0.0.1` / `localhost`；
- `198.18.*`（代理虚拟网卡）；
- `100.*`（VPN/Tailscale）；
- `172.*` 或其他 WSL 虚拟网卡地址。

在 WSL 中更新当前网络配置，IP 必须替换为本次 `ipconfig` 的 WLAN IPv4：

```bash
cat > ~/.config/smartcar/ros_network.env <<'EOF'
ROS_IP=192.168.31.252
EOF
```

验证 WSL 也看见同一地址：

```bash
source ~/.config/smartcar/ros_network.sh
printf 'ROS_IP=%s\nROS_MASTER_URI=%s\n' "$ROS_IP" "$ROS_MASTER_URI"
```

若输出不是该 WLAN 地址，先检查 `.wslconfig` 的 mirrored networking 是否生效；不要继续。

### 2.2 找到小车 IP（即 `CAR_IP`）

优先在路由器 DHCP 客户端列表找小车；也可在 Windows PowerShell 执行：

```powershell
arp -a
```

在同一 `192.168.31.*` 子网中识别小车后，先做 SSH 验证：

```powershell
ssh ucar@<CAR_IP> hostname
```

只有 `hostname` 显示小车名称且能登录，才能把它当作 `CAR_IP`。新的 Wi-Fi 下 IP 会变化，
无法 SSH 时不要猜测或扫描整个网段；先在路由器确认 DHCP 地址和 Wi-Fi 隔离设置。

### 2.3 Windows 防火墙（每台新电脑一次）

小车必须能访问 WSL 的 ROS Master（11311 和 ROS 动态 TCPROS）与 bridge（11313）。在**管理员
PowerShell** 为可信局域网创建/调整规则：

```powershell
New-NetFirewallRule -DisplayName 'ROS TCPROS from UCar to WSL' `
  -Direction Inbound -Protocol TCP -RemoteAddress LocalSubnet -Action Allow
New-NetFirewallHyperVRule -Name 'ROS TCPROS from UCar to WSL' `
  -DisplayName 'ROS TCPROS from UCar to WSL' -Direction Inbound `
  -RemoteAddresses LocalSubnet -Action Allow
New-NetFirewallRule -DisplayName 'SimBridge 11313 from UCar to WSL' `
  -Direction Inbound -Protocol TCP -LocalPort 11313 `
  -RemoteAddress LocalSubnet -Action Allow
New-NetFirewallHyperVRule -Name 'SimBridge 11313 from UCar to WSL' `
  -DisplayName 'SimBridge 11313 from UCar to WSL' -Direction Inbound `
  -LocalPorts 11313 -RemoteAddresses LocalSubnet -Action Allow
```

规则只在受信任的赛场局域网中使用 `LocalSubnet`。换 Wi-Fi 后无需把规则绑到新 IP；但必须重复
第 2.1、2.2 节的地址确认。

## 3. 一次性部署真车任务到小车

在 Windows PowerShell，设定本次小车地址后逐文件上传。不要把多个源文件投到包根目录：

```powershell
$CAR = 'ucar@<CAR_IP>'
Set-Location D:\SmartCar\ucar_source_code
scp ucar_ws/src/ucar_2026/scripts/production_task_2026.py "${CAR}:~/ucar_ws/src/ucar_2026/scripts/production_task_2026.py"
scp ucar_ws/src/ucar_2026/scripts/production_task_geometry.py "${CAR}:~/ucar_ws/src/ucar_2026/scripts/production_task_geometry.py"
scp ucar_ws/src/ucar_2026/scripts/start_2026.sh "${CAR}:~/ucar_ws/src/ucar_2026/scripts/start_2026.sh"
scp ucar_ws/src/ucar_2026/launch/2026.launch "${CAR}:~/ucar_ws/src/ucar_2026/launch/2026.launch"
scp ucar_ws/src/ucar_2026/test/test_production_task_geometry.py "${CAR}:~/ucar_ws/src/ucar_2026/test/test_production_task_geometry.py"
ssh $CAR 'chmod 0755 ~/ucar_ws/src/ucar_2026/scripts/production_task_2026.py ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh'
```

上传后应逐文件比对 SHA-256，再仅在小车端构建/回归。把 `<MASTER_IP>` 换成本次第 2.1 节的地址：

```bash
# 在小车 Ubuntu 18.04 终端执行；不启动 roscore
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
unset ROS_HOSTNAME
MASTER_IP=<MASTER_IP>
export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
catkin_make --pkg ucar_2026 -DCATKIN_ENABLE_TESTING=ON
catkin_make -DCATKIN_ENABLE_TESTING=ON run_tests
catkin_test_results --verbose build/test_results/ucar_2026
```

预期测试结果无 error/failure。构建或测试不发送底盘速度。

## 4. WSLg COPY MODE：启动 GUI 前的强制预检

带界面的 Gazebo/RViz 只能在两项检查都通过时启动。在 WSL 终端执行：

```bash
grep -c 'use_gfxredir = 0' /mnt/wslg/weston.log
findmnt -no FSTYPE /mnt/shared_memory
```

合格条件：第一行必须为 `0`，第二行必须为 `tmpfs`。若 Gazebo/RViz 标题含
`[WARN:COPY MODE]`，也视为不合格。

不合格时**不得带病启动仿真**。关闭所有 WSL 终端后，在 Windows PowerShell 按顺序执行：

```powershell
wsl --terminate Ubuntu-20.04
```

重新打开 WSL 并复检。若 `use_gfxredir = 0` 仍出现，再执行：

```powershell
wsl --shutdown
```

这会让 Docker Desktop 临时显示 Stopped，属于正常现象。重启 WSL 后再次复检，合格才继续。

## 5. 启动顺序：五个保持打开的终端

从这里开始，不要将任一启动命令放入一次性后台 PowerShell/WSL 命令。每个终端保持打开，
按标题命名，便于发生异常时精确 Ctrl-C。

### 终端 A：WSL 真车 Master（11311）

```bash
unset ROS_IP ROS_HOSTNAME ROS_MASTER_URI
source ~/.config/smartcar/ros_network.sh
~/start_ros_master.sh
```

必须出现类似：

```text
Starting ROS Master at http://192.168.31.252:11311 (ROS_IP=192.168.31.252, XML-RPC=HTTP/1.0)
```

若出现 `localhost`、`127.0.0.1`、`198.18.*`、`100.*`，按 Ctrl-C 停止，回到第 2.1 节。

### 终端 B：WSL 仿真专用 Master（11312）

```bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
roscore -p 11312
```

这个 Master 只服务仿真，不能填给小车。

### 终端 C：WSL GUI 仿真准备（Gazebo + RViz）

先完成第 4 节 COPY MODE 检查，然后执行：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
roslaunch car3 task3_prepare.launch gui:=true rviz:=true
```

等待 Gazebo 与 RViz 显示，且日志出现 `calibrated initial arm pose applied smoothly`。比赛/验收时
两个窗口必须保持可见，不能最小化或遮挡。不要将本终端的 ROS Master 改为 11311。

### 终端 D：WSL bridge（11313）

bridge 已随仓库克隆，直接在仿真工作区运行：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
python3 bridge/sim_bridge.py
```

只在输出以下两行后，bridge 才可用：

```text
SIMULATION_BRIDGE_READY
simulation bridge listening on 0.0.0.0:11313 (state=waiting)
```

### 终端 E：小车先做网络检查，再启动 manual

在小车端运行。`<MASTER_IP>` 必须是终端 A 显示的 WSL WLAN 地址：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <MASTER_IP> check
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <MASTER_IP> manual
```

`check` 必须显示小车自动计算出的 `ROS_IP`，并报告 Master 连接成功。若失败，不得启动
`manual` 或 `mission`，先按第 9 节排查。

## 6. 进入 mission 前的必要检查（不通过就停止）

`manual` 正在运行时，在另一个小车终端依次检查：

```bash
rostopic echo -n 3 /odom_raw
timeout 5 rosrun tf tf_echo odom base_link
timeout 5 rosrun tf tf_echo map base_link
rostopic hz /scan
```

必须同时满足：

- `/odom_raw` 连续三条均为有限数，不能有 `NaN`；
- 两个 TF 都能连续输出，日志没有 `TF_NAN_INPUT`；
- `/scan` 有稳定新数据；
- `manual` 启动日志没有 `crc16`、`head_len error`、`sensor not active`、`No such device`；
- 小车确实在物理起点 `(-0.25, 2.75, 0)`，车头按赛场起点方向摆放；
- USB 扬声器不得与 CP2102 底盘串口共用同一 Hub；若发生过 CRC，先拔掉/改接扬声器并重新检查；
- Gazebo/RViz 已可见，bridge 是 `waiting`，终端 C 的 `/map` 已可读：

  ```bash
  # 在终端 C 或 D（ROS_MASTER_URI 必须是 11312）
  rostopic echo -n 1 /map
  rosservice list | grep -E '^/(gazebo/get_link_state|move_base/clear_costmaps)$'
  ```

- 小车到 bridge 的连通性通过：

  ```bash
  timeout 2 bash -c '>/dev/tcp/<MASTER_IP>/11313' && echo BRIDGE_TCP_OK
  curl -s http://<MASTER_IP>:11313/status
  ```

如果任何 odom/TF 条目异常：保持不动，先执行第 8 节“紧急停止”，重启导航/底盘里程计链路，
再次从 manual 开始检查。**不要**因为急着跑流程而跳过此步骤。

确认无误后，在运行 manual 的终端 Ctrl-C，等 `2026.launch` 完全退出；不要让 manual 和 mission
同时运行。

## 7. 启动主流程

在小车主终端执行：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <MASTER_IP> mission
```

出现“是否已把车放回起点”时，只有确认真实摆放正确才输入 `yes`。

随后听到提示，先说唤醒词“**小飞小飞**”，再完整说出双类别指令，例如：

```text
前往物品领取区，取得日用品，放置在对应仓库，并领取仿真环境中需要的食品放置在对应仓库
```

`食品`、`日用品`、`电子产品` 中选择两个不同类别；二维码阶段会从现场二维码获取实际物品名。
任务顺序是：

1. 扫齐二维码，**在前往点 3 前**播报收集结果和物品所属仓库；
2. 巡航识别，仿真类别即使先发现也只记录，必须先停入并播报实物；
3. 停入仿真物品后 bridge 收到 `/start`，Gazebo 中开始仿真；小车最多等待 **150 秒**完成；
4. 仿真完成后前往 441，任务成功会自动交接 lane_proto 巡线至 `STOPPED`。

任务主路线未找到类别时会补跑边界路线；若仍没有，将直接去 441 交接。边界角点的完整原地旋转
净空必须先经现场确认，首次使用改速后的路线务必安排操作员看护急停。

运行时观察：

```bash
# 小车终端
rostopic echo /ucar_2026/task_state
rostopic echo /ucar_2026/task_result

# WSL 另一终端
curl -s http://<MASTER_IP>:11313/status
```

## 8. 停止与清理（必须做，不留后台进程）

### 立即停止真车任务

在任意新的小车终端执行：

```bash
MASTER_IP=<MASTER_IP> bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
```

该脚本先发布零速度，再停止 `ucar_2026 2026.launch`；不会停止 WSL Master。若已自动交接
lane_proto，先观察 `/lane_proto/state`，确认要停止的准确 `roslaunch lane_proto` PID 后，
对该 PID 发送 `kill -INT <PID>`。不要按进程名宽泛杀进程。

### 正常结束顺序

1. 小车：确认 2026/lane 任务已退出；
2. 终端 D：bridge 按 Ctrl-C 退出；
3. 终端 C：`task3_prepare.launch` 按 Ctrl-C 退出，Gazebo/RViz 关闭；
4. 终端 B：仿真 `roscore` 按 Ctrl-C 退出；
5. 终端 A：真车 `roscore` 按 Ctrl-C 退出。

最后在 WSL 检查没有本次残留：

```bash
pgrep -af 'sim_bridge|task3_prepare|task3_execute|gzserver|gzclient|roscore|rosmaster|roslaunch'
```

只处理能明确确认属于**本次**运行的 PID；不要按宽泛模式结束其他人的 ROS/仿真工作。

## 9. 常见故障速查

| 现象 | 处理 |
| --- | --- |
| 小车 `check` 连不上 Master | 核对终端 A 的 `MASTER_IP` 是 WLAN IPv4，非 127/198.18/100/WSL 虚拟地址；确认同网段、路由器未隔离客户端、Windows 防火墙规则存在。 |
| 小车能 SSH，但 `rosnode list` 超时 | 多为 ROS 动态 TCPROS 防火墙未放行或小车 `ROS_IP` 取错。重新运行 `start_2026.sh ... check`，检查输出的小车 IP 是否为当前 WLAN 地址。 |
| bridge `connection refused` / 小车 POST 超时 | 确认终端 D 已显示 `SIMULATION_BRIDGE_READY`，在小车用 `curl http://<MASTER_IP>:11313/status`，再检查 11313 两条防火墙规则。 |
| bridge 一直等 `/map` | 终端 C 没启动成功、未 source Noetic/devel，或 ROS Master 错用了 11311。C/D 必须都是 `127.0.0.1:11312`。先让 `/map` 有数据后重启 bridge。 |
| bridge 返回 409 `already finished` | bridge 每次只服务一轮。Ctrl-C 结束终端 D，再重新启动 bridge，状态回到 `waiting`。 |
| Gazebo/RViz 黑屏、卡顿或标题含 COPY MODE | 立即关闭，不要带病跑；严格按第 4 节 `wsl --terminate` / 必要时 `wsl --shutdown` 后复检。 |
| 关闭 WSL 窗口后仿真消失 | 不要用一次性后台命令承载 ROS；使用第 5 节的常驻终端，并确认 `.wslconfig` 已设置 `vmIdleTimeout=-1`。 |
| `/odom_raw` 有 NaN 或 TF_NAN_INPUT | 不运行 mission。使用停止脚本发布零速度，检查电池/CP2102/USB Hub，重启导航和底盘里程计链路，再从 manual 安全检查重新开始。 |
| 日志出现 CRC16 | USB 音频扬声器不要与 CP2102 同一 Hub；改到独立 USB 口或 3.5 mm 音频。数据链路未恢复前不得行驶。 |
| 仿真等超过 150 秒 | 小车会按安全策略中止。不要在小车停到加工区后临时补开仿真；先停止、回起点，从第 5 节重新让仿真和 bridge 就绪。 |
| 2026 与 lane_proto 同时抢相机/串口 | 两者不能同时启动。先用停止脚本确保 2026 全部退出，自动交接应自行等待；人工启动 lane 前复核进程表。 |

## 10. 出发前 30 秒勾选

- [ ] 本次 `MASTER_IP` 是 Windows WLAN IPv4，且小车和电脑同网段。
- [ ] 终端 A 是 11311，终端 B/C/D 是 11312；没有小车端 roscore。
- [ ] COPY MODE 两项检查合格，Gazebo 与 RViz 可见。
- [ ] `/map`、Gazebo 服务、bridge `waiting`、小车到 11313 的 curl 均正常。
- [ ] manual 下 odom、两个 TF、雷达、底盘日志均正常且无 NaN/TF/CRC。
- [ ] 车辆已物理放回起点，扬声器未与 CP2102 共 Hub，操作员准备急停。
- [ ] 已停止 manual，准备从 mission 重新启动。
