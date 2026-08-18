# 新电脑部署文档：WSL 仿真与小车通信

本文用于在一台全新的 Windows 电脑上配置 WSL 仿真、Gazebo/RViz、真车—仿真
HTTP bridge、动态局域网通信和 Windows 防火墙。部署完成后，按下面三份操作文档
启动对应主流程：

- [标准主流程](operations.md)：ucar_2026
- [国赛主流程](operations-national.md)：ucar_2026_national
- [额外 OCR 主流程](operations-extra.md)：ucar_2026_extra

## 1. 先确认通信拓扑

当前源码中的正式启动脚本会由小车端 start_2026.sh 临时托管小车自己的 ROS Master；
电脑 WSL 只托管仿真专用 Master 和 HTTP bridge。不要让两套系统共用同一个 Master。

> 版本一致性说明：当前源码和最近的本地改动记录均采用“小车本机 Master”架构；如果你
> 使用的上层 AGENTS.md 仍要求“WSL 托管真车 Master、禁止小车 roscore”，那与当前三套
> start_2026.sh 的实际行为冲突。不能混用两套架构；必须先统一脚本和规范，再按本文启动。

~~~text
小车 Ubuntu 18.04 / ROS Melodic
  └─ 本车 ROS Master：TCP 11311
       ├─ 底盘、定位、导航、视觉、任务节点
       └─ HTTP → 电脑局域网地址:11313

电脑 Windows + WSL Ubuntu 20.04 / ROS Noetic
  ├─ 仿真 ROS Master：127.0.0.1:11312
  ├─ Gazebo + RViz
  └─ sim_bridge.py：0.0.0.0:11313
~~~

本流程只需要让小车访问电脑的 TCP 11313，不需要把小车的 ROS 11311 或 ROS
动态 TCPROS 端口暴露到 Windows。每次换 Wi-Fi 都重新发现地址，不得照抄旧 IP。

文档中的变量含义如下：

| 变量 | 含义 | 从哪里获取 |
| --- | --- | --- |
| PC_LAN_IP | 电脑当前 WLAN/有线网卡的局域网 IPv4；传给 start_2026.sh | Windows ipconfig 或第 4 节 PowerShell 命令 |
| CAR_IP | 小车当前局域网 IPv4；只用于 SSH 和车端登录 | 路由器 DHCP 列表，随后用 SSH 验证 |
| 127.0.0.1:11312 | WSL 仿真专用 ROS Master | 只在 WSL 仿真终端使用 |

## 2. Windows 一次性安装

### 2.1 系统要求

- Windows 11，已启用 CPU 虚拟化。
- WSL 2、Ubuntu 20.04、WSLg；不要使用 Ubuntu 18.04 作为本机仿真系统。
- 小车仍使用 Ubuntu 18.04 / ROS Melodic；小车代码不能在 Windows 或 WSL 编译后
  当作车端构建结果使用。
- 电脑和小车必须连接同一个可信 IPv4 局域网；路由器不能开启 AP/client isolation。

管理员 PowerShell 执行：

~~~powershell
wsl --install --no-launch
wsl --set-default-version 2
wsl --install -d Ubuntu-20.04
wsl --update
wsl -l -v
~~~

确认列表中 Ubuntu-20.04 的 VERSION 为 2。如果安装命令提示重启，先重启 Windows，
再重新执行 wsl -l -v。

检查 WSLg 是否可用：

~~~powershell
wsl --version
wsl -d Ubuntu-20.04 -- bash -lc 'echo WSL_OK; test -d /mnt/wslg && echo WSLG_OK'
~~~

### 2.2 配置 WSL 网络和常驻时间

在 Windows 用户目录创建或修改
C:\Users\<Windows用户名>\.wslconfig：

~~~ini
[wsl2]
networkingMode=mirrored
vmIdleTimeout=-1
~~~

networkingMode=mirrored 让 WSL 内的 bridge 更容易被局域网小车访问；
vmIdleTimeout=-1 防止最后一个 WSL 终端退出后仿真进程被系统回收。保存后必须执行：

~~~powershell
wsl --shutdown
wsl -d Ubuntu-20.04
~~~

如果 Windows 版本不支持 mirrored networking，使用第 5.2 节的 NAT + portproxy
方案；不要把 WSL 的 172.* 地址直接传给小车。

### 2.3 安装 ROS Noetic、Gazebo 和构建工具

在 Ubuntu-20.04 WSL 中执行：

~~~bash
sudo apt update
sudo apt install -y curl gnupg2 lsb-release ca-certificates git build-essential \
  python3-pip python3-opencv python3-rosdep python3-catkin-tools

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full \
  ros-noetic-navigation ros-noetic-map-server \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control ros-noetic-rviz

sudo rosdep init
rosdep update
~~~

把 ROS 环境加入 WSL 用户的 shell：

~~~bash
grep -qxF 'source /opt/ros/noetic/setup.bash' ~/.bashrc || echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash
~~~

不要把 ROS_MASTER_URI=http://127.0.0.1:11311 固定写入 .bashrc。每个仿真终端都要
显式设置 ROS_MASTER_URI=http://127.0.0.1:11312，避免误连到其他 ROS Master。

## 3. 获取并构建仿真仓库

在 WSL 中执行。统一仓库同时包含 `simulation/` 和 `ucar_source_code/`；以下目录名可以
更换，但后续操作文档必须使用实际目录：

~~~bash
cd ~
git clone --branch main https://github.com/MDSIXONE/smartcar2026.git smartcar2026
cd ~/smartcar2026/simulation
source /opt/ros/noetic/setup.bash
sudo apt install -y python3-pip python3-opencv ros-noetic-navigation ros-noetic-gazebo-ros-control
python3 -m pip install --user -r requirements-vision.txt
find src/car3/scripts -type f -name '*.py' -exec chmod +x {} +
catkin_make -j2
source devel/setup.bash
~~~

确认关键模型和 bridge 存在：

~~~bash
test -f src/car3/models/vision/cube_yolov5_best.onnx && echo ONNX_OK
test -d src/car3/models/cube/meshes && echo MESH_OK
test -f bridge/sim_bridge.py && echo BRIDGE_OK
test -f src/car3/launch/task3_prepare.launch && echo PREPARE_LAUNCH_OK
~~~

如果 catkin_make 失败，先保留完整错误，不要复制其他电脑的 build/、devel/、install/
到新电脑；删除构建产物后重新在本机 WSL 构建：

~~~bash
cd ~/smartcar2026/simulation
rm -rf build devel
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
~~~

## 4. 每次换 Wi-Fi：发现电脑和小车地址

### 4.1 找电脑地址

在 Windows PowerShell 执行：

~~~powershell
$LAN = Get-NetIPConfiguration | Where-Object { $_.NetAdapter.Status -eq 'Up' -and $_.IPv4DefaultGateway } | Select-Object -First 1
$PC_LAN_IP = ($LAN.IPv4Address | Select-Object -First 1).IPAddress
$LAN | Format-List InterfaceAlias,IPv4Address,IPv4DefaultGateway
"PC_LAN_IP=$PC_LAN_IP"
~~~

必须选择有默认网关、状态为 Up 的 WLAN 或有线网卡。以下地址不能传给小车：

- 127.0.0.1、localhost；
- WSL NAT 的 172.*；
- 代理虚拟网卡常见的 198.18.*；
- VPN/Tailscale 常见的 100.*；
- Docker、Hyper-V、VirtualBox 等虚拟网卡地址。

也可以用 ipconfig 复核：使用 Wireless LAN adapter WLAN 或实际联网的以太网适配器
IPv4，而不是 vEthernet/WSL 适配器。

### 4.2 找小车地址

优先从当前路由器 DHCP 客户端列表获取小车地址，然后在 PowerShell 验证：

~~~powershell
$CAR_IP = '<CAR_IP>'
ssh "ucar@$CAR_IP" hostname
ssh "ucar@$CAR_IP" 'uname -a; test -x ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh && echo UCAR_LAUNCHER_OK'
~~~

只有 SSH 登录成功且主机名确实是小车，才能继续。换 Wi-Fi 后不要沿用旧的
192.168.* 地址；如果 SSH 失败，先检查 DHCP 地址和 AP 隔离，不要盲目扫描整个网段。

## 5. Windows 防火墙和 WSL 外部访问

### 5.1 推荐：mirrored networking

将 Windows 当前网络设为“专用网络”前，必须确认这是可信局域网。管理员 PowerShell：

~~~powershell
Get-NetConnectionProfile
~~~

如果当前联网适配器不是 Private，确认它确实是可信赛场/家庭局域网后执行；公共 Wi-Fi
不要改成 Private：

~~~powershell
$LAN | Set-NetConnectionProfile -NetworkCategory Private
Get-NetConnectionProfile
~~~

确认 NetworkCategory 为 Private 后创建仅允许局域网访问的 bridge 规则：

~~~powershell
New-NetFirewallRule -DisplayName 'SmartCar SimBridge TCP 11313' -Direction Inbound -Action Allow -Protocol TCP -LocalPort 11313 -Profile Private -RemoteAddress LocalSubnet
~~~

检查规则和端口：

~~~powershell
Get-NetFirewallRule -DisplayName 'SmartCar SimBridge TCP 11313' | Get-NetFirewallPortFilter
Get-NetTCPConnection -LocalPort 11313 -State Listen -ErrorAction SilentlyContinue
~~~

不需要为当前架构开放 Windows 入站 TCP 11311，也不要开放所有端口。bridge 启动后
小车只访问 PC_LAN_IP:11313。

### 5.2 NAT 备用方案：端口代理

仅当 mirrored networking 不可用时使用。每次 WSL 重启后 WSL IP 可能变化，因此必须
重新执行以下命令，不能把旧的 WSL_IP 写进固定文档：

~~~powershell
$WSL_IP = (wsl.exe -d Ubuntu-20.04 -- hostname -I).Trim().Split()[0]
"WSL_IP=$WSL_IP"
netsh interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=11313
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=11313 connectaddress=$WSL_IP connectport=11313
New-NetFirewallRule -DisplayName 'SmartCar SimBridge TCP 11313' -Direction Inbound -Action Allow -Protocol TCP -LocalPort 11313 -Profile Private -RemoteAddress LocalSubnet
netsh interface portproxy show all
~~~

bridge 必须在 WSL 中监听 0.0.0.0:11313，而不是只监听 127.0.0.1:11313。如果
端口代理已配置但小车仍然无法访问，先确认 connectaddress 是本次启动的 WSL 地址，
再检查 Windows 防火墙规则。

## 6. 首次通信验收

### 6.1 一键启动仿真栈

仿真专用 Master、Gazebo/RViz 和 HTTP bridge 已合并为一个入口。脚本会自动设置
`ROS_MASTER_URI=http://127.0.0.1:11312`，执行 GUI 启动前后 COPY MODE 预检，并等待
`/map` 就绪后才开放 bridge：

~~~bash
cd ~/smartcar2026/simulation
bash scripts/start_simulation_stack.sh
~~~

无界面联调：

~~~bash
bash scripts/start_simulation_stack.sh --headless
~~~

保持一键脚本终端打开，等待 /map 和 Gazebo 服务出现：

~~~bash
source /opt/ros/noetic/setup.bash
source ~/smartcar2026/simulation/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
rostopic echo -n 1 /map
rosservice list | grep -E '^/(gazebo/get_link_state|move_base/clear_costmaps)$'
~~~

必须看到：

~~~text
SIMULATION_BRIDGE_READY
simulation bridge listening on 0.0.0.0:11313 (state=waiting)
~~~

在 Windows 可先验证端口：

~~~powershell
Test-NetConnection $PC_LAN_IP -Port 11313
~~~

在小车上验证 HTTP：

~~~bash
curl -sS "http://<PC_LAN_IP>:11313/status"
~~~

返回 state=waiting 后，才允许启动任一主流程。

## 7. WSLg COPY MODE 强制预检

任何带 GUI 的 task3_prepare.launch 之前，在 WSL 执行：

~~~bash
grep -c 'use_gfxredir = 0' /mnt/wslg/weston.log
findmnt -no FSTYPE /mnt/shared_memory
~~~

只有以下两个条件同时满足才合格：

~~~text
use_gfxredir = 0 的计数：0
/mnt/shared_memory 文件系统：tmpfs
~~~

窗口标题出现 [WARN:COPY MODE]（用户有时会写成 WORN COPY MODE）也判定为不合格。
不合格时不要启动 Gazebo/RViz：

~~~powershell
wsl --terminate Ubuntu-20.04
~~~

重新打开 WSL 后再次检查。如果 weston.log 仍有 use_gfxredir = 0：

~~~powershell
wsl --shutdown
~~~

然后重新打开 Ubuntu-20.04，再重复两条预检。wsl --shutdown 会让 Docker Desktop
短暂显示 Stopped，这是 WSL 完全重启的正常现象。预检通过前不得用 --no-gui 绕过，
也不得先启动仿真再修复 COPY MODE。

从 PowerShell 检查 weston.log 时，不要把包含 Bash $() 的命令放在 PowerShell 双引号
中；否则 $() 会被 PowerShell 提前展开，可能出现 grep: =: No such file or directory。

## 8. 车端首次验收与构建边界

新电脑只负责提供仿真和上传文件；实车代码必须在小车 Ubuntu 18.04 / ROS Melodic 上
构建。当前三套脚本都可先做网络检查：

~~~bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <PC_LAN_IP> check
bash ~/ucar_ws/src/ucar_2026_national/scripts/start_2026.sh <PC_LAN_IP> check
bash ~/ucar_ws/src/ucar_2026_extra/scripts/start_2026.sh <PC_LAN_IP> check
~~~

check 只验证 bridge TCP 可达和车端 Python 2/ROS 环境，不启动导航；命令返回后脚本
会清理自己启动的临时 Master。三套脚本中选一套继续操作，不要同时启动多个主流程，
它们会争用同一底盘、相机和 /cmd_vel。

若本次确实同步了车端源码，只在小车执行构建；不要在 Windows 或 WSL 编译后把 devel/
直接复制到小车：

~~~bash
cd ~/ucar_ws
source /opt/ros/melodic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="cym_planner;lane_proto;ucar_2026;ucar_2026_national;ucar_2026_extra"
source devel/setup.bash
~~~

构建完成后，后续每次 source 都重新设置车端当前 ROS 环境；启动命令会按到电脑的路由
自动选择小车 ROS_IP，不需要写死 Wi-Fi 地址。

## 9. 部署完成检查表

- [ ] wsl -l -v 显示 Ubuntu-20.04 为 WSL 2。
- [ ] .wslconfig 已配置 mirrored networking 和 vmIdleTimeout=-1，并已 wsl --shutdown。
- [ ] ROS Noetic、Gazebo、RViz、navigation、map_server 已安装。
- [ ] 仿真工作区 catkin_make 成功，ONNX、模型 mesh、bridge 和 prepare launch 都存在。
- [ ] Windows 防火墙只为可信局域网放行 TCP 11313。
- [ ] 每次换 Wi-Fi 都重新获取 PC_LAN_IP 和 CAR_IP。
- [ ] COPY MODE 两项预检通过，Gazebo/RViz 可见。
- [ ] 仿真 Master 是 127.0.0.1:11312，bridge 显示 state=waiting。
- [ ] 小车 check 通过，且没有运行其他主流程或独立 lane_follow.py。

## 10. 部署故障处理

| 现象 | 处理 |
| --- | --- |
| 小车提示 11313 不可达 | 确认传入的是 Windows WLAN IPv4，不是 WSL 172.*；确认 bridge 已显示 0.0.0.0:11313；确认防火墙规则的配置文件和 LocalSubnet 正确。NAT 模式再按第 5.2 节重建 portproxy。 |
| Windows 能通，车端 curl 不通 | 在车端先执行 timeout 2 bash -c "exec 3<>/dev/tcp/<PC_LAN_IP>/11313"；检查 Wi-Fi 是否隔离、Windows 网络是否为 Private、portproxy 的 WSL IP 是否已过期。 |
| bridge connection refused | 一键启动脚本未启动、端口被旧 bridge 占用或 bridge 绑定错误。停止旧 bridge 后重新运行脚本；不要同时运行两份 bridge。 |
| bridge 一直不是 waiting | 一键脚本未等到 /map，或仿真 Master 端口不一致。确认脚本使用 127.0.0.1:11312，并先确认 rostopic echo -n 1 /map。 |
| Gazebo/RViz 黑屏、卡顿或标题含 COPY MODE | 立即关闭 GUI，按第 7 节先 wsl --terminate Ubuntu-20.04；仍有 use_gfxredir = 0 时执行 wsl --shutdown，复检通过后才能重启。 |
| wsl --shutdown 后端口代理失效 | NAT 模式下 WSL IP 变了；重新获取 WSL_IP 并重建 11313 portproxy。mirrored 模式不需要这一步。 |
| 小车启动后 /odom_raw 是 NaN 或 TF 报 TF_NAN_INPUT | 立即停止并发布零速度，重启导航/底盘里程计链路；只有 /odom_raw 有限且 odom -> base_link、map -> base_link 恢复后才能继续。 |
| 日志出现 crc16、head_len、sensor not active | 不进入 mission，不发送导航目标；先排查 CP2102、USB Hub、串口线束和底盘供电。 |
| bridge 返回 409 already finished | 当前 bridge 只服务一轮任务；在一键启动脚本终端按 Ctrl-C 完整停止，再重新运行脚本使状态回到 waiting。 |
