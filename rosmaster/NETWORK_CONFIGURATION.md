# ROS 1 动态网络配置：WSL Master 与小车

本文件是控制电脑（WSL Ubuntu 20.04）与小车更换 Wi-Fi 后的网络配置清单。它只涉及 ROS 网络连通性，不会发送底盘或导航命令。

## 先理解这三个地址

| 名称 | 所在设备 | 作用 |
| --- | --- | --- |
| `MASTER_IP` | 控制电脑的物理 Wi-Fi / 网线网卡 | 小车访问 ROS Master 的地址 |
| `VEHICLE_IP` | 小车当前 Wi-Fi / 网线网卡 | 小车节点向 ROS 网络公布自身的回连地址 |
| `ROS_MASTER_URI` | 每个 ROS 节点 | 固定写法为 `http://MASTER_IP:11311` |

ROS 1 不会自动发现 Master。两台设备即使连接到同一 Wi-Fi，仍必须让小车知道 `MASTER_IP`，并让每台设备正确设置自己的 `ROS_IP`。不要使用 `127.0.0.1`、WSL 虚拟网络 `198.18.*` 或 VPN / 代理地址作为小车通信地址。

## 推荐流程：每次更换 Wi-Fi 后

1. 让控制电脑和小车连接到同一个 IPv4 局域网，确认两者地址在同一子网。例如控制电脑为 `192.168.31.252/24` 时，小车应为 `192.168.31.*`。
2. 在控制电脑 WSL 中启动 Master：

   ```bash
   ~/start_ros_master.sh
   ```

   脚本会显示 `Starting ROS Master at ...`。其中 `ROS_IP` 与 URI 中的 IP 即本次的 `MASTER_IP`；记下它。
3. 在控制电脑另一个 WSL 终端启动 RViz：

   ```bash
   ~/start_rviz.sh
   ```

   它和 Master 共用同一份动态网络配置。
4. 在小车终端按下文“小车端”设置完成 ROS 环境，再启动节点。**小车端不得运行 `roscore`。**
5. 小车端执行 `rosnode list`；控制电脑执行 `rosparam list`。两个命令都应能读取同一个 Master。

## 控制电脑 / WSL Master

### 自动发现（默认）

`/home/car/start_ros_master.sh` 与 `/home/car/start_rviz.sh` 都加载：

```text
/home/car/.config/smartcar/ros_network.sh
```

它按以下顺序选择地址：命令行 `ROS_IP`、命令行 `ROS_INTERFACE`、Windows 当前具有默认网关的物理网卡、WSL 路由地址、其他 WSL 全局 IPv4 地址。当前 WSL 为镜像网络模式，因此 WSL 可以直接使用 Windows 的 Wi-Fi 局域网地址。

通常不要填写任何固定地址；直接运行两个启动脚本即可。

### 需要手动覆盖时

编辑这个文件：

```bash
nano ~/.config/smartcar/ros_network.env
```

该文件默认所有值都被注释。只取消注释所需的一项：

```bash
# 直接指定本机局域网地址
ROS_IP=192.168.31.252

# 或指定 Linux 网卡；不要同时填写 ROS_IP
# ROS_INTERFACE=eth1

# 仅当要连接外部 Master 时才填写；本机启动 roscore 时保持注释
# ROS_MASTER_URI=http://192.168.31.252:11311
```

一次性覆盖而不改文件：

```bash
ROS_IP=192.168.31.252 ~/start_ros_master.sh
ROS_INTERFACE=eth1 ~/start_rviz.sh
```

查看 Windows 当前可用于小车的物理网卡地址：

```bash
powershell.exe -NoProfile -Command "(Get-NetIPConfiguration | Where-Object { $_.NetAdapter.HardwareInterface -and $_.IPv4DefaultGateway }).IPv4Address.IPAddress"
```

## 小车端

每次启动小车 ROS 节点的终端，必须在所有 `source` 命令之后执行以下内容。将 `MASTER_IP` 替换为本次控制电脑启动脚本显示的地址：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash

unset ROS_HOSTNAME
MASTER_IP=192.168.31.252
export ROS_MASTER_URI="http://${MASTER_IP}:11311"

# 自动挑选小车到 Master 所走网卡的 IPv4 地址。
export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"

printf 'ROS_IP=%s\nROS_MASTER_URI=%s\n' "$ROS_IP" "$ROS_MASTER_URI"
```

输出的 `ROS_IP` 必须是小车当前局域网地址，而 `ROS_MASTER_URI` 必须指向控制电脑。随后再执行实际的 `roslaunch`；不要在小车上执行 `roscore`。

若小车有自己的启动脚本，应将上述片段放在该脚本中所有 `source` 之后，而不是写死旧 Wi-Fi 的地址。小车端的工作区和账号未在本机挂载时，需在小车上单独完成这项修改。

## 不想每次抄写 MASTER_IP：三种方案

1. **手动记录（立即可用）**：使用控制电脑启动脚本打印的 `MASTER_IP`，填入小车终端的 `MASTER_IP=...`。这是当前默认流程。
2. **路由器 DHCP 地址保留（推荐用于固定赛场路由器）**：为控制电脑的 Wi-Fi MAC 地址保留一个局域网地址。地址由路由器分配，但在同一赛场网络中保持稳定；小车只需配置该地址。
3. **mDNS 主机名（推荐用于经常更换网络）**：将控制电脑发布为如 `rosmaster.local`，并确保小车可以解析该名称。小车端改为：

   ```bash
   MASTER_HOST=rosmaster.local
   export ROS_MASTER_URI="http://${MASTER_HOST}:11311"
   MASTER_IP="$(getent hosts "$MASTER_HOST" | awk 'NR == 1 { print $1 }')"
   export ROS_IP="$(ip -4 route get "$MASTER_IP" | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')"
   ```

   必须先验证 `getent hosts rosmaster.local` 有输出；ROS 1 本身不提供 mDNS 服务，需要由系统或路由器提供名称解析。

## 防火墙与连通性

控制电脑处于 WSL 镜像网络模式，但 Windows 防火墙仍必须允许小车访问 ROS 的 XML-RPC 与动态 TCPROS 端口。旧规则若只允许旧 IP（如 `192.168.8.231`），换 Wi-Fi 后必须改为当前局域网范围。

在 **管理员 PowerShell** 中，将既有规则限制为当前本地子网，而不是单一小车 IP：

```powershell
Get-NetFirewallRule -DisplayName 'ROS TCPROS from UCar to WSL' |
  Get-NetFirewallAddressFilter |
  Set-NetFirewallAddressFilter -RemoteAddress LocalSubnet

Get-NetFirewallHyperVRule -DisplayName 'ROS TCPROS from UCar to WSL' |
  Set-NetFirewallHyperVRule -RemoteAddresses LocalSubnet
```

若规则尚不存在，先按当前 Windows / WSL 网络策略创建对应的 TCP 入站和 Hyper-V 规则；ROS 使用动态 TCPROS 端口，不能只开放 `11311`。仅在受信任的专用局域网中使用 `LocalSubnet`。

只读连通性检查：

```bash
# 小车上：确认 TCP 11311 可达；成功时命令返回 0。
timeout 2 bash -c ">/dev/tcp/${MASTER_IP}/11311"

# 小车上：确认 ROS API 可读。
rosnode list

# 控制电脑 WSL 上：确认 Master 正在运行。
rosparam list
```

## 常见故障

| 现象 | 原因与处理 |
| --- | --- |
| 小车 `rosnode list` 超时 | 检查 `ROS_MASTER_URI` 是否为本次 `MASTER_IP`，并检查防火墙与 11311 端口。 |
| topic 列得出但节点互相连不上 | 小车或控制电脑的 `ROS_IP` 设置成了旧地址、回环地址或 VPN 地址；重新执行本页配置。 |
| Master 显示 `198.18.*` / `100.*` | 这是代理、VPN 或虚拟网卡；在控制电脑 `ros_network.env` 中设置正确的 `ROS_IP` 或 `ROS_INTERFACE`。 |
| 小车启动了自己的 roscore | 停止它；小车所有节点必须连接控制电脑 WSL 的唯一 Master。 |
| 更换 Wi-Fi 后旧 shell 仍失败 | 关闭旧终端，重新 `source` 环境并按本页重新导出变量；不要复用旧 shell 的 ROS 环境变量。 |
