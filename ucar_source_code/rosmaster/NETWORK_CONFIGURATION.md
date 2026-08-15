# ROS 1 动态网络配置：小车本机 Master

主流程的 ROS Master 运行在小车上；电脑不再运行或提供真车 ROS Master。电脑只运行仿真
bridge 的 HTTP 服务（默认 `11313`）以及其独立的仿真 ROS Master（`127.0.0.1:11312`）。

| 名称 | 所在设备 | 用途 |
| --- | --- | --- |
| `VEHICLE_IP` | 小车当前局域网地址 | 小车 ROS Master 和节点回连地址 |
| `SIMULATION_HOST` | 电脑当前局域网地址 | 小车访问电脑 bridge 的 HTTP 地址 |
| `ROS_MASTER_URI` | 小车所有主流程节点 | `http://VEHICLE_IP:11311` |

每次换 Wi-Fi 后不写死任何 IP。`start_2026.sh` 根据到 `SIMULATION_HOST` 的路由自动选择
本次 `VEHICLE_IP`，前台监管它启动的 `roscore` 与 `roslaunch`；任务退出或按 Ctrl-C 后会
一并停止，因此不会在后台遗留 Master。

## 小车端启动

先在电脑上启动仿真 bridge，再从小车运行：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <电脑LAN_IP> mission
```

例如电脑 bridge 可达地址为 `192.168.31.252`：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh 192.168.31.252 mission
```

脚本会显示：

```text
小车 ROS Master = http://<小车当前IP>:11311
电脑仿真服务 = <电脑当前IP>
```

仅检查本机 Master 和网络、不开导航：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <电脑LAN_IP> check
```

停止时在另一个小车终端执行：

```bash
bash ~/ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh
```

它向前台 `roslaunch` 发送 SIGINT；启动脚本随后停止自己创建的 Master。不要手工后台启动
`roscore`，也不要让电脑的 `11311` 充当主流程 Master。

## 电脑侧

电脑只需允许小车访问 bridge 的 TCP `11313`。不需要开放 ROS `11311` 或动态 TCPROS 端口。
仿真仍使用本机隔离环境：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
python3 bridge/sim_bridge.py
```

Windows 防火墙示例（仅可信局域网）：

```powershell
New-NetFirewallRule -DisplayName 'SimBridge 11313 from UCar' `
  -Direction Inbound -Protocol TCP -LocalPort 11313 `
  -RemoteAddress LocalSubnet -Action Allow
```

## 故障排查：车端 No route to host（/start 连不上仿真）

车端日志出现 `PRODUCTION_SIMULATION_START_RETRY ... error="<urlopen error [Errno 113] No route to
host>"` 时，是车到 `<电脑LAN_IP>:11313` 网络层不可达，与 bridge 是否就绪无关。按顺序检查：

1. **填错 IP（最常见）**：`start_2026.sh` 的地址必须是电脑的 **Windows 局域网 IP**（在电脑上
   `ipconfig` 查 Wi-Fi 适配器 IPv4），**不是** WSL 里的 `ip addr` 结果——WSL2 的 `172.x` 是
   NAT 内部网段，小车在场地 Wi-Fi 网段无法路由到它（网关回 ICMP unreachable 即 Errno 113）。
   Wi-Fi 环境变化后 IP 会变，每次出发前都要重新确认。
2. **bridge 未监听**：电脑终端必须显示 `simulation bridge listening on 0.0.0.0:11313`。
3. **WSL2 网络模式**：NAT 模式下 WSL 内监听的端口对局域网不可见。推荐在
   `C:\Users\<用户>\.wslconfig` 设置 `networkingMode=mirrored`（Windows 11 22H2+）；或
   Windows 上用端口代理：
   `netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=11313 connectaddress=<WSL_IP> connectport=11313`
   （`<WSL_IP>` 用 WSL 里 `ip addr` 查询），并配下方防火墙规则。
4. **防火墙**：确认入站规则放行 TCP 11313（`RemoteAddress LocalSubnet`）。

小车侧快速验证（车在场地内）：

```bash
timeout 2 bash -c "exec 3<>/dev/tcp/<电脑LAN_IP>/11313" && echo TCP_OK
```

`start_2026.sh` 的 `mission` 模式启动时会先做 TCP 11313 预检，不可达会直接报错退出；不要忽略
该提示强行出发，否则任务会在仿真区因无法启动仿真而中止。

## 只读连通性检查

主流程启动后，在小车另一个 shell 中用脚本显示的 `VEHICLE_IP`：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_IP=<VEHICLE_IP>
export ROS_MASTER_URI="http://${ROS_IP}:11311"
rosnode list
```

静止时 `/odom_raw` 可以保持位置不变，但仍必须持续发布新时间戳。若 TF 报
`Lookup would require extrapolation into the future`，且请求时间比最新 TF 晚数秒，说明
里程计/串口/时间链路没有及时更新，不是“车不动”的正常现象；不得据此继续做定位或导航。
