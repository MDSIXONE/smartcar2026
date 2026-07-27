# 动态 ROS 网络配置文档

## 目的

移除操作指引中“控制电脑与小车地址永久固定”的假设，为更换 Wi-Fi 后仍处于同一局域网的 ROS 1 部署提供统一配置流程。

## 涉及文件

- `rosmaster/NETWORK_CONFIGURATION.md`：控制电脑 WSL、RViz、小车端、mDNS / DHCP、防火墙和只读验证的完整清单。
- `docs/operations.md`：在旧固定地址操作段前添加动态配置文档入口。
- `AGENTS.md`：将唯一 Master 规则改为动态发现或显式配置，而非旧固定地址。
- `/home/car/.config/smartcar/ros_network.{sh,env}`：控制电脑启动脚本使用的共享网络配置。

## 验证

- `~/start_ros_master.sh` 与 `~/start_rviz.sh` 均加载共享网络配置。
- 在当前网络，自动发现结果为 `ROS_IP=192.168.31.252`、`ROS_MASTER_URI=http://192.168.31.252:11311`。
- 启动脚本与共享 shell 配置均已通过 Bash 语法检查；未启动 ROS 节点或发送运动命令。

## 已知限制

ROS 1 不提供 Master 自动发现。控制电脑的地址可自动发现，但小车必须通过本次显示的 `MASTER_IP`、路由器 DHCP 保留地址或可解析的 mDNS 主机名来定位 Master。小车端文件不在本机挂载时，需按文档在小车上单独更新启动环境。
