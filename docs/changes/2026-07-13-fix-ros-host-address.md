# 固定小车 ROS 主机地址

日期：2026-07-13

## 问题

- 小车主机名 `ucar-mini` 仅解析到无作用域 IPv6 link-local 地址。
- roslaunch 使用该地址进行自身 XML-RPC 回连时无法连接，导致 launch 在节点启动前卡住。

## 改动

- 在小车启动命令中显式设置 `ROS_IP=192.168.8.231`。
- 显式设置 `ROS_MASTER_URI=http://192.168.8.231:11311`，使小车自身与控制电脑使用同一 ROS Master 地址。

## 验证

- 已使用上述环境变量实际启动；roslaunch 成功启动 rosmaster、move_base、lidar_loc 与 navigation_2026，不再卡在 XML-RPC 自检阶段。
