# 修复 WSL 与小车之间的 ROS XML-RPC HTTP/1.1 卡死

## 目的

解决小车连接 WSL ROS Master 后，`roslaunch` 间歇卡在
`load_parameters starting ...`、节点无法启动的问题，同时保持 WSL 为唯一 Master。

## 根因

小车使用 Linux 4.9，WSL 使用新内核。Noetic Master 根据服务端 WSL 内核启用
HTTP/1.1，但当前 WSL 镜像网络到小车的路径会间歇性丢失这类 XML-RPC 短响应。
最小空 `system.multicall` 在默认 HTTP/1.1 下 10 次有 7 次超时；强制 Master
使用 HTTP/1.0 后连续 20 次全部成功。

## 涉及文件

- `rosmaster/start_ros_master.sh`
- `rosmaster/python_http10_compat/sitecustomize.py`
- `rosmaster/NETWORK_CONFIGURATION.md`
- `docs/operations.md`
- `犯错档案.md`
- `docs/changes/2026-07-28-fix-wsl-vehicle-xmlrpc-http11.md`

## 验证结果

- HTTP/1.1 原始最小复现：10 次中 7 次在 2 秒内无响应。
- HTTP/1.0 隔离 Master：空 multicall 连续 20/20 成功。
- 正式启动脚本和兼容层已部署到 WSL 用户目录，SHA-256 与仓库文件一致，脚本权限为
  `0755`；正式 Master 下空 multicall 再次连续 20/20 成功。
- 小车 `ucar_2026 2026.launch` 已越过 `load_parameters`，14 个节点全部启动，
  CymPlanner 完成初始化，证明原始 XML-RPC 故障已消除。
- 兼容层只修改 ROS XML-RPC 请求处理器，不修改系统 ROS 安装或 TCPROS。

## 已知限制

- 该兼容层是当前 WSL 镜像网络与 Linux 4.9 小车组合的定向修复；未来小车内核或
  WSL 网络模式升级后，可重新评估是否仍需强制 HTTP/1.0。
- 本次后续静态安全门发现小车上游 `usb 1-2` Hub 会整体断开并重新枚举；
  `base_driver` 单节点隔离测试仍可复现，因此完整静态安全门尚未通过。该硬件链路
  问题与已经修复的 XML-RPC 卡死相互独立，处理前不得发送导航目标。
- 物理拔除 `UACDemoV1.0` USB 扬声器后，设备已从 USB 拓扑消失，但新的
  `base_driver` 隔离测试仍在约 11 秒内出现 IMU CRC 错误和 Hub disconnect；
  扬声器不是唯一原因，剩余排查对象为 Hub、本体上游线缆和公共供电。
