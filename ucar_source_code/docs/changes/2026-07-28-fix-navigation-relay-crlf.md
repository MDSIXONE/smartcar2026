# 修复真机 navigation scan relay 的 CRLF 启动失败

## 目的

修复从 Windows 工作区部署后，`navigation_scan_relay.py` 的 shebang 被 CRLF 破坏，
导致真机导航链路无法建立 `/scan` 和 `map` TF 的问题。

## 涉及文件

- `.gitattributes`
- `docs/operations.md`
- `犯错档案.md`
- `docs/changes/2026-07-28-fix-navigation-relay-crlf.md`

车端重新同步并规范化：

- `ucar_ws/src/ucar_2026/scripts/navigation_scan_relay.py`
- `ucar_ws/src/ucar_2026/scripts/stop_2026_task.sh`

## 修复

- 新增 `*.py text eol=lf` 与 `*.sh text eol=lf`，防止 Windows 检出把可执行脚本转换为
  CRLF。
- 将当前两个 `ucar_2026` 脚本规范化为 LF，重新上传后设置权限为 `0755`。
- 部署文档增加本地 `git ls-files --eol`、车端 shebang 十六进制和执行权限检查。

## 验证结果

- 修复前可稳定复现 `/usr/bin/env: ‘python2\r’: No such file or directory`。
- 修复后本地与车端 shebang 都以单个 `0a` 结束，不含 `0d`。
- `navigation_scan_relay` 启动成功并报告 `/scan_raw -> /scan is ready`，静态墙体掩码就绪。
- `/odom_raw` 全部为有限值，线速度和角速度均为零。
- `odom -> base_link` 与 `map -> base_link` TF 均可读取。
- `/scan_filtered` 与 `/scan_global_obstacles` 均约为 `12 Hz`。
- `/cmd_vel` 只有 `/move_base` 发布、`/base_driver` 订阅。
- 本地 RViz 使用正式真机配置启动，成功订阅 `/map` 与 `/usb_cam/image_raw`。
- 测试全程没有发送导航目标；停止前再次发布零速度。
- RViz、车端 launch 和 WSL ROS Master 均已正常停止，两端无残留 ROS 启动进程。

## 已知限制

- 本次为无目标启动验收，没有进行路径运动测试。
- 运行期间偶尔出现激光时间戳略超前于最新 odom TF 的外推警告；relay 按 fail-safe
  策略丢弃对应全局障碍扫描，两个扫描话题仍持续约 `12 Hz`。后续实际导航若频繁出现
  Costmap TF 超时，应单独测量时间差并调整 TF 时序，不能通过放宽 NaN 安全门绕过。
