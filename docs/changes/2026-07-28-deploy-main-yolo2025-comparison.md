# 部署 main 的 yolo2025 做真机静态对照

## 目的

将 GitHub `main` 分支提交 `d23d198` 的完整 `yolo2025` 包部署到小车，通过关闭自动
起点目标的静态启动，判断底盘 CRC 与 USB 异常是否由当前 `simulation_real`
分支中的 yolo2025 改动引起。

## 涉及文件

车端替换：

- `~/ucar_ws/src/yolo2025/`：完整替换为 `origin/main:ucar_ws/src/yolo2025`

本地记录：

- `docs/operations.md`
- `犯错档案.md`
- `docs/changes/2026-07-28-deploy-main-yolo2025-comparison.md`

本地当前工作树和 `simulation_real` 分支源码没有被切换或覆盖。车端没有保留旧包
备份目录或归档。

## 部署与验证结果

- `origin/main` 与 GitHub 远端均指向 `d23d1983e7aeac9c1fd78a16a30cce233dac0de3`。
- `yolo2025` 在小车 Ubuntu 18.04 / ROS Melodic 中以
  `PYTHON_EXECUTABLE=/usr/bin/python2` 构建成功。
- 车端 `2026.launch`、`2026.py` 和 `CMakeLists.txt` 的 Git blob 哈希与
  `origin/main` 完全一致；脚本权限已设为 `0755`，shebang 为 LF。
- `roslaunch --nodes` 确认包含 `/base_driver`、`/move_base` 和
  `/navigation_2026`。
- 静态启动显式设置
  `/navigation_2026/startup_goal_enabled=false`，没有发送导航目标。
- 启动后 45 秒内先后出现 `check crc16 faild(imu)` 与
  `check crc16 faild(ahrs)`；当时内核 Hub disconnect 计数仍为 `2`，尚未新增
  整体 Hub 掉线。按照安全门立即发布零速度并停止。

## 结论

底盘串口 CRC 在 GitHub `main` 的 yolo2025 上仍可复现，因此不是当前
`simulation_real` 的 yolo2025 修改造成。问题仍位于 yolo2025 之外的底盘串口、
USB Hub、上游线缆或公共供电链路。

## 已知限制

- 因 CRC 已触发安全门，没有继续等待 Hub 掉线，也没有执行运动测试。
- `main` 启动会尝试旧语音合成，扬声器已拔除时返回错误码 `11212`；该错误不发送
  底盘速度。
- `main` 的旧扫描回调在关闭阶段会报告 `publish() to a closed topic`，当前分支已
  有对应 shutdown 防护。
- 小车当前保留的是 `main` 的 yolo2025；恢复当前分支版本需重新部署。
