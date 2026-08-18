# usb_cam USB Hub 热重连恢复（2026-08-18）

## 目的

USB Hub 整体断开后，摄像头会重新枚举为新的 `/dev/videoN`，旧版 `usb_cam` 仍持有断开前的 fd，导致点 52 启动 QR 相机时在 `VIDIOC_QBUF` 返回 `ENODEV(19)`。本改动让节点在摄像头热重连后自动重新打开稳定别名、重新设置格式并重新申请采集缓冲区。

## 实现

- `usb_cam` 增加 `capture_requested` 状态，区分“用户主动停止”与“设备异常断开”。
- `start_capture` 遇到 `ENOENT/ENODEV/ENXIO/EPIPE/EIO` 时，在 `reconnect_timeout=8s` 内按 `reconnect_interval=0.5s` 重试。
- 运行中 `read_frame` 遇到上述设备断开错误后，释放旧 fd、mmap buffer 和图像缓冲；ROS 定时器继续尝试重新打开 `/dev/ucar_camera` 并恢复采集。
- 启动时摄像头暂时不存在不再直接退出 `usb_cam` 节点；后续 QR 启动服务可以触发重连。
- 手动调用 `/usb_cam/stop_capture` 会清除重连请求，不会在用户主动暂停后自行打开相机。

## 涉及文件

- `ucar_ws/src/usb_cam/include/usb_cam/camera_driver.h`
- `ucar_ws/src/usb_cam/src/camera_driver.cpp`
- `ucar_ws/src/usb_cam/src/usb_cam.cpp`
- 三套 `ucar_ws/src/ucar_2026*/launch/2026.launch`
- `ucar_ws/src/usb_cam/launch/usb_cam.launch`

## 验证

- 本地 launch XML 和差异检查通过；不在本机编译 ROS/C++。
- 已同步 7 个相关源码/launch 文件到车端，并在 Ubuntu 18.04 / ROS Melodic 完成 `usb_cam` 白名单构建。
- 车端新二进制已包含 `USB_CAM_RECONNECT capture resumed`；`roslaunch --nodes ucar_2026_national 2026.launch task_enabled:=true` 解析通过。
- 尚未人为断开 USB Hub 做实车回归：当前车端已有用户 ROS 主流程运行，未强制停止或重启，也未发送运动命令。
- 需在下一轮车辆静止条件下验证：启动新相机节点、确认图像发布，断开并恢复 USB Hub 后观察重连日志、`/dev/ucar_camera` 新目标和图像恢复。

## 已知限制

自动恢复依赖 USB Hub 和摄像头在超时时间内重新枚举；超过 8 秒仍未出现时，当前一次 `/usb_cam/start_capture` 请求会失败，但节点保留并可由下一次请求再次尝试。硬件持续掉电、供电不足或 USB Hub 驱动不再恢复时，软件不能替代硬件修复。
