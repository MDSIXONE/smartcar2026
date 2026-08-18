# 摄像头改用稳定 udev 别名（2026-08-18）

## 问题

车端 RHX USB 摄像头的实际设备从 `/dev/video0` 重新枚举为 `/dev/video1`，而主流程仍固定打开 `/dev/video0`，导致点 52 到达后 QR 阶段启动 `/usb_cam/start_capture` 返回 `Video4linux ... (19)` 并中止任务。

## 改动

- 新增 `ucar_ws/src/startup_scripts/ucar_camera.rules`，按已确认的 USB 摄像头 `idVendor=0edc`、`idProduct=2050`、`serial=20190827` 创建稳定别名 `/dev/ucar_camera`。
- 标准、省赛/国赛、额外任务的 `2026.launch` 将相机设备集中为 `camera_device` 参数，默认使用 `/dev/ucar_camera`。
- 生产任务、QR、巡线交接、数据采集和常用相机入口统一使用 `/dev/ucar_camera`；不再把可变的 `/dev/video0` 作为运行时默认值。
- `usb_cam` 驱动的裸节点默认值也改为 `/dev/ucar_camera`；该 C++ 改动必须在车端 Ubuntu 18.04 / ROS Melodic 编译，不能在本机编译后上传构建产物。
- `/dev/ucar_camera` 是符号链接，实际目标仍可能是 `/dev/video0`、`/dev/video1` 等内核枚举节点，因此不需要在程序中维护端口编号映射。

## 车端安装

先停止本轮任务节点，再把规则安装到车端并刷新 udev：

~~~bash
sudo install -m 0644 ~/ucar_ws/src/startup_scripts/ucar_camera.rules \
  /etc/udev/rules.d/99-ucar-camera.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
ls -l /dev/ucar_camera
readlink -f /dev/ucar_camera
~~~

Python/launch 改动只需重启下一次 2026 主流程即可读取；`usb_cam` C++ 默认值改动则需在车端完成对应白名单构建。当前运行中的 ROS 节点不会热加载已经启动时的 `video_device` 参数，因此本轮不要仅替换文件后直接重试 QR。

## 验证

- 本地完成 launch XML、Python AST、udev 规则文本和运行时默认值静态检查；未在本机编译 ROS/C++。
- 车端已同步 28 个相关文件并完成关键文件 SHA-256 校验；`usb_cam` 白名单构建通过。
- 车端已安装 `/etc/udev/rules.d/99-ucar-camera.rules`，并验证 `/dev/ucar_camera -> /dev/video0`、硬件身份为 `0edc:2050` / `20190827`。
- 车端 `v4l2-ctl --device=/dev/ucar_camera --get-fmt-video` 成功读取 `1920/1080 MJPG`；未启动 ROS 主流程或发送车辆运动命令。
- 尚未执行真实 QR 流程，因此 `/usb_cam/start_capture` 的端到端验证留到下一次按安全门启动主流程时完成。

## 已知限制

规则绑定当前已确认的 RHX 摄像头硬件序列号；若更换摄像头，需按新的 `idVendor`、`idProduct`、`serial` 更新规则并重新加载 udev。
