# 2026-08-15 摄像头设备名迁移 /dev/ucar_video → /dev/video0

## 背景

摄像头原本插在小车固定 USB 端口，靠 udev 规则（固定 `KERNELS` 路径 + `SYMLINK+="ucar_video"`）生成 `/dev/ucar_video` 符号链接。现摄像头改插 USB hub，固定端口路径匹配失效，符号链接不再生成。小车仅有一个 USB 摄像头，直接使用内核默认设备名 `/dev/video0`。

## 涉及文件

- `ucar_ws/src/yolo2025/launch/qrcode.launch` — `video_device` 参数改为 `/dev/video0`
- `ucar_ws/src/lane_proto/launch/lane_proto.launch` — `video_device` 默认值改为 `/dev/video0`
- `ucar_ws/src/lane_proto/scripts/lane_follow.py` — 默认设备及管线注释改为 `/dev/video0`
- `ucar_ws/src/ucar_2026/launch/2026.launch` — 两处 `video_device` 参数改为 `/dev/video0`
- `ucar_ws/src/ucar_2026/launch/yolo_dataset_capture.launch` — 同上
- `ucar_ws/src/ucar_2026/scripts/handoff_lane.sh` — 设备存在性检查改为 `/dev/video0`
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py` — `~video_device` 默认值改为 `/dev/video0`
- `ucar_ws/src/ucar_2026/test/test_production_camera_ocr.py` — 测试参数改为 `/dev/video0`
- `ucar_ws/src/ucar_cam/launch/ucar_cam.launch` — `video_device` 参数改为 `/dev/video0`
- `ucar_ws/src/startup_scripts/initdev_mini.sh`、`initdev_xiao.sh` — 删除摄像头 udev 固定端口规则（已失效）
- `docs/operations.md` — 摄像头设备描述同步更新

## 验证结果

- 全仓 `ucar_video` 引用仅剩 `docs/changes/` 两处历史快照文档（保持原样）
- 替换使用字节级写回，保留各文件原有行尾与 UTF-8 编码

## 已知限制

- udev 摄像头规则删除后，小车端 `/etc/udev/rules.d/ucar.rules` 中旧规则仅在重新执行 initdev 脚本或手动清理后移除；旧规则因端口路径不匹配本身不生效，无副作用。
- 依赖唯一 USB 摄像头被内核命名为 `/dev/video0`；若小车插入其他视频设备（如 USB 转 HDMI 采集卡）导致编号偏移，需重新考虑映射。
