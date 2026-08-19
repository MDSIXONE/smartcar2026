# 2026-08-20 二维码固定面与 OCR 旋转速度调整

## 目的

缩短二维码固定面之间的同点转向时间，并提高 OCR 完整 360°扫描速度。

## 最终参数

- `fixed_heading_rotation_speed=0.70 rad/s`：二维码固定面切换，原为 `0.35 rad/s`。
- `ocr_scan_rotation_speed=0.35 rad/s`：OCR 完整 360°扫描，原为 `0.18 rad/s`。
- `qr_rotation_speed=0.18 rad/s`：二维码完整 360°扫描，保持不变。

三套任务脚本和三套 `2026.launch` 的默认值、显式值及现场参数文档已同步为以上配置。

## 验证与部署

- 本地三套 Python 脚本编译检查、三套 launch XML 解析、参数契约和 `git diff --check` 通过。
- 六个运行文件已同步到 `ucar-mini (192.168.8.231)`，本地/车端 SHA-256 一致。
- 车端三套 Python2 `py_compile` 和三套 `roslaunch --nodes` 通过；未重启 ROS、未发送运动指令。

## 生效条件

运行中的 Python 2 任务节点不会热加载源码和 launch 参数；车辆安全停止后，下一次启动对应
`2026.launch` 才会生效。调整后需要通过车端日志确认固定面和完整旋转的实际运动效果。
