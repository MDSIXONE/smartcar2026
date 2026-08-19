# 2026-08-20 OCR 旋转速度分离

## 目的

固定朝向的短距离原地转向耗时过长；OCR 的完整 360° 扫描需要保留较慢速度以保证画面覆盖和识别稳定性。

## 改动

- 三套实际车端任务脚本新增 `fixed_heading_rotation_speed`，固定朝向原地旋转使用 `0.35 rad/s`。
- 三套 `2026.launch` 显式配置 `fixed_heading_rotation_speed=0.35`。
- 三套 OCR 完整旋转的 `ocr_scan_rotation_speed` 从 `0.30` 调整为 `0.18 rad/s`；脚本默认值同步为 `0.18`。
- QR 完整旋转继续使用 `qr_rotation_speed=0.18`，不与固定朝向速度混用。

## 验证

- 三套任务脚本 Python AST/语法检查通过。
- 三套 launch XML 解析通过，参数值核对为固定朝向 `0.35`、OCR 360° `0.18`、QR 360° `0.18`。
- `git diff --check` 通过。

## 生效条件

运行中的 Python 2 任务节点不会热加载源码和 launch 参数；车辆安全停止后，下一次启动对应 `2026.launch` 才会生效。
