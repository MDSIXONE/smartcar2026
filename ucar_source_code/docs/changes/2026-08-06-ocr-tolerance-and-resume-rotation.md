# OCR 对准容差放宽与转圈续转优化

## 目的

2026-08-06 晚两轮 mission 运行中，点 16/17 出现「一直在重复对准食品」：本轮食品类别未被成功记录（observations 仅 1 条点25 日用品），而转圈扫描每次遇到「食品加工车间」候选都会停下对准；18 px 对准容差过严，反复 `horizontal_error_px` 震荡（-186→-10）导致 aligned 失败、类别不记录，随后继续转圈又遇到同一标签，形成死循环。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`

## 改动

1. **对准容差放宽**：`ocr_alignment_tolerance_px` 18.0 → 35.0（launch 190 行显式参数 + py 207 行默认值同步），640px 宽图像约 5.5%，允许目标不严格居中即判定 aligned，减少反复 PD 对准。
2. **对准完从当前朝向继续转圈**：删除 `handle_candidate` 中对准后的 `restore_ocr_capture_yaw(response, scan_label + " resume")`。对准改变了 yaw 后直接从当前朝向继续累计转圈进度，不再回到捕获候选时的朝向重复扫描同一段墙面；对准前的 `restore_ocr_capture_yaw` 保留（observe_wall 需要从捕获朝向测墙点）。
3. **转圈旋转速度加快**：`ocr_scan_rotation_speed` 0.18 → 0.25（launch 170 行显式参数 + py 197 行默认值同步）。底盘实际角速度约命令值 55%（0.10 → 0.14 rad/s），一圈 360° 转圈约 63s → 46s；`rotation_timeout_scale` 超时按 `2π/speed*scale` 自适应，无需调整。

## 验证结果

- 本地已改、scp 同步小车，`grep` 确认小车端 launch=35.0、py 默认值=35.0，resume restore 已删除。
- 当前运行中的任务（pid 12844，21:38 启动）仍是旧代码，**需重启任务后生效**；未在小车端验证运行行为。
- **2026-08-07 实车验证（mission 22:02 启动，run_20260806_220333）**：OCR 修复生效。点 13「食品加工车间」第二轮转圈仅 1 次 attempt 以 `horizontal_error_px=-23.5`（≤35）对准成功并记录；点 25「日用品加工车间」6 次 attempt（-169.1→-124→-78→-44.5→-20.4）对准成功并记录，wall_point=160。点 16 转圈期间检测到已记录的「日用品」候选不再停下对准（修复②生效），转满一圈后继续。
- 该轮任务最终在点 26 途中因 USB 硬件故障中止（`usb 1-2.1` 枚举失败循环 → odom/imu 发布间隙 1.09s → move_base recovery 卡死 → guard cancel 5s 超时），与 OCR 修改无关，详见犯错档案 2026-08-07 条目。

## 已知限制

- 35.0 为经验值，若仍对不准可继续上调或检查 PD 修正本身。
- 对准后不恢复 yaw 会导致剩余转圈弧段偏少（原逻辑保证完整一圈），因候选检测是异步全向的，实际影响很小。
