# OCR 对齐最小转向速度（MCU 死区修复）

## 目的

修复点 25 OCR 对齐实机 ABORT：`PRODUCTION_OCR_TURN_025_日用品 protected alignment did not
reach 0.022 rad of measured yaw within 2.5 s (actual=0.004 required=0.012)`——PD 对齐输出
0.073 rad/s 低于底盘 MCU 旋转死区，车几乎没转。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `docs/operations.md`
- `犯错档案.md`

## 根因

- 对齐 PD（`alignment_angular_speed`，production_task_perception.py:31-49）无最小速度
  输出限制；`base_driver.cpp` 只裁剪最大值（angular_speed_min_=0 未参与钳制）；死区在
  底盘 MCU 固件层。
- 证据链：QR 阶段 0.18 rad/s 能转（第 4 次实测成功）、点 25 对齐 0.073 rad/s 转不动
  （actual=0.004），死区阈值在 0.073~0.18 rad/s 之间。

## 行为

- 新增参数 `ocr_alignment_min_speed`（默认 0.12，2026.launch 已加
  `<param name="ocr_alignment_min_speed" value="0.12"/>`）。
- `rotate_in_place_for_yaw` 发布速度前钳制：`abs(speed) < min_speed` 时提升到
  `min_speed * direction`（保持正确定向）。
- 超转风险可控：progress 每帧累计（`positive_turn_increment`）、达到
  `required = target - tolerance` 立即停转并确认底盘停止；小量超转由下一轮 PD 反向修正。

## 验证结果

- 本机 py_compile 通过；小车端 python2 py_compile 通过（VEHICLE_OK）。
- 实车验证待做：观察节点日志 `PRODUCTION_OCR_ALIGNMENT_TURN`（command_speed 应 ≥0.12）。

## 已知限制

- 死区阈值因电池电压/负载变化会漂移，0.12 为经验值；若仍出现
  `did not reach` 可继续上调。
- 未处理线速度死区（OCR 对齐只用旋转，不影响）。
