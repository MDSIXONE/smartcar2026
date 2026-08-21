# OCR 对齐验证入口改为直接发送 /cmd_vel

## 目的

`roslaunch ucar_2026 ocr_alignment.launch` 原本把 Twist 发布到 `/cmd_vel/navigation`，再经
`cmd_vel_owner`（默认 mission 模式）仲裁转发到 `/cmd_vel` 底盘驱动。该入口只做原地连续 OCR
对齐验证，不参与导航/巡线仲裁，因此改为让 `ocr_alignment` 节点直接发布 `/cmd_vel`，跳过
`cmd_vel_owner` 转发层，使该入口只有这一个速度发布者。

## 涉及文件

- `ucar_ws/src/ucar_2026/launch/ocr_alignment.launch`：`cmd_vel_topic` 由
  `/cmd_vel/navigation` 改为 `/cmd_vel`，并同步更新顶部行为注释（本机与车端已一致）。

## 行为边界

新链路为 `ocr_alignment.py → /cmd_vel → base_driver → 底盘`。include 的 `2026.launch`
（`task_enabled:=false`）中 `cmd_vel_owner` 与 `move_base` 组仍会被启动，但 `move_base` 无
goal 时不发布速度，因此实际写入 `/cmd_vel` 的只有 `ocr_alignment` 节点。`wait_for_chassis_stop`
的零速确认、`stop_confirmation_timeout` 停止判定等逻辑不受影响，零速直接送达底盘。

## 验证

- 本机回归测试 `test_ocr_alignment_launch.py` `4/4` 通过（现有断言未涉及 `cmd_vel_topic`，无需改动）。
- launch XML 车端解析通过，`cmd_vel_topic=/cmd_vel` 已在车端确认。
- 本地/车端 `ocr_alignment.launch` SHA-256 一致：
  `94cd0beb09ed5e71cefd36e05fec7651bfabcddeeddcb0c06ab5607ee42734ab`。
- 车端无 `production_task_2026` 残留进程。

## 已知限制

- 车端在同步前已有一个 19:28 启动的旧版 `ocr_alignment.launch` 实例（含 `move_base`），
  需停止该实例并重新启动后才能加载新参数。
- 若未来在验证期间另起会写 `/cmd_vel` 的节点，会与 `ocr_alignment` 竞争该 topic；验证前
  应确认没有其他运动源运行。