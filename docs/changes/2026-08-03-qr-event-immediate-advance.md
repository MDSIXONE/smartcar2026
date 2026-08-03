# QR 识别事件即时推进

## 目的

二维码在当前观察朝向识别成功后立即开始下一次观察，不把配置的等待上限误用为固定停留时间。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `docs/operations.md`

## 实现

- QR 回调设置线程事件；等待循环由该事件立即唤醒，并先再次检查序列号与去重规则。
- 每次朝向的序列号基线在开始朝向导航前记录；因此转向/停稳期间收到的新二维码在
  朝向完成时会立即接受，不会被错误当作旧消息丢弃。
- `qr_search_timeout` 是未识别时才生效的上限，默认仍为 4 秒；`qr_hold_seconds` 保留为旧配置兼容回退。
- 搜索空闲时最多每 50 ms 重新检查安全状态与 ROS shutdown；这不是识别成功后的停留。

## 验证

- 新增回归用例：回调在等待期间到达时，`wait_for_fresh_qr()` 返回识别内容且不等到搜索上限；
  以及朝向导航期间收到二维码时不进入搜索等待。
- 本机仅做 Python 语法和无 ROS 测试发现；按仓库约束，完整 Python 2 ROS 测试与构建须在小车 Ubuntu 18.04 上完成。

## 已知限制

- 本改动不启动相机、不连接 ROS Master，也不改变相机的按需启停策略。
