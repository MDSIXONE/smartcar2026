# 2026-08-19：OCR 对准强制使用下一帧

## 现象

移动 OCR 对准时，连续日志出现相同的 `horizontal_error_px`，例如
`-192.7 → -192.6 → -192.3 → -192.8`，但任务同时持续发布旋转速度。

## 根因

ROS 相机取帧逻辑原先只检查最近图像不超过 `camera_frame_timeout=1.0s`，没有要求
`camera_sequence` 在本次抓取前后增加。因此连续 OCR 请求可能在 1 秒窗口内重复保存同一帧，
导致车辆已经转动而视觉误差不更新。

## 最小修复

- 标准、省赛、国赛三套 `production_task_2026.py` 在保存 ROS 图像前记录当前
  `camera_sequence`。
- 只有收到 `sequence > baseline_sequence` 的下一帧才保存并送入 OCR。
- 保持 `ocr_alignment_tolerance_px=20`、`ocr_candidate_min_bbox_area_px=2400`、旋转速度和
  对准次数不变；若等待窗口内无新帧，按现有相机失败路径中止，暴露相机链路问题。

## 验证与生效

新增三套相机取帧回归用例，验证旧帧存在时会等待异步到达的新帧。修改只涉及 Python2 任务脚本，
不需要 catkin 编译；同步后必须在车辆零速、`/odom_raw`、两个 TF 和 `/scan` 安全检查通过后，
重启实际任务节点才会生效。当前运行中的节点不会热加载。
