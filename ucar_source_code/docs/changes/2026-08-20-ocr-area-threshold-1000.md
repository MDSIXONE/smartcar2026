# OCR 候选框面积门调整为 1000px²

## 改动

三套 2026 主流程将 `ocr_candidate_min_bbox_area_px` 从 `500px²` 调整为
`1000px²`，并同步任务脚本默认值、launch 参数和 perception 回归测试。

## 行为

- OCR bbox 面积小于 `1000px²` 的候选继续被忽略，不停车、不进入视觉对准。
- 面积等于或大于 `1000px²` 的候选允许进入原有置信度和文本判断流程。
- `ocr_alignment_tolerance_px=30` 与 `ocr_alignment_attempts=12` 不变。

## 生效

运行中的 Python2 任务不会热加载参数；需要在车辆安全停止后重启实际使用的
`2026.launch` 才会加载新门槛。本次不启动 ROS、不发送运动指令。
