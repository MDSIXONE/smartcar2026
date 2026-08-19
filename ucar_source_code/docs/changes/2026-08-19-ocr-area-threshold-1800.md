# 2026-08-19：OCR 候选框面积门、对准容差与尝试次数调整

## 改动

三套 2026 主流程将 `ocr_alignment_tolerance_px` 调整为 `30px`，将
`ocr_candidate_min_bbox_area_px` 调整为 `500px²`，并将
`ocr_alignment_attempts` 从 `6` 调整为 `12`。
任务脚本默认值与三套 launch 保持一致。

## 目的

降低面积门，避免近距离有效 OCR 框被面积门提前过滤；面积小于 `500px²` 的极小残缺框仍会被过滤。
放宽横向对准验收并增加连续修正次数，给点 15 这类误差收敛较慢的目标更多时间。

## 验证与生效

- 三套 perception 回归更新为 `500px²`，`400px²` 小框拒绝、`1938px²` 边界框接受、`2501px²` 有效框接受。
- 修改只涉及 Python2 脚本、launch 和测试，不需要 catkin 编译。
- 同步后必须在车辆零速、`/odom_raw`、两个 TF 和 `/scan` 安全检查通过后重启实际任务节点；
  运行中的 Python2 节点不会热加载。
