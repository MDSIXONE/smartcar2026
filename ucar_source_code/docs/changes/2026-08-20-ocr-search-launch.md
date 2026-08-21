# 独立 OCR 搜索 launch

## 目的

参照省赛 `ucar_2026` 主流程，先拆出一个只用于单功能调试的 OCR 搜索程序。默认认为车辆已经在 3 号点，启动时原地朝向 13 号点，然后按 `428→429→430→431→432→433→434→435→436→445→444→437→419→427` 逐点导航并做完整 360° OCR 搜索。发现候选后以当前帧为起点连续居中对齐，每点最多 15 秒；超时失败就跳过当前点，继续下一个点，不增加像素容差。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/ocr_search.py`：独立 ROS Python2 节点。
- `ucar_ws/src/ucar_2026/launch/ocr_search.launch`：独立入口，复用底层导航和相机，但关闭原任务节点。
- `ucar_ws/src/ucar_2026/scripts/ocr_alignment.py`：只验证连续 OCR 对齐并播报完成的 ROS Python2 节点。
- `ucar_ws/src/ucar_2026/launch/ocr_alignment.launch`：独立对齐验证入口，不导航路线。
- `ucar_ws/src/ucar_2026/test/test_ocr_search_launch.py`：入口隔离和默认起始点回归测试。
- `ucar_ws/src/ucar_2026/test/test_ocr_alignment_launch.py`：对齐验证入口回归测试。
- `ucar_ws/src/ucar_2026/CMakeLists.txt`：安装独立节点并注册测试。
- `docs/operations.md`：单功能调试启动与安全前置条件。

## 行为边界

搜索节点只做导航、原地起始朝向、OCR 360°搜索和候选框连续居中对齐，保存识别候选及图片路径；点位被动态障碍物占用时跳过；单点连续对齐 15 秒失败时跳过当前点；所有点无成功 OCR 结果时导航到 441。不执行省赛主流程的 QR、墙边停车、搬运、仿真或巡线交接。OCR 搜索速度为 `0.70rad/s`，对齐阈值固定为 `30px`，不按固定帧数重试或放宽像素容差；固定朝向旋转逻辑保持不变。对齐验证节点只使用当前姿态连续旋转，成功后发布 `ALIGNMENT_COMPLETED` 并通过 TTS 播报“对齐完成”。原 `launch/2026.launch` 与 `scripts/production_task_2026.py` 未接入这些独立节点。

## 验证

- 本机 Python 语法检查通过。
- 独立 launch XML 和原省赛 launch XML 解析通过。
- 两个独立入口回归测试 `9/9` 通过；标准 `ucar_2026` 测试 `131` 项通过、`90` 项因本机无 ROS 环境跳过。
- `git diff --check` 通过。

## 已知限制

本机未加载 ROS Melodic/Python2，也未连接车端，因此没有启动 ROS、调用 OCR TensorRT 或发送运动指令。车端第一次使用前仍需按 `operations.md` 检查 `/odom_raw`、`odom -> base_link`、`map -> base_link` 和车辆零速。
