# YOLO 数据集定时采集

## 目的

提供不驱动车辆的独立采集入口：通过 ROS 相机连续保存 300 张图像、每张间隔 0.5 秒，并直接生成
可用于 YOLO 标注的 `images/{train,val}` 与 `labels/{train,val}` 目录配对。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/yolo_dataset_capture.py`
- `ucar_ws/src/ucar_2026/launch/yolo_dataset_capture.launch`
- `ucar_ws/src/ucar_2026/CMakeLists.txt`
- `docs/operations.md`
- `犯错档案.md`

## 行为

- 默认 300 张、0.5 秒间隔、80/20 交错划分，得到 240 个训练图和 60 个验证图。
- 每张 JPEG 都创建一个同名空 label 文件；类别与框尚未生成，必须由后续标注工具写入标准 YOLO 行。
- launch 只启动 `/usb_cam` 和采集节点；采集节点通过 start/stop 服务按需打开/关闭视频流，不发布
  导航或底盘命令。
- `capture_manifest.json` 记录每张图的分区、相对路径与 ROS 图像时间戳，便于审计。

## 验证与限制

- 由于小车断电，本轮没有部署或启动该新功能；只完成源码和操作路径审查。
- 小车重新供电后，必须先在 Ubuntu 18.04 / ROS Melodic 构建 `ucar_2026`，确认 300 张、240/60
  分区、同名空标签和采集结束后的相机停止，再开始标注或与生产任务组合使用。
- 采集不包含自动标注，空标签不代表背景真值；请在标注前检查模糊、重复和曝光异常帧。
