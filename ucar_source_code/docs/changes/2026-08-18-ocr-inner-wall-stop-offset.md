# OCR 识别后内墙停车偏移调整（2026-08-18）

## 目的

将 OCR 识别成功、完成墙面交点测量后生成的内墙停车坐标，从墙面交点向场内内缩 `0.25m` 调整为 `0.29m`。

## 改动

- 三套任务脚本新增 `ocr_stop_offset_m` 参数，并将该参数直接传给墙面停车坐标计算函数。
- `stop_point_for_wall_point()` 从按网格边长一半计算偏移改为接收显式内缩距离；网格几何中的 `square_side_m=0.5` 保持不变。
- `ucar_2026`、`ucar_2026_national`、`ucar_2026_extra` 三套 `launch/2026.launch` 均设置 `ocr_stop_offset_m=0.29`。
- 对应三套几何测试补充运行时测试夹具参数，并将墙面停车坐标断言锁定为 29cm 内缩。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026_national/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_geometry.py`
- 三套 `scripts/production_task_2026.py`
- 三套 `launch/2026.launch`
- 三套 `test/test_production_task_geometry.py`
- `docs/operations.md`

## 验证

- 三套 Python 语法检查、几何测试、launch XML 解析与 `git diff --check` 已通过。
- 未在本机编译 ROS、启动主流程或发送车辆运动命令。

## 已知限制

- `0.29m` 尚未在 Ubuntu 18.04 / ROS Melodic 车端现场验证实际停车误差；部署后必须重启对应任务节点。
