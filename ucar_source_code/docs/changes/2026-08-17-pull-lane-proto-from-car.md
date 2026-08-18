# 从小车拉取最新 lane_proto 巡线代码（2026-08-17）

## 目的

将小车 `192.168.8.231:/home/ucar/ucar_ws/src/lane_proto` 的最新巡线实现同步到本地
`ucar_ws/src/lane_proto/`。

## 同步内容

- `CMakeLists.txt`、`package.xml`、`config/`、`launch/`；
- 根目录 `test_board.py`；
- `scripts/`、`test/`、`tools/` 中的源码；
- `cuda/` 的构建脚本、源码和生成代码；
- `lib/` 运行库及说明文件。

排除了现场 `dump/`、抓拍图片、Python 字节码/缓存，以及 CUDA 训练权重和编译中间产物。

## 验证

- 车端与本地同步范围内 31 个文件 SHA-256 全部一致；
- 本机 Python 语法检查通过；
- `test_lane_runtime.py`：8 项中 5 项通过、3 项因本机没有 ROS Melodic Python 运行时跳过；
- 未启动 ROS、未发布速度、未启动车辆。

## 限制

完整 ROS Melodic/Catkin 回归仍需在小车 Ubuntu 18.04 上执行。
