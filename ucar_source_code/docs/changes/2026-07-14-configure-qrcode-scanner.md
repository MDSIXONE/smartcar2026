# 2026-07-14：配置车载二维码扫描与接口查询

## 目的

将车载 USB 摄像头二维码识别整理为可直接启动的 ROS 节点。识别结果发布到兼容旧任务代码的 `/qr_result`；可选地对二维码中的 `a` 至 `i` 查询 `http://192.168.8.1:3663/<字母>`，并将完整 JSON 结果发布到 `/qr_api_result`。

## 涉及文件

- `ucar_ws/src/yolo2025/scripts/qrcode_scanner.py`
  - 新增 WeChat QR 优先、OpenCV 回退的识别节点；支持扫描开关、去重和异步接口查询；直接转换 `rgb8` 图像，避开 Python 2 专用 `cv_bridge`。
- `ucar_ws/src/yolo2025/launch/qrcode.launch`
  - 启动 USB 摄像头与扫描节点；不再启动无关的图像翻转节点，避免其 Python 2 `cv_bridge` 错误。
- `ucar_ws/src/yolo2025/CMakeLists.txt`
  - 将二维码扫描脚本安装到 catkin 可执行目录。
- `docs/operations.md`
  - 记录启动、接口查询与停止扫描命令。

## 验证

- 本地通过 Python 语法校验。
- 小车端通过 `/home/ucar/myenv/bin/python3 -m py_compile` 校验并完成 `yolo2025` 包构建。
- 小车端 `roslaunch --nodes yolo2025 qrcode.launch` 静态检查通过；修复后实际启动节点为 `/usb_cam`、`/qrcode_scanner`。
- 实机扫描验证通过：二维码原文为 `http://192.168.8.1:3663/i`，`/qr_api_result` 返回 HTTP `200`，JSON 中 `code: 200`、`result: i`。
- 识别节点不使用 `cv_bridge`，成功规避了 Python 3 运行 WeChat QR 时加载 ROS Python 2 二进制模块的错误。

## 已知限制

- 识别准确率受摄像头对焦、二维码尺寸、角度和光照影响。
- 接口服务不在线时，二维码原始内容仍会发布到 `/qr_result`，接口错误会发布到 `/qr_api_result`。
