# 原生相机 OCR、雷达墙点匹配与新生产路线

## 目的

- 保持已有 1–418 编号不变，补齐中间区域可见线端点以及左右墙缺失的可匹配点。
- 将生产路线替换为：
  `3 → 2 → 1 → 11 → 21 → 31 → 32 → 33 → 34 → 35 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 20 → 30 → 40 → 39 → 38 → 37`。
- 初始 52 点三方向二维码流程结束后继续使用 ROS `usb_cam` 图像；生产阶段由
  Python 2 任务节点保存当前帧，Python 3 helper 调用车端 `live_ppocr.py`
  CUDA OCR，再完成视觉水平居中和前向雷达墙点匹配。
- 保存每个状态点的图片与审计记录，并选出三个不同墙点及其 OCR 内容。

## 涉及文件

- `production_full_grid_all_numbered.json`
- `production_full_grid_all_numbered.png`
- `tools/update_production_grid_assets.py`
- `ucar_ws/src/ucar_2026/config/production_full_grid_all_numbered.json`
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/scripts/production_camera_ocr.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_geometry.py`
- `ucar_ws/src/ucar_2026/scripts/production_task_perception.py`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/CMakeLists.txt`
- `ucar_ws/src/ucar_2026/package.xml`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_perception.py`
- `ucar_ws/src/ucar_2026/test/test_production_camera_ocr.py`
- `docs/operations.md`
- `docs/quickstart.md`

## 地图编号

- 419–445：PNG 中间区 27 个黑色线端点，按从上到下、从左到右编号。
- 446–451：中间区左右墙缺失的 6 个顶点。
- 452–459：中间区左右墙缺失的 8 个边中点。
- 顶层 `wall_reference_point_numbers` 明确列出 56 个真实中间墙候选点；
  雷达匹配不会在中心点或灰色网格点中搜索。
- 根目录 JSON 与 ROS 包内副本保持字节一致；PNG 由工具做确定性标签叠加。

## 任务行为

- `mission` 模式下 `/usb_cam` 在二维码和生产 OCR 阶段均持续运行；Python 2
  任务节点通过 `cv_bridge` 保存 `/usb_cam/image_raw` 当前帧，Python 3 helper
  只读取该图片，不再争抢 `/dev/ucar_video`。
- 每个生产状态点的目标 yaw 按当前点指向下一点确定，37 沿用向西方向。
- Python 2 ROS 节点通过持久 Python 3 helper 复用
  `/home/ucar/ocr3/ppocr_trt/python/live_ppocr.py` 的 `PPOcr`、中文白名单和三类候选词；
  引擎与相机只初始化一次，不使用 Tesseract。
- OCR 框水平误差超过 18 px 时只发布受限角速度；每次转动后停车、重新拍照确认。
- 雷达使用新的 `/scan` 帧、正前方 ±3° 中位数和该帧时间的 `map ← laser` TF；
  墙点误差超过 0.18 m 的结果不计入正式三点。
- 结果原子写入
  `~/.ros/ucar_2026_observations/run_*/observations.json`。

## 验证结果

- 本机 Python 语法检查通过：
  `production_task_geometry.py`、`production_task_perception.py`、
  `production_task_2026.py` 和地图更新工具。
- 本机无 ROS 单测 15 项运行：13 项通过，2 项因本机没有 ROS Python 模块按设计跳过。
- 两份 JSON 均为 459 个唯一编号点、56 个墙候选点，SHA-256 完全一致。
- launch 与 package XML 解析通过，`git diff --check` 通过。
- PNG 已人工查看：419–445 与 446–459 标签位置可读，旧编号和底图保持不变。
- 已同步到小车 `~/ucar_ws/src/ucar_2026`；10 个部署文件与本机 SHA-256 全部一致。
- 小车 Ubuntu 18.04 的 Python 2/3 语法检查通过，Catkin `ucar_2026` 构建通过，
  Python 2 单测 15/15 通过。
- `live_ppocr.py` CUDA 引擎加载成功，中文白名单命中 12 类字符；原生相机探针实际识别
  “食品加工车间”，分类置信度约 73.36%，图片和文字框均成功返回，helper 正常退出。
- 2026-07-29 完整 supervisor 会自动拉回 `/usb_cam`，原生相机模式在约 8 秒内
  连续 66 次遇到设备忙；因此生产 OCR 已切换为 ROS 图像保存模式。车端重新构建成功，
  原有 15 项测试全部通过，Python 3 相机测试 1/1 通过。
- ROS 图像模式实车运行成功经过
  `3 → 2 → 1 → 11 → 21 → 31 → 32 → 33 → 34 → 35 → 4 → 5 → 6 → 7`；
  `34 → 35 → 4` 新路线已实际验证。前往 8 时在距目标约 `0.21 m` 处被车体轮廓
  碰撞检查连续挡停，并以 move_base status 4 安全结束。
- `roslaunch --nodes ucar_2026 2026.launch task_enabled:=true` 静态解析通过；
  没有启动任务、相机 ROS 节点或底盘运动。

## 已知限制

- 未在本机编译车端 ROS；已按仓库规则只在小车 Ubuntu 18.04 构建。
- ROS 图像 OCR 已在实车运行，但多次识别框水平误差仍约 `280 px`，6 次受限旋转内
  未收敛；尚未得到三个完成居中并通过雷达误差门限的正式墙点结果。
- `7 → 8` 的全局规划是近似水平直线，但实车在点 7 后出现约 `0.10 m` 横向偏差，
  随后局部车体轮廓检查持续判定前方受阻。未确定新的安全路线或定位修正前不得强制通过。
- 首次实车必须先跑 manual 安全门，再分别验证“仅释放/拍照”“单点低速居中”
  和完整任务。若出现 NaN、`TF_NAN_INPUT`、CRC、`head_len` 或传感器掉线，
  必须先停车并重启导航/底盘里程计链路。
- `lidar_loc` 启动参数仍以起点 `(-0.25, 2.75, 0)` 为初值。任务中途停车并停止
  完整链路后，不得在车辆仍位于其他点时直接重启 mission；必须先把车放回起点，
  否则地图位姿会与实车位置不一致。
- `MODEL_ROUTING.md` 当前为 0 字节；本次按用户要求使用了并行子智能体，但没有可读取的
  额外模型路由细则。
