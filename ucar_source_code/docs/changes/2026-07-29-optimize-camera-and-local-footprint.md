# 相机与局部车体投影性能优化

## 目的

- 相机仅在 QR 或连续 OCR 实际使用期间采集，降低 USB、V4L2 和图像转换负载。
- 无 RViz 时关闭 CymPlanner OpenCV 调试图像的分配、绘制和发布。
- 车体投影改用每个控制周期的一份 local Costmap2D 快照，移除全局
  OccupancyGrid 订阅和完整地图传输依赖。
- 在不削弱静态墙和雷达断流门禁的前提下，把 global costmap
  `always_send_full_costmap` 改为 `false`。

## 涉及文件

- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/include/cym_planner/global_cost_semantics.h`
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/config/README.md`
- `ucar_ws/src/cym_planner/test/global_cost_semantics_test.cpp`
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
- `ucar_ws/src/ucar_2026/package.xml`
- `ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_params.yaml`
- `ucar_ws/src/ucar_nav/config/testnav20260721/local_costmap_common.yaml`
- `ucar_ws/src/ucar_nav/config/testnav20260721/global_costmap_params.yaml`
- `docs/operations.md`
- `犯错档案.md`

## 实现

### 相机

- `2026.launch` 默认 `camera_start_suspended=true`，无论是否启用自动任务，
  不使用 RViz/相机时都不采集；手工确需立即图像时可显式设为 `false`。
- 到达 52 并确认停车后调用 `/usb_cam/start_capture`，收到启动后的新鲜预热帧
  才允许 QR。
- QR 结束后暂停相机；OCR 模型加载期间保持暂停。连续生产 OCR 前重新开启并等待
  新鲜帧。
- 成功、异常和 shutdown 都先停车并停止 OCR，再 best-effort 调用
  `/usb_cam/stop_capture`。
- 停止服务会有界重试一次；最终清理仍失败时精确停止 `/usb_cam` 节点，
  防止任务结束后继续占用采集资源。
- `stop_capture` 执行 `VIDIOC_STREAMOFF`，降低采集负载但不释放 fd；原生 V4L2
  模式仍沿用精确停止 `/usb_cam` 节点的路径。

### CymPlanner

- `debug_images_enabled=false` 时不注册调试图话题，不分配或填充调试 Mat。
- local costmap 加入 StaticLayer `/map`，保留 ObstacleLayer
  `/scan_filtered`；雷达源 `expected_update_rate=0.30 s`。
- 每周期先要求 `costmap_ros_->isCurrent()`，然后锁内复制一份约 2 m × 2 m 的
  local Costmap2D，锁外完成全部路径与 Twist 投影。
- 当前 footprint、最近路径点起的 `0.30 m` 前视、实际 Twist 未来 `0.40 s`
  扫掠全部使用同一快照。
- raw 253 作为已膨胀格不重复阻挡完整 footprint；254 和 255 失败关闭。
- 删除 `/move_base/global_costmap/costmap` 订阅后，GlobalPlanner 仍使用进程内
  global Costmap2D，因此 `always_send_full_costmap=false` 不影响全局规划。

## 验证

- TDD 红灯：
  - C++ 新增 local raw 代价值测试因 helper 不存在而编译失败。
  - Python 新增相机生命周期测试因方法不存在而失败，其余 18 项通过。
- 完整绿灯：
  - 车端 Ubuntu 18.04 成功构建 `cym_planner` 和 `ucar_2026`。
  - `run_tests_cym_planner`：54 项，0 error，0 failure。
  - `run_tests_ucar_2026`：30 项，0 error，0 failure；其中任务几何、
    状态机、相机新鲜帧和停止兜底 Python 测试 23 项通过。
  - `roslaunch --nodes ucar_2026 2026.launch task_enabled:=true` 解析成功。
  - 参数展开确认 `/usb_cam/create_suspended=true`，任务节点的
    `camera_starts_suspended=true`。
  - 4 份改动 YAML 均在车端 Python 2/PyYAML 独立解析成功。
- 2026-08-03 首次实车链路验证：安全门、52 导航和三组 QR 均成功；切换
  `body_projection` 后，前往点 3 时 footprint 命中 local StaticLayer 的 raw 254
  致命格 `(-1.455, 1.455~1.485)`，move_base 依次重规划、清图和旋转恢复仍无
  安全解，最终 status 4。任务已发布零速并停止；没有降低阈值或绕过碰撞检查。

## 已知限制

- 已确认 local StaticLayer 会阻挡真实墙；点 3 的此处转向/footprint 几何仍须在
  下一轮测试前单独做静态路径与轮廓验证。`/scan_filtered` 断流使 local costmap
  变为 not current、以及 global planner 在增量发布配置下的静态验收仍未完成。
- 尚未测量优化前后控制周期和 CPU。首轮测试如重复出现
  `control cycle exceeded 50 ms` 必须立即停车。
- 完整自动任务须等上述静态检查和低速人工急停验证通过后再运行。
