# 8 方位定点 OCR 扫描（ocr_search / ocr_alignment / 备用方案二）

## 目的

实车调试中 360° 连续旋转扫描存在转速快时抓帧覆盖不稳、单圈耗时长的问题。
将独立 OCR 搜索/对齐流程改为**定点停靠扫描**：车辆不再连续旋转一整圈，而是按固定
方位数（默认 8 个，每 45°）逐一转到位停约 1 秒抓帧，转速提高为 1.5 rad/s。

## 变更内容

### 1. `ucar_2026/scripts/ocr_search.py`（独立 OCR 搜索）

- 重写 `scan_point`：连续 360° 旋转 → **8 方位定点停靠扫描**。
  - 以起始车头朝向为基准，每方位相对递增 45°，用 `positive_turn_increment`
    累计实际转角判定到位（恒正向旋转）。
  - 每方位：转到位 → `stop_motion` + `wait_for_chassis_stop` → 停留
    `ocr_scan_dwell_seconds`（默认 1 秒，抓帧耗时不足则补齐）→ 静止 `capture_ocr`
    抓帧一次 → `is_navigation_ocr_candidate` 检查。
  - 命中候选 → `finish_candidate`（PD 连续对齐，15s 预算，30px 容差不变）；
    **对齐失败不再跳过当前路线点，继续扫描剩余方位**（发布
    `SCAN_CONTINUE_AFTER_ALIGN_FAIL_%03d` 状态）。
  - 8 方位全部无候选 → 返回 None，外层逻辑不变（进入下一路线点；全路线无结果→441）。
- 新增参数：`ocr_scan_positions`（默认 8）、`ocr_scan_dwell_seconds`（默认 1.0）。
- 移除对 `ocr_scan_poll_period` 的使用（异步轮询逻辑删除；参数读取保留兼容）。
- 时间估算：8×(45°@1.5rad/s≈0.52s + 停 1s) ≈ 12s/点，明显短于原 360° 连续转预算。

### 2. `ucar_2026/scripts/ocr_alignment.py`（独立对齐验证）

- 从"当前姿态单帧对齐"升级为 **8 方位定点扫描 + 对齐**：
  - 新增 `scan_positions()`：逐方位转到位停 1 秒抓帧，命中强候选（置信度
    ≥ `ocr_scan_candidate_confidence`、bbox ≥ `ocr_candidate_min_bbox_area_px`）
    后调用原有 `align()` 连续对齐；对齐失败继续下一方位。
  - `odom_cb` 增加姿态 yaw 提取，新增 `current_odom_yaw()` 支持方位旋转判定。
  - `run()` 改为调用 `scan_positions()`，成功仍播报“对齐完成”。
- 新增参数：`ocr_scan_rotation_speed`（默认 1.5）、`ocr_scan_positions`（8）、
  `ocr_scan_dwell_seconds`（1.0）、`ocr_scan_candidate_confidence`（60.0）、
  `ocr_candidate_min_bbox_area_px`（1000.0）、`rotation_timeout_scale`（3.5）、
  `rotation_completion_tolerance_rad`（0.03）。

### 3. 备用方案二 `production_task_2026_alt2.py` + `2026_alt2.launch`

- 基于备用方案一（`production_task_2026_alt1.py` / `2026_alt1.launch`）复制。
- 唯一差别：OCR 扫描方式由 360° 连续旋转（`rotate_full_revolution_for_ocr`）
  改为 `scan_ocr_positions()` 8 方位定点停靠扫描（转速 1.5 rad/s、每方位停 1 秒、
  对齐失败继续剩余方位）。其余逻辑（QR、导航、守卫、连续对齐、测距、停车、
  仿真联动、巡线交接、直发 /cmd_vel）与 alt1 完全一致。
- 新增参数：`ocr_scan_positions`、`ocr_scan_dwell_seconds`，`ocr_scan_rotation_speed`
  由 0.35 改为 1.5；`rotation_timeout_scale` 由 1.8 改为 3.5（launch 覆盖）。
- 启动方式：`roslaunch ucar_2026 2026_alt2.launch task_enabled:=true` 或
  `bash ~/ucar_ws/src/ucar_2026/scripts/start_2026.sh <电脑LAN_IP> mission_alt2`。

### 4. 配套

- `CMakeLists.txt`：新增 `production_task_2026_alt2.py` 与 `test_2026_alt2_launch.py`。
- `start_2026.sh`：新增 `mission_alt2` 模式（启动 `2026_alt2.launch`）。
- 测试：新增 `test_2026_alt2_launch.py`；`test_ocr_search_launch.py`、
  `test_ocr_alignment_launch.py` 更新断言（转速 1.5、positions=8、dwell=1.0、`scan_positions` 方法）。

## 验证

- 本地：`py_compile` 三个脚本通过；XML 解析 4 个 launch 通过；135 项 unittest
  全部通过（88 项为 ROS 依赖跳过）。
- 车端（192.168.8.231，同步后）：三个脚本 Python2 AST 通过；11 个文件 SHA-256
  与本地一致；4 个 launch XML 解析通过；三个相关 unittest 通过；`chmod +x`
  已设置。同步后未重启 ROS、未启动任务、未发送运动指令。

## 已知限制

- `ocr_scan_poll_period` 在 ocr_search.py 中不再使用（保留读取与默认值以兼容旧配置）。
- `rotate_full_revolution_for_ocr` 在 alt2 中仍保留定义但不再被 `scan_production_point`
  调用（原调用已切换到 `scan_ocr_positions`），保留作为回退。
- 8 方位扫描覆盖全周（8×45°），但候选只在其对应方位停靠窗口内抓取；若目标墙在
  两次停靠间隙被短暂遮挡，需下一轮路线重扫。