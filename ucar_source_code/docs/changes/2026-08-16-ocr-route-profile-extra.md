# 2026-08-16 OCR 快捷任务模板（extra 包，ucar_2026_extra）

## 目的

国赛特殊任务内容未知，需要在比赛现场通过修改一个 YAML 文件快速切换
"点顺序、数量、到达朝向、旋转角度、旋转方向、停车模式、目标文字"，
而不改动代码。本改动为 `ucar_2026_extra`（额外任务包）新增
`~ocr_route_profile` rosparam：非空时按模板逐点巡航（独立任务，不走国赛
双物品/QR/类别停车/仿真联动流程），空/缺省时完全沿用国赛
`production_route_numbers` 逻辑，行为逐字不变。

> 注意：该功能此前在 `ucar_2026_national`（实车主版本）实现并验证过，
> 随后按用户要求改到 extra 包、national 已还原。**本改动只涉及
> `ucar_2026_extra`，`ucar_2026_national` 与 `ucar_2026` 均未改动。**

## 涉及文件

- `ucar_ws/src/ucar_2026_extra/config/ocr_route_profile.yaml`（新增）
  - 模板文件：`ocr_route_profile: []` 为默认（不启用快捷模式）；
    注释给出每个条目的字段说明与示例。
  - 字段：
    - `point` 必填，网格编号点（须存在于 production_full_grid_all_numbered.json）
    - `heading_deg` 可选，到达朝向（度），缺省 None（保持当前车头朝向到达）
    - `rotate_angle_deg` 可选，到点后旋转扫描角度（度），缺省 360
    - `rotate_dir` 可选，ccw/cw，缺省 ccw
    - `stop_mode` 可选，wall（国赛激光测墙）/ free（随机位置直接记录位姿），缺省 wall
    - `target_texts` 可选，目标文字列表；非空时"检测文本包含任一目标文字"即命中，
      为空时沿用国赛三类别（日用品/食品/电子产品）
- `ucar_ws/src/ucar_2026_extra/launch/2026.launch`
  - production_task_2026 节点参数区（production_route_numbers 附近）新增
    `<rosparam file="$(find ucar_2026_extra)/config/ocr_route_profile.yaml"
    param="ocr_route_profile"/>`；文件不存在时 rosparam 加载失败会直接暴露。
- `ucar_ws/src/ucar_2026_extra/scripts/production_task_2026.py`
  - 模块级 `_TEXT_TYPES = (basestring,)`（本文件纯 Python 2，str/unicode
    均需接受；注释已说明）。
  - `__init__`：`production_route_numbers` 加载后新增
    `self.ocr_route_profile = rospy.get_param("~ocr_route_profile", [])`；
    新增 `validate_ocr_route_profile()` 在 `require_points` 之后逐条校验
    （条目必须为 dict、point 存在且 int 化后存在于 grid、heading_deg 可转 float、
    rotate_angle_deg 有限且 >0、rotate_dir 仅 ccw/cw、stop_mode 仅 wall/free、
    target_texts 为 list of str），非法值抛 TaskDefinitionError。
  - `rotate_full_revolution_for_ocr(label, candidate_handler=None,
    target_progress=None, direction=1.0)`：硬编码 360°/ccw 改为参数化
    （direction 显式 float 化；target_progress None 时默认 2π）；
    timeout 计算已用 target_progress 自动适配；末尾 MissionAbort 文案改为
    动态 "%.1f-radian OCR turn"。其余逻辑（capture_task、candidate_handler、
    _ocr_turn_stop_flag、deadline 延长）不动。
  - `scan_production_point(..., rotate_angle_rad=None, direction=1.0,
    target_texts=None, stop_mode="wall")`：target_texts 非空时用包含匹配
    （检测文本须为文本类型、strip 后包含任一目标文字即命中，category 值为
    命中的文字），为空时保持 normalize_production_category；记录分支新增
    stop_mode=="free"：aligned 且 position_map 存在即 append 到 observations
    （跳过 served_wall_points 去重），target_category 为 None 或命中时置
    _ocr_turn_stop_flag，outcome=processing_category_recorded；wall 分支逐字不变。
    调用 `rotate_full_revolution_for_ocr` 处传 target_progress=rotate_angle_rad、
    direction=direction；调用 `observe_wall` 处传 stop_mode=stop_mode
    （此前实现曾漏传导致 free 模式失效，本次已确认传参）。
  - `observe_wall(..., stop_mode="wall")`：stop_mode=="free" 时对准成功后
    跳过测墙点逻辑（wait_for_fresh_front_distance / laser_map_pose /
    forward_ray_wall_intersection / nearest_numbered_point / range_residual），
    直接 `current_map_pose` 记录 position_map + stop_mode + front_distance_m=None
    并返回；wall 分支逐字不变。
  - 新增 `cruise_ocr_profile_route(profile)`（置于 cruise_production_route
    之后）：逐点 publish_state → navigate_to（heading_deg None 时保持当前
    车头朝向）→ scan_production_point（旋转角/方向/目标文字/停车模式来自
    模板）。导航复用现有 `navigate_to` → `navigate_coordinates`（cancel 由
    rotate 前的 cancel_all_goals 处理、require_plan/到达确认
    arrival_tolerance/require_safe 均在 navigate_coordinates 内），
    未新写导航函数。
  - `run_mission`：入口处新增 profile 分支——profile 非空时跳过
    WAITING_FOR_ITEM 语音等待与 QR 阶段（两者仅服务国赛类别获取），
    直接安全启动（wait_for_safe_start）→ switch_to_point_mode →
    prepare_result_directory → 相机/原生 OCR 启动 → cruise_ocr_profile_route
    → stop_native_ocr → save_observation_summary → finish_at_destination
    （441 导航 + lane 交接）→ return；profile 为空时国赛路径逐字不变。
- `ucar_ws/src/ucar_2026_extra/test/test_production_task_geometry.py`
  - 两处测试 mock（`rotate_full_revolution_for_ocr` 的 lambda / 内嵌函数）
    增加 `**_kwargs`，以接受新签名引入的 `target_progress` / `direction`
    关键字参数；否则 scan_production_point 的新传参会使测试 TypeError。
    该适配与本次签名改动配套，必要且保留。
  - 新增 `ProductionTaskOcrProfileValidationTest`（类级 skipIf
    task_module 导入失败即本机无 ROS/Python2 环境）：
    空 profile 合法、全字段合法条目通过、10 类非法条目（非 dict、缺 point、
    point 不在网格/非整数、非法 heading、旋转角 0/负/非数字、非法
    rotate_dir/stop_mode、target_texts 非 list/含非字符串）全部抛
    TaskDefinitionError；run_mission profile 模式跳过语音等待（mock
    wait_for_item_inputs 使其抛错，断言未触发、巡航被调用、终点完成）。

## 验证结果

- `python3 -m py_compile scripts/production_task_2026.py`（WSL Ubuntu 20.04，
  Python 3.8.10）：通过。
- 现有单测（WSL，python3，纯函数可跑，含本次新增的
  ProductionTaskOcrProfileValidationTest 用例）：
  - test_production_task_geometry.py：无 ROS/Python2 环境时
    production_task_2026 导入失败，几何/感知纯函数用例通过、
    依赖 task_module 的用例 skip（既有行为）；
  - test_production_task_perception.py：通过；
  - test_production_camera_ocr.py：通过。
- 未做真车/仿真运行验证（本次仅代码与配置改动，未部署）。

## 已知限制

- profile 模式是独立任务：不执行语音输入、QR 收集、类别停车播报与
  仿真联动（simulation_request_start/simulation_wait_done），终点前
  无仿真流程。
- free 停车模式不做激光测墙与墙点去重，多次命中同一目标文字会重复记录
  到 observations（由 target_texts 列表与 _ocr_turn_stop_flag 控制
  单点停转，跨点不合并）。
- rotate_angle_deg 旋转超时由 rotation_timeout_scale 放大，
  与 360° 扫描共用同一比例；小角度旋转时 timeout 下限仍为
  target_progress/speed*scale + 2.0 s。
- heading_deg 缺省时保持"当前车头朝向"到达（导航目标 yaw 取当前 map
  位姿 yaw），不额外旋转。
- yaml 中文 target_texts 在车上（Python 2）加载为 unicode；
  匹配为子串包含（非正则）。
- 本机（Windows，无 ROS）无法 import production_task_2026.py，
  ProductionTaskOcrProfileValidationTest 在无 ROS 环境整体 skip；
  完整校验用例需在装有 ROS Melodic（Python 2）的车端或等价环境执行。
- extra 包无 sprint 加速段、话题为 /ucar_2026_extra/*、
  result_directory 默认 ~/.ros/ucar_2026_extra_observations，
  与 national 的结构差异均已按 extra 自身代码适配。
