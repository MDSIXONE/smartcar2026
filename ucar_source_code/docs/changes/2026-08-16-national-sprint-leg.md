# 2026-08-16：国赛起点→52 拆三段（起点→70→288→52），CymPlanner 新增 mode3_sprint 参数集

## 目的

国赛路线"起点→52"（staging）途中 70 与 65 之间的路段需要加速通过：把原来
起点→52 的直航拆成 起点→70→288→52 三段，其中 70→288 为沿 y=1.75 的
直线加速段（朝向 180°），该段切换 CymPlanner 新增的 `mode3_sprint`
参数集（linear_x_gain 5.0、max_vel_x 1.20）通过，到达 288 后切回 `point`，再走原
288→52 段进入 staging。

## 涉及文件

- `ucar_ws/src/cym_planner/include/cym_planner.h`
  - 新增成员 `PlannerTuning sprint_tuning_;` 与 `bool sprint_enabled_;`。
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
  - 匿名命名空间新增 `sprintDefaults()`（linear_x_gain 5.0、
    max_vel_x 1.20、angular_gain 5.0、final_yaw_tolerance 0.05；
    2026-08-16 实车两轮调参：先 max_vel_x 翻倍
    0.60→1.20 仍慢，根因是点模式稳态速度 = 前视距离 0.20 ×
    linear_x_gain，gain 1.5 时稳态仅 ~0.3 m/s，max_vel_x 只是上限；
    随后 linear_x_gain 1.5→5.0，稳态 ≈1.0 m/s；第二轮航向环 P 翻倍
    angular_gain 2.5→5.0，朝向容差 final_yaw_tolerance 0.15→0.05；
    第三轮航向环 P 再翻倍 angular_gain 5.0→10.0（受 max_vel_theta
    0.80 钳制，只加快小角度收敛））；
  - `pointDefaults()` 默认 `final_yaw_tolerance` 0.10→0.05（与 yaml
    mode1_point 当前值一致，yaml 总是加载、此值仅兜底）；
  - `initialize()` 新增 `readTuning("mode3_sprint", ...)` 与
    `sanitizeTuning(sprint_tuning_)`；
  - 构造函数初始化列表新增 `sprint_enabled_(false)`；
  - `navigationModeCallback()` 改为三态（point / body_projection /
    sprint，`"sprint"`、`"fast"` 进入 sprint）；
  - `activeTuning()` 优先返回 `sprint_tuning_`（`selectPlannerTuning`
    保持两参版本不动）；
  - 启动日志追加 mode3 sprint 摘要行。
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - 新增 `mode3_sprint:` 参数段（max_vel_x 1.20，其余同 mode1 当前实车值）。
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
  - `__init__` 新增 `sprint_enabled` / `sprint_start_point_number` /
    `sprint_end_point_number`（默认 70 / 288，sprint_enabled 默认 False）；
  - `all_required_numbers` 在 sprint_enabled 时追加起点/终点编号；
  - `switch_to_point_mode()` 改为调用新的通用
    `switch_navigation_mode(mode)`（"point"/"sprint"，保持
    `SET_POINT_NAVIGATION_MODE` 状态字符串与原有发布语义不变）；
  - `run_mission()`：sprint_enabled 时走 起点→70（point）→288
    （sprint，发布 `PRODUCTION_SPRINT_LEG 70 -> 288`）→52（point）；
    否则保持原起点→52 直航（"STAGING_52" 状态不变，测试兼容）。
- `ucar_ws/src/ucar_2026_national/launch/2026.launch`
  - production_task_2026 节点内新增 `sprint_enabled=true`、
    `sprint_start_point_number=70`、`sprint_end_point_number=288`。
- `docs/operations.md`
  - 新增 sprint 改动的部署/验证步骤（见该文件"2026 主流程三场比赛副本"
    章节后的 `## 国赛 sprint 加速段（70→288）` 小节）。

## 改动说明

- sprint 参数集 `linear_x_gain = 5.0`、`max_vel_x = 1.20`、
  `angular_gain = 5.0`（航向环 P 翻倍）、`final_yaw_tolerance = 0.05`
  （2026-08-16
  实车两轮调参：先 max_vel_x 翻倍 0.60→1.20 仍慢，根因是点模式稳态速度 =
  前视距离 0.20 m × linear_x_gain，gain 1.5 时稳态仅 ~0.3 m/s，
  max_vel_x 只是上限；随后 gain 1.5→5.0，稳态 ≈1.0 m/s；
  第二轮 angular_gain 2.5→5.0、final_yaw_tolerance 0.15→0.05），其余与
  mode1_point 当前实车调优值
  一致：终点只对朝向（final_linear_x_gain 0.0）、无命令扫掠
  （command_sweep_time 0.0）、保留前视判障（obstacle_lookahead_distance
  0.25）。
- `mode1_point.final_yaw_tolerance` 0.15→0.05（2026-08-16 第二轮）：
  到达点朝向容差严格到 ±2.9°，保证起点→70 的 180° 对准与后续
  point 导航到达朝向更准；影响所有 point 模式到达（生产路线 OCR
  站点、终点 441 等），时间影响约每点 1-2 s。
- 70→288 为开阔直线段，一般不会触发 elastic 绕行；若触发，速度会被
  `elastic_max_vel_x 0.07` 钳制（属既有保护，非本改动引入）。
- 非 sprint 路径行为完全不变：`sprint_enabled=false` 时 run_mission 仍只发
  送起点→52 一个导航目标，"STAGING_52" 状态保留；`switch_to_point_mode`
  名称与 `SET_POINT_NAVIGATION_MODE` 状态字符串不变，既有测试
  （mock switch_to_point_mode、object.__new__ 构造）不受影响。
- 任务节点用 `getattr(self, "sprint_enabled", False)` 兼容无 `__init__`
  的测试对象。

## 验证

- 本机 Python 3：
  - `python -m py_compile production_task_2026.py` 通过（仅语法检查；
    该文件为 Python 2 运行时脚本，本机不执行）。
  - YAML 解析：`mode3_sprint.linear_x_gain == 5.0`、
    `mode3_sprint.max_vel_x == 1.2` 通过。
  - XML 解析 `2026.launch` 通过。
- 已部署小车（192.168.8.231，2026-08-16）：
  - 5 个文件 scp 同步，本地/车端 SHA-256 逐一比对一致；
  - 车端 `catkin_make -DCATKIN_WHITELIST_PACKAGES="ucar_2026;lane_proto;
    ucar_2026_national;ucar_2026_extra;cym_planner" --pkg cym_planner`
    编译通过，`devel/lib/libcym_planner.so` 重新生成
    （白名单现值已含 cym_planner，operations.md 已同步）；
  - `production_task_2026.py` 权限复核 0755。
  - 未启动任务/导航，sprint 段实车表现待实际跑任务时按日志
    （`PRODUCTION_SPRINT_LEG 70 -> 288`、`cym_planner switched to
    mode3_sprint`）确认。

## 已知限制

- sprint 段仍为 move_base 导航，实际速度受 CymPlanner 判障/弹性绕行
  影响；若 elastic 激活，速度会被 `elastic_max_vel_x 0.07` 钳制——
  70→288 为开阔直线一般不会激活。
- 修改 `ucar_cym_planner_params.yaml` 后需重启 2026.launch 生效；
  move_base 不会热加载参数。
- 修改 C++ 后需在小车端重新编译 cym_planner 并同步
  （见 docs/operations.md 部署步骤）。

---

# 第四轮（同日追加）：接近目标自动减速 + 冲刺终点提前坡顶中点

## 目的

70→288 冲刺段中间有坡，坡顶在 67 (0.75,1.75) 与 290 (1.0,1.75) 连线中点
= (0.875, 1.75)；原冲刺终点 288 (0.0,1.75) 在坡顶下坡侧，1.0 m/s 冲刺
惯性会冲过头刹不住。方案：
1. 冲刺终点提前到坡顶中点 (0.875, 1.75)（任务节点 `sprint_end_x/y`
   参数，非空时优先于编号点 288）；
2. CymPlanner sprint 模式新增"接近目标自动减速"：距终点
   `approach_decel_distance`（1.0 m）起，速度上限按剩余距离线性压降到
   `approach_min_vel_x`（0.12 m/s）；
3. sprint 参数集 `final_linear_x_gain` 0.0→0.6：终点阶段允许位置回拉
   修正（轻微冲过头时倒回）。

point/body 模式行为不变：approach 参数默认 0.0 = 禁用。

## 涉及文件（第四轮增量）

- `ucar_ws/src/cym_planner/include/cym_planner/planner_tuning.h`
  - `PlannerTuning` 新增 `approach_decel_distance` / `approach_min_vel_x`
    （构造函数默认 0.0 = 禁用；`selectPlannerTuning` / `headingSpeedScale`
    未动）。
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
  - `sprintDefaults()`：`final_linear_x_gain` 0.0→0.6，新增
    `approach_decel_distance = 1.0`、`approach_min_vel_x = 0.12`；
  - `readTuning()` 新增两个 approach 参数读取；
  - `sanitizeTuning()` 新增 approach 参数 clamp（0.0~2.0 / 0.0~1.0）；
  - `computeVelocityCommands()`：主循环速度计算改为
    `approach_max_vel`（距终点 < approach_decel_distance 时速度上限
    按 `max(approach_min_vel_x, max_vel_x * 剩余距离/approach_decel_distance)`
    线性压降），elastic 钳制与 pose_adjusting 分支未动。
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - `mode3_sprint` 段：`final_linear_x_gain 0.0→0.6`，新增
    `approach_decel_distance: 1.0`、`approach_min_vel_x: 0.12`，注释追加
    第四轮说明。
- `ucar_ws/src/ucar_2026_national/scripts/production_task_2026.py`
  - `__init__` 新增 `sprint_end_x` / `sprint_end_y` / `sprint_end_xy`
    （均非空时转 float 元组，否则 None）；
  - `run_mission()` sprint 分支：`sprint_end_xy` 非空时用其作为冲刺终点
    （label `sprint end midpoint (x, y)`），否则退回编号点 288。
- `ucar_ws/src/ucar_2026_national/launch/2026.launch`
  - 新增 `sprint_end_x=0.875`、`sprint_end_y=1.75`，注释更新。

## 验证（本机，未部署）

- `python -m py_compile production_task_2026.py` 通过。
- YAML 解析断言：`approach_decel_distance == 1.0`、
  `approach_min_vel_x == 0.12`、`final_linear_x_gain == 0.6` 通过。
- XML 解析 `2026.launch` 通过。
- grep 复查：approach 参数在 planner_tuning.h /
  readTuning / sanitizeTuning / computeVelocityCommands / yaml 全部落地；
  `sprint_end_xy` 在 py 落地。
- C++ 未本机编译（只能在车上 18.04 编译）；本轮未部署小车，部署由
  后续流程执行。

## 已知限制（第四轮追加）

- 接近减速按 `final_distance`（global_plan 终点相对 base_link 距离）
  计算；若 move_base 全局路径终点与任务目标点偏差大（重规划后），
  减速区间按实际路径终点生效。
- `approach_min_vel_x` 是速度上限下限，不是目标速度；PD 输出低于该值
  时仍按 PD 输出（不会强制提速）。
- 终点提前后 288→52 段变长（坡顶 → 52），点模式 max_vel_x 0.35 正常
  通行；冲刺段总长缩短约 0.875 m。

## 第五轮（同日追加）：速度再次加大

- `mode3_sprint.linear_x_gain` 5.0→8.0（稳态 0.20×8.0=1.6 m/s）、
  `max_vel_x` 1.20→1.60，稳态顶满上限；`sprintDefaults()` 同步。
- 第六轮（同日）：加到 2.0——`linear_x_gain` 8.0→10.0（稳态
  0.20×10=2.0 m/s）、`max_vel_x` 1.60→2.0，稳态顶满 2.0；
  底盘裁剪上限 `linear_speed_max` 3.0 仍有余量。
- 末端停车仍由 approach 自动减速兜底（1.0 m 起压到 0.12 m/s），
  本轮未触碰。
- 涉及文件：`ucar_cym_planner_params.yaml`、`cym_planner.cpp`
  （sprintDefaults 默认值，与 yaml 一致）。
- 已部署小车（192.168.8.231，2026-08-16 05:53）：scp 同步两文件，
  车端 `catkin_make --pkg cym_planner` 编译通过，
  `libcym_planner.so` 重新生成；车上 yaml 确认
  `linear_x_gain: 10.0`、`max_vel_x: 2.0`。

