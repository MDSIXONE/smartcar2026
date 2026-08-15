# 2026-08-16 仿真物理步长调整为 333 Hz（实时率约 0.6）

将 `simulation/src/car3/world/math.world` 的 `<physics>` 时间参数由官方
1000 Hz 物理步长调整为 333 Hz 步长（`max_step_size=0.003`）并保留
`real_time_update_rate=200`，用于解决本机仿真时间慢（官方 1000 Hz 参数下
实时率约 0.25）以及 200 Hz 步长 + RTF 1.0 时任务速度过快翻车的问题。
负责人授权：world 的时间参数可调整，模型部分不得修改。

## 背景

- 官方 `math.world` 使用 `max_step_size=0.001`（1000 Hz）、
  `real_time_update_rate=1000`；当前机器上实测实时率约 `0.25`，
  仿真时间严重滞后于墙钟。
- 第一次调整：`max_step_size=0.005`（200 Hz）+ `real_time_update_rate=200`，
  实测 RTF=1.0，任务速度过快导致翻车。
- 第二次调整（当前）：`max_step_size=0.003`（约 333 Hz，步长更细，
  接触/碰撞更稳定）+ `real_time_update_rate=200`（钳制上限），
  RTF = 200 步 × 0.003 s = **0.6**，计算负载为官方的 1/5。
- 机械臂轨迹等待逻辑基于 `/clock` 仿真时间，不受 RTF 变化影响。
- 下游链路频率均低于 333 Hz（grasp_attach 跟随 100 Hz、传感器 20 Hz、
  控制环 100 Hz），无需联动修改。

## 涉及文件

- `simulation/src/car3/world/math.world`：仅改 `<physics>` 段时间参数
  （`max_step_size` 0.001 → 0.003、`real_time_update_rate` 1000 → 200），
  并加注释说明授权范围与调整原因；模型/场景/挂牌等其余内容未动。
- `simulation/src/car3/test/test_task3_realtime_budget.py`：
  `test_world_keeps_official_physics_settings` 改名为
  `test_world_uses_200hz_max_update_rate`，断言同步为 0.003 / 200.0。
- `simulation/src/car3/test/test_grasp_attach_state.py`：更新 world
  SHA-256 哈希为 `48045b11...`（URDF 哈希不变），注释说明时间参数授权
  可调、模型部分仍锁定。
- `simulation/DEPLOYMENT.md`：第 7 节 physics 示例更新为 0.003 / 200 值；
  移除"禁止修改 world / 独立实验分支"过时表述，改为时间参数授权可调。
- `simulation/FAQ.md`：更新物理参数现状与"禁止调整物理步长"的旧表述。

## 验证结果

- `python -m unittest test_task3_realtime_budget.py -v`：7 项全部通过
  （含新的 `test_world_uses_200hz_max_update_rate`）。
- `test_grasp_attach_state.py` 的 world 哈希已按测试归一化规则重算并更新
  （模型相关断言不受影响；该文件需在 WSL ROS 环境运行完整验证）。
- WSL 部署（`/home/car/smartcar2026-simulation`）实跑 `task3_prepare.launch`
  （GUI + RViz + 导航全负载）验证：
  - 21 项回归测试全部通过（test_task3_realtime_budget + test_grasp_attach_state）；
  - 200 Hz 步长（0.005）阶段实测 `/clock` **200.000 Hz**（RTF ≈ 1.0），
    任务速度过快翻车，经负责人确认改为 0.003 / 200（RTF ≈ 0.6）；
  - 333 Hz 步长（0.003）阶段：RTF 探针 6 秒窗口实测
    `clock_rate=190.7 Hz`、**RTF=0.572**（钳制在 0.6 目标附近；启动初期
    Gazebo 追赶仿真时间时 `/clock` 瞬时可达 333 Hz）；机械臂初始姿态
    平滑到位，`/odom` 正常发布；
  - 启动日志无新增错误（既有 `cannot set gravity on car3::arm_link6`
    警告为历史行为，不影响任务）。

## 已知限制

- world 哈希回归测试无法区分"时间参数调整"与"模型改动"：任何对
  `math.world` 的后续修改都会使哈希不一致，须同步更新本记录与测试。
- 0.003 s 步长下高速碰撞穿透距离约为 1000 Hz 的 3 倍；若实跑仍出现
  穿墙/接触异常，可继续收细步长或回退 `0.001/1000`，并同步两个回归测试。
- `simulation/bridge/test_sim_bridge.py` 引用当前 `sim_bridge.py` 中已不
  存在的 `GazeboPhysicsRateController`（既有失效测试，与本改动无关）。
