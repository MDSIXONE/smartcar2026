# 2026-08-16 仿真物理步长调整为 200 Hz

将 `simulation/src/car3/world/math.world` 的 `<physics>` 时间参数由官方
1000 Hz 物理步长调整为 200 Hz，用于解决本机仿真时间明显慢于墙钟
（官方 1000 Hz 参数下实时率约 0.25）的问题。负责人授权：world 的
时间参数可调整，模型部分不得修改。

## 背景

- 官方 `math.world` 使用 `max_step_size=0.001`（1000 Hz）、
  `real_time_update_rate=1000`；当前机器上实测实时率约 `0.25`，
  仿真时间严重滞后于墙钟。
- 将物理步长降为 200 Hz（`max_step_size=0.005`、`real_time_update_rate=200`，
  200 步 × 0.005 s = 1 s 仿真时间），计算负载降为原来的 1/5，
  目标实时率仍为 1，机械臂轨迹等待逻辑基于 `/clock` 仿真时间，不受影响。
- 下游链路频率均低于 200 Hz（grasp_attach 跟随 100 Hz、传感器 20 Hz、
  控制环 100 Hz），无需联动修改。

## 涉及文件

- `simulation/src/car3/world/math.world`：仅改 `<physics>` 段两个时间参数
  （`max_step_size` 0.001 → 0.005、`real_time_update_rate` 1000 → 200），
  并加注释说明授权范围；模型/场景/挂牌等其余内容未动。
- `simulation/src/car3/test/test_task3_realtime_budget.py`：
  `test_world_keeps_official_physics_settings` 改名为
  `test_world_uses_200hz_physics_step`，断言同步为 0.005 / 200.0。
- `simulation/src/car3/test/test_grasp_attach_state.py`：更新 world
  SHA-256 哈希为 `c1ecf61d...`（URDF 哈希不变），注释说明时间参数授权
  可调、模型部分仍锁定。
- `simulation/DEPLOYMENT.md`：第 7 节 physics 示例更新为 200 Hz 值；
  移除"禁止修改 world / 独立实验分支"过时表述，改为时间参数授权可调。
- `simulation/FAQ.md`：更新物理参数现状与"禁止调整物理步长"的旧表述。

## 验证结果

- `python -m unittest test_task3_realtime_budget.py -v`：7 项全部通过
  （含新的 `test_world_uses_200hz_physics_step`）。
- `test_grasp_attach_state.py` 的 world 哈希已按测试归一化规则重算并更新
  （模型相关断言不受影响；该文件需在 WSL ROS 环境运行完整验证）。
- 尚未实跑 Gazebo；预期实时率从约 0.25 提升到接近 1，实跑后需观察
  碰撞/接触稳定性与抓取效果。

## 已知限制

- world 哈希回归测试无法区分"时间参数调整"与"模型改动"：任何对
  `math.world` 的后续修改都会使哈希不一致，须同步更新本记录与测试。
- 200 Hz 步长下高速碰撞穿透距离约为 1000 Hz 的 5 倍（每步 0.005 s），
  若实跑出现穿墙/接触异常，需回退到 `0.001/1000` 或折中值
  （如 `0.002/500`）并同步两个回归测试。
- `simulation/bridge/test_sim_bridge.py` 引用当前 `sim_bridge.py` 中已不
  存在的 `GazeboPhysicsRateController`（既有失效测试，与本改动无关）。
