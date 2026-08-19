# 2026-08-19 441 终点独立位置收敛

## 目的

实车到生产任务终点 441 时，任务层复核的 `map -> base_link` 位置偏差仍偏大。问题来自普通 `mode1_point` 在规划器进入末端位置包络后将 `final_linear_x_gain` 设为 `0`，末端只继续校正航向。

本次只为 441 增加独立的 `destination` 规划模式：末端位置进入阈值为 `0.04m`，并保留 `final_linear_x_gain=1.0` 的位置微调。其他普通导航点继续使用 `point` 模式，位置进入阈值统一为 `0.07m`。

## 改动范围

- `cym_planner` 新增 `mode4_destination` 参数组和 `destination` 导航模式；位置进入阈值改为按模式读取。
- 三套 `production_task_2026.py` 在 `finish_at_destination()` 发出 `destination` 模式后再导航到 441。
- 既有 441 成功后的 lane handoff、`SUCCEEDED` 和 `signal_shutdown()` 保持不变；没有增加任务层停车等待或结束停车命令。
- 新增规划器参数单测和三套任务入口的模式发布回归用例。

## 验证

- 三套 Python2 任务源码可被本机 Python 语法编译；规划器 YAML 可解析；`git diff --check` 通过。
- 车端 `ucar-mini`（动态解析地址 `192.168.8.231`）已同步普通/额外任务和国赛所需的运行/测试文件，国赛任务脚本、几何/感知依赖、launch 与共享规划器共 8 个文件 SHA-256 与本地一致；`CATKIN_WHITELIST_PACKAGES` 保留为 `cym_planner`。
- 车端 Ubuntu 18.04 / ROS Melodic 已成功构建 `cym_planner`，`run_tests_cym_planner` 的 31 项 gtest 全部通过；三套任务脚本 Python2 语法、YAML 和 launch 节点解析通过。
- 本机 pytest 环境缺少 `pluggy`，未重复执行三套 ROS Python 用例；本机未执行 ROS Melodic/Catkin 构建。

## 已知限制

修改不会热加载到运行中的 `move_base`。部署后必须在车辆零速、`/odom_raw`、TF 和 `/scan` 安全检查通过后重启实际 2026 主流程，并在低速条件下观察 441 的复核误差；本次未发送运动命令。
