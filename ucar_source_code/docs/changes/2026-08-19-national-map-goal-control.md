# 2026-08-19 国赛终点地图坐标闭环

## 目的

替换终点视觉命中后的“两面墙雷达拟合”停车逻辑。现场日志出现 `两面墙不完整(x=2,y=0)` 时，节点会保持零速度等待雷达拟合超时；新逻辑不再用墙控制，也不调用终点导航。

## 行为

- 终点视觉命中后进入 `MAP_GOAL_ADJUST`。
- 从生产网格 `production_full_grid_all_numbered.json` 读取目标：物理左支路为点 120 `(2.25,-2.75)`，物理右支路为点 111 `(-2.25,-2.75)`，中间支路为点 111。
- 左右支路目标航向为 `-90°`，中间支路目标航向为 `180°`。
- 先原地调整航向，再将地图坐标误差转换到 `base_link`，用 `x/y` 速度闭环对齐。
- 连续 5 帧进入 `±0.04m` 坐标误差后发布 `GOAL`，并播报“任务完成”。TF 缺失时发布零速度；30 秒未到位时以 `ABORT` 停车。
- 国赛入口将 `use_lidar=false`；`/scan` 仍可供拦路板检测使用，但不参与终点停车。

## 涉及文件

- `ucar_ws/src/lane_proto/scripts/lane_common.py`
- `ucar_ws/src/lane_proto/scripts/lane_follow.py`
- `ucar_ws/src/lane_proto/launch/lane_proto.launch`
- `ucar_ws/src/lane_proto/package.xml`
- `ucar_ws/src/lane_proto/test/test_map_goal_control.py`
- `ucar_ws/src/lane_proto/test/test_lane_runtime.py`
- `ucar_ws/src/ucar_2026_national/launch/2026.launch`
- `docs/operations.md`
- `docs/operations-national.md`

## 验证结果

- 本机 `python -m unittest discover -s ucar_ws/src/lane_proto/test -p 'test_*.py' -v`：修正映射后 24 项通过，3 项因无 ROS Melodic Python2 运行时跳过。
- 本机 `py_compile`、两个 launch XML 解析、`git diff --check`：通过。
- 已按动态 DNS `ucar-mini` 同步修正后的 5 个运行文件；车端 Python2 语法、XML、111/120 网格 JSON 和 SHA-256 校验通过。不启动 ROS、不重启任务、不动车辆。

## 已知限制

- 现场首轮验证发现点号坐标本身正确，但“物理左/右支路”与地图 `x` 正负的映射曾反转：实际左支路触发时 `map x` 约为 `+2.18m`，旧代码却追踪 111 的 `-2.25m`，因此穿过中线；已按用户确认修正为物理左→120、物理右→111。
- 尚未进行 Ubuntu 18.04 / ROS Melodic 实车运动验证；正式启动前必须确认 `/odom_raw` 有限、`odom -> base_link` 与 `map -> base_link` TF 有限且车辆零速。
- 目标点依赖车端地图与定位坐标系一致；若地图版本或定位原点变化，必须重新核对 111/120 坐标。
