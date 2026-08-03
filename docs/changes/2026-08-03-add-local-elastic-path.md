# Local footprint 弹性路径

## 目的

避免 global path 的中心线在 global costmap 可通过、但局部真实车体 footprint 即将接触
墙或障碍时反复获得相同的全局重规划并中止。

## 涉及文件

- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/include/cym_planner/local_elastic_path.h`
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
- `ucar_ws/src/cym_planner/config/README.md`
- `ucar_ws/src/cym_planner/test/local_elastic_path_test.cpp`
- `ucar_ws/src/cym_planner/CMakeLists.txt`
- `docs/operations.md`

## 实现

- 当前 footprint 已经命中 254/255、未知/越界、TF 或 local costmap 不可用时保持零速并
  返回失败，绝不进行弹性偏移。
- 仅当前 footprint 清晰、前视原路径阻挡时，在 `0.25 m` 带内依次尝试左右
  `0.02..0.10 m` 的正弦平滑偏移，并以 `0.015 m` 间隔投影完整 footprint。
- 所有候选均使用合并 local master snapshot；254/255、越界和非有限值均淘汰。候选启用后
  速度受 `0.07 m/s` 与 `0.30 rad/s` 上限限制；每段同时插值位置和变形后切线朝向，
  旋转采样不超过 `0.05 rad`，最终命令仍须通过既有 0.40 秒 footprint sweep。
- 无候选才向 move_base 返回失败，允许全局规划作为兜底；不改变受限横移的硬关闭状态。
- 等价的周期性 global plan 会保留已验证带和 0.4 秒搜索计时；等价性由整段弧长 7 点
  的位置（`≤0.04 m`）和切线角（`≤0.20 rad`）共同决定，中段绕行也会被识别为变化。

## 验证

- 新增纯函数测试，覆盖带状路径在起点/终点回接原路径、非有限输入失败关闭、旋转造成
  的额外 footprint 采样，以及等价/变化 global plan 的带状态判据。
- 本机仅运行了可用的 Python 测试；C++/ROS Melodic 构建与 gtest 必须在小车 Ubuntu 18.04
  部署后完成，当前未启动 ROS 或车辆。

## 已知限制

- 合并 master costmap 不含各 Layer 来源，因此第一版对静态墙和动态障碍采用相同的严格
  候选验证，不根据 raw 254 推断来源。
- 当前版本是保守带状采样，不是完整 TEB 优化器；首次部署必须先做静态 local footprint
  回放与低速、可急停的实车验证。
