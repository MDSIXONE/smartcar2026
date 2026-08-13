# 终点自动交接 lane_proto 速度参数对齐手动调参命令

## 目的

主流程（2026.launch 任务 SUCCEEDED 后 `handoff_lane.sh` 自动交接）启动的
lane_proto 跑得慢，手动直接 roslaunch 同样命令却能跑到 16Hz。把交接脚本参数
与手动调参命令对齐。

## 原因

`handoff_lane.sh` 中硬编码的 lane_proto 参数是旧值，与手动调参命令不一致：

| 参数 | 修改前 | 修改后 |
| ---- | ---- | ---- |
| linear_speed | 0.2 | 0.35 |
| gain | 1.0 | 1.2 |
| template | red_template_band.png | red_template_band2.png |
| align_offset | 0.15 | 0.14 |
| start_offset | 0.25 | 0.23 |
| rate | 默认 | 20 |
| dump_every | 0 | 3 |

速度上限 0.2→0.35、增益 1.0→1.2、控制循环 rate 未显式指定（低于 20），
共同导致交接后车速与响应频率偏低。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/handoff_lane.sh`：roslaunch 参数更新，并加
  2026-08-12 对齐说明注释。
- `docs/operations.md`：`### 终点自动交接 lane_proto` 一节的手动命令与参数
  含义说明同步更新。

## 保留项

- `goal_y_lo:=0.85` 保留：该参数控制终点横线识别条带位置（2026-08-11 调大
  以补偿后轮过线），与速度无关，手动命令未传则走 launch 默认值；如后续确认
  默认值即 0.85 可再删除该显式参数。

## 验证与限制

- lane_proto 包不在本仓库，无法本地静态验证 launch 默认 rate；参数对齐后
  需在小车端跑一次完整流程（或手动交接）确认交接后速度与手动一致（约 16Hz）。
- 脚本本身为 bash，本地仅做语法级核对；部署走小车端 18.04（本机不编译）。
