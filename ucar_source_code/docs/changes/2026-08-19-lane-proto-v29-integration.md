# lane_proto V29 接入省赛/国赛主流程（2026-08-19）

## 目的

将用户提供的 `tmp/lane_proto_v29.zip` 作为最新 `lane_proto` 运行包，接入标准、省赛、
国赛和额外主流程，并清理上一版临时 `goal_control_mode` 接口。

## 参数约定

- V29 使用 `goal_mode=visual|both`、`goal_map_xy`，不使用 `goal_control_mode`、
  `goal_grid_path`、`goal_point_111/120`。
- `use_lidar=self` 与 `use_lidar=true` 都由 `lane_follow` 自己读取 `/scan` 做终点
  `CORNER_ADJUST` 雷达闭环；`use_lidar=false` 才是旧的 50cm 盲推路径。
- 国赛：`use_lidar=true`、`goal_mode=visual`、`board_in_lane=true`、
  `go_around=true`。
- 省赛/额外：`use_lidar=self`、`goal_mode=visual`、`board_in_lane=false`、
  `go_around=false`。
- 三套入口统一使用用户指定的 V29 视觉、绕板和速度参数；独立 handoff 脚本同步使用
  `take_cam_on_start=true`，主流程常驻节点仍使用共享相机和底盘。

## 影响文件

- `ucar_ws/src/lane_proto/`：由 V29 压缩包覆盖；保留工作区中不在压缩包内的其他测试文件。
- `ucar_ws/src/ucar_2026/launch/2026.launch`
- `ucar_ws/src/ucar_2026_national/launch/2026.launch`
- `ucar_ws/src/ucar_2026_extra/launch/2026.launch`
- 三套 `scripts/handoff_lane.sh`
- `ucar_ws/src/ucar_2026_extra/config/ocr_route_profile.yaml`：补齐车端缺失的额外流程必需资源。
- `docs/operations.md`、`operations-national.md`、现场参数表和运行记录。

## 验证

本地完成 V29 核心文件 SHA-256、三套 launch XML、Python AST/语法、旧接口扫描和
`lane_proto` 回归测试；车端部署需额外执行 `chmod +x lane_follow.py`，然后完成
Python2 语法、三套 `roslaunch --nodes`、远端哈希和进程清理验收。

车端首次启动暴露 `lane_follow.py` 权限为 `644` 的部署问题；补充执行 `chmod +x` 后，
文件权限为 `755`，无运动 `roslaunch --nodes` 已能列出 `/lane_follow`。

## 限制

V29 的 `goal_mode=both` 只接受单个 `goal_map_xy`；当前主流程使用 `visual`，避免将
一个固定地图点错误套到不同支路。若以后恢复地图点触发，必须先明确每条支路的目标坐标
和动态切换机制，再改 launch/节点并补充实车回归。
