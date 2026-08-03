# 完成生产任务实车验收并增强 Master 冷启动等待

## 目的

从配置起点完整运行一次 ucar_2026 正式任务，并根据实测完善快速启动流程。
首次静态启动紧跟 WSL Master 进程创建，Master 尚在检查日志目录，旧脚本的单次
3 秒检查过早失败，因此同时增加有界自动等待。

## 涉及文件

- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
- `docs/quickstart.md`
- `docs/operations.md`
- `docs/changes/2026-07-28-add-ucar-2026-production-mission.md`
- `docs/changes/2026-07-28-fix-body-projection-global-lethal.md`
- `犯错档案.md`

## 验证结果

- 冷启动静态安全门：
  - 起点定位约 `(-0.23, 2.73, -6.3°)`；
  - `/odom_raw≈20 Hz`、`/imu≈50 Hz`、`/scan_filtered≈12 Hz`；
  - 连续里程计无 NaN/Inf，两条 TF 正常；
  - 到 52 的全局规划返回 1658 个路径点。
- 完整任务：
  - 52 到达误差 `0.031 m`；
  - 二维码依次为 `…/a`、`…/d`、`…/i`；
  - 12、24、16、28、19 均到达并完成 360°；
  - 转后位置误差最大 `0.055 m`；
  - 170 到达误差 `0.018 m`，朝向 319 校验通过；
  - 最终状态 `SUCCEEDED`，结果 `success: true`。
- 全程没有 CRC、`head_len`、NaN、TF_NAN 或串口掉线。
- 更新后的 `start_2026.sh`：
  - 车端 `bash -n` 通过；
  - 在 WSL Master 尚未完成启动时立即执行 `check`，脚本显示自动等待；
  - Master 就绪后自动返回“连接成功”，没有启动导航或产生运动。

## 已知限制

- 全局规划器在部分路段仍会出现瞬时 `NO PATH`，但 action 保持活动并自行恢复；
  连续失败或 action aborted 时仍必须停车。
- `lidar_loc` 偶尔出现几十毫秒级未来外推警告；本次均立即恢复。若 TF 停滞、
  超龄持续增长或任务安全门失败，不能继续运动。
- 操作者在任务成功后手动移动了车辆，因此移动后的 TF 不能用于复核任务终点；
  验收采用移动前任务节点即时记录的 170 位置与朝向校验。
