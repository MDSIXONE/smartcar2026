# 仿真 /start 失败兜底：不中止任务，120 秒后继续主流程

## 目的

真车到达仿真区后 POST `/start` 请求仿真 bridge 启动仿真；此前 3 次重试失败会直接
`MissionAbort` 中止整个任务（2026-08-15 现场事故：`No route to host` 导致任务中断）。
本次改为：`/start` 失败不中止任务，转入 `/status` 兜底轮询，`simulation_done_timeout`
（120 秒）到期后继续小车终点与巡线流程——与 2026-08-14 的状态轮询超时继续语义一致。

同时在小车启动脚本中增加 TCP 11313 预检，把"IP 填错/WSL 端口不可达"问题提前到出发前暴露，
而不是任务进行到仿真区才失败。

## 修改

- `ucar_ws/src/ucar_2026/scripts/production_task_2026.py`
  - `simulation_request_start()`：重试耗尽后不再 `raise MissionAbort`，改为记录
    `PRODUCTION_SIMULATION_START_FAILED_CONTINUE`（含最后一次错误）并返回 `False`；
    成功路径返回 `True`。HTTP 409（already running）与 `accepted=False` 仍中止任务。
  - `run_mission()` 调用处保持不变，加注释说明兜底语义。
- `ucar_ws/src/ucar_2026/scripts/start_2026.sh`
  - `mission` 模式启动时对 `<电脑LAN_IP>:11313` 做 TCP 连通预检（`/dev/tcp`，2 秒超时），
    不可达打印排查指引（Windows 局域网 IP / bridge 已启动 / WSL2 mirrored 或 portproxy /
    防火墙）并退出；`check`/`manual` 模式仅告警不阻断。
- `ucar_ws/src/ucar_2026/test/test_production_task_geometry.py`
  - `test_simulation_request_start_aborts_after_all_retries` 改为
    `test_simulation_request_start_continues_after_all_retries`：断言不抛异常、返回
    `False`、重试 3 次、出现 `PRODUCTION_SIMULATION_START_FAILED_CONTINUE` 日志。
- `rosmaster/NETWORK_CONFIGURATION.md`：新增"故障排查：车端 No route to host"小节，
  明确必须填 Windows 局域网 IP（非 WSL `ip addr`）、WSL2 网络模式与防火墙检查、车侧
  TCP 验证命令。
- `docs/operations.md`：`start_2026.sh mission` 说明中补充 TCP 11313 启动预检行为。

## 验证

- 两个 Python 文件（Python2）通过 `ast.parse` 语法检查；`start_2026.sh` 通过 `bash -n`。
- 定向测试改断言后仅做语法检查（依赖 Python2/ROS，车上运行）。
- 现场行为：`/start` 3 次失败 → 终端保留 `PRODUCTION_SIMULATION_START_RETRY` ×3 与
  `PRODUCTION_SIMULATION_START_FAILED_CONTINUE` → 进入 `/status` 兜底轮询 120 秒 →
  到期 `PRODUCTION_SIMULATION_WAIT_TIMEOUT_CONTINUE` 继续任务。

## 已知限制

- 兜底轮询期间仿真未启动也不会自动启动：如果网络/仿真在 120 秒窗口内仍未恢复，任务按
  超时继续，仿真结果视为未确认（终端日志供赛后核查）。
- TCP 预检只验证 bridge 端口可达，不验证仿真环境（`/map`）本身已就绪。
