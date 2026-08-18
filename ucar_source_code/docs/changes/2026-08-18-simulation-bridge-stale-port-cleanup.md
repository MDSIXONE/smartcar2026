# 2026-08-18：仿真 bridge 残留端口清理

## 目的

修复上一轮仿真完成后旧 HTTP bridge 仍监听 `11313`，下一轮启动的新 bridge 因
`Address already in use` 退出，但启动器误读旧 bridge `/status` 并把它当成新 bridge
已就绪的问题。

## 涉及文件

- `simulation/scripts/start_simulation_stack.sh`
- `simulation/bridge/sim_bridge.py`
- `ucar_source_code/docs/operations.md`

## 结果

- 启动 Gazebo 前检查 `11313` 的实际监听 PID；已有监听时直接报 PID 并停止启动。
- 等待 bridge 就绪时同时确认 HTTP 响应和监听端口归属于本次启动的 `BRIDGE_PID`。
- 将 `SIMULATION_BRIDGE_READY` 延后到 HTTP socket 成功 bind 之后；环境就绪但尚未 bind
  时改用 `SIMULATION_ENVIRONMENT_READY`，避免日志标记产生歧义。
- Ctrl-C 清理对子进程使用有界等待，5 秒内未响应时依次升级为 SIGTERM、SIGKILL。
- 操作文档增加只按已核对 PID 清理残留 bridge 的命令，禁止宽泛 `pkill`。
- 端口预检失败时直接打印当前 PID 的核对、SIGTERM 和 SIGKILL 命令，便于现场清理。

## 验证

- WSL 当前现场确认 `11313` 由旧 `sim_bridge.py` PID `78097` 监听，`/status` 返回上一轮
  `state=done`，与日志中的端口冲突一致。
- WSL 使用本地修改版通过 `bash -n` 和 `sim_bridge.py` Python 语法检查；端口所有权在空闲、
  临时监听和旧 PID 场景验证；`--wait-ready-timeout 0` 真实启动 bridge 后确认 `/status`、
  标记顺序和清理升级，最终 `11313` 无监听。随后只将两个运行文件精确部署到 WSL，SHA-256
  与本地一致；WSL 启动脚本 Bash 语法、帮助命令和 bridge Python 语法通过。不在本机编译
  Ubuntu 18.04 车端代码，也不发送运动命令。

## 已知限制

- 启动器不会自动杀掉未知监听进程，避免误伤另一份正在运行的仿真；需先按 PID 核对命令行。
- WSL `/home/car/smartcar2026/simulation` 仍有既有 tracked/untracked 改动；本轮只精确覆盖
  两个运行文件，未做整仓库 fast-forward 或完整 Gazebo/RViz 启动回归。

## 端口占用提示补充

启动器已精确部署到 WSL；当 `11313` 被 PID `303639` 占用时，实际输出了可复制的 `ps`、
`kill -TERM` 和 `kill -KILL` 命令。当前 `state=done` 的 bridge 按用户会话保留，未自动终止。
