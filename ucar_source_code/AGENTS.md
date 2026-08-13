# Workspace Rules
- 本目录为统一仓库 [MDSIXONE/smartcar2026](https://github.com/MDSIXONE/smartcar2026) 中的小车源代码部分；历史来源为原 smartcar2026 仓库 `simulation_real` 分支。
- 小车的系统为：UBUNTU 18.04，代码只能在车上的18.04编译，不能在本机编译后上传。
- 小车端的 ROS 节点、诊断终端和启动命令可以以本机 WSL Ubuntu 20.04 的 ROS Master 的 为Master；地址按 `rosmaster/NETWORK_CONFIGURATION.md` 动态发现或显式配置。所有 `source` 完成后均须设置当前 `ROS_MASTER_URI`，不得使用或启动小车本机的 `roscore`。
- 小车诊断或运动前，若日志中出现 `wheelodom`、`/odom_raw` 的位置为 `NaN`，或 TF 报 `TF_NAN_INPUT`，必须先发布零速度并重启导航/底盘里程计链路；仅在 `/odom_raw` 为有限值且 `odom -> base_link`、`map -> base_link` TF 均恢复后，才允许继续定位、导航或旋转测试。
- 每次完成代码、配置或资源文件改动后，必须在 `docs/changes/` 新增或更新对应的本地改动文档，记录目的、涉及文件、验证结果和已知限制。
- 每次改动涉及构建、启动、部署、验证或回滚命令时，必须同步更新 `docs/operations.md` 中的本地操作命令文档。
- 每次需要备份时，只能把本地的上传到github对应分支；小车端不得保留备份目录或归档文件。
- 若用户修改了本地参数，就把本地参数同步到小车，若用户在下次端修改了，就把小车端的也同步到本机
- 每次启动完后不得在后端残留启动终端，需要停止后才能结束对话
- 启动带 GUI 的本机仿真（Gazebo/RViz，如 `task3_prepare.launch`）**之前**，必须先做 WSLg COPY MODE 预检；窗口标题出现 `[WARN:COPY MODE]` 或 weston.log 出现 `use_gfxredir = 0` 时，不得带病启动仿真。预检与修复（详见 `docs/ai-records/mistakes/2026-08-10.md`）：
  ```bash
  # 预检：两条必须同时满足才算健康
  grep -c 'use_gfxredir = 0' /mnt/wslg/weston.log   # 必须为 0
  findmnt /mnt/shared_memory                         # 必须为 tmpfs 挂载
  ```
  预检不通过时先 `wsl --terminate Ubuntu-20.04`；若 weston.log 仍含 `use_gfxredir = 0`，必须 `wsl --shutdown` 完全重启 WSL（Docker Desktop 会临时 Stopped，无影响），重启后复检两条预检通过，才允许启动 GUI 仿真。注意：从 PowerShell 检查 weston.log 时，双引号内的 Bash `$()` 会被 PowerShell 提前展开（报 `grep: =: No such file or directory`），应使用 Base64 脚本或避免嵌套 `$()`。
- 编写时不能只考虑固定 IP；由于 Wi-Fi 网络环境会变化，必须支持 IP 动态变化或提供相应的配置/发现机制。
- 编写代码时参考(MODEL_ROUTING.md)来部署子智能体

## Agent skills

### Issue tracker

Issues and PRDs are tracked in this repository's GitHub Issues. See
`docs/agents/issue-tracker.md`.

### Triage labels

Use the default five-role triage vocabulary. See `docs/agents/triage-labels.md`.

### Domain docs

This is a single-context repository. See `docs/agents/domain.md`.

## Commit & Pull Request Guidelines

Use a Chinese Emoji subject in the form `Emoji 范围：简短动作`, for example `🧭 导航：优化目标点路径规划`. Use one coherent change per commit. Preferred scopes include `🤖 ROS`、`🧭 导航`、`👁️ 视觉`、`🔤 OCR`、`🗺️ 地图`、`🐛 修复`、`🧪 测试`、`🔧 配置` and `📚 文档`. Do not rewrite published history merely to restyle messages. Pull requests should explain affected sections, safety implications, validation, linked issues, and layout screenshots when needed.
