# 2026-08-11：WSLg COPY MODE 预检固化为工作区规则

## 目的

带 GUI 的本机仿真（Gazebo/RViz，`task3_prepare.launch`）启动前漏做 WSLg COPY MODE
预检是反复出现的操作错误。本次把预检与修复流程固化为 `AGENTS.md` 的工作区规则，
保证每次启动 GUI 仿真前都必须先检查，避免带病启动。

## 涉及文件

- `AGENTS.md`：新增「启动带 GUI 的本机仿真之前必须先做 WSLg COPY MODE 预检」规则。
  内容要点（详细方法见 `犯错档案/2026-08-10.md`）：
  - 预检两条必须同时满足才算健康：
    - `grep -c 'use_gfxredir = 0' /mnt/wslg/weston.log` 必须为 `0`
    - `findmnt /mnt/shared_memory` 必须为 tmpfs 挂载
  - 窗口标题出现 `[WARN:COPY MODE]` 或 weston.log 出现 `use_gfxredir = 0` 时不得带病启动仿真。
  - 修复顺序：先 `wsl --terminate Ubuntu-20.04`；仍含 `use_gfxredir = 0` 时
    `wsl --shutdown` 完全重启 WSL（Docker Desktop 临时 Stopped，无影响），
    重启后复检通过才允许启动 GUI 仿真。
  - 注意：PowerShell 双引号内的 Bash `$()` 会被提前展开（报
    `grep: =: No such file or directory`），应使用 Base64 脚本或避免嵌套 `$()`。

## 验证结果

- 规则文本已写入 `AGENTS.md` 第 12-18 行，无重复条目。
- 本次会话启动 GUI 仿真（task3_prepare.launch）前后均未出现
  `[WARN:COPY MODE]` / `use_gfxredir = 0`，Gazebo、RViz 进程正常显示。

## 已知限制

- 预检只能降低带病启动概率，不能修复 WSLg 底层图形会话；异常仍按规则完整重启 WSL。
- `wsl --shutdown` 会使 Docker Desktop 临时停止，不影响小车链路（小车不依赖本机 WSL）。
